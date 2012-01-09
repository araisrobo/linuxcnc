/********************************************************************
* Description:  wou_stepgen.c
*               This file, 'wou_stepgen.c', is a HAL component that
*               It was based on stepgen.c by John Kasunich.
*
* Author: Yi-Shin Li
* License: GPL Version 2
*
* Copyright (c) 2009-2010 All rights reserved.
*
* Last change:
********************************************************************/
/** This file, 'wou_stepgen.c', is a HAL component that provides software
    based step pulse generation.  The maximum step rate will depend
    on the speed of the PC, but is expected to exceed 5KHz for even
    the slowest computers, and may reach 25KHz on fast ones.  It is
    a realtime component.

    It supports up to 8 pulse generators.  Each generator can produce
    several types of outputs in addition to step/dir, including
    quadrature, half- and full-step unipolar and bipolar, three phase,
    and five phase.  A 32 bit feedback value is provided indicating
    the current position of the motor in counts (assuming no lost
    steps), and a floating point feedback in user specified position
    units is also provided.

    The number of step generators and type of outputs is determined
    by the insmod command line parameter 'step_type'.  It accepts
    a comma separated (no spaces) list of up to 8 stepping types
    to configure up to 8 channels.  A second command line parameter
    "ctrl_type", selects between position and velocity control modes
    for each step generator.  (ctrl_type is optional, the default
    control type is position.)

    So a command line like this:

	insmod stepgen step_type=0,0,1,2  ctrl_type=p,p,v,p

    will install four step generators, two using stepping type 0,
    one using type 1, and one using type 2.  The first two and
    the last one will be running in position mode, and the third
    one will be running in velocity mode.

    The driver exports three functions.  'wou.stepgen.make-pulses', is
    responsible for actually generating the step pulses.  It must
    be executed in a fast thread to reduce pulse jitter.  The other
    two functions are normally called from a much slower thread.
    'wou.stepgen.update-freq' reads the position or frequency command
    and sets internal variables used by 'wou.stepgen.make-pulses'.
    'wou.stepgen.capture-position' captures and scales the current
    values of the position feedback counters.  Both 'update-freq' and
    'capture-position' use floating point, 'make-pulses' does not.

    Polarity:

    All signals from this module have fixed polarity (active high
    in most cases).  If the driver needs the opposite polarity,
    the signals can be inverted using parameters exported by the
    hardware driver(s) such as ParPort.

    Timing parameters:

    There are five timing parameters which control the output waveform.
    No step type uses all five, and only those which will be used are
    exported to HAL.  The values of these parameters are in nano-secondindex_enables,
    so no recalculation is needed when changing thread periods.  In
    the timing diagrams that follow, they are identfied by the
    following numbers:

    (1): 'wou.stepgen.n.steplen' = length of the step pulse
    (2): 'wou.stepgen.n.stepspace' = minimum space between step pulses
	  (actual space depends on frequency command, and is infinite
	  if the frequency command is zero)
    (3): 'wou.stepgen.n.dirhold' = minimum delay after a step pulse before
	  a direction change - may be longer
    (4): 'wou.stepgen.n.dirsetup' = minimum delay after a direction change
	  and before the next step - may be longer
    (5): 'wou.stepgen.n.dirdelay' = minimum delay after a step before a
	 step in the opposite direction - may be longer

    Stepping Types:

    This module supports Type-2 only.

    Type 2:  Quadrature (aka Gray/Grey code)
    State   Phase A   Phase B
      0        1        0
      1        1        1
      2        0        1
      3        0        0
      0        1        0
*/

/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to www.linuxcnc.org.
*/

#ifndef RTAPI
#error This is a realtime component only!
#endif

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */
#include "tc.h"                 /* motion state */
#include <math.h>
#include <string.h>
#include <float.h>
#include <assert.h>
#include <stdio.h>
#include "rtapi_math.h"
#include "motion.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <wou.h>
#include <wb_regs.h>
#include <mailtag.h>
#include <sync_cmd.h>

#define REQUEST_TICK_SYNC_AFTER 500 // after 500 tick count, request risc to sync tick count
#define MAX_CHAN 8
#define MAX_STEP_CUR 255
#define PLASMA_ON_BIT 0x02
#define FRACTION_BITS 16
#define FIXED_POINT_SCALE   65536.0     // (double (1 << FRACTION_BITS))
#define FRACTION_MASK 0x0000FFFF
#define PID_LOOP 8
// to disable DP(): #define TRACE 0
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
// FILE *dptrace = fopen("dptrace.log","w");
static FILE *dptrace;
#endif

// to disable MAILBOX dump: #define MBOX_LOG 0
#define MBOX_LOG 0
#if (MBOX_LOG)
#define MBOX_DEBUG_VARS     0      // extra MBOX VARS for debugging
static FILE *mbox_fp;
#endif

#define DEBUG_LOG 0
#if (DEBUG_LOG)
static FILE *debug_fp;
#endif

#define VEL_UPDATE_FREQ         (0.00065536)   // in second
#define VEL_UPDATE_BP           (VEL_UPDATE_FREQ * 1526)

/* module information */
MODULE_AUTHOR("Yi-Shin Li");
MODULE_DESCRIPTION("Wishbone Over USB for EMC HAL");
MODULE_LICENSE("GPL");
int step_type[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };

RTAPI_MP_ARRAY_INT(step_type, MAX_CHAN,
		   "stepping types for up to 8 channels");
const char *ctrl_type[MAX_CHAN] =
    { "p", "p", "p", "p", "p", "p", "p", "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type, MAX_CHAN,
		      "control type (pos or vel) for up to 8 channels");

const char *bits = "\0";
RTAPI_MP_STRING(bits, "FPGA bitfile");

const char *bins = "\0";
RTAPI_MP_STRING(bins, "RISC binfile");

int pulse_type = -1;
RTAPI_MP_INT(pulse_type, "WOU Register Value for pulse type");

int enc_type = -1;
RTAPI_MP_INT(enc_type, "WOU Register Value for encoder type");

//obsolete: int adc_spi_en = 0;
//obsolete: RTAPI_MP_INT(adc_spi_en, "Analog to Digital Converter Enable Signal");

int servo_period_ns = -1;   // init to '-1' for testing valid parameter value
RTAPI_MP_INT(servo_period_ns, "used for calculating new velocity command, unit: ns");


//obsolete: int gpio_leds_sel = -1;
//obsolete: RTAPI_MP_INT(gpio_leds_sel, "WOU Register Value for GPIO_LEDS_SEL");

int step_cur[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
RTAPI_MP_ARRAY_INT(step_cur, MAX_CHAN,
		   "current limit for up to 8 channel of stepping drivers");

int num_gpio_in = 64;
RTAPI_MP_INT(num_gpio_in, "Number of WOU HAL PINs for gpio input");
int num_gpio_out = 32;
RTAPI_MP_INT(num_gpio_out, "Number of WOU HAL PINs for gpio output");

//const char *thc_velocity = "1.0"; // 1mm/s
//RTAPI_MP_STRING(thc_velocity, "Torch Height Control velocity");

#define NUM_PID_PARAMS  14
const char **pid_str[MAX_CHAN];
const char *j0_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j0_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[0]");

const char *j1_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j1_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[1]");

const char *j2_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j2_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[2]");

const char *j3_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j3_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[3]");

const char *j4_pid_str[NUM_PID_PARAMS] =
        { NULL, "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j4_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[4]");

const char *j5_pid_str[NUM_PID_PARAMS] =
        { NULL, "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j5_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[5]");

const char *j6_pid_str[NUM_PID_PARAMS] =
        { NULL, "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j6_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[6]");

const char *j7_pid_str[NUM_PID_PARAMS] =
        { NULL, "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j7_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[7]");


const char *max_vel_str[MAX_CHAN] =
    { "100.0", "100.0", "100.0", "100.0", "100.0", "100.0", "100.0", "100.0" };
RTAPI_MP_ARRAY_STRING(max_vel_str, MAX_CHAN,
                      "max velocity value for up to 8 channels");

const char *max_accel_str[MAX_CHAN] =
    { "100.0", "100.0", "100.0", "100.0", "100.0", "100.0", "100.0", "100.0" };
RTAPI_MP_ARRAY_STRING(max_accel_str, MAX_CHAN,
                      "max acceleration value for up to 8 channels");

const char *pos_scale_str[MAX_CHAN] =
    { "1.0", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0" };
RTAPI_MP_ARRAY_STRING(pos_scale_str, MAX_CHAN,
                      "pos scale value for up to 8 channels");

const char *ferror_str[MAX_CHAN] =
    { "0", "0", "0", "0", "0", "0", "0", "0" };
RTAPI_MP_ARRAY_STRING(ferror_str, MAX_CHAN,
                      "max following error value for up to 8 channels");

const char *ahc_polarity = "POSITIVE";  // normally joint is lifted up when feedback level below reference
RTAPI_MP_STRING(ahc_polarity,
		"auto height control behavior");

const char *ahc_joint_str = "2";
RTAPI_MP_STRING(ahc_joint_str,
		"auto height control joint");

const char *ahc_ch_str ="0"; // ANALOG_0: analog input0
RTAPI_MP_STRING(ahc_ch_str,
                "auto height control analog channel");

const char *ahc_level_max_str ="1700";
RTAPI_MP_STRING(ahc_level_max_str,
                "auto height control: max level");

const char *ahc_level_min_str ="1100";
RTAPI_MP_STRING(ahc_level_min_str,
                "auto height control: min level");

const char *machine_param_str = "XYZA"; // XYZY, XYZY_
RTAPI_MP_STRING(machine_param_str,
                "specify machine type");

const char *pattern_type_str ="NO_TEST"; // ANALOG_0: analog input0
RTAPI_MP_STRING(pattern_type_str,
                "indicate test pattern type");

const char *probe_pin_type="DIGITAL_PIN"; // ANALOG_IN
RTAPI_MP_STRING(probe_pin_type,
                "indicate probing type");

const char *probe_pin_id= "0";         // probing input channel
RTAPI_MP_STRING(probe_pin_id,
                "indicate probing channel");

const char *probe_decel_cmd= "0";         // deceleration command for probing in user-unit/s
RTAPI_MP_STRING(probe_decel_cmd,
                "deceleration for probing");

const char *probe_analog_ref_level= "2048";
RTAPI_MP_STRING(probe_analog_ref_level,
                "indicate probing level used by analog probing");

const char *act_jnt_num="4";
RTAPI_MP_STRING(act_jnt_num,
               "actual joints controlled by risc");

int alr_output = 0x00000000;
RTAPI_MP_INT(alr_output, "Digital Output when E-Stop presents");

static int test_pattern_type = 0;  // use dbg_pat_str to update dbg_pat_type

// sim: static const char *board = "7i43u";
static const char wou_id = 0;
static wou_param_t w_param;
static int pending_cnt;
static int normal_move_flag[MAX_CHAN] = {0, 0, 0, 0, 0, 0, 0, 0};
//static uint8_t is_probing = 0;

#define JNT_PER_WOF     2       // SYNC_JNT commands per WOU_FRAME


//trace INDEX_HOMING: static int debug_cnt = 0;


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/** This structure contains the runtime data for a single generator. */

/* structure members are ordered to optimize caching for makepulses,
   which runs in the fastest thread */

typedef struct {
    // hal_pin_*_newf: variable has to be pointer
    // hal_param_*_newf: varaiable not necessary to be pointer
    /* stuff that is read but not written by makepulses */
    int64_t rawcount;
    hal_bit_t prev_enable;
    hal_bit_t *enable;		/* pin for enable stepgen */
    hal_u32_t step_len;		/* parameter: step pulse length */
    int step_type;		/* stepping type - see list { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };above */
    int num_phases;		/* number of phases for types 2 and up */
    hal_bit_t *phase[5];	/* pins for output signals */
    /* stuff that is not accessed by makepulses */
    int pos_mode;		/* 1 = position command mode, 0 = velocity command mode */
    hal_u32_t step_space;	/* parameter: min step pulse spacing */
    hal_s32_t *pulse_pos;	/* pin: pulse_pos to servo drive, captured from FPGA */
    int32_t   prev_enc_pos;     /* previous encoder position for "vel-fb" calculation */
    hal_s32_t *enc_pos;		/* pin: encoder position from servo drive, captured from FPGA */
    hal_float_t *switch_pos;	/* pin: scaled home switch position in absolute motor position */
    hal_float_t *index_pos;	/* pin: scaled index position in absolute motor position */
    hal_bit_t *index_enable;	/* pin for index_enable */
    hal_float_t pos_scale;	/* param: steps per position unit */
    hal_float_t *pos_scale_pin;  /* param: steps per position unit */
    double scale_recip;		/* reciprocal value used for scaling */
    double accel_cmd;           /* accel_cmd: difference between vel_cmd and prev_vel_cmd */
    double vel_cmd;	        /* pin: velocity command (pos units/sec) */
    double prev_vel_cmd;        /* prev vel cmd: previous velocity command */
    hal_float_t *pos_cmd;	/* pin: position command (position units) */
    double prev_pos_cmd;        /* prev pos_cmd: previous position command */
    hal_float_t *pos_fb;	/* pin: position feedback (position units) */
    hal_float_t *vel_fb;        /* pin: velocity feedback */
    double      prev_pos_fb;    /* previous position feedback for calculating vel_fb */
    hal_bit_t *ctrl_type_switch;/* switch between vel ctrl and pos ctrl (valid for vel ctrl only) */
    hal_bit_t *ferror_flag;     /* following error flag from risc */
    int32_t    prev_ctrl_type_switch;
    hal_float_t freq;		/* param: frequency command */
    hal_float_t maxvel;		/* param: max velocity, (pos units/sec) */
    hal_float_t maxaccel;	/* param: max accel (pos units/sec^2) */
    int printed_error;		/* flag to avoid repeated printing */

    hal_s32_t *home_state;	/* pin: home_state from homing.c */
    hal_s32_t prev_home_state;	/* param: previous home_state for homing */
    double sum_err_0;
    double sum_err_1;
    
    /* pid info */
    hal_float_t *pid_cmd;
    hal_float_t *cmd_error;       /* cmd error */
    hal_float_t *pid_output;      /* pid output */
    
    /* motion type be set */
    int32_t motion_type;          /* motion type wrote to risc */
    
    /* debug info */
    hal_s32_t   *joint_cmd;       /* increased pulse counts of this BP */

} stepgen_t;

typedef struct {
    // Digital I/O: 16in 8out
    // Analog I/O: 32bit
    hal_float_t *a_in[1];
} gpio_t;

typedef struct {
    // Analog input: 0~4.096VDC, up to 16 channel
    hal_s32_t *in[16];
    // TODO: may add *out[] here
} analog_t;

typedef struct {
    /* plasma control */
    hal_bit_t *thc_enbable;
    //TODO: replace plasma enable with output enable for each pin.
    hal_bit_t *plasma_enable;
    /* sync input pins (input to motmod) */
    hal_bit_t   *in[64];
    hal_bit_t   *in_n[64];
    uint32_t    prev_in0;
    uint32_t    prev_in1;
    hal_float_t *analog_ref_level;
    double prev_analog_ref_level;
    hal_bit_t *sync_in_trigger;
    hal_float_t *sync_in;		//
    hal_float_t *wait_type;
    hal_float_t *timeout;
    double prev_timeout;
    int num_gpio_in;
    
    hal_u32_t   *wou_bp_tick;   /* host side bp counter */
    hal_u32_t   *bp_tick;       /* base-period tick obtained from fetchmail */
    uint32_t    prev_bp;        /* previous base-period tick */
    hal_u32_t   *dout0;         /* the DOUT value obtained from fetchmail */
    hal_u32_t   *crc_error_counter;

    /* sync output pins (output from motmod) */
    hal_bit_t *out[32];
    int num_gpio_out;
    uint32_t prev_out;		//ON or OFF

    double     prev_ahc_state;
    hal_float_t *ahc_state;     // 0: disable 1:enable 2: suspend
    hal_float_t *ahc_level;
    double prev_ahc_level;
    hal_float_t *ahc_max_offset;
    double      prev_ahc_max_level;
    hal_float_t *ahc_max_level;
    hal_float_t *ahc_min_level;
    hal_u32_t   *control_mode;       // for state machine in risc
    hal_float_t *probe_retract_dist; // for risc probing
    hal_float_t *probe_vel;          // for risc probing
    hal_float_t *probe_disct;        // for risc probing
    /* motion state tracker */
    hal_s32_t *motion_state;
    int32_t prev_motion_state;
    /* command channel for emc2 */
    hal_u32_t *wou_cmd;
    uint32_t prev_wou_cmd;
    hal_u32_t *wou_status;
    uint32_t a_cmd_on_going;
    /* spindle */
//obsolete:    hal_bit_t *spindle_enable;
//obsolete:    hal_float_t *spindle_vel_cmd;
//obsolete:    hal_float_t *spindle_vel_fb;
    hal_bit_t *spindle_index_enable;    // TODO: move spindle-index sync into motion
    hal_bit_t *spindle_at_speed;
    hal_float_t *spindle_revs;
    double   spindle_revs_integer;
//    double  last_spindle_index_pos;
    int32_t last_spindle_index_pos;
    double  last_spindle_index_pos_int;
    int32_t spindle_enc_count;          // predicted spindle-encoder count
//    double  prev_spindle_irevs;         // to calculate index position
    int32_t prev_spindle_irevs;
    /* test pattern  */
    hal_s32_t *test_pattern;
    /* MPG */
    hal_s32_t *mpg_count;
    /* DEBUG */
//    hal_s32_t *debug;
    hal_s32_t *debug[8];
    /* tick */
    hal_u32_t *tick[14];
    /* application parameter*/
    hal_s32_t *app_param[16];
    int32_t prev_app_param[16];
    hal_bit_t *send_app_param; // IO: trigger parameters to be sent
    hal_s32_t *encoder_count[8]; // encoder count from risc
    hal_s32_t *pulse_count[8];
    hal_s32_t *ferror[8];
} machine_control_t;

/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t *stepgen_array;
static gpio_t *gpio;
static analog_t *analog;
static machine_control_t *machine_control;
static int32_t actual_joint_num;
/* file handle for wou step commands */
// static FILE *wou_fh;

/* lookup tables for stepping types 2 and higher - phase A is the LSB */

static const unsigned char master_lut[][10] = {
    {1, 3, 2, 0, 0, 0, 0, 0, 0, 0},	/* type 2: Quadrature */
    {1, 2, 4, 0, 0, 0, 0, 0, 0, 0},	/* type 3: Three Wire */
    {1, 3, 2, 6, 4, 5, 0, 0, 0, 0},	/* type 4: Three Wire Half Step */
    {1, 2, 4, 8, 0, 0, 0, 0, 0, 0},	/* 5: Unipolar Full Step 1 */
    {3, 6, 12, 9, 0, 0, 0, 0, 0, 0},	/* 6: Unipoler Full Step 2 */
    {1, 7, 14, 8, 0, 0, 0, 0, 0, 0},	/* 7: Bipolar Full Step 1 */
    {5, 6, 10, 9, 0, 0, 0, 0, 0, 0},	/* 8: Bipoler Full Step 2 */
    {1, 3, 2, 6, 4, 12, 8, 9, 0, 0},	/* 9: Unipolar Half Step */
    {1, 5, 7, 6, 14, 10, 8, 9, 0, 0},	/* 10: Bipolar Half Step */
    {1, 2, 4, 8, 16, 0, 0, 0, 0, 0},	/* 11: Five Wire Unipolar */
    {3, 6, 12, 24, 17, 0, 0, 0, 0, 0},	/* 12: Five Wire Wave */
    {1, 3, 2, 6, 4, 12, 8, 24, 16, 17},	/* 13: Five Wire Uni Half */
    {3, 7, 6, 14, 12, 28, 24, 25, 17, 19}	/* 14: Five Wire Wave Half */
};

static const unsigned char cycle_len_lut[] =
    { 4, 3, 6, 4, 4, 4, 4, 8, 8, 5, 5, 10, 10 };

static const unsigned char num_phases_lut[] =
    { 2, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, };

#define MAX_STEP_TYPE 14

#define STEP_PIN	0	/* output phase used for STEP signal */
#define DIR_PIN		1	/* output phase used for DIR signal */
#define UP_PIN		0	/* output phase used for UP signal */
#define DOWN_PIN	1	/* output phase used for DOWN signal */

//#define PICKOFF		26	/* bit location in DDS accum */

/* other globals */
static int comp_id;		/* component ID */
static int num_chan = 0;	/* number of step generators configured */
static double dt;		/* update_freq period in seconds */
static double recip_dt;		/* reciprocal of period, avoids divides */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_stepgen(int num, stepgen_t * addr, int step_type,
			  int pos_mode);
static int export_gpio(gpio_t * addr);
static int export_analog(analog_t * addr);
static int export_machine_control(machine_control_t * machine_control);
static void update_freq(void *arg, long period);


void endian_swap(uint32_t  *x)
{
    *x = (*x>>24) | ((*x<<8) & 0x00FF0000) |((*x>>8) & 0x0000FF00) |(*x<<24);
}

/************************************************************************
 * mailbox callback function for libwou                                 *
 ************************************************************************/
// not in use: static void get_crc_error_counter(int32_t crc_error_counter)
// not in use: {
// not in use:     // store crc_error_counter
// not in use: 
// not in use:     return;
// not in use: }
static void fetchmail(const uint8_t *buf_head)
{
    int         i;
    uint16_t    mail_tag;
    uint32_t    *p, din[2], dout[1];
    // obsolete: double      pos_scale;
    stepgen_t   *stepgen;
    uint32_t    bp_tick;    // served as previous-bp-tick
    uint32_t    ferror_flag;
#if (MBOX_LOG)
    char        dmsg[1024];
    int         dsize;
    //obsolete: static      uint32_t prev_din[2];
#elif (DEBUG_LOG)
    char        dmsg[1024];
	int         dsize;
#endif

    memcpy(&mail_tag, (buf_head + 2), sizeof(uint16_t));
    switch(mail_tag)
    {
    case MT_MOTION_STATUS:

        /* for PLASMA with ADC_SPI */

        // BP_TICK
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
        *machine_control->bp_tick = bp_tick;
        assert(actual_joint_num>=num_chan);
        stepgen = stepgen_array;
        for (i=0; i<actual_joint_num; i++) {
            if (i<num_chan) {
                // PULSE_POS
                p += 1;
                *(stepgen->pulse_pos) = *p;
                *(stepgen->pid_cmd) = (*(stepgen->pulse_pos))*(stepgen->scale_recip);
                *(machine_control->pulse_count[i]) = (int32_t)*p;
                // enc counter
                p += 1;
                *(stepgen->enc_pos) = *p;
                *(machine_control->encoder_count[i]) = (int32_t) *p;
                // pid output
                p +=1;
                // *(stepgen->pid_output) = ((int32_t)*p)*(stepgen->scale_recip);
                *(stepgen->pid_output) = ((int32_t)*p)*(1.0);
                // cmd error
                p += 1;
                // *(stepgen->cmd_error) = ((int32_t)*p)*(stepgen->scale_recip);
                *(stepgen->cmd_error) = ((int32_t)*p)*(1.0);
                *(machine_control->ferror[i]) = (int32_t)*p;
                // joint_cmd of this BP
                p += 1;
                *(stepgen->joint_cmd) = ((int32_t)*p);

                stepgen += 1;   // point to next joint
            } else {
                // PULSE_POS
                p += 1;
//                *(stepgen->pulse_pos) = *p;
//                *(stepgen->pid_cmd) = (*(stepgen->pulse_pos))*(stepgen->scale_recip);
                *(machine_control->pulse_count[i]) = *p;
                // enc counter
                p += 1;
//                *(stepgen->enc_pos) = *p;
                *(machine_control->encoder_count[i]) = *p;
                // pid output
                p +=1;
                // *(stepgen->pid_output) = ((int32_t)*p)*(stepgen->scale_recip);
//                *(stepgen->pid_output) = ((int32_t)*p)*(1.0);
                // cmd error
                p += 1;
                // *(stepgen->cmd_error) = ((int32_t)*p)*(stepgen->scale_recip);
//                *(stepgen->cmd_error) = ((int32_t)*p)*(1.0);
                *(machine_control->ferror[i]) = (int32_t)*p;
                // joint_cmd of this BP
                p += 1;
//                *(stepgen->joint_cmd) = ((int32_t)*p);
            }
            
        }

        // digital inpout
        p += 1;
        din[0] = *p;
        p += 1;
        din[1] = *p;
        // digital output
        p += 1;
        dout[0] = *p;
        *machine_control->dout0 = dout[0];

        // update gpio_in[31:0]
        // compare if there's any GPIO.DIN bit got toggled
        // WOU_SIM
//        if (memcmp (&(machine_control->prev_in0), &din[0], sizeof(uint32_t))) {
//            // avoid for-loop to save CPU cycles
//            memcpy(&(machine_control->prev_in0), &din[0], sizeof(uint32_t));
//            for (i = 0; i < 32; i++) {
//                *(machine_control->in[i]) = ((machine_control->prev_in0) >> i) & 0x01;
//            }
//        }
        // TODO: update gpio_in[63:32]

        // ADC_SPI (raw ADC value)
        p += 1;
        *(gpio->a_in[0]) = (((double)*p)/20.0);
        *(analog->in[0]) = *p;
        p += 1; *(analog->in[1]) = *p;
        p += 1; *(analog->in[2]) = *p;
        p += 1; *(analog->in[3]) = *p;
        p += 1; *(analog->in[4]) = *p;
        p += 1; *(analog->in[5]) = *p;
        p += 1; *(analog->in[6]) = *p;
        p += 1; *(analog->in[7]) = *p;
        p += 1; *(analog->in[8]) = *p;
        p += 1; *(analog->in[9]) = *p;
        p += 1; *(analog->in[10]) = *p;
        p += 1; *(analog->in[11]) = *p;
        p += 1; *(analog->in[12]) = *p;
        p += 1; *(analog->in[13]) = *p;
        p += 1; *(analog->in[14]) = *p;
        p += 1; *(analog->in[15]) = *p;

        // MPG
        p += 1;
        *(machine_control->mpg_count) = *p;
        // the MPG on my hand is 1-click for a full-AB-phase-wave.
        // therefore the mpg_count will increase by 4.
        // divide it by 4 for smooth jogging.
        // otherwise, there will be 4 units of motions for every MPG click.
        *(machine_control->mpg_count) >>= 2;
        //debug: fprintf (stdout, "MPG: 0x%08X\n", *(machine_control->mpg_count));

        // FERROR FLAG
        p += 1;
        ferror_flag = *p;
        stepgen = stepgen_array;
        for (i=0; i<num_chan; i++) {
            *stepgen->ferror_flag = ferror_flag & (1 << i);
            stepgen += 1;   // point to next joint
        }

        // DEBUG  : MOVE to MT_DEBUG

#if (MBOX_LOG)
        //obsolete: if (din[0] != prev_din[0]) {
        //obsolete:     prev_din[0] = din[0];
        //obsolete:     fprintf (stderr, "DIN[0]: 0x%08X\n", din[0]);
        //obsolete: }
        //obsolete: if (din[1] != prev_din[1]) {
        //obsolete:     prev_din[1] = din[1];
        //obsolete:     fprintf (stderr, "DIN[1]: 0x%08X\n", din[1]);
        //obsolete: }
        dsize = sprintf (dmsg, "%10d  ", bp_tick);  // #0
        // fprintf (mbox_fp, "%10d  ", bp_tick);
        stepgen = stepgen_array;
        for (i=0; i<num_chan; i++) {
            dsize += sprintf (dmsg + dsize, "%10d %10d %10f %10f %10d  ",
                              *(stepgen->pulse_pos),
                              *(stepgen->enc_pos),
                              *(stepgen->pid_output),
                              *(stepgen->cmd_error),
                              *(stepgen->joint_cmd)
                             );
            stepgen += 1;   // point to next joint
        }
        dsize += sprintf (dmsg + dsize, "0x%04X 0x%04X 0x%04X", // #21 #22 #23
                          din[0],
                          din[1],
                          dout[0]);
        dsize += sprintf (dmsg + dsize, "  %10d 0x%04X", *(analog->in[0]), ferror_flag); // #24 #25


        // number of debug words: to match "send_joint_status() at common.c
        for (i=0; i<MBOX_DEBUG_VARS; i++) {
            p += 1;
            dsize += sprintf (dmsg + dsize, "%10d ", *p);
        }
        assert (dsize < 1023);
        fprintf (mbox_fp, "%s\n", dmsg);
#endif
        break;
    case MT_ERROR_CODE:
        // error code
        p = (uint32_t *) (buf_head + 4);
        p += 1;
        bp_tick = *p;
        p += 1;
        switch (*p) {
        case ERROR_BASE_PERIOD:
            fprintf(stderr, "ERROR_BASE_PERIOD occurs with code(%d) bp_tick(%d) \n", *p, bp_tick);
            break;
        }

#if (MBOX_LOG)
//        if(*p < 100) {
            fprintf(mbox_fp, "# error occure with code(%d) bp_tick(%d)\n",*p, bp_tick);
//        }
//        fprintf(mbox_fp, "# error occure with code(%d)\n",bp_tick);
#endif
        break;
    case MT_USB_STATUS:
            // update wou status only if a cmd ongoing
        p = (uint32_t *) (buf_head + 4);
        /* probe status */
        p += 1;
        if (*machine_control->wou_cmd != USB_CMD_NOOP) {
            *machine_control->wou_status = *p;
        } else if (*p == USB_STATUS_RISC_PROBE_ERROR) {
            // section report status normally
            *machine_control->wou_status = *p;
        } else {
            *machine_control->wou_status = USB_STATUS_READY;
        }
//        fprintf(stderr, "wou_stepgen.c: probe_level(%d) probe_ref(%d)\n", *(p+1), *(p+2));
        break;
    case MT_DEBUG:
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
#if (DEBUG_LOG)
        dsize = sprintf (dmsg, "%10d  ", bp_tick);  // #0
#endif
        for (i=0; i<8; i++) {
            p += 1;
            *machine_control->debug[i] = *p;

#if (DEBUG_LOG)
            dsize += sprintf (dmsg + dsize, "%10d", *p);
            assert (dsize < 1023);
#endif
        }
#if (DEBUG_LOG)
        fprintf (debug_fp, "%s\n", dmsg);
#endif
        break;
    case MT_TICK:
        p = (uint32_t *) (buf_head + 4);

        for (i=0; i<14; i++) {
            p += 1;
            *machine_control->tick[i] = *p;
        }
        break;
    default:
        fprintf(stderr, "ERROR: wou_stepgen.c unknown mail tag\n");
        assert(0);
#if (MBOX_LOG)
        fprintf(mbox_fp, "# unknown mail tag\n"
                );
#endif
        break;
    }

}

static void write_mot_param (uint32_t joint, uint32_t addr, int32_t data)
{
    uint16_t    sync_cmd;
    uint8_t     buf[MAX_DSIZE];
    int         j;

    for(j=0; j<sizeof(int32_t); j++) {
        sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), buf);
        // wou_flush(&w_param);
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ADDR(addr) | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
    // sim:         sizeof(uint16_t), buf);
    // sim: wou_flush(&w_param);

    return;
}

static void write_machine_param (uint32_t addr, int32_t data)
{
    uint16_t    sync_cmd;
    uint8_t     buf[MAX_DSIZE];
    int         j;

    for(j=0; j<sizeof(int32_t); j++) {
        sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), buf);
        // sim: // wou_flush(&w_param);
    }

    sync_cmd = SYNC_MACH_PARAM | PACK_MACH_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
    // sim:         sizeof(uint16_t), buf);
    // sim: wou_flush(&w_param);

    return;
}

static void parse_usb_cmd (uint32_t usb_cmd)
{
    int i;
    stepgen_t   *stepgen;

    if (machine_control->a_cmd_on_going == 0) {

        // issue a command relative to the usb_cmd
        switch(usb_cmd) {
        case USB_CMD_PROBE_HIGH:
            machine_control->a_cmd_on_going = 1;
            write_machine_param(PROBE_CMD, PROBE_HIGH);
            break;
        case USB_CMD_PROBE_LOW:
            machine_control->a_cmd_on_going = 1;
            write_machine_param(PROBE_CMD, PROBE_LOW);
            break;
        case USB_CMD_ABORT:  // for risc probing
            fprintf(stderr,"output PROBE_END\n");
            write_machine_param(PROBE_CMD, PROBE_END);
            break;
        }
    } else if (usb_cmd == USB_CMD_STATUS_ACK){

        // issue a command to clear or abort previous command
        switch(machine_control->prev_wou_cmd) {
        case USB_CMD_PROBE_HIGH:
        case USB_CMD_PROBE_LOW:
//            machine_control->a_cmd_on_going = 0;
            fprintf(stderr,"output PROBE_END\n");
            write_machine_param(PROBE_CMD, PROBE_END);
            break;
        }

    } else if (usb_cmd == USB_CMD_ABORT) {
        switch(machine_control->prev_wou_cmd) {
            case USB_CMD_PROBE_HIGH:
            case USB_CMD_PROBE_LOW:
                machine_control->a_cmd_on_going = 0;
                write_machine_param(PROBE_CMD, PROBE_END);
            break;
        }

    } else if (usb_cmd == USB_CMD_NOOP) {
        machine_control->a_cmd_on_going = 0;
        write_machine_param(PROBE_CMD, PROBE_STOP_REPORT);
        *machine_control->wou_status = USB_STATUS_READY;
    } else if (usb_cmd == USB_CMD_WOU_CMD_SYNC)  {
        // align prev pos cmd and pos cmd;
        // machine_control->a_cmd_on_going = 0;
        stepgen = stepgen_array;
        for (i=0; i<num_chan; i++) {
            stepgen->prev_pos_cmd = *stepgen->pos_cmd;
            stepgen++;
        }
    } else {
        fprintf(stderr, "issue command while another command is ongoing.\n");
        assert(0);
    }
}

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval, i;

    uint8_t data[MAX_DSIZE];
    int32_t immediate_data;
    double max_vel, max_accel, pos_scale, value, max_following_error, probe_decel;
    int msg;

    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_ALL);

#if (TRACE!=0)
    // initialize file handle for logging wou steps
    dptrace = fopen("wou_stepgen.log", "w");
    /* prepare header for gnuplot */
    DPS("#%10s  %15s%15s%15s%3s  %15s%15s%15s%3s  %15s%15s%15s%3s  %15s%15s%15s%3s  %15s\n",
         "dt",
         "int_pos_cmd[0]", "prev_pos_cmd[0]", "pos_fb[0]", "H0",  //H0: home_state for J0
         "int_pos_cmd[1]", "prev_pos_cmd[1]", "pos_fb[1]", "H1",
         "int_pos_cmd[2]", "prev_pos_cmd[2]", "pos_fb[2]", "H2",
         "int_pos_cmd[3]", "prev_pos_cmd[3]", "pos_fb[3]", "H3",
         "spindle_revs"
       );
#endif

    pending_cnt = 0;

    /* test for bitfile string: bits */
    if ((bits == 0) || (bits[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"WOU: ERROR: no fpga bitfile string: bits\n");
	return -1;
    } else {
	// initialize FPGA with bitfile(bits)
	// sim: wou_init(&w_param, board, wou_id, bits);
	// sim: if (wou_connect(&w_param) == -1) {
	// sim:     rtapi_print_msg(RTAPI_MSG_ERR,
	// sim: 		    "WOU: ERROR: Connection failed\n");
	// sim:     return -1;
	// sim: }
    }

    /* test for risc image file string: bins */
    if ((bins == 0) || (bins[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"WOU: ERROR: no risc binfile string: bins\n");
	return -1;
    } else {
	// programming risc with binfile(bins)
        // sim: wou_prog_risc(&w_param, bins);
#if (MBOX_LOG)
        mbox_fp = fopen ("./mbox.log", "w");
        fprintf (mbox_fp, "%10s  ", "bp_tick");
        for (i=0; i<4; i++) {
            fprintf (mbox_fp, "%9s%d  %9s%d %9s%d %9s%d %9s%d  ",
                              "pls_pos-", i,
                              "enc_pos-", i,
                              "pid_out-", i,
                              "cmd_err-", i,
                              "jnt_cmd-", i
                    );
        }
        fprintf (mbox_fp, "\n");
#endif
#if (DEBUG_LOG)
        debug_fp = fopen ("./debug.log", "w");
#endif
        // sim: // set mailbox callback function
        // sim: wou_set_mbox_cb (&w_param, fetchmail);
        // sim: // set crc counter callback function
        // sim: wou_set_crc_error_cb (&w_param, get_crc_error_counter);
    }

    if(pulse_type != -1) {
        data[0] = pulse_type;
        // sim: wou_cmd (&w_param, WB_WR_CMD,
        // sim:         (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE),
        // sim:         (uint8_t) 1, data);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no pulse type: pulse_type\n");
        return -1;
    }

    if(enc_type != -1) {
        data[0] = enc_type;
        // sim: wou_cmd (&w_param, WB_WR_CMD,
        // sim:         (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE),
        // sim:         (uint8_t) 1, data);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no encoder type: enc_type\n");
        return -1;
    }
    // config machine type
    if (strcmp(machine_param_str, "XYZA") == 0) {
        // config machine type normal (no synchronized joints)
        write_machine_param (MACHINE_TYPE, XYZA);
    } else if (strcmp(machine_param_str, "XYZY") == 0) {
        // config machine type YY (one kind of gantry)
        write_machine_param (MACHINE_TYPE, XYZY);
    } else if (strcmp(machine_param_str, "XYZY_") == 0) {
       // config machine type YY_ (one kind of gantry)
        write_machine_param (MACHINE_TYPE, XYZY_);
    } else {
        fprintf(stderr, "wou_stepgen.c: non-supported machine type\n");
        assert(0);
    }

    // configure alarm output (for E-Stop)
    write_machine_param(ALR_OUTPUT, alr_output);
    // config probe parameters
    // probe_decel_cmd
    immediate_data = atoi(probe_pin_id);
    fprintf(stderr,"wou_stgepgen.c: probe_pin_id(%d)\n", immediate_data);
    write_machine_param(PROBE_PIN_ID, immediate_data);
    
    immediate_data = atoi(probe_analog_ref_level);
    write_machine_param(PROBE_ANALOG_REF_LEVEL, immediate_data);
    
    if (strcmp(probe_pin_type, "ANALOG_PIN") == 0) {
        immediate_data = ANALOG_PIN;
    } else if (strcmp(probe_pin_type, "DIGITAL_PIN") == 0){
        immediate_data = DIGITAL_PIN;
    } else {
        fprintf(stderr,"ERROR: wou_stepgen.c unknown probe_pin_type\n");
        assert(0);
    }

    write_machine_param(PROBE_PIN_TYPE, immediate_data);
    // we use this value later.
    probe_decel = atoi(probe_decel_cmd);

    // config auto height control behavior
    immediate_data = atoi(ahc_ch_str);
    write_machine_param(AHC_ANALOG_CH, immediate_data);
    immediate_data = atoi(ahc_joint_str);
    pos_scale = atof(pos_scale_str[immediate_data]);
    write_machine_param(AHC_JNT, immediate_data);
    immediate_data = atoi(ahc_level_min_str);
    write_machine_param(AHC_LEVEL_MIN, immediate_data);
    immediate_data = atoi(ahc_level_max_str);
    write_machine_param(AHC_LEVEL_MAX, immediate_data);

    if (strcmp(ahc_polarity, "POSITIVE") == 0) {
    	if (pos_scale >=0) {
    		// set risc positive
    		write_machine_param(AHC_POLARITY, AHC_POSITIVE);
    	} else {
    		// set risc negative
    		write_machine_param(AHC_POLARITY, AHC_NEGATIVE);
    	}
    } else if (strcmp(ahc_polarity, "NEGATIVE") == 0) {
    	if (pos_scale >= 0) {
    		// set risc negative
    		write_machine_param(AHC_POLARITY, AHC_NEGATIVE);
    	} else {
    		// set risc positive
    		write_machine_param(AHC_POLARITY, AHC_POSITIVE);
    	}
    } else {
    	fprintf(stderr, "wou_stepgen.c: non-supported ahc polarity config\n");
    	assert(0);
    }

    // config debug pattern
    if (strcmp(pattern_type_str, "NO_TEST") == 0) {
        write_machine_param(TEST_PATTERN_TYPE, NO_TEST);
        test_pattern_type = NO_TEST;
    } else if (strcmp(pattern_type_str, "ANALOG_IN") == 0) {
        fprintf(stderr, "output to risc  test pattern(ANALOG_IN)\n");
        write_machine_param(TEST_PATTERN_TYPE, ANALOG_IN);
        test_pattern_type = ANALOG_IN;
    } else if (strcmp(pattern_type_str, "DIGITAL_IN") == 0) {
        fprintf(stderr, "output to risc  test pattern(DIGITAL_IN)\n");
        write_machine_param(TEST_PATTERN_TYPE, DIGITAL_IN);
        test_pattern_type = DIGITAL_IN;
    } else {
        fprintf(stderr, "wou_stepgen.c: unknow test pattern type (%s)\n", pattern_type_str);
        assert(0);
    }

//obsolete:    if (1 == adc_spi_en) {
//obsolete:        rtapi_print_msg(RTAPI_MSG_INFO,
//obsolete:                        "WOU: enable ADC_SPI\n");
//obsolete:        // MCP3204: set ADC_SPI_SCK_NR to generate 19 SPI_SCK pulses
//obsolete:        data[0] = 19; 
//obsolete:        //MCP3202: // MCP3202: set ADC_SPI_SCK_NR to generate 17 SPI_SCK pulses
//obsolete:        //MCP3202: data[0] = 17;   // MCP3202
//obsolete:        wou_cmd (&w_param, WB_WR_CMD,
//obsolete:                 (uint16_t) (SPI_BASE | ADC_SPI_SCK_NR),
//obsolete:                 (uint8_t) 1, data);
//obsolete:
//obsolete:        // enable ADC_SPI with LOOP mode
//obsolete:        
//obsolete:        // MCP3204: (p.19 of adc_mcp3204.pdf)
//obsolete:        // ADC_SPI_CMD: 0x10: { (1)START_BIT,
//obsolete:        //                      (0)Differential mode,
//obsolete:        //                      (0)D2 ... dont care,
//obsolete:        //                      (0)D1 ... Ch0 = IN+,
//obsolete:        //                      (0)D0 ... CH1 = IN-   }
//obsolete:        // ADC_SPI_CMD: 0x18: { (1)START_BIT,
//obsolete:        //                      (1)single-end mode,
//obsolete:        //                      (0)D2 ... dont care,
//obsolete:        //                      (00){D1,D0} ... CH0 }
//obsolete:        data[0] = ADC_SPI_EN_MASK | ADC_SPI_LOOP_MASK
//obsolete:                  | (ADC_SPI_CMD_MASK & 0x18);
//obsolete:
//obsolete:        //MCP3202: // MCP3202: 
//obsolete:        //MCP3202: // ADC_SPI_CMD: 0x04: { (1)START_BIT,
//obsolete:        //MCP3202: //                      (0)Differential mode,
//obsolete:        //MCP3202: //                      (0)SIGN   Ch0 = IN+,
//obsolete:        //MCP3202: //                                CH1 = IN-   }
//obsolete:        //MCP3202: data[0] = ADC_SPI_EN_MASK | ADC_SPI_LOOP_MASK
//obsolete:        //MCP3202:           | (ADC_SPI_CMD_MASK & 0x04);  // MCP3202
//obsolete:        wou_cmd (&w_param, WB_WR_CMD,
//obsolete:                 (uint16_t) (SPI_BASE | ADC_SPI_CTRL),
//obsolete:                 (uint8_t) 1, data);
//obsolete:    }

    /* to clear PULSE/ENC/SWITCH/INDEX positions for 4 axes */
    // issue a WOU_WRITE 
    data[0] = 0x0F;
    // sim: wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    // sim: wou_flush(&w_param);
    
    data[0] = 0x00;
    // sim: wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    // sim: wou_flush(&w_param);

//obsolete:     /* test for GPIO_MASK_IN0: gpio_mask_in0 */
//obsolete:     if ((gpio_mask_in0 == -1)) {
//obsolete: 	rtapi_print_msg(RTAPI_MSG_ERR,
//obsolete: 			"WOU: ERROR: no value for GPIO_MASK_IN0: gpio_mask_in0\n");
//obsolete: 	return -1;
//obsolete:     } else {
//obsolete: 	// un-mask HOME-SWITCH inputs (bits_i[5:2])
//obsolete: 	data[0] = (uint8_t) gpio_mask_in0;
//obsolete: 	wou_cmd(&w_param, WB_WR_CMD, GPIO_BASE | GPIO_MASK_IN0, 1, data);
//obsolete:     }

//obsolete:    /* test for GPIO_MASK_IN1: gpio_mask_in1 */
//obsolete:    if ((gpio_mask_in1 == -1)) {
//obsolete:	rtapi_print_msg(RTAPI_MSG_ERR,
//obsolete:			"WOU: ERROR: no value for GPIO_MASK_IN1: gpio_mask_in1\n");
//obsolete:	return -1;
//obsolete:    } else {
//obsolete:	// un-mask HOME-SWITCH inputs (bits_i[5:2])
//obsolete:	data[0] = (uint8_t) gpio_mask_in1;
//obsolete:	wou_cmd(&w_param, WB_WR_CMD, GPIO_BASE | GPIO_MASK_IN1, 1, data);
//obsolete:    }
    
//obsolete:    /* test for GPIO_LEDS_SEL: gpio_leds_sel */
//obsolete:    if ((gpio_leds_sel == -1)) {
//obsolete:	rtapi_print_msg(RTAPI_MSG_ERR,
//obsolete:			"WOU: ERROR: no value for GPIO_LEDS_SEL: gpio_leds_sel\n");
//obsolete:	return -1;
//obsolete:    } else {
//obsolete:	// select signals for LEDs
//obsolete:	data[0] = (uint8_t) gpio_leds_sel;
//obsolete:	wou_cmd(&w_param, WB_WR_CMD, GPIO_BASE | GPIO_LEDS_SEL, 1, data);
//obsolete:    }

    /* test for stepping motor current limit: step_cur */
    num_chan = 0;
    for (n = 0; n < MAX_CHAN && step_cur[n] != -1; n++) {
	if ((step_cur[n] > MAX_STEP_CUR) || (step_cur[n] < 0)) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "WOU: ERROR: bad step_cur[%d]: '%i'\n",
			    n, step_type[n]);
	    return -1;
	}
	data[n] = (uint8_t) step_cur[n];
	num_chan++;
    }
    if (num_chan > 0) {
	// wirte SSIF_MAX_PWM to USB with Automatically Address Increment
        // sim: wou_cmd(&w_param,
        // sim: 	WB_WR_CMD, (SSIF_BASE | SSIF_MAX_PWM), num_chan, data);
    }

    // sim: wou_flush(&w_param);

    num_chan = 0;
    for (n = 0; n < MAX_CHAN && step_type[n] != -1; n++) {
	if ((step_type[n] > MAX_STEP_TYPE) || (step_type[n] < 0)) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "STEPGEN: ERROR: bad stepping type '%i', axis %i\n",
			    step_type[n], n);
	    return -1;
	}
	if ((ctrl_type[n][0] == 'p') || (ctrl_type[n][0] == 'P')) {
	    ctrl_type[n] = "p";
	} else if ((ctrl_type[n][0] == 'v') || (ctrl_type[n][0] == 'V')) {
	    ctrl_type[n] = "v";
	} else {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
			    ctrl_type[n], n);
	    return -1;
	}
	num_chan++;
    }
    if (num_chan == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: no channels configured\n");
	return -1;
    }
    /* test for dt: servo_period_ns */
    if ((servo_period_ns == -1)) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no value for servo_period_ns\n");
        return -1;
    } else {
        /* precompute some constants */
        dt = (double) servo_period_ns * 0.000000001;
        recip_dt = 1.0 / dt;
    }

    pid_str[0] = j0_pid_str;
    pid_str[1] = j1_pid_str;
    pid_str[2] = j2_pid_str;
    pid_str[3] = j3_pid_str;
    pid_str[4] = j4_pid_str;
    pid_str[5] = j5_pid_str;
    pid_str[6] = j6_pid_str;
    pid_str[7] = j7_pid_str;

    /* configure motion parameters */
    for(n=0; n<num_chan; n++) {
        // const char **pid_str;

        /* compute fraction bits */
        // compute proper fraction bit for command
        // compute fraction bit for velocity
        // accurate 0.0001 mm
        pos_scale   = fabs(atof(pos_scale_str[n]));
        max_vel     = atof(max_vel_str[n]);
        max_accel   = atof(max_accel_str[n]);
        
        /* config MAX velocity */
        immediate_data = ((uint32_t)(max_vel * pos_scale * dt * FIXED_POINT_SCALE));
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_vel(%d) = %f*%f*%f*%f\n", 
                        n, immediate_data, max_vel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data>0);
        write_mot_param (n, (MAX_VELOCITY), immediate_data);

        /* config acceleration */
        immediate_data = ((uint32_t)(max_accel * pos_scale * dt * FIXED_POINT_SCALE * dt));
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_accel(%d) = %f*%f*(%f^2)*(%f)\n",
                        n, immediate_data, max_accel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_ACCEL), immediate_data);

        /* config acceleration recip */
        immediate_data = (uint32_t)(FIXED_POINT_SCALE / (max_accel * pos_scale * dt * dt));
        rtapi_print_msg(RTAPI_MSG_DBG, 
                        "j[%d] max_accel_recip(%d) = (%f/(%f*%f*(%f^2)))\n",
                        n, immediate_data, FIXED_POINT_SCALE, max_accel, pos_scale, dt);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_ACCEL_RECIP), immediate_data);

        /* config max following error */
        // following error send with unit pulse
        max_following_error = atof(ferror_str[n]);
        immediate_data = (uint32_t)(ceil(max_following_error * pos_scale));
        rtapi_print_msg(RTAPI_MSG_DBG, "max ferror(%d)\n", immediate_data);
        write_mot_param (n, (MAXFOLLWING_ERR), immediate_data);

        // set probe decel cmd
        immediate_data = (uint32_t)(probe_decel * pos_scale * dt * FIXED_POINT_SCALE * dt);
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] probe_decel(%d) = %f*%f*(%f^2)*%f\n",
                        n, immediate_data, probe_decel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data > 0);
        write_mot_param (n, (PROBE_DECEL_CMD), immediate_data);

        // set move type as normal by default
        immediate_data = NORMAL_MOVE;
        write_mot_param (n, (MOTION_TYPE), immediate_data);
    }

    // config PID parameter
    for (n=0; n < PID_LOOP; n++) {
        if (pid_str[n][0] != NULL) {
            rtapi_print_msg(RTAPI_MSG_INFO, "J%d_PID: ", n);
            rtapi_print_msg(RTAPI_MSG_INFO,"#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO 14:PE 15:PB\n");
            // all gains (P, I, D, FF0, FF1, FF2) varie from 0(0%) to 65535(100%)
            // all the others units are '1 pulse'
            for (i=0; i < (MAXOUTPUT-P_GAIN+1); i++) {
                value = atof(pid_str[n][i]);
                immediate_data = (int32_t) (value);
                write_mot_param (n, (P_GAIN + i), immediate_data);
                rtapi_print_msg(RTAPI_MSG_INFO, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }

            value = 0;
            immediate_data = (int32_t) (value);
            write_mot_param (n, (ENABLE), immediate_data);
            rtapi_print_msg(RTAPI_MSG_INFO, "\n");
        }
    }

    actual_joint_num = atoi(act_jnt_num);

    /* to send position compensation velocity  of Z*/
//    thc_vel = atof(thc_velocity);
//    pos_scale = atof(pos_scale_str[2]);
//    immediate_data = (uint32_t)(thc_vel*pos_scale*dt*(1 << FRACTION_BITS));
//    immediate_data = immediate_data > 0? immediate_data:-immediate_data;
//    write_mot_param (2, (COMP_VEL), immediate_data);

    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo/stepper interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    // TODO: RTL: remove SSIF_EN (always enable SSIF)
    // FIXME: WORKAROUND: always enable SSIF_EN by SW
    // SSIF_EN = 1
    data[0] = 2;
    // sim: wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
    // sim: // RISC ON
    // sim: data[0] = 1;        
    // sim: wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, data);
    // sim: wou_flush(&w_param);
    /* have good config info, connect to the HAL */
    comp_id = hal_init("wou_sim");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: hal_init() failed\n");
	return -1;
    }

    /* allocate shared memory for counter data */
    stepgen_array = hal_malloc(num_chan * sizeof(stepgen_t));
    if (stepgen_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
    
    gpio = hal_malloc(sizeof(gpio_t));
    if (gpio == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"GPIO: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
    
    analog = hal_malloc(sizeof(analog_t));
    if (analog == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"ANALOG: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
    
    machine_control = hal_malloc(sizeof(machine_control_t));
    if (machine_control == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"MACHINE_CONTROL: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }

    /* export all the variables for each pulse generator */
    for (n = 0; n < num_chan; n++) {
	/* export all vars */
	retval = export_stepgen(n, &(stepgen_array[n]),
				step_type[n], (ctrl_type[n][0] == 'p'));
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "STEPGEN: ERROR: stepgen %d var export failed\n",
			    n);
	    hal_exit(comp_id);
	    return -1;
	}
    }

    retval = export_gpio(gpio);	// 16-in, 8-out
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"GPIO: ERROR: gpio var export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    retval = export_analog(analog);	// up to 16-ch-adc-in
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"ANALOG: ERROR: analog var export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    /* put export machine_control below */
    // static int export_m_control (m_control_t *m_control)
    retval = export_machine_control(machine_control);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"MACHINE_CONTROL: ERROR:  machine_control var export failed\n");
	hal_exit(comp_id);
	return -1;
    }
/* put export machine_control above */
    retval = hal_export_funct("wou.stepgen.update-freq", update_freq,
			      stepgen_array, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: freq update funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,
		    "STEPGEN: installed %d step pulse generators\n",
		    num_chan);

/*   restore saved message level*/
    rtapi_set_msg_level(msg);

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
#if (TRACE!=0)
    fclose(dptrace);
#endif
    hal_exit(comp_id);
}


/***********************************************************************
*              REALTIME STEP PULSE GENERATION FUNCTIONS                *
************************************************************************/

// This function was invented by Jeff Epler.
// It forces a floating-point variable to be degraded from native register
// size (80 bits on x86) to C double size (64 bits).
static double force_precision(double d) __attribute__ ((__noinline__));
static double force_precision(double d)
{
    return d;
}

static void update_freq(void *arg, long period)
{
    stepgen_t *stepgen;
    int n, i, enable;
    double physical_maxvel, maxvel;	// max vel supported by current step timings & position-scal

    // int32_t wou_cmd_accum;
    uint16_t sync_cmd;
    int wou_pos_cmd, integer_pos_cmd;
    uint8_t data[MAX_DSIZE];    // data[]: for wou_cmd()
    uint32_t sync_out_data;

    // for homing:
    uint8_t r_load_pos;
    uint8_t r_switch_en;
//TODO: RISC_HOMING:    uint8_t r_index_en;
//TODO: RISC_HOMING:    uint8_t r_index_lock;
//TODO: RISC_HOMING:    static uint8_t prev_r_index_en = 0;
//TODO: RISC_HOMING:    static uint8_t prev_r_index_lock = 0;
    static uint32_t host_tick = 0;

    int32_t immediate_data = 0;
//obsolete: double fp_req_vel, fp_cur_vel, fp_diff;
#if (TRACE!=0)
    static uint32_t _dt = 0;
#endif
    /* FIXME - while this code works just fine, there are a bunch of
       internal variables, many of which hold intermediate results that
       don't really need their own variables.  They are used either for
       clarity, or because that's how the code evolved.  This algorithm
       could use some cleanup and optimization. */
    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    int msg;
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_ALL);

    // update host tick for risc

    write_machine_param(HOST_TICK, host_tick);
    *machine_control->wou_bp_tick = host_tick;
    if (host_tick == REQUEST_TICK_SYNC_AFTER) {

        sync_cmd = SYNC_BP ;
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), data);
        // sim: wou_flush(&w_param);
    }
    host_tick += 1;
    //    DP("before wou_update()\n");
    // sim: wou_update(&w_param);
    // read SSIF_INDEX_LOCK
//TODO: implement RISC homing:    memcpy(&r_index_lock,
//TODO: implement RISC homing:	   wou_reg_ptr(&w_param, SSIF_BASE + SSIF_INDEX_LOCK), 1);
//TODO: implement RISC homing:    if (r_index_lock != prev_r_index_lock) {
//TODO: implement RISC homing:	rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: index_lock(0x%02X) prev_index_lock(0x%02X)\n",
//TODO: implement RISC homing:	                                r_index_lock, prev_r_index_lock);
//TODO: implement RISC homing:	prev_r_index_lock = r_index_lock;
//TODO: implement RISC homing:    }

    /* begin: sending debug pattern */
    if (test_pattern_type != NO_TEST) {
        write_machine_param(TEST_PATTERN, *(machine_control->test_pattern));
    }
    /* end: sending debug pattern*/

    /* begin set analog trigger level*/
    if (*machine_control->analog_ref_level != machine_control->prev_analog_ref_level) {
        write_machine_param(ANALOG_REF_LEVEL, (uint32_t)(*(machine_control->analog_ref_level)));
        fprintf(stderr,"wou_stepgen.c: analog_ref_level(%d) \n", (uint32_t)*machine_control->analog_ref_level);
    }
    machine_control->prev_analog_ref_level = *machine_control->analog_ref_level;
    /* end: */

    /* begin: handle usb cmd */
    if ((*machine_control->wou_cmd) != machine_control->prev_wou_cmd) {
        // call api to parse wou_cmd to risc
        parse_usb_cmd (*machine_control->wou_cmd);
        fprintf(stderr, "wou_cmd(%d) prev_wou_cmd(%d)\n", *machine_control->wou_cmd,
                machine_control->prev_wou_cmd);
    }
    machine_control->prev_wou_cmd = *machine_control->wou_cmd;
    /* end: handle usb cmd */

    /* begin: handle AHC state, AHC level */
    if (*machine_control->ahc_level != machine_control->prev_ahc_level) {
        immediate_data = (uint32_t)(*(machine_control->ahc_level));
        write_machine_param(AHC_LEVEL, immediate_data);
        machine_control->prev_ahc_level = *(machine_control->ahc_level);
        fprintf(stderr,"wou_stepgen.c: ahc_level(%d)\n",
                        (uint32_t) *(machine_control->ahc_level));
        machine_control->prev_ahc_level = *(machine_control->ahc_level);

    }

    if (((uint32_t)*machine_control->ahc_state) !=
            ((uint32_t)machine_control->prev_ahc_state)) {
        immediate_data = (uint32_t)(*(machine_control->ahc_state));
        write_machine_param(AHC_STATE, immediate_data);
        fprintf(stderr,"wou_stepgen.c: ahc_state(%d)\n",
                        (uint32_t)*(machine_control->ahc_state));
        machine_control->prev_ahc_state = *machine_control->ahc_state;
    }
    /* end: handle AHC state, AHC level */

    /* begin: handle ahc max offset */
    if (*(machine_control->ahc_max_level) != (machine_control->prev_ahc_max_level)) {
        int32_t max_offset, pos_scale;
        stepgen = arg;
        stepgen += atoi(ahc_joint_str);
        pos_scale = (stepgen->pos_scale);
        max_offset = *(machine_control->ahc_max_offset);
        max_offset = max_offset >= 0? max_offset:0;
        fprintf(stderr,"wou_stepgen.c: ahc_max_offset(%d) ahc_joint(%d) \n",
                            (uint32_t)abs(max_offset * (pos_scale)),
                            atoi(ahc_joint_str));

        /* ahc max_offset */
        write_machine_param(AHC_MAX_OFFSET, (uint32_t)
                abs((max_offset)) * (pos_scale));
        machine_control->prev_ahc_max_level = *(machine_control->ahc_max_level);
    }
    /* end: handle ahc max offset */

    /* begin: setup sync wait timeout */
    if (*machine_control->timeout != machine_control->prev_timeout) {
        immediate_data = (uint32_t)(*(machine_control->timeout)/(servo_period_ns * 0.000000001)); // ?? sec timeout / one tick interval
        // immediate_data = 1000; // ticks about 1000 * 0.00065536 sec
        // transmit immediate data
        fprintf(stderr,"wou_stepgen.c: setup wait timeout(%u) \n", immediate_data);
        write_machine_param(WAIT_TIMEOUT, immediate_data);
        machine_control->prev_timeout = *machine_control->timeout;
    }
    /* end: setup sync wait timeout */

    /* begin: process motion synchronized input */
    if (*(machine_control->sync_in_trigger) != 0) {
        assert(*(machine_control->sync_in) >= 0);
        assert(*(machine_control->sync_in) < num_gpio_in);
        fprintf(stderr,"wou_stepgen.c: risc singal wait trigged(input(%d) type (%d))\n",(uint32_t)*machine_control->sync_in,
                       (uint32_t)*(machine_control->wait_type));
        // begin: trigger sync in and wait timeout 
        sync_cmd = SYNC_DIN | PACK_IO_ID((uint32_t)*(machine_control->sync_in)) |
                                           PACK_DI_TYPE((uint32_t)*(machine_control->wait_type));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:                               sizeof(uint16_t), (uint8_t *) &sync_cmd);
        // end: trigger sync in and wait timeout
        *(machine_control->sync_in_trigger) = 0;
    }
    /* end: process motion synchronized input */
    
    // for sim, activate ESTOP
    *(machine_control->in[0]) = 1;

    /* begin: process motion synchronized output */
    sync_out_data = 0;
    //obsolete: j = 0;
    for (i = 0; i < machine_control->num_gpio_out; i++) {
        if(((machine_control->prev_out >> i) & 0x01) !=
                (*(machine_control->out[i]))) {
            {
                // write a wou frame for sync output into command FIFO
                fprintf(stderr,"wou_stepgen.c: gpio_%02d => (%d)\n",
                        i,*(machine_control->out[i]));
                sync_cmd = SYNC_DOUT | PACK_IO_ID(i) | PACK_DO_VAL(*(machine_control->out[i]));
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                // sim: wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);
            }
            //obsolete: j ++;
        }

	sync_out_data |= ((*(machine_control->out[i])) << i);
    }
    //obsolete: if (j > 0) {        //
    machine_control->prev_out = sync_out_data;
    //obsolete: }
    /* end: process motion synchronized output */

    /* begin: send application parameters */
    // use carefully
    for (i=0; i<16; i++) {
        // PARAM0 to PARAM15
        if (machine_control->prev_app_param[i] != (*machine_control->app_param[i])) {
            fprintf(stderr, "send application parameters PARAM%d (%d) \n", i, (*machine_control->app_param[i]));
            write_machine_param(PARAM0+i, (*machine_control->app_param[i]));
            machine_control->prev_app_param[i] = *machine_control->app_param[i];
        }
    }
    /* end: send application parameters */

    /* point at stepgen data */
    stepgen = arg;

#if (TRACE!=0)
    _dt++;
    if (*stepgen->enable) {
	DPS("%11u", _dt);	// %11u: '#' + %10s
    }
#endif

    // num_chan: 4, calculated from step_type;
    /* loop thru generators */
// sim:
// sim:    r_load_pos = 0;
// sim:    r_switch_en = 0;
// sim:    r_index_en = prev_r_index_en;
// sim:    /* begin: homing logic */
// sim:    for (n = 0; n < num_chan; n++) {
// sim:	if ((*stepgen->home_state != HOME_IDLE) && stepgen->pos_mode) {
// sim:	    static hal_s32_t prev_switch_pos;
// sim:	    hal_s32_t switch_pos_tmp = 0;
// sim:	    hal_s32_t index_pos_tmp = 0;
// sim:	    /* update home switch and motor index position while homing */
// sim:	    //TODO: to get switch pos and index pos from risc
// sim:	    memcpy((void *) &switch_pos_tmp,
// sim:		   wou_reg_ptr(&w_param,
// sim:			       SSIF_BASE + SSIF_SWITCH_POS + n * 4), 4);
// sim://	    memcpy((void *) &index_pos_tmp,
// sim://		   wou_reg_ptr(&w_param,
// sim://			       SSIF_BASE + SSIF_INDEX_POS + n * 4), 4);
// sim:
// sim:	    *(stepgen->switch_pos) = switch_pos_tmp * stepgen->scale_recip;
// sim:	    *(stepgen->index_pos) = index_pos_tmp * stepgen->scale_recip;
// sim:	    if(prev_switch_pos != switch_pos_tmp) {
// sim://                fprintf(stderr, "wou: switch_pos(0x%04X)\n",switch_pos_tmp);
// sim:                prev_switch_pos = switch_pos_tmp;
// sim:	    }
// sim:
// sim:	    /* check if we should wait for HOME Switch Toggle */
// sim:	    if ((*stepgen->home_state == HOME_INITIAL_BACKOFF_WAIT) ||
// sim:		(*stepgen->home_state == HOME_INITIAL_SEARCH_WAIT) ||
// sim:		(*stepgen->home_state == HOME_FINAL_BACKOFF_WAIT) ||
// sim:		(*stepgen->home_state == HOME_RISE_SEARCH_WAIT) ||
// sim:		(*stepgen->home_state == HOME_FALL_SEARCH_WAIT) ||
// sim:		(*stepgen->home_state == HOME_INDEX_SEARCH_WAIT)) {
// sim:		if (stepgen->prev_home_state != *stepgen->home_state) {
// sim:		    // set r_switch_en to locate SWITCH_POS
// sim:		    // r_switch_en is reset by HW
// sim://		    r_switch_en |= (1 << n);
// sim:		    r_switch_en &= (1 << n); // reset by SW while SIM
// sim:
// sim:		    if((*stepgen->home_state == HOME_INITIAL_SEARCH_WAIT)) {
// sim:                        immediate_data = SEARCH_HOME_HIGH;
// sim:                        machine_control->in[i+2] = 1;
// sim:                    } else if((*stepgen->home_state == HOME_FINAL_BACKOFF_WAIT)) {
// sim:                        immediate_data = SEARCH_HOME_LOW;
// sim:                        machine_control->in[i+2] = 0;
// sim:                    } else if((*stepgen->home_state == HOME_INITIAL_BACKOFF_WAIT)) {
// sim:                        immediate_data = SEARCH_HOME_LOW;
// sim:                        machine_control->in[i+2] = 0;
// sim:                    } else if((*stepgen->home_state == HOME_RISE_SEARCH_WAIT)) {
// sim:                        immediate_data = SEARCH_HOME_HIGH;
// sim:                        machine_control->in[i+2] = 1;
// sim:                    } else if ((*stepgen->home_state == HOME_FALL_SEARCH_WAIT)) {
// sim:                        immediate_data = SEARCH_HOME_LOW;
// sim:                        machine_control->in[i+2] = 0;
// sim:                    } else if(*stepgen->home_state == HOME_INDEX_SEARCH_WAIT){
// sim:                        immediate_data = SEARCH_INDEX;
// sim:
// sim:                    }
// sim:                    write_mot_param (n, (MOTION_TYPE), immediate_data);
// sim:
// sim:
// sim:		}
// sim:
// sim:	    } else if ((*stepgen->home_state == HOME_SET_SWITCH_POSITION) ||
// sim:	                (*stepgen->home_state == HOME_SET_INDEX_POSITION)) {
// sim:                if(normal_move_flag[n] == 1) {
// sim:                    immediate_data = NORMAL_MOVE;
// sim:                    write_mot_param (n, (MOTION_TYPE), immediate_data);
// sim:                    normal_move_flag[n] = 0;
// sim:                }
// sim:
// sim:                (stepgen->prev_pos_cmd) = *(stepgen->pos_fb);
// sim:                (*stepgen->pos_cmd) = *(stepgen->pos_fb);
// sim:            } else if(*stepgen->home_state == HOME_START) {
// sim:                normal_move_flag[n] = 1;
// sim:            }
// sim:
// sim:	    /* check if we should wait for Motor Index Toggle */
// sim:	    if (*stepgen->home_state == HOME_INDEX_SEARCH_WAIT) {
// sim:		if (stepgen->prev_home_state != *stepgen->home_state) {
// sim:		    // set r_index_en while getting into HOME_INDEX_SEARCH_WAIT state
// sim:		    r_index_en |= (1 << n);
// sim:		} else if (r_index_lock & (1 << n)) {
// sim:		    // the motor index is locked
// sim:		    // reset r_index_en by SW
// sim:		    r_index_en &= (~(1 << n));	// reset index_en[n]
// sim:		    *(stepgen->index_enable) = 0;
// sim://		    rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: J[%d] index_en(0x%02X) prev_r_index_en(0x%02X)\n",
// sim://		                                    n, r_index_en, prev_r_index_en);
// sim://		    rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: J[%d] index_pos(%f) INDEX_POS(0x%08X)\n",
// sim://		                                    n, *(stepgen->index_pos), index_pos_tmp);
// sim://		    rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: J[%d] switch_pos(%f) SWITCH_POS(0x%08X)\n",
// sim://		                                    n, *(stepgen->switch_pos), switch_pos_tmp);
// sim:		}
// sim://		stepgen->prev_pos_cmd = *stepgen->pos_cmd;
// sim:	    }
// sim:
// sim:	    if (stepgen->prev_home_state == HOME_IDLE) {
// sim:                /**
// sim:                 * r_load_pos: set to ONE to load PULSE_POS, SWITCH_POS, and
// sim:                 * INDEX_POS with enc_counter at beginning of homing
// sim:                 * (HOME_START state)
// sim:                 *
// sim:		 * reset to ZERO one cycle after setting this register
// sim:                 **/
// sim:		r_load_pos |= (1 << n);
// sim:                if (*(stepgen->enc_pos) != *(stepgen->pulse_pos)) {
// sim:                    /* accumulator gets a half step offset, so it will step half
// sim:                       way between integer positions, not at the integer positions */
// sim:                    stepgen->rawcount = *(stepgen->enc_pos);
// sim://                    (stepgen->prev_pos_cmd) = (double) (stepgen->rawcount) * stepgen->scale_recip;
// sim:                }
// sim:                fprintf(stderr, "j[%d] enc_counter(%d) pulse_pos(%d)\n",
// sim:                        n/*, stepgen->accum*/, *(stepgen->enc_pos), *(stepgen->pulse_pos));
// sim:	    }
// sim:	}
// sim:        stepgen->prev_home_state = *stepgen->home_state;
// sim:	/* move on to next channel */
// sim:	stepgen++;
// sim:    }
    /* enc: homing logic */
    /* check if we should update SWITCH/INDEX positions for HOMING */
    if (r_load_pos != 0) {
	// issue a WOU_WRITE
	// sim: wou_cmd(&w_param,
	// sim: 	WB_WR_CMD, SSIF_BASE | SSIF_LOAD_POS, 1, &r_load_pos);
	// sim: fprintf(stderr, "wou: r_load_pos(0x%x)\n", r_load_pos);
	// sim: wou_flush(&w_param);
    }

    /* check if we should enable HOME Switch Detection */
    if (r_switch_en != 0) {
	// issue a WOU_WRITE
	// sim: wou_cmd(&w_param,
	// sim: 	WB_WR_CMD, SSIF_BASE | SSIF_SWITCH_EN, 1, &r_switch_en);
	// sim: // fprintf(stderr, "wou: r_switch_en(0x%x)\n", r_switch_en);
	// sim: wou_flush(&w_param);
    }

//TODO: RISC_HOMING:    /* check if we should enable MOTOR Index Detection */
//TODO: RISC_HOMING:    if (r_index_en != prev_r_index_en) {
//TODO: RISC_HOMING:	// issue a WOU_WRITE
//TODO: RISC_HOMING:	wou_cmd(&w_param,
//TODO: RISC_HOMING:		WB_WR_CMD, SSIF_BASE | SSIF_INDEX_EN, 1, &r_index_en);
//TODO: RISC_HOMING:	fprintf(stderr, "wou: r_index_en(0x%x)\n", r_index_en);
//TODO: RISC_HOMING:	wou_flush(&w_param);
//TODO: RISC_HOMING:	prev_r_index_en = r_index_en;
//TODO: RISC_HOMING:    }

    pending_cnt += 1;
    if (pending_cnt == JNT_PER_WOF) {
        pending_cnt = 0;

        // send WB_RD_CMD to read registers back
        /* TODO: replace switch pos and index pos with mailbox siwtch pos and index pos.
         *       Also clean index lock in risc.
        */
        // sim: wou_cmd (&w_param,
        // sim:         WB_RD_CMD,
        // sim:         (SSIF_BASE | SSIF_INDEX_LOCK),
        // sim:         1,
        // sim:         data);

        // sim: wou_cmd (&w_param,
        // sim:         WB_RD_CMD,
        // sim:         (SSIF_BASE | SSIF_SWITCH_POS),
        // sim:         16,
        // sim:         data);

//         wou_cmd (&w_param,
//                 WB_RD_CMD,
//                 (SSIF_BASE | SSIF_INDEX_POS),
//                 16,
//                 data);

        // sim: wou_flush(&w_param);

    }

    // check wou.stepgen.00.enable signal directly
    stepgen = arg;
    if (*stepgen->enable != stepgen->prev_enable) {
        stepgen->prev_enable = *stepgen->enable;
        fprintf(stderr,"enable changed(%d)\n", *stepgen->enable);
        if (*stepgen->enable) {
            for(i=0; i<num_chan; i++) {
                immediate_data = 1;
                write_mot_param (i, (ENABLE), immediate_data);
                immediate_data = NORMAL_MOVE;
                write_mot_param (i, (MOTION_TYPE), immediate_data);
            }

        } else {
//            data[0] = 0; // RESET GPIO_OUT
//            wou_cmd (&w_param, WB_WR_CMD,
//                     (uint16_t) (GPIO_BASE | GPIO_OUT),
//                     (uint8_t) 1, data);
            for(i=0; i<num_chan; i++) {
                immediate_data = 0;
                write_mot_param (i, (ENABLE), immediate_data);
            }
        }

    }

    i = 0;
    stepgen = arg;
    enable = *stepgen->enable;            // take enable status of first joint
    for (n = 0; n < num_chan; n++) {
        *stepgen->pos_scale_pin = stepgen->pos_scale; // export pos_scale
        *(stepgen->pos_fb) = (*stepgen->enc_pos) * stepgen->scale_recip;
//        if (*stepgen->enc_pos != stepgen->prev_enc_pos) {
//        if ((*machine_control->bp_tick - machine_control->prev_bp) > ((int32_t)VEL_UPDATE_BP)) {
            // update velocity-feedback only after encoder movement
//            *(stepgen->vel_fb) = ((*stepgen->pos_fb - stepgen->prev_pos_fb) * recip_dt
//                                  / (*machine_control->bp_tick - machine_control->prev_bp)
//                                 );
            *(stepgen->vel_fb) = stepgen->vel_cmd;
            stepgen->prev_pos_fb = *stepgen->pos_fb;
            stepgen->prev_enc_pos = *stepgen->enc_pos;
            if (n == (num_chan - 1)) {
                // update bp_tick for the last joint
                //DEBUG: rtapi_print_msg(RTAPI_MSG_WARN,
                //DEBUG:                 "WOU: j[%d] bp(%u) prev_bp(%u) vel_fb(%f)\n",
                //DEBUG:                 n,
                //DEBUG:                 machine_control->bp_tick,
                //DEBUG:                 machine_control->prev_bp,
                //DEBUG:                 *(stepgen->vel_fb)
                //DEBUG:                );
//                machine_control->prev_bp = *machine_control->bp_tick;
//            }
        }

	/* test for disabled stepgen */
	if (enable == 0) {
	    /* AXIS not PWR-ON */
	    /* keep updating parameters for better performance */
	    stepgen->scale_recip = 1.0 / stepgen->pos_scale;

	    /* set velocity to zero */
	    stepgen->freq = 0;

	    /* update pos fb*/
	    // fetchmail will update enc_pos
	    //*(stepgen->pos_fb) = (*stepgen->enc_counter) * stepgen->scale_recip;

            /* to prevent position drift while toggeling "PWR-ON" switch */
	    (stepgen->prev_pos_cmd) = *stepgen->pos_cmd;

	    /* clear vel status when enable = 0 */
	    stepgen->prev_pos_fb = *stepgen->pos_fb;
	    stepgen->vel_cmd = 0;
	    stepgen->prev_vel_cmd = 0;
	    *(stepgen->vel_fb) = 0;
	    *(stepgen->ctrl_type_switch) = 0;

            r_load_pos = 0;
            if (stepgen->rawcount != *(stepgen->enc_pos)) {
                r_load_pos |= (1 << n);
                /**
                 * accumulator gets a half step offset, so it will step
                 * half way between integer positions, not at the integer
                 * positions.
                 **/
                stepgen->rawcount = *(stepgen->enc_pos);
                /* load SWITCH, INDEX, and PULSE positions with enc_counter */
                // sim: wou_cmd(&w_param,
                // sim:         WB_WR_CMD, SSIF_BASE | SSIF_LOAD_POS, 1, &r_load_pos);
//                fprintf(stderr, "j[%d] enc_counter(%d) pulse_pos(%d)\n",
//                        n/*, stepgen->accum*/, *(stepgen->enc_pos), *(stepgen->pulse_pos));
//                fprintf(stderr, "j[%d] prev_pos_cmd(%f) pos_cmd(%f) rawcount(%lld)\n",
//                        n, (stepgen->prev_pos_cmd), *(stepgen->pos_cmd), stepgen->rawcount);
            }

            assert (i == n); // confirm the JCMD_SYNC_CMD is packed with all joints
            i += 1;
            // sim: wou_flush(&w_param);
            // sim: wou_pos_cmd = 0;
            // sim: sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);

            memcpy(data + 2*n* sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));
            sync_cmd = 0;

            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd,
                               sizeof(uint16_t));
            if (n == (num_chan - 1)) {
                // send to WOU when all axes commands are generated
                // sim: wou_cmd(&w_param,
                // sim:         WB_WR_CMD,
                // sim:         (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_chan, data);
            }

            // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds
            {
                /* step_len is for electron characteristic: pulse duration. */
                double min_ns_per_step = stepgen->step_len;
                double max_steps_per_s = 1.0e9 / min_ns_per_step;

                physical_maxvel = max_steps_per_s / fabs(stepgen->pos_scale);
                physical_maxvel = force_precision(physical_maxvel);

                if (stepgen->maxvel < 0.0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                                    "stepgen.%02d.maxvel < 0, setting to its absolute value\n",
                                    n);
                    stepgen->maxvel = fabs(stepgen->maxvel);
                }

                if (stepgen->maxvel > physical_maxvel) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                                    "stepgen.%02d.maxvel is too big for current step timings & position-scale, clipping to max possible\n",
                                    n);
                    stepgen->maxvel = physical_maxvel;
                }

    //obsolete:	    if (stepgen->maxvel == 0.0) {
    //obsolete:		maxvel = physical_maxvel;
    //obsolete:	    } else {
    //obsolete:		maxvel = stepgen->maxvel;
    //obsolete:	    }
            }

            /* and skip to next one */
            stepgen++;
	    continue;
	}
        
	//
	// first sanity-check our maxaccel and maxvel params
	//


	/* at this point, all scaling, limits, and other parameter
	   changes hrawcount_diff_accumave been handled - time for the main control */

	if (stepgen->pos_mode) {
	    /* position command mode */
	    stepgen->vel_cmd = ((*stepgen->pos_cmd) - (stepgen->prev_pos_cmd)) * recip_dt;
	} else {
	    /* velocity command mode */
	    if (stepgen->prev_ctrl_type_switch != *stepgen->ctrl_type_switch) {
	        stepgen->prev_vel_cmd = 0;
	        stepgen->prev_pos_cmd = *stepgen->pos_cmd;
	        // do more thing if necessary.
	    }
	    if (*stepgen->ctrl_type_switch == 0) {
                stepgen->vel_cmd = (*stepgen->pos_cmd) ; // notice: has to wire *pos_cmd-pin to velocity command input

                /* begin:  ramp up/down spindle */
                maxvel = stepgen->maxvel;   /* unit/s */
                if (stepgen->vel_cmd > maxvel) {
                    stepgen->vel_cmd = maxvel;
                } else if(stepgen->vel_cmd < -maxvel){
                    stepgen->vel_cmd = -maxvel;
                }
                stepgen->accel_cmd = (stepgen->vel_cmd - stepgen->prev_vel_cmd) * recip_dt; /* unit/s^2 */

                if (stepgen->accel_cmd > stepgen->maxaccel) {
                    stepgen->accel_cmd = stepgen->maxaccel;
                    stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd * dt;
                } else if (stepgen->accel_cmd < -(stepgen->maxaccel)) {
                    stepgen->accel_cmd = -(stepgen->maxaccel);
                    stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd * dt;
                }
                /* end: ramp up/down spindle */
//                fprintf(stderr,"prev_vel(%f) accel_cmd(%f) \n", stepgen->prev_vel_cmd, stepgen->accel_cmd);
	    } else {
	        stepgen->vel_cmd = ((*stepgen->pos_cmd) - (stepgen->prev_pos_cmd)) * recip_dt;
	    }
	    stepgen->prev_ctrl_type_switch = *stepgen->ctrl_type_switch;
//obsolete:            if(*machine_control->spindle_enable)
//obsolete:                stepgen->vel_cmd = *machine_control->spindle_vel_cmd * 360.0;
//obsolete:            else
//obsolete:                stepgen->vel_cmd = 0;
	}
	{
            
            //wou_pos_cmd = (int32_t)((stepgen->vel_cmd * dt *(stepgen->pos_scale)) * (1 << pulse_fraction_bit[n]));
            integer_pos_cmd = (int32_t)((stepgen->vel_cmd * dt *(stepgen->pos_scale)) * (1 << FRACTION_BITS));

            /* extract integer part of command */
            wou_pos_cmd = abs(integer_pos_cmd) >> FRACTION_BITS;
            
            if(wou_pos_cmd >= 8192) {
                fprintf(stderr,"j(%d) pos_cmd(%f) prev_pos_cmd(%f) home_state(%d) vel_cmd(%f)\n",
                        n ,
                        (*stepgen->pos_cmd), 
                        (stepgen->prev_pos_cmd), 
                        *stepgen->home_state, 
                        stepgen->vel_cmd);
                fprintf(stderr,"wou_stepgen.c: wou_pos_cmd(%d) too large\n", wou_pos_cmd);
                assert(0);
            }
            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            
            if (integer_pos_cmd >= 0) {
                // wou_cmd_accum = wou_pos_cmd << FRACTION_BITS;
                sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);
            } else {
                // wou_cmd_accum = -(wou_pos_cmd << FRACTION_BITS);
                sync_cmd = SYNC_JNT | DIR_N | (POS_MASK & wou_pos_cmd);
            }
            memcpy(data + (2 * n * sizeof(uint16_t)), &sync_cmd, sizeof(uint16_t));
            
            wou_pos_cmd = (abs(integer_pos_cmd)) & FRACTION_MASK;


            /* packing fraction part */
            sync_cmd = (uint16_t) wou_pos_cmd;
            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));

            // (stepgen->prev_pos_cmd) += (double) ((wou_cmd_accum * stepgen->scale_recip)/(1<<FRACTION_BITS));
            (stepgen->prev_pos_cmd) += (double) ((integer_pos_cmd * stepgen->scale_recip)/(1<<FRACTION_BITS));

            stepgen->prev_vel_cmd = stepgen->vel_cmd;
            *(stepgen->enc_pos) =  (stepgen->prev_pos_cmd) * stepgen->pos_scale;  //SIM
            if (stepgen->pos_mode == 0) {
                // TODO: find a better way for spindle control
                // TODO: let TRAJ-PLANNER judge the index/revolution
                // TODO: remove this section from wou_stepgen.c
                double delta;
                int32_t spindle_pos;
                int32_t spindle_irevs;
                double pos_scale;
                pos_scale = stepgen->pos_scale;
                // machine_control->spindle_enc_count += (wou_cmd_accum/(1<<FRACTION_BITS));
                machine_control->spindle_enc_count += (integer_pos_cmd >> FRACTION_BITS);
                spindle_pos = machine_control->spindle_enc_count;
                spindle_irevs = (machine_control->spindle_enc_count % ((int32_t)(pos_scale)));

                delta = ((double)(spindle_irevs - machine_control->prev_spindle_irevs))/pos_scale;

                machine_control->prev_spindle_irevs = spindle_irevs;

                {
                    // 用 revs 記錄小數點以上的旋轉圈數
                    // 用 rev 計算小數點以下的旋轉圈數
                    // 超過一圈時，更新 index 點的 encoder 位置 (last_spindle_index_pos)
                    // 超過一圈時，更新 revs
                    static double revs;
                    double rev;

                    if ((*machine_control->spindle_index_enable == 1) && (*machine_control->spindle_at_speed)) {

                        if (delta < -0.5) {
                            // ex.: 0.9 -> 0.1 (forward)
                            machine_control->last_spindle_index_pos = machine_control->spindle_enc_count;
                            *machine_control->spindle_index_enable = 0;
                            revs = 0;

                        } else if (delta > 0.5) {
                            // ex.: 0.1 -> 0.9 (backward)
                            machine_control->last_spindle_index_pos = machine_control->spindle_enc_count;
                            *machine_control->spindle_index_enable = 0;
                            revs = 0;
                        }
                    }

                    //orig: *machine_control->spindle_revs = (((int32_t)(spindle_pos - machine_control->last_spindle_index_pos))/pos_scale);

                    rev = ((int32_t)(spindle_pos - machine_control->last_spindle_index_pos))/pos_scale;
                    *machine_control->spindle_revs = revs + rev;
                    if (rev < -1.0) {
                        // rotate toward negative encoder position
                        // and, larger than a full revolution (after passing index point)
                        machine_control->last_spindle_index_pos -= (int32_t) pos_scale;
                        revs -= 1.0;
                    } else if (rev > 1.0){
                        // rotate toward positive encoder position
                        // and, larger than a full revolution (after passing index point)
                        machine_control->last_spindle_index_pos += (int32_t) pos_scale;
                        revs += 1.0;
                    }
                }
            }

	}

        if (n == (num_chan - 1)) {
            // send to WOU when all axes commands are generated
            // sim: wou_cmd(&w_param,
            // sim:         WB_WR_CMD,
            // sim:         (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_chan, data);
        }
	DPS("  0x%13X%15.7f%15.7f%3d",
	    integer_pos_cmd, 
            (stepgen->prev_pos_cmd), 
            *stepgen->pos_fb,
            *stepgen->home_state);

	/* move on to next channel */
	stepgen++;
    }

    DPS("  %15.7f", *machine_control->spindle_revs);
    // send velocity status to RISC
//    if(*machine_control->ahc_state == 1) {

    if (*machine_control->motion_state != machine_control->prev_motion_state) {
        if (*machine_control->motion_state == ACCEL_S3) {
            sync_cmd = SYNC_VEL | 0x0001;
//             fprintf(stderr,"wou_stepgen.c: motion_state == ACCEL_S3\n");
        } else {
            sync_cmd = SYNC_VEL;
//                 fprintf(stderr,"motion_state != ACCEL_S3\n");
        }
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), data);

//        if (is_probing) {
//            if (*machine_control->motion_state == ACCEL_S7 ||
//                    *machine_control->motion_state == ACCEL_S6) {
//                *machine_control->probe_type = PROBE_NONE;
//                is_probing = 0;
//                fprintf(stderr, "clean probe state  probe_type(%0f) prev_probe_type(%0f)\n",
//                        *machine_control->probe_type, machine_control->prev_probe_type);
//            }
//            fprintf(stderr, "motion_state(%d)\n", *machine_control->motion_state);
//        }
    }
    machine_control->prev_motion_state = *machine_control->motion_state;
//        fprintf(stderr,"prev_motion_state(%d)\n", machine_control->prev_motion_state);
//    }

#if (TRACE!=0)
    if (*(stepgen - 1)->enable) {
	DPS("\n");
    }
#endif

/* restore saved message level */
    rtapi_set_msg_level(msg);
    /* done */
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_gpio(gpio_t * addr)
{
    int i, retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_WARN);
    rtapi_set_msg_level(RTAPI_MSG_ALL);

    // export Analog IN
    for (i = 0; i < 1; i++) {
        retval = hal_pin_float_newf(HAL_OUT, &(addr->a_in[i]), comp_id,
                                  "wou.gpio.a_in.%02d", i);
        if (retval != 0) {
            return retval;
        }
	*(addr->a_in[i]) = 0;
    }

    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
} // export_gpio ()

static int export_analog(analog_t * addr)
{
    int i, retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_WARN);
    rtapi_set_msg_level(RTAPI_MSG_ALL);

    // export Analog IN
    for (i = 0; i < 16; i++) {
        retval = hal_pin_s32_newf(HAL_OUT, &(addr->in[i]), comp_id,
                                  "wou.analog.in.%02d", i);
        if (retval != 0) {
            return retval;
        }
	*(addr->in[i]) = 0;
    }
    
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
} // export_analog ()


static int export_stepgen(int num, stepgen_t * addr, int step_type,
			  int pos_mode)
{
    int retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_WARN);
    rtapi_set_msg_level(RTAPI_MSG_ALL);
    
    /* debug info */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->joint_cmd), comp_id,
			      "wou.stepgen.%d.joint_cmd", num);
    if (retval != 0) {
	return retval;
    }

    /* export pin for counts captured by wou_update() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->pulse_pos), comp_id,
			      "wou.stepgen.%d.pulse_pos", num);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc_pos), comp_id,
			      "wou.stepgen.%d.enc_pos", num);
    if (retval != 0) {
	return retval;
    }

    /* export pin for scaled switch position (absolute motor position) */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->switch_pos), comp_id,
				"wou.stepgen.%d.switch-pos", num);
    if (retval != 0) {
	return retval;
    }

    /* export pin for scaled index position (absolute motor position) */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->index_pos), comp_id,
				"wou.stepgen.%d.index-pos", num);
    if (retval != 0) {
	return retval;
    }

    /* export pin for index_enable */
    retval = hal_pin_bit_newf(HAL_IO, &(addr->index_enable), comp_id,
			      "wou.stepgen.%d.index-enable", num);
    if (retval != 0) {
	return retval;
    }

    /* export parameter for position scaling */
    retval = hal_param_float_newf(HAL_RW, &(addr->pos_scale), comp_id,
				  "wou.stepgen.%d.position-scale", num);
    if (retval != 0) {
	return retval;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos_scale_pin), comp_id,
                  "wou.stepgen.%d.position-scale-pin", num);
    if (retval != 0) {
    return retval;
    }


    /* export pin for pos/vel command */
    retval = hal_pin_float_newf(HAL_IN, &(addr->pos_cmd), comp_id,
				    "wou.stepgen.%d.position-cmd", num);
    if (retval != 0) {
	return retval;
    }

    /* export parameter to obtain homing state */
    retval = hal_pin_s32_newf(HAL_IN, &(addr->home_state), comp_id,
			      "wou.stepgen.%d.home-state", num);
    if (retval != 0) {
	return retval;
    }

    /* export pin for enable command */
    addr->prev_enable = 0;
    retval = hal_pin_bit_newf(HAL_IN, &(addr->enable), comp_id,
			      "wou.stepgen.%d.enable", num);
    if (retval != 0) {
	return retval;
    }
    
    /* export pin for scaled position captured by update() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos_fb), comp_id,
				"wou.stepgen.%d.position-fb", num);
    if (retval != 0) {
	return retval;
    }
    
    /* export pin for velocity feedback (unit/sec) */
    retval = hal_pin_float_newf(HAL_IN, &(addr->vel_fb), comp_id,
				    "wou.stepgen.%d.velocity-fb", num);
    if (retval != 0) {
	return retval;
    }

    /* export param for scaled velocity (frequency in Hz) */
    retval = hal_param_float_newf(HAL_RO, &(addr->freq), comp_id,
				  "wou.stepgen.%d.frequency", num);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for max frequency */
    retval = hal_param_float_newf(HAL_RW, &(addr->maxvel), comp_id,
				  "wou.stepgen.%d.maxvel", num);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for max accel/decel */
    retval = hal_param_float_newf(HAL_RW, &(addr->maxaccel), comp_id,
				  "wou.stepgen.%d.maxaccel", num);
    if (retval != 0) {
	return retval;
    }
    /* every step type uses steplen */
    retval = hal_param_u32_newf(HAL_RW, &(addr->step_len), comp_id,
				"wou.stepgen.%d.steplen", num);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for pid error term */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->cmd_error), comp_id,
                              "wou.stepgen.%d.pid.error", num);
    if (retval != 0) {
        return retval;
    }
    /* export parameter for pid output in risc */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pid_output), comp_id,
                              "wou.stepgen.%d.pid.output", num);
    if (retval != 0) {
        return retval;
    }
    /* export parameter for cmd in risc*/
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pid_cmd), comp_id,
                              "wou.stepgen.%d.pid.cmd", num);
    if (retval != 0) {
        return retval;
    }
    if (step_type < 2) {
	/* step/dir and up/down use 'stepspace' */
	retval = hal_param_u32_newf(HAL_RW, &(addr->step_space),
				    comp_id, "wou.stepgen.%d.stepspace",
				    num);
	if (retval != 0) {
	    return retval;
	}
    }

    /* export pin for control type switch (for vel control only) */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->ctrl_type_switch), comp_id,
                              "wou.stepgen.%d.ctrl-type-switch", num);
    if (retval != 0) {
        return retval;
    }

    /* export pin for following error */
    retval = hal_pin_bit_newf(HAL_OUT, &(addr->ferror_flag), comp_id,
                              "wou.stepgen.%d.ferror-flag", num);
    if (retval != 0) {
        return retval;
    }

    /* set default parameter values */
    addr->pos_scale = 1.0;
    addr->scale_recip = 0.0;
    addr->freq = 0.0;
    addr->maxvel = 0.0;
    addr->maxaccel = 0.0;
    addr->step_type = step_type;
    addr->pos_mode = pos_mode;
    /* timing parameter defaults depend on step type */
    addr->step_len = 1;
    if (step_type < 2) {
	addr->step_space = 1;
    } else {
	addr->step_space = 0;
    }
    //obsolete: if (step_type == 0) {
    //obsolete:     addr->dir_hold_dly = 1;
    //obsolete:     addr->dir_setup = 1;
    //obsolete: } else {
    //obsolete:     addr->dir_hold_dly = 1;
    //obsolete:     addr->dir_setup = 0;
    //obsolete: }
    /* init the step generator core to zero output */
//    addr->cur_pos = 0.0;
    /* accumulator gets a half step offset, so it will step half
       way between integer positions, not at the integer positions */
//    addr->accum = 1L << (PICKOFF-1);
    addr->rawcount = 0;

    addr->prev_pos_cmd = 0;
    addr->prev_pos_fb = 0;
    addr->sum_err_0 = 0;
    addr->sum_err_1 = 0;

    *(addr->enable) = 0;
    /* other init */
    addr->printed_error = 0;
    // addr->old_pos_cmd = 0.0;
    /* set initial pin values */
    *(addr->pulse_pos) = 0;
    addr->prev_enc_pos = 0;
    *(addr->enc_pos) = 0;
    *(addr->pos_fb) = 0.0;
    *(addr->vel_fb) = 0;
    *(addr->switch_pos) = 0.0;
    *(addr->index_pos) = 0.0;
    *(addr->pos_cmd) = 0.0;
    (addr->vel_cmd) = 0.0;
    (addr->prev_vel_cmd) = 0.0;
    (addr->accel_cmd) = 0.0;
    *(addr->home_state) = HOME_IDLE;
    addr->prev_home_state = HOME_IDLE;
    *(addr->ctrl_type_switch) = 0;
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}


static int export_machine_control(machine_control_t * machine_control)
{
    int i, retval, msg;

    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_WARN);
    rtapi_set_msg_level(RTAPI_MSG_ALL);
    machine_control->num_gpio_in = num_gpio_in;
    machine_control->num_gpio_out = num_gpio_out;

    // export input status pin
     for (i = 0; i < machine_control->num_gpio_in; i++) {
         retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->in[i]), comp_id,
                                   "wou.gpio.in.%02d", i);
         if (retval != 0) {
             return retval;
         }
         *(machine_control->in[i]) = 0;
         
         retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->in_n[i]), comp_id,
                                   "wou.gpio.in.%02d.not", i);
         if (retval != 0) {
             return retval;
         }
         *(machine_control->in_n[i]) = 0;
     }

    retval =
	hal_pin_bit_newf(HAL_IO, &(machine_control->sync_in_trigger), comp_id,
			 "wou.sync.in.trigger");
    *(machine_control->sync_in_trigger) = 0;	// pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    retval =
	hal_pin_float_newf(HAL_IN, &(machine_control->sync_in), comp_id,
			 "wou.sync.in.index");
    *(machine_control->sync_in) = 0;	// pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->wait_type), comp_id,
			      "wou.sync.in.wait_type");
    if (retval != 0) {
	return retval;
    }
    *(machine_control->wait_type) = 0;
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->timeout), comp_id,
				"wou.sync.in.timeout");
    if (retval != 0) {
	return retval;
    }
    *(machine_control->timeout) = 0.0;

    for (i = 0; i < num_gpio_out; i++) {
	retval =
	    hal_pin_bit_newf(HAL_IN, &(machine_control->out[i]), comp_id,
			     "wou.gpio.out.%02d", i);
	if (retval != 0) {
	    return retval;
	}
	*(machine_control->out[i]) = 0;
    }

    retval =
            hal_pin_float_newf(HAL_IN, &(machine_control->analog_ref_level), comp_id,
                             "wou.sync.analog_ref_level");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->analog_ref_level) = 0;    // pin index must not beyond index

    retval =
            hal_pin_float_newf(HAL_IN, &(machine_control->ahc_state), comp_id,
                            "wou.ahc.state");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->ahc_state) = 0;

    retval =
            hal_pin_float_newf(HAL_IN, &(machine_control->ahc_level), comp_id,
                            "wou.ahc.level");
    if (retval != 0) {
            return retval;
    }
    *(machine_control->ahc_level) = 0;

    retval =
                hal_pin_float_newf(HAL_IN, &(machine_control->ahc_max_offset), comp_id,
                                "wou.ahc.max_offset");
    if (retval != 0) {
            return retval;
    }
    *(machine_control->ahc_max_offset) = 0;

    retval =
                hal_pin_float_newf(HAL_IN, &(machine_control->ahc_max_level), comp_id,
                                "wou.ahc.max_level");
    if (retval != 0) {
            return retval;
    }
    *(machine_control->ahc_max_level) = 0;

    retval =
                hal_pin_float_newf(HAL_IN, &(machine_control->ahc_min_level), comp_id,
                                "wou.ahc.min_level");
    if (retval != 0) {
            return retval;
    }
    *(machine_control->ahc_min_level) = 0;

    /* auto height control switches */
// TODO: replace by CL
//    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->thc_enbable), comp_id,
//                                "wou.thc_enable");
//    if (retval != 0) {
//        return retval;
//    } else {
//        *machine_control->thc_enbable = 1; // default enabled
//    }
// TODO: replace by CL
//    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->plasma_enable), comp_id,
//                            "wou.plasma_enable");
//    if (retval != 0) {
//        return retval;
//    } else {
//        *machine_control->plasma_enable = 1;    // default enabled
//    }

    /* wou command */
    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->wou_cmd), comp_id,
                             "wou.motion.cmd");
    *(machine_control->wou_cmd) = 0;    // pin index must not beyond index
    machine_control->prev_wou_cmd = 0;
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->wou_status), comp_id,
                             "wou.motion.status");
    *(machine_control->wou_status) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->motion_state), comp_id,
                             "wou.motion-state");
    *(machine_control->motion_state) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(machine_control->spindle_revs), comp_id,
                                     "wou.spindle.spindle-revs");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->spindle_revs) = 0;

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->spindle_index_enable), comp_id,
                                    "wou.spindle.index-enable");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->spindle_index_enable) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->spindle_at_speed), comp_id,
                                       "wou.spindle.at-speed");
    if (retval != 0) {
       return retval;
    }
    *(machine_control->spindle_at_speed) = 0;
    
    retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->mpg_count), comp_id,
                             "wou.mpg_count");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->mpg_count) = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->test_pattern), comp_id,
                             "wou.test_pattern");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->test_pattern) = 0;

    for (i=0; i<14; i++) {
        retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->tick[i]), comp_id,
                                     "wou.risc.tick.count-%d", i);
        if (retval != 0) {
            return retval;
        }
        *(machine_control->tick[i]) = 0;
    }

    for (i=0; i<8; i++) {
        retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->debug[i]), comp_id,
                                     "wou.debug.value-%d", i);
        if (retval != 0) {
            return retval;
        }
        *(machine_control->debug[i]) = 0;
    }

    // dout0: the DOUT value obtained from RISC
    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->dout0), comp_id,
                                     "wou.dout0");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->dout0) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->bp_tick), comp_id,
                                 "wou.bp_tick");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->bp_tick) = 0;


    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->crc_error_counter), comp_id,
                                 "wou.crc-error-counter");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->crc_error_counter) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->wou_bp_tick), comp_id,
                                 "wou.wou_bp_tick");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->wou_bp_tick) = 0;

    /* application parameters */
    for (i=0; i<16; i++) {
        retval = hal_pin_s32_newf(HAL_IN, &(machine_control->app_param[i]), comp_id,
                                     "wou.risc.param.%02d", i);
        if (retval != 0) {
            return retval;
        }
        *(machine_control->app_param[i]) = 0;
    }

    assert(actual_joint_num<=8);
    for (i=0; i<actual_joint_num; i++) {
        retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->encoder_count[i]), comp_id,
                "wou.risc.motion.joint.encoder-count.%02d", i);
        *(machine_control->encoder_count[i]) = 0;

    }
    for (i=0; i<actual_joint_num; i++) {
        retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->pulse_count[i]), comp_id,
                "wou.risc.motion.joint.pulse-count.%02d", i);
        *(machine_control->pulse_count[i]) = 0;

    }
    for (i=0; i<actual_joint_num; i++) {
        retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->ferror[i]), comp_id,
                "wou.risc.motion.joint.ferror.%02d", i);
        *(machine_control->ferror[i]) = 0;

    }

    machine_control->prev_out = 0;

    machine_control->last_spindle_index_pos = 0;
    machine_control->prev_spindle_irevs = 0;
    machine_control->spindle_revs_integer  = 0;

    /* restore saved message level*/
    rtapi_set_msg_level(msg);
    return 0;
}				// export_gpio ()


// vim:sw=4:sts=4:et:
