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

/* module information */
MODULE_AUTHOR("Yi-Shin Li");
MODULE_DESCRIPTION("Wishbone Over USB for EMC HAL");
MODULE_LICENSE("GPL");

//obsolete: int step_type[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
//obsolete: RTAPI_MP_ARRAY_INT(step_type, MAX_CHAN,
//obsolete: 		   "stepping types for up to 8 channels");

const char *ctrl_type[MAX_CHAN] =
    { " ", " ", " ", " ", " ", " ", " ", " " };
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

int servo_period_ns = -1;   // init to '-1' for testing valid parameter value
RTAPI_MP_INT(servo_period_ns, "used for calculating new velocity command, unit: ns");

//obsolete: int step_cur[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
//obsolete: RTAPI_MP_ARRAY_INT(step_cur, MAX_CHAN,
//obsolete: 		   "current limit for up to 8 channel of stepping drivers");

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

const char *max_jerk_str[MAX_CHAN] =
    { "100.0", "100.0", "100.0", "100.0", "100.0", "100.0", "100.0", "100.0" };
RTAPI_MP_ARRAY_STRING(max_jerk_str, MAX_CHAN,
                      "max jerk value for up to 8 channels");

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

// const char *probe_pin_type="DIGITAL_PIN"; // ANALOG_IN
// RTAPI_MP_STRING(probe_pin_type,
//                 "indicate probing type");

const char *probe_config= "0x00010000";         // probing input channel
RTAPI_MP_STRING(probe_config,
                "probe config for RISC");

const char *jog_config_str[MAX_CHAN] =
    { "0x00020000", "0x00020000", "0x00020000", "0x00020000",
        "0x00020000", "0x00020000", "0x00020000", "0x00020000" };
RTAPI_MP_ARRAY_STRING(jog_config_str, MAX_CHAN,
                      "jog config for RISC");



//const char *probe_decel_cmd= "0";         // deceleration command for probing in user-unit/s
//RTAPI_MP_STRING(probe_decel_cmd,
//                "deceleration for probing");

const char *probe_analog_ref_level= "2048";
RTAPI_MP_STRING(probe_analog_ref_level,
                "indicate probing level used by analog probing");

//obsolete: const char *act_jnt_num="4";
//obsolete: RTAPI_MP_STRING(act_jnt_num,
//obsolete:                "actual joints controlled by risc");

// int alr_output = 0x00000000;
// RTAPI_MP_INT(alr_output, "Digital Output when E-Stop presents");
const char *alr_output= "0";
RTAPI_MP_STRING(alr_output,
                "Digital Output when E-Stop presents");

static int test_pattern_type = 0;  // use dbg_pat_str to update dbg_pat_type

static const char *board = "7i43u";
static const char wou_id = 0;
static wou_param_t w_param;
static int pending_cnt;
//static int normal_move_flag[MAX_CHAN] = {0, 0, 0, 0, 0, 0, 0, 0};
//static uint8_t is_probing = 0;

#define JNT_PER_WOF     2       // SYNC_JNT commands per WOU_FRAME


//trace INDEX_HOMING: static int debug_cnt = 0;


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/** This structure contains the runtime data for a single generator. */

/* structure members are ordered to optimize caching for makepulses,
   which runs in the fastest thread */

// #pragma pack(push)  /* push current alignment to stack */
// #pragma pack(1)     /* set alignment to 1 byte boundary */
typedef struct {
    // hal_pin_*_newf: variable has to be pointer
    // hal_param_*_newf: varaiable not necessary to be pointer
    /* stuff that is read but not written by makepulses */
    int32_t rawcount;
    hal_bit_t prev_enable;
    hal_bit_t *enable;		/* pin for enable stepgen */
    hal_u32_t step_len;		/* parameter: step pulse length */
    /* stuff that is not accessed by makepulses */
    int pos_mode;		/* 1 = position command mode, 0 = velocity command mode */
    //obsolete: hal_u32_t step_space;	/* parameter: min step pulse spacing */
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
    hal_float_t *vel_cmd;	        /* pin: velocity command (pos units/sec) */
    double prev_vel_cmd;        /* prev vel cmd: previous velocity command */
    hal_float_t *pos_cmd;	/* pin: position command (position units) */
    double prev_pos_cmd;        /* prev pos_cmd: previous position command */
    hal_float_t *probed_pos;
    hal_bit_t *align_pos_cmd;
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

//    hal_s32_t *home_state;	/* pin: home_state from homing.c */
//    hal_s32_t prev_home_state;	/* param: previous home_state for homing */
    
    /* pid info */
    hal_float_t *pid_cmd;
    hal_float_t *cmd_error;       /* cmd error */
    hal_float_t *pid_output;      /* pid output */
    
    /* motion type be set */
    int32_t motion_type;          /* motion type wrote to risc */
    
    hal_s32_t     *cmd_fbs;     /* position command retained by RISC (unit: pulse) */
    hal_float_t   *cmd_fbf;     /* position command retained by RISC (cmd_fbs divided by scale) */
    uint32_t      jog_config;   /* for risc jogging */
    hal_float_t   *jog_scale;   /* for risc jogging */
    hal_bit_t     *jog_enable;
    int8_t        prev_jog_enable;
    double       prev_jog_scale;
//    hal_bit_t     *tp_enable;  /* connect to joint jog active */
} stepgen_t;
// #pragma pack(pop)   /* restore original alignment from stack */

typedef struct {
    // Analog input: 0~4.096VDC, up to 16 channel
    hal_s32_t *in[16];
    // TODO: may add *out[] here
} analog_t;

// machine_control_t:
typedef struct {
    hal_bit_t *ignore_ahc_limit;
    hal_bit_t *align_pos_cmd;
    hal_bit_t *ignore_host_cmd;
    int32_t     prev_vel_sync;
    hal_float_t *vel_sync_scale;
    hal_float_t *current_vel;
    hal_float_t *requested_vel;
    hal_float_t *feed_scale;
    hal_bit_t   *vel_sync;  /* A pin to determine when (vel * feedscale) beyond (req_vel * vel_sync_scale) */
    hal_bit_t   *rt_abort;  // realtime abort to FPGA
    hal_bit_t   *cl_abort;  // realtime abort from CL to FPGA
    /* plasma control */
    hal_bit_t   *thc_enbable;
    //TODO: replace plasma enable with output enable for each pin.
    hal_bit_t   *plasma_enable;
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
    uint32_t      prev_ahc_max_level;
    hal_u32_t    *ahc_max_level;
    uint32_t      prev_ahc_min_level;
    hal_u32_t    *ahc_min_level;
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
    hal_bit_t *spindle_index_enable;    // TODO: move spindle-index sync into motion
    hal_bit_t *spindle_at_speed;
    hal_float_t *spindle_revs;
    double   spindle_revs_integer;
    int32_t last_spindle_index_pos;
    double  last_spindle_index_pos_int;
    int32_t spindle_enc_count;          // predicted spindle-encoder count
    int32_t prev_spindle_irevs;
    /* test pattern  */
    hal_s32_t *test_pattern;
    /* MPG */
    hal_s32_t *mpg_count;
    /* DEBUG */
    hal_s32_t *debug[8];
    /* tick */
    hal_u32_t *tick[14];
    /* application parameter*/
    hal_s32_t *app_param[16];
    int32_t prev_app_param[16];
    hal_bit_t *send_app_param; // IO: trigger parameters to be sent
    hal_u32_t *usb_cmd;
    hal_u32_t *last_usb_cmd;
    hal_float_t *usb_cmd_param[4];
    hal_float_t *last_usb_cmd_param[4];
//    hal_bit_t *prog_is_running;
} machine_control_t;

/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t *stepgen_array;
//obsolete: static gpio_t *gpio;
static analog_t *analog;
static machine_control_t *machine_control;
//obsolete: static int32_t actual_joint_num;
/* file handle for wou step commands */
// static FILE *wou_fh;

/* lookup tables for stepping types 2 and higher - phase A is the LSB */

#define MAX_STEP_TYPE 14

/* other globals */
static int comp_id;		/* component ID */
static int num_joints = 0;	/* number of step generators configured */
static double dt;		/* update_freq period in seconds */
static double recip_dt;		/* reciprocal of period, avoids divides */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_stepgen(int num, stepgen_t * addr, /* obsolete: int step_type, */
			  int pos_mode);
//obsolete: static int export_gpio(gpio_t * addr);
static int export_analog(analog_t * addr);
static int export_machine_control(machine_control_t * machine_control);
static void update_freq(void *arg, long period);
static void update_rt_cmd(void);


void endian_swap(uint32_t  *x)
{
    *x = (*x>>24) | ((*x<<8) & 0x00FF0000) |((*x>>8) & 0x0000FF00) |(*x<<24);
}

/************************************************************************
 * callback functions for libwou                                        *
 ************************************************************************/
static void get_crc_error_counter(int32_t crc_error_counter)
{
    // store crc_error_counter
    return;
}

static void fetchmail(const uint8_t *buf_head)
{
    // char        *buf_head;
    int         i;
    uint16_t    mail_tag;
    uint32_t    *p, din[2], dout[1];
    stepgen_t   *stepgen;
    uint32_t    bp_tick;    // served as previous-bp-tick
    uint32_t    ferror_flag;
#if (MBOX_LOG)
    char        dmsg[1024];
    int         dsize;
#elif (DEBUG_LOG)
    char        dmsg[1024];
    int         dsize;
#endif
    
    // buf_head = (char *) wou_mbox_ptr (&w_param);
    memcpy(&mail_tag, (buf_head + 2), sizeof(uint16_t));

    // BP_TICK
    p = (uint32_t *) (buf_head + 4);
    bp_tick = *p;
    if (machine_control->prev_bp == bp_tick) {
        // skip mailbox parsing because there isn't new bp_tick
        return;
    }
    *machine_control->bp_tick = bp_tick;

    switch(mail_tag)
    {
    case MT_PID:
        stepgen = stepgen_array;
        for (i=0; i<num_joints; i++) {
            p += 1;
            *(stepgen->pid_output) = (hal_float_t)((int32_t)*p);
            // cmd error
            p += 1;
            *(stepgen->cmd_error) = (hal_float_t)((int32_t)*p);
            stepgen += 1;   // point to next joint
        }
        break;
    case MT_MOTION_STATUS:
        /* for PLASMA with ADC_SPI */
        //redundant: p = (uint32_t *) (buf_head + 4); // BP_TICK
        stepgen = stepgen_array;
        for (i=0; i<num_joints; i++) {
            // PULSE_POS
            p += 1;
            *(stepgen->pulse_pos) = *p;
            //TODO: confirm necessary: 
            *(stepgen->pid_cmd) = (*(stepgen->pulse_pos))*(stepgen->scale_recip);
            // enc counter
            p += 1;
            *(stepgen->enc_pos) = *p;
            p += 1;
#ifndef __ARMEL__
            // for unknown reason, the next expression will cause alignment
            // trap for ARM processor (observed with dmesg)
            *(stepgen->cmd_fbs) = ((int32_t)*p);
            //TODO: confirm necessary: 
            *(stepgen->cmd_fbf) = ((int32_t)*p) * (stepgen->scale_recip);
#endif
            p += 1;
            *(stepgen->switch_pos) = (*p) * (stepgen->scale_recip);
            stepgen += 1;   // point to next joint
        }

        // digital input
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
        if (machine_control->prev_in0 != din[0]) {
            // avoid for-loop to save CPU cycles
            machine_control->prev_in0 = din[0];
            for (i = 0; i < 32; i++) {
                *(machine_control->in[i]) = ((machine_control->prev_in0) >> i) & 0x01;
                *(machine_control->in_n[i]) = (~(*(machine_control->in[i]))) & 0x01;
            }
        }
        
        // update gpio_in[63:32]
        // compare if there's any GPIO.DIN bit got toggled
        if (machine_control->prev_in1 != din[1]) {
            // avoid for-loop to save CPU cycles
            machine_control->prev_in1 = din[1];
            for (i = 32; i < 64; i++) {
                *(machine_control->in[i]) = ((machine_control->prev_in1) >> (i-32)) & 0x01;
                *(machine_control->in_n[i]) = (~(*(machine_control->in[i]))) & 0x01;
            }
        }

        // ADC_SPI (raw ADC value)
        p += 1;
        //obsolete: *(gpio->a_in[0]) = (((double)*p)/20.0);
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
        for (i=0; i<num_joints; i++) {
            *stepgen->ferror_flag = ferror_flag & (1 << i);
            stepgen += 1;   // point to next joint
        }

        // DEBUG  : MOVE to MT_DEBUG

#if (MBOX_LOG)
        dsize = sprintf (dmsg, "%10d  ", bp_tick);  // #0
        // fprintf (mbox_fp, "%10d  ", bp_tick);
        stepgen = stepgen_array;
        for (i=0; i<num_joints; i++) {
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
        p = (uint32_t *) (buf_head + 4);    // prev_bp_tick - bp_offset
        p += 1;
        bp_tick = *p;                      
        p += 1;
        if (ERROR_BASE_PERIOD == *p) {
            DP("ERROR_BASE_PERIOD occurs with code(%d) bp_tick(%d) \n", *p, bp_tick);
        }
        break;

    case MT_USB_STATUS:
            // update wou status only if a cmd ongoing
        p = (uint32_t *) (buf_head + 4);
        /* probe status */
        p += 1;
//        if (*machine_control->wou_cmd != USB_CMD_NOOP) {
        *machine_control->wou_status = *p;
//        fprintf(stderr,"usb status(0x%0X)\n", *p);
//        } else if (*p == USB_STATUS_RISC_PROBE_ERROR) {
//            // section report status normally
//            *machine_control->wou_status = *p;
//        } else {
//            *machine_control->wou_status = USB_STATUS_READY;
//        }
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
    case MT_PROBED_POS:
        stepgen = stepgen_array;
        p = (uint32_t *) (buf_head + 4);

        for (i=0; i<num_joints; i++) {
            p += 1;
//            fprintf(stderr,"(%d) probed pos(%d)\n", i, (int32_t)*p);
            *(stepgen->probed_pos) = (double) ((int32_t)*p) * (stepgen->scale_recip);
            stepgen += 1;   // point to next joint

        }
        break;
        break;
    default:
        fprintf(stderr, "ERROR: wou_stepgen.c unknown mail tag (%d)\n", mail_tag);
        *(machine_control->bp_tick) = machine_control->prev_bp;  // restore bp_tick
        //? assert(0);
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
        // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        //         sizeof(uint16_t), buf);
        // wou_flush(&w_param);
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ADDR(addr) | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
    //         sizeof(uint16_t), buf);
    // wou_flush(&w_param);

    return;
}
static void write_usb_cmd(machine_control_t *mc)
{
  /* write parameters */
  int32_t i,j,data, n;
  uint8_t     buf[MAX_DSIZE];
  uint16_t sync_cmd;
  double pos_scale;

    switch(*mc->usb_cmd) {
    case PROBE_CMD_TYPE:
      *mc->last_usb_cmd = *mc->usb_cmd;
      for (i=0; i<4; i++) {
          *mc->last_usb_cmd_param[i] =
              *mc->usb_cmd_param[i];
      }
      for (i=0; i<4; i++) {
          fprintf(stderr,"get probe command (%d)(%d)\n", i, (int32_t)(*mc->usb_cmd_param[i]));
          data = (int32_t)(*mc->usb_cmd_param[i]);
          for(j=0; j<sizeof(int32_t); j++) {
              sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
              memcpy(buf, &sync_cmd, sizeof(uint16_t));
              // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
              // sim:       sizeof(uint16_t), buf);
          }
      }
      /* write command */
        sync_cmd = SYNC_USB_CMD | *mc->usb_cmd; // TODO: set in control.c or do homing.c
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), buf);
        // sim: wou_flush(&w_param);
      break;
    case HOME_CMD_TYPE:
      *mc->last_usb_cmd = *mc->usb_cmd;
      for (i=0; i<4; i++) {
          *mc->last_usb_cmd_param[i] =
              *mc->usb_cmd_param[i];
      }
      n = ((int32_t)(*mc->usb_cmd_param[0]) & 0xFFFFFFF0) >> 4;
      fprintf(stderr,"get home command for (%d) joint\n", n);
      pos_scale   = fabs(atof(pos_scale_str[n]));
      data = (int32_t)(*mc->usb_cmd_param[0]);
      fprintf(stderr,"get home command param0 (0x%0X) joint\n", data);
      for(j=0; j<sizeof(int32_t); j++) {
          sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
          memcpy(buf, &sync_cmd, sizeof(uint16_t));
          // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
          // sim:       sizeof(uint16_t), buf);
      }
      data = (int32_t)(*mc->usb_cmd_param[1] * pos_scale * dt * FIXED_POINT_SCALE);
      fprintf(stderr,"get home command param1 (0x%0d) joint\n", data);
      for(j=0; j<sizeof(int32_t); j++) {
          sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
          memcpy(buf, &sync_cmd, sizeof(uint16_t));
          // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
          // sim:       sizeof(uint16_t), buf);
      }
      data = (int32_t)(*mc->usb_cmd_param[2] * pos_scale * dt * FIXED_POINT_SCALE);
      fprintf(stderr,"get home command param2 (0x%0d) joint\n", data);
      for(j=0; j<sizeof(int32_t); j++) {
          sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
          memcpy(buf, &sync_cmd, sizeof(uint16_t));
          // sim:wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
          // sim:      sizeof(uint16_t), buf);
      }
      /* write command */
        sync_cmd = SYNC_USB_CMD | *mc->usb_cmd; // TODO: set in control.c or do homing.c
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), buf);
        // sim: wou_flush(&w_param);
      break;
    case SPECIAL_CMD_TYPE:
      *mc->last_usb_cmd = *mc->usb_cmd;
      for (i=0; i<4; i++) {
          *mc->last_usb_cmd_param[i] =
              *mc->usb_cmd_param[i];
      }
      for (i=0; i<4; i++) {
//          fprintf(stderr,"SPEC_CMD type command (%d)(%d)\n", i, (int32_t)(*mc->usb_cmd_param[i]));
          data = (int32_t)(*mc->usb_cmd_param[i]);
            for(j=0; j<sizeof(int32_t); j++) {
                sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
                memcpy(buf, &sync_cmd, sizeof(uint16_t));
                // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                //       sizeof(uint16_t), buf);
            }
      }
      /* write command */
        sync_cmd = SYNC_USB_CMD | *mc->usb_cmd; // TODO: set in control.c or do homing.c
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        // sim: wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        // sim:         sizeof(uint16_t), buf);
        // sim: wou_flush(&w_param);
    default:
      // do nothing, don't write command if it is invalid.
      break;
    }
  *mc->usb_cmd = 0;
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
        // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        //         sizeof(uint16_t), buf);
        // wou_flush(&w_param);
    }

    sync_cmd = SYNC_MACH_PARAM | PACK_MACH_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
    //         sizeof(uint16_t), buf);
    // wou_flush(&w_param);


    return;
}

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval, i;

    uint8_t data[MAX_DSIZE];
    int32_t immediate_data;
    uint32_t jog_config_value, max_pulse_tick;
    double max_vel, max_accel, pos_scale, value, max_following_error;//, probe_decel;
    double max_jerk;
    int msg;

    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_ALL);
    // rtapi_set_msg_level(RTAPI_MSG_INFO);
    rtapi_set_msg_level(RTAPI_MSG_DBG);

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

    machine_control = NULL;
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
        // wou_prog_risc(&w_param, bins);
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
        // set mailbox callback function
        // sim: wou_set_mbox_cb (&w_param, fetchmail);
        
        // set crc counter callback function
        // sim: wou_set_crc_error_cb (&w_param, get_crc_error_counter);
        
        // set rt_cmd callback function
        // sim: wou_set_rt_cmd_cb (&w_param, update_rt_cmd);
    }

    if(pulse_type != -1) {
        data[0] = pulse_type;
        // wou_cmd (&w_param, WB_WR_CMD,
        //         (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE),
        //         (uint8_t) 1, data);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no pulse type: pulse_type\n");
        return -1;
    }

    if(enc_type != -1) {
        data[0] = enc_type;
        // wou_cmd (&w_param, WB_WR_CMD,
        //         (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE),
        //         (uint8_t) 1, data);
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
    write_machine_param(ALR_OUTPUT, (uint32_t) strtoul(alr_output, NULL, 16));
    fprintf(stderr, "ALR_OUTPUT(%08X)",(uint32_t) strtoul(alr_output, NULL, 16));
    // config probe parameters
    // probe_decel_cmd
    write_machine_param(PROBE_CONFIG, (uint32_t) strtoul(probe_config, NULL, 16));
    fprintf(stderr, "PROBE_CONFIG(%08X)",(uint32_t) strtoul(probe_config, NULL, 16));
    
    immediate_data = atoi(probe_analog_ref_level);
    write_machine_param(PROBE_ANALOG_REF_LEVEL, immediate_data);
    
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

    num_joints = 0;
    for (n = 0; n < MAX_CHAN && (ctrl_type[n][0] != ' ') /* obsolete: step_type[n] != -1 */; n++) {
	if ((ctrl_type[n][0] == 'p') || (ctrl_type[n][0] == 'P')) {
	    ctrl_type[n] = "p";
	} else if ((ctrl_type[n][0] == 'v') || (ctrl_type[n][0] == 'V')) {
	    ctrl_type[n] = "v";
	} else {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "STEPGEN: ERROR: bad control type '%s' for joint %i (must be 'p' or 'v')\n",
			    ctrl_type[n], n);
	    return -1;
	}
	num_joints++;
    }
    assert (num_joints <=6);  // support up to 6 joints for USB/7i43 
    if (num_joints == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: no channels configured\n");
	return -1;
    }

    /* to clear PULSE/ENC/SWITCH/INDEX positions for all joints*/
    // issue a WOU_WRITE to RESET SSIF position registers
    data[0] = (1 << num_joints) - 1;  // bit-map-for-num_joints
    // wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    // wou_flush(&w_param);
    
    // issue a WOU_WRITE to clear SSIF_RST_POS register
    data[0] = 0x00;
    // wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    // wou_flush(&w_param);

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
    for(n=0; n<num_joints; n++) {
        // const char **pid_str;

        /* compute fraction bits */
        // compute proper fraction bit for command
        // compute fraction bit for velocity
        // accurate 0.0001 mm
        pos_scale   = fabs(atof(pos_scale_str[n]));
        max_vel     = atof(max_vel_str[n]);
        max_accel   = atof(max_accel_str[n]);
        max_jerk    = atof(max_jerk_str[n]);
        
        /* config MAX velocity */
        // +1: rounding to last fixed point unit
        immediate_data = ((uint32_t)(max_vel * pos_scale * dt * FIXED_POINT_SCALE) + 1);    
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_vel(%d) = %f*%f*%f*%f\n", 
                        n, immediate_data, max_vel, pos_scale, dt, FIXED_POINT_SCALE);
        max_pulse_tick = immediate_data;
        assert(immediate_data>0);
        write_mot_param (n, (MAX_VELOCITY), immediate_data);

        /* config acceleration */
        // +1: rounding to last fixed point unit
        immediate_data = ((uint32_t)(max_accel * pos_scale * dt * FIXED_POINT_SCALE * dt) + 1);
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

        /* config max jerk */
        immediate_data = (uint32_t)((max_jerk * pos_scale * dt * dt * dt * FIXED_POINT_SCALE )+1);
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_jerk(%d) = (%f * %f * %f * %f^3)))\n",
                        n, immediate_data, FIXED_POINT_SCALE, max_jerk, pos_scale, dt);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_JERK), immediate_data);
        
        /* config max jerk recip */
        immediate_data = (uint32_t)(FIXED_POINT_SCALE/(max_jerk * pos_scale * dt * dt * dt));
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_jerk_recip(%d) = %f/(%f * %f * %f^3)))\n",
                        n, immediate_data, FIXED_POINT_SCALE, max_jerk, pos_scale, dt);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_JERK_RECIP), immediate_data);

        /* config max following error */
        // following error send with unit pulse
        max_following_error = atof(ferror_str[n]);
        immediate_data = (uint32_t)(ceil(max_following_error * pos_scale));
        rtapi_print_msg(RTAPI_MSG_DBG, "max ferror(%d)\n", immediate_data);
        write_mot_param (n, (MAXFOLLWING_ERR), immediate_data);
        /* config jog setting */
        jog_config_value = strtoul(jog_config_str[n],NULL, 16);
        jog_config_value &= 0x000FFFFF;
        jog_config_value |= ((max_pulse_tick << 4) & 0xFFF00000); // ignore fraction part
        rtapi_print_msg(RTAPI_MSG_DBG,
                  "j[%d] JOG_CONFIG(0x%0X)\n",
                  n, jog_config_value);
        write_mot_param (n, (JOG_CONFIG), jog_config_value);
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
    
    // configure NUM_JOINTS after all joint parameters are set
    write_machine_param(NUM_JOINTS, (uint32_t) num_joints);

    //obsolete: actual_joint_num = atoi(act_jnt_num);

    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo/stepper interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    // TODO: RTL: remove SSIF_EN (always enable SSIF)
    // FIXME: WORKAROUND: always enable SSIF_EN by SW
    // SSIF_EN = 1
    data[0] = 2;
    // wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
    // RISC ON
    data[0] = 1;        
    // wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, data);
    // wou_flush(&w_param);
    /* have good config info, connect to the HAL */
    comp_id = hal_init("wou_sim");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: hal_init() failed\n");
	return -1;
    }

    /* allocate shared memory for counter data */
    stepgen_array = hal_malloc(num_joints * sizeof(stepgen_t));
    if (stepgen_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: hal_malloc() failed\n");
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
    for (n = 0; n < num_joints; n++) {
	/* export all vars */
	retval = export_stepgen(n, &(stepgen_array[n]),
				/* step_type[n],*/ (ctrl_type[n][0] == 'p'));
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "STEPGEN: ERROR: stepgen %d var export failed\n",
			    n);
	    hal_exit(comp_id);
	    return -1;
	}
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
		    num_joints);
    
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
//obsolete: static FILE *f_abort;
static void update_rt_cmd(void)
{
    uint8_t data[MAX_DSIZE];    // data[]: for wou_cmd()
    int32_t immediate_data = 0;
    if (machine_control) {
        if (*machine_control->rt_abort == 1 ||
                *machine_control->cl_abort == 1) {
            immediate_data = RT_ABORT;
            memcpy(data, &immediate_data, sizeof(uint32_t));
            // rt_wou_cmd (&w_param, 
            //             WB_WR_CMD, 
            //             (uint16_t) (JCMD_BASE | OR32_RT_CMD),
            //             sizeof(uint32_t), 
            //             data);
            // rt_wou_flush(&w_param);
//            *machine_control->align_pos_cmd = 2;
        }
    }
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
    static uint32_t host_tick = 0;
    uint32_t jog_var, new_jog_config;
    int32_t immediate_data = 0;
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
    // rtapi_set_msg_level(RTAPI_MSG_ALL);
    rtapi_set_msg_level(RTAPI_MSG_WARN);
    
    //obsolete: rtapi_print_msg(RTAPI_MSG_ERR,
    //obsolete:                 "update_freq:debug: begin\n");
    //obsolete: if (*machine_control->rt_abort == 1) {
    //obsolete:     immediate_data = RT_ABORT;
    //obsolete:     memcpy(data, &immediate_data, sizeof(uint32_t));
    //obsolete:     rt_wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | OR32_RT_CMD),
    //obsolete:             sizeof(uint32_t), data);
    //obsolete:     rt_wou_flush(&w_param);
    //obsolete: }
    //obsolete: 

    // update host tick for risc
    write_machine_param(HOST_TICK, host_tick);
    *machine_control->wou_bp_tick = host_tick;
    if (host_tick == REQUEST_TICK_SYNC_AFTER) {
        sync_cmd = SYNC_BP ;
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        //         sizeof(uint16_t), data);
        // wou_flush(&w_param);
    }
    host_tick += 1;


    // wou_update(&w_param);
    //obsolete: fetchmail();

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
    /* OLD ONE */
//    if ((*machine_control->wou_cmd) != machine_control->prev_wou_cmd) {
//        // call api to parse wou_cmd to risc
//        parse_usb_cmd (*machine_control->wou_cmd);
//        fprintf(stderr, "wou_cmd(%d) prev_wou_cmd(%d)\n", *machine_control->wou_cmd,
//                machine_control->prev_wou_cmd);
//    }
//    machine_control->prev_wou_cmd = *machine_control->wou_cmd;
    /* NEW ONE*/
    if (*machine_control->usb_cmd != 0) {
        write_usb_cmd(machine_control);
        *machine_control->usb_cmd = 0; // reset usb_cmd
    }

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

    // obsolste: /* begin: handle ahc max offset */
    // obsolste: if (*(machine_control->ahc_max_level) != (machine_control->prev_ahc_max_level)) {
    // obsolste:     int32_t max_offset, pos_scale;
    // obsolste:     stepgen = arg;
    // obsolste:     stepgen += atoi(ahc_joint_str);
    // obsolste:     pos_scale = (stepgen->pos_scale);
    // obsolste:     max_offset = *(machine_control->ahc_max_offset);
    // obsolste:     max_offset = max_offset >= 0? max_offset:0;
    // obsolste:     fprintf(stderr,"wou_stepgen.c: ahc_max_offset(%d) ahc_joint(%d) \n",
    // obsolste:                         (uint32_t)abs(max_offset * (pos_scale)),
    // obsolste:                         atoi(ahc_joint_str));

    // obsolste:     /* ahc max_offset */
    // obsolste:     write_machine_param(AHC_MAX_OFFSET, (uint32_t)
    // obsolste:             abs((max_offset)) * (pos_scale));
    // obsolste:     machine_control->prev_ahc_max_level = *(machine_control->ahc_max_level);
    // obsolste: }
    // obsolste: /* begin: process ahc limit control */

    /* begin: process ahc limit control */
    if (*(machine_control->ahc_max_level) != (machine_control->prev_ahc_max_level)) {
        fprintf(stderr,"wou_stepgen.c: ahc_max_level has been changed (%u) \n", *machine_control->ahc_max_level);
        write_machine_param(AHC_LEVEL_MAX, (uint32_t)*(machine_control->ahc_max_level ));
    }
    machine_control->prev_ahc_max_level = *(machine_control->ahc_max_level);
    if (*(machine_control->ahc_min_level) != (machine_control->prev_ahc_min_level)) { 
        fprintf(stderr,"wou_stepgen.c: ahc_min_level has been changed (%u) \n", *machine_control->ahc_min_level);
        write_machine_param(AHC_LEVEL_MIN, (uint32_t)*(machine_control->ahc_min_level ));
    }
    machine_control->prev_ahc_min_level = *(machine_control->ahc_min_level);
    /* end: process ahc limit control */
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
        // wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
        //                               sizeof(uint16_t), (uint8_t *) &sync_cmd);
        // end: trigger sync in and wait timeout
        *(machine_control->sync_in_trigger) = 0;
    }
    /* end: process motion synchronized input */

    /* begin: process motion synchronized output */
    sync_out_data = 0;
    for (i = 0; i < machine_control->num_gpio_out; i++) {
        if(((machine_control->prev_out >> i) & 0x01) !=
                (*(machine_control->out[i]))) {
            {
                // write a wou frame for sync output into command FIFO
//                fprintf(stderr,"wou_stepgen.c: gpio_%02d => (%d)\n",
//                        i,*(machine_control->out[i]));
//                fprintf(stderr,"wou_stepgen.c: num_joints(%d)\n",
//                        num_joints);
                sync_cmd = SYNC_DOUT | PACK_IO_ID(i) | PACK_DO_VAL(*(machine_control->out[i]));
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                // wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);
            }
        }

	sync_out_data |= ((*(machine_control->out[i])) << i);
    }
    machine_control->prev_out = sync_out_data;
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

    // num_joints: calculated from ctrl_type;
    /* loop thru generators */
    r_load_pos = 0;
    r_switch_en = 0;
    /* check if we should update SWITCH/INDEX positions for HOMING */
    if (r_load_pos != 0) {
	// issue a WOU_WRITE
	// wou_cmd(&w_param,
	// 	WB_WR_CMD, SSIF_BASE | SSIF_LOAD_POS, 1, &r_load_pos);
	// fprintf(stderr, "wou: r_load_pos(0x%x)\n", r_load_pos);
	// wou_flush(&w_param);
    }

    /* check if we should enable HOME Switch Detection */
    if (r_switch_en != 0) {
	// issue a WOU_WRITE
	// wou_cmd(&w_param,
	// 	WB_WR_CMD, SSIF_BASE | SSIF_SWITCH_EN, 1, &r_switch_en);
	// // fprintf(stderr, "wou: r_switch_en(0x%x)\n", r_switch_en);
	// wou_flush(&w_param);
    }


    // check wou.stepgen.00.enable signal directly
    stepgen = arg;
    if (*stepgen->enable != stepgen->prev_enable) {
        stepgen->prev_enable = *stepgen->enable;
        fprintf(stderr,"enable changed(%d)\n", *stepgen->enable);
        if (*stepgen->enable) {
            for(i=0; i<num_joints; i++) {
                immediate_data = 1;
                write_mot_param (i, (ENABLE), immediate_data);
//                immediate_data = NORMAL_MOVE;
//                write_mot_param (i, (MOTION_TYPE), immediate_data);
            }

        } else {
            for(i=0; i<num_joints; i++) {
                immediate_data = 0;
                write_mot_param (i, (ENABLE), immediate_data);
            }
        }

    }
    
    // in[0] == 1 (ESTOP released)
    *(machine_control->in[0]) = 1;
    if (*(machine_control->in[0]) == 0) {
        // force updating prev_out and ignore out[] if ESTOP is pressed
        machine_control->prev_out =  *machine_control->dout0;
    }

    i = 0;
    stepgen = arg;
    enable = *stepgen->enable;            // take enable status of first joint
    for (n = 0; n < num_joints; n++) {
        /* begin: handle jog config for RISC */
        if (*stepgen->jog_scale != stepgen->prev_jog_scale) {
            /* config jog setting */
            jog_var = (stepgen->jog_config & 0xFFF00000) >> 20; // fetch jog velocity
            jog_var = (uint32_t)(((double)jog_var) * (*stepgen->jog_scale));
            new_jog_config = (jog_var << 20) | (stepgen->jog_config & 0x000FFFFF);
            new_jog_config = (new_jog_config & 0xFFF0FFFF);
            new_jog_config |= (*stepgen->jog_enable) << 16;
            write_mot_param (n, (JOG_CONFIG), new_jog_config);
            fprintf(stderr, "wou_stepgen.c: j (%d) jog-scale has been changed new jog_config(0x%0X)\n",
                n, new_jog_config);
        }
        stepgen->prev_jog_scale = *stepgen->jog_scale;

        if (stepgen->prev_jog_enable != *stepgen->jog_enable) {
            new_jog_config = (stepgen->jog_config & 0xFFF0FFFF);
            new_jog_config |= (*stepgen->jog_enable) << 16;
            jog_var = (stepgen->jog_config & 0xFFF00000) >> 20; // fetch jog velocity
            jog_var = (uint32_t)(((double)jog_var) * (*stepgen->jog_scale));
            new_jog_config = (jog_var << 20) | (new_jog_config & 0x000FFFFF);
            write_mot_param (n, (JOG_CONFIG), new_jog_config);
            fprintf(stderr, "wou_stepgen.c: j (%d) jog-enable has been changed new jog_config(0x%0X)\n",
                n, new_jog_config);
        }
        stepgen->prev_jog_enable = *stepgen->jog_enable;
        /* end: handle jog config for RISC */


        *stepgen->pos_scale_pin = stepgen->pos_scale; // export pos_scale
        *(stepgen->pos_fb) = (*stepgen->enc_pos) * stepgen->scale_recip;
        // update velocity-feedback only after encoder movement
        if ((*machine_control->bp_tick - machine_control->prev_bp) > 0/* ((int32_t)VEL_UPDATE_BP) */) {
            *(stepgen->vel_fb) = ((*stepgen->pos_fb - stepgen->prev_pos_fb) * recip_dt
                                  / (*machine_control->bp_tick - machine_control->prev_bp)
                                 );
            stepgen->prev_pos_fb = *stepgen->pos_fb;
            stepgen->prev_enc_pos = *stepgen->enc_pos;
            if (n == (num_joints - 1)) {
                // update bp_tick for the last joint
                //DEBUG: rtapi_print_msg(RTAPI_MSG_WARN,
                //DEBUG:                 "WOU: j[%d] bp(%u) prev_bp(%u) vel_fb(%f)\n",
                //DEBUG:                 n,
                //DEBUG:                 machine_control->bp_tick,
                //DEBUG:                 machine_control->prev_bp,
                //DEBUG:                 *(stepgen->vel_fb)
                //DEBUG:                );
                machine_control->prev_bp = *machine_control->bp_tick;
            }
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
	    *stepgen->vel_cmd = 0;
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
                // wou_cmd(&w_param,
                //         WB_WR_CMD, SSIF_BASE | SSIF_LOAD_POS, 1, &r_load_pos);
            }

            assert (i == n); // confirm the JCMD_SYNC_CMD is packed with all joints
            i += 1;
            // wou_flush(&w_param);
            wou_pos_cmd = 0;
            sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);

            memcpy(data + 2*n* sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));
            sync_cmd = 0;

            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd,
                               sizeof(uint16_t));
            if (n == (num_joints - 1)) {
                // send to WOU when all axes commands are generated
                // wou_cmd(&w_param,
                //         WB_WR_CMD,
                //         (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_joints, data);
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
	    if (*machine_control->align_pos_cmd == 1 ||
	       *machine_control->ignore_host_cmd) {
	        (stepgen->prev_pos_cmd) = (*stepgen->pos_cmd);
//	        fprintf(stderr,"ignore_host_cmd(%d) align_pos_cmd(%d)\n",
//	            *machine_control->ignore_host_cmd, *machine_control->align_pos_cmd);

	    }
	    *stepgen->vel_cmd = ((*stepgen->pos_cmd) - (stepgen->prev_pos_cmd)) * recip_dt;

	} else {
	    /* velocity command mode */
	    if (stepgen->prev_ctrl_type_switch != *stepgen->ctrl_type_switch) {
	        stepgen->prev_vel_cmd = 0;
	        stepgen->prev_pos_cmd = *stepgen->pos_cmd;
	        // do more thing if necessary.
	    }
	    if (*stepgen->ctrl_type_switch == 0) {
                *stepgen->vel_cmd = (*stepgen->pos_cmd) ; // notice: has to wire *pos_cmd-pin to velocity command input

                /* begin:  ramp up/down spindle */
                maxvel = stepgen->maxvel;   /* unit/s */
                if (*stepgen->vel_cmd > maxvel) {
                    *stepgen->vel_cmd = maxvel;
                } else if(*stepgen->vel_cmd < -maxvel){
                    *stepgen->vel_cmd = -maxvel;
                }
                stepgen->accel_cmd = (*stepgen->vel_cmd - stepgen->prev_vel_cmd) * recip_dt; /* unit/s^2 */

                if (stepgen->accel_cmd > stepgen->maxaccel) {
                    stepgen->accel_cmd = stepgen->maxaccel;
                    *stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd * dt;
                } else if (stepgen->accel_cmd < -(stepgen->maxaccel)) {
                    stepgen->accel_cmd = -(stepgen->maxaccel);
                    *stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd * dt;
                }
                /* end: ramp up/down spindle */
//                fprintf(stderr,"prev_vel(%f) accel_cmd(%f) \n", stepgen->prev_vel_cmd, stepgen->accel_cmd);
	    } else {
	        *stepgen->vel_cmd = ((*stepgen->pos_cmd) - (stepgen->prev_pos_cmd)) * recip_dt;
	    }
	    stepgen->prev_ctrl_type_switch = *stepgen->ctrl_type_switch;
	}
	{
            
            integer_pos_cmd = (int32_t)((*stepgen->vel_cmd * dt *(stepgen->pos_scale)) * (1 << FRACTION_BITS));

            /* extract integer part of command */
            wou_pos_cmd = abs(integer_pos_cmd) >> FRACTION_BITS;
            
            if(wou_pos_cmd >= 8192) {
                fprintf(stderr,"j(%d) pos_cmd(%f) prev_pos_cmd(%f) vel_cmd(%f)\n",
                        n ,
                        (*stepgen->pos_cmd), 
                        (stepgen->prev_pos_cmd), 
//                        *stepgen->home_state,
                        *stepgen->vel_cmd);
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
            stepgen->prev_vel_cmd = *stepgen->vel_cmd;
	    *(stepgen->pos_fb) = *stepgen->pos_cmd; // for sim:

            if (stepgen->pos_mode == 0) {
                    // TODO: find a better way for spindle control
                    // TODO: let TRAJ-PLANNER judge the index/revolution
                    // TODO: remove this section from wou_stepgen.c
                    double delta;
//                    double spindle_pos;
                    int32_t spindle_pos;
//                    double spindle_irevs;
                    int32_t spindle_irevs, pos_cmd, tmp;
                    double pos_scale;
                    pos_scale = stepgen->pos_scale;
                    pos_cmd = integer_pos_cmd >> FRACTION_BITS;

                    // machine_control->spindle_enc_count += (wou_cmd_accum/(1<<FRACTION_BITS));
//                    machine_control->spindle_enc_count += (integer_pos_cmd >> FRACTION_BITS);

                    /* limit the spindle encoder count in a revolution and
                     * keep tracking the revolution of the spindle cmd */
                    tmp = pos_cmd + machine_control->spindle_enc_count;
                    if (tmp >= (int32_t) abs(pos_scale)) {
                        machine_control->spindle_enc_count = tmp - abs(pos_scale);
                        machine_control->spindle_revs_integer += 1;
                    } else if (tmp <-(int32_t)(abs(pos_scale))) {
                        machine_control->spindle_enc_count = tmp + abs(pos_scale);
                        machine_control->spindle_revs_integer -= 1;
                    } else {
                        machine_control->spindle_enc_count = tmp;
                    }

                    spindle_pos = machine_control->spindle_enc_count;
                    spindle_irevs = (machine_control->spindle_enc_count % ((int32_t)(pos_scale)));

                    delta = ((double)(spindle_irevs - machine_control->prev_spindle_irevs))/pos_scale;

                    /* communicate with motion control */
                    if ((*machine_control->spindle_index_enable == 1) && (*machine_control->spindle_at_speed)) {

                        if (delta < -0.5) {
                            // ex.: 0.9 -> 0.1 (forward)
                           machine_control->last_spindle_index_pos = machine_control->spindle_enc_count;
                           machine_control->last_spindle_index_pos_int = machine_control->spindle_revs_integer;
                            *machine_control->spindle_index_enable = 0;
//                            fprintf(stderr,"revs_int(%d)\n", machine_control->spindle_revs_integer);
                        } else if (delta > 0.5) {
                            // ex.: 0.1 -> 0.9 (backward)
                           machine_control->last_spindle_index_pos = machine_control->spindle_enc_count;
                           machine_control->last_spindle_index_pos_int = machine_control->spindle_revs_integer;
                            *machine_control->spindle_index_enable = 0;

                        }
                    }
                    /* compute the absolute position of the spindle */
                    *machine_control->spindle_revs = (spindle_pos -
                                       machine_control->last_spindle_index_pos)/pos_scale +
                                       machine_control->spindle_revs_integer -
                                       machine_control->last_spindle_index_pos_int;
                   machine_control->prev_spindle_irevs = spindle_irevs;
            }

	}

        if (n == (num_joints - 1)) {
            // send to WOU when all axes commands are generated
            // wou_cmd(&w_param,
            //         WB_WR_CMD,
            //         (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_joints, data);
        }
	DPS("  0x%13X%15.7f%15.7f%3d",
	    integer_pos_cmd, 
            (stepgen->prev_pos_cmd), 
            *stepgen->pos_fb,
            *stepgen->home_state);

	/* move on to next channel */
	stepgen++;
    }
//    *machine_control->align_pos_cmd = 0;

    DPS("  %15.7f", *machine_control->spindle_revs);
    sync_cmd = SYNC_VEL;
    // send velocity status to RISC
    if ( (*machine_control->vel_sync_scale) *
        (*machine_control->feed_scale) *
        (*(machine_control->requested_vel)) <
            *machine_control->current_vel) {
        sync_cmd = SYNC_VEL | 0x0001;
        *machine_control->vel_sync = 1;
    } else {
        sync_cmd = SYNC_VEL;
        *machine_control->vel_sync = 0;
    }
    if (sync_cmd != machine_control->prev_vel_sync) {
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        // wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
        //             sizeof(uint16_t), data);
        // debug: fprintf(stderr, "sent new vel sync cmd (0x%x)\n", sync_cmd);
    }
    machine_control->prev_vel_sync = sync_cmd;
    machine_control->prev_motion_state = *machine_control->motion_state;

#if (TRACE!=0)
    if (*(stepgen - 1)->enable) {
	DPS("\n");
    }
#endif

/* restore saved message level */
    rtapi_set_msg_level(msg);
    /* done */
    //obsolete: rtapi_print_msg(RTAPI_MSG_ERR,
    //obsolete:                 "update_freq:debug: end\n");
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_analog(analog_t * addr)
{
    int i, retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);
    // rtapi_set_msg_level(RTAPI_MSG_ALL);

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


static int export_stepgen(int num, stepgen_t * addr,/* obsolete: int step_type,*/
			  int pos_mode)
{
    int retval, msg;
    uint32_t max_pulse_tick, jog_config_value;
    double pos_scale, max_vel;
    pos_scale   = fabs(atof(pos_scale_str[num]));
    max_vel     = atof(max_vel_str[num]);



    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);
    // rtapi_set_msg_level(RTAPI_MSG_ALL);
    
    /* debug info */
//    Encryptedretval = hal_pin_s32_newf(HAL_OUT, &(addr->joint_cmd), comp_id,
//			      "wou.stepgen.%d.joint_cmd", num);
//    if (retval != 0) {
//	return retval;
//    }
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->cmd_fbs), comp_id,
			      "wou.stepgen.%d.cmd-fbs", num);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_float_newf(HAL_OUT, &(addr->cmd_fbf), comp_id,
			      "wou.stepgen.%d.cmd-fbf", num);
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

    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel_cmd), comp_id,
                                    "wou.stepgen.%d.vel-cmd", num);
    if (retval != 0) {
        return retval;
    }
    /* export pin for pos/vel command */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->probed_pos), comp_id,
                                    "wou.stepgen.%d.probed-pos", num);
    if (retval != 0) {
        return retval;
    }
    /* export parameter to obtain homing state */
//    retval = hal_pin_s32_newf(HAL_IN, &(addr->home_state), comp_id,
//			      "wou.stepgen.%d.home-state", num);
//    if (retval != 0) {
//	return retval;
//    }

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

    retval = hal_pin_float_newf(HAL_IN, &(addr->jog_scale), comp_id,
                                    "wou.stepgen.%d.jog-scale", num);
    if (retval != 0) {
        return retval;
    }


    retval = hal_pin_bit_newf(HAL_IN, &(addr->jog_enable), comp_id,
                                    "wou.stepgen.%d.jog-enable", num);
    if (retval != 0) {
        return retval;
    }

//    retval = hal_pin_bit_newf(HAL_IN, &(addr->tp_enable), comp_id,
//                                    "wou.stepgen.%d.tp-enable", num);
//    if (retval != 0) {
//        return retval;
//    }
//    *addr->tp_enable = 1;
    /* set default parameter values */
    addr->pos_scale = 1.0;
    addr->scale_recip = 0.0;
    addr->freq = 0.0;
    addr->maxvel = 0.0;
    addr->maxaccel = 0.0;
    //obsolete: addr->step_type = step_type;
    addr->pos_mode = pos_mode;
    /* timing parameter defaults depend on step type */
    addr->step_len = 1;
    /* init the step generator core to zero output */
    /* accumulator gets a half step offset, so it will step half
       way between integer positions, not at the integer positions */
    addr->rawcount = 0;
    addr->prev_pos_cmd = 0;
    addr->prev_pos_fb = 0;

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
    *(addr->vel_cmd) = 0.0;
    (addr->prev_vel_cmd) = 0.0;
    (addr->accel_cmd) = 0.0;
    *(addr->ctrl_type_switch) = 0;
    *(addr->jog_scale) = 0.6;
    addr->prev_jog_scale = 1.0;
    /* config jog setting */
    max_pulse_tick = ((uint32_t)(max_vel * pos_scale * dt * FIXED_POINT_SCALE) + 1);
    jog_config_value = strtoul(jog_config_str[num],NULL, 16);
    jog_config_value &= 0x000FFFFF;
    jog_config_value |= ((max_pulse_tick << 4) & 0xFFF00000); // ignore fraction part
    addr->jog_config = jog_config_value;

    *addr->jog_enable = ((addr->jog_config) & 0x000F0000) >> 16;
    addr->prev_jog_enable = *addr->jog_enable;
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}

static int export_machine_control(machine_control_t * machine_control)
{
    int i, retval, msg;

    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);


    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->vel_sync), comp_id,
                              "wou.motion.vel-sync");
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_float_newf(HAL_IN, &(machine_control->vel_sync_scale), comp_id,
                             "wou.motion.vel-sync-scale");
    if (retval != 0) { return retval; }
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->current_vel), comp_id,
                             "wou.motion.current-vel");
    if (retval != 0) { return retval; }
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->feed_scale), comp_id,
                             "wou.motion.feed-scale");
    if (retval != 0) { return retval; }
    *(machine_control->feed_scale) = 0;    // pin index must not beyond index
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->requested_vel), comp_id,
                             "wou.motion.requested-vel");
    if (retval != 0) { return retval; }
    *(machine_control->requested_vel) = 0;    // pin index must not beyond index
    // rtapi_set_msg_level(RTAPI_MSG_ALL);
    machine_control->num_gpio_in = num_gpio_in;
    machine_control->num_gpio_out = num_gpio_out;
    
    // rt_abort: realtime abort command to FPGA
    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->rt_abort), comp_id,
                              "wou.rt.abort");
    if (retval != 0) { return retval; }
    *(machine_control->rt_abort) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->cl_abort), comp_id,
                              "wou.cl.abort");
    if (retval != 0) { return retval; }
    *(machine_control->cl_abort) = 0;

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->align_pos_cmd), comp_id,
                                    "wou.align-pos-cmd");
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->ignore_host_cmd), comp_id,
                                    "wou.ignore-host-cmd");
    if (retval != 0) {
        return retval;
    }
    *machine_control->ignore_host_cmd = 0;
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

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->sync_in_trigger), comp_id,
                              "wou.sync.in.trigger");
    if (retval != 0) { return retval; }
    *(machine_control->sync_in_trigger) = 0;	// pin index must not beyond index

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

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->ahc_max_level), comp_id,
                                "wou.ahc.max_level");
    if (retval != 0) {
            return retval;
    }
    *(machine_control->ahc_max_level) = atoi(ahc_level_max_str);
    machine_control->prev_ahc_max_level = *(machine_control)->ahc_max_level;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->ahc_min_level), comp_id,
                                "wou.ahc.min_level");
    if (retval != 0) {
            return retval;
    }
    
    *(machine_control->ahc_min_level) = atoi(ahc_level_min_str);
    machine_control->prev_ahc_min_level = *(machine_control)->ahc_min_level;

    /* wou command */
    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->wou_cmd), comp_id,
                             "wou.motion.cmd");
    *(machine_control->wou_cmd) = 0;    // pin index must not beyond index
    machine_control->prev_wou_cmd = 0;
    if (retval != 0) {
        return retval;
    }

    /* usb command */
    retval = hal_pin_u32_newf(HAL_IO, &(machine_control->usb_cmd), comp_id,
                             "wou.usb.cmd");
    *(machine_control->usb_cmd) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->last_usb_cmd), comp_id,
                             "wou.usb.last-cmd");
    *(machine_control->last_usb_cmd) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    for (i = 0; i < 4; i++) {
        retval =
            hal_pin_float_newf(HAL_IN, &(machine_control->usb_cmd_param[i]), comp_id,
                             "wou.usb.param-%02d", i);
        if (retval != 0) {
            return retval;
        }
        *(machine_control->usb_cmd_param[i]) = 0;

        retval =
            hal_pin_float_newf(HAL_OUT, &(machine_control->last_usb_cmd_param[i]), comp_id,
                             "wou.usb.last-param-%02d", i);
        if (retval != 0) {
            return retval;
        }
        *(machine_control->last_usb_cmd_param[i]) = 0;
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
//    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->prog_is_running), comp_id,
//                                    "wou.prog-is-running");
//    if (retval != 0) {
//        return retval;
//    }
//    *(machine_control->prog_is_running) = 1;

    machine_control->prev_out = 0;

    machine_control->last_spindle_index_pos = 0;
    machine_control->prev_spindle_irevs = 0;
    machine_control->spindle_revs_integer  = 0;

    /* restore saved message level*/
    rtapi_set_msg_level(msg);
    return 0;
}


// vim:sw=4:sts=4:et:
