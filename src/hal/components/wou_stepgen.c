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
#define MAX_CHAN 8
#define MAX_STEP_CUR 255
#define PLASMA_ON_BIT 0x02
#define FRACTION_BITS 15
#define FRACTION_MASK 0x0000FFFF  
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
#define MBOX_DEBUG_VARS     1      // extra MBOX VARS for debugging
static FILE *mbox_fp;
#endif


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

//obsolete: int gpio_mask_in0 = -1;
//obsolete: RTAPI_MP_INT(gpio_mask_in0, "WOU Register Value for GPIO_MASK_IN0");
//obsolete: 
//obsolete: int gpio_mask_in1 = -1;
//obsolete: RTAPI_MP_INT(gpio_mask_in1, "WOU Register Value for GPIO_MASK_IN1");

int gpio_alm_out0 = -1;
RTAPI_MP_INT(gpio_alm_out0, "WOU Register Value for GPIO_ALM_OUT0");

int gpio_alm_out1 = -1;
RTAPI_MP_INT(gpio_alm_out1, "WOU Register Value for GPIO_ALM_OUT1");

//obsolete: int gpio_leds_sel = -1;
//obsolete: RTAPI_MP_INT(gpio_leds_sel, "WOU Register Value for GPIO_LEDS_SEL");

int step_cur[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
RTAPI_MP_ARRAY_INT(step_cur, MAX_CHAN,
		   "current limit for up to 8 channel of stepping drivers");

int num_gpio_in = 64;
RTAPI_MP_INT(num_gpio_in, "Number of WOU HAL PINs for gpio input");
int num_gpio_out = 64;
RTAPI_MP_INT(num_gpio_out, "Number of WOU HAL PINs for gpio output");

const char *thc_velocity = "1.0"; // 1mm/s
RTAPI_MP_STRING(thc_velocity, "Torch Height Control velocity");

#define NUM_PID_PARAMS  16
const char **pid_str[MAX_CHAN];
const char *j0_pid_str[NUM_PID_PARAMS] =
        { NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RTAPI_MP_ARRAY_STRING(j0_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[0]");

const char *j1_pid_str[NUM_PID_PARAMS] =
        { NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RTAPI_MP_ARRAY_STRING(j1_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[1]");

const char *j2_pid_str[NUM_PID_PARAMS] =
        { NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RTAPI_MP_ARRAY_STRING(j2_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[2]");

const char *j3_pid_str[NUM_PID_PARAMS] =
        { NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RTAPI_MP_ARRAY_STRING(j3_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[3]");

const char *gantry_str[NUM_PID_PARAMS]=
        { NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RTAPI_MP_ARRAY_STRING(gantry_str, NUM_PID_PARAMS,
                      "gantry parameters");


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

const char *home_use_index_str[MAX_CHAN] =
    { "NO", "NO", "NO", "NO", "NO", "NO", "NO", "NO" };
RTAPI_MP_ARRAY_STRING(home_use_index_str, MAX_CHAN,
                      "home use index flag for up to 8 channels");

static int home_use_index[MAX_CHAN] = {0, 0, 0, 0, 0, 0, 0, 0};
static int pulse_fraction_bit[MAX_CHAN] = { 7, 7, 7, 7, 7, 7, 7, 7 };
static int param_fraction_bit[MAX_CHAN] = { 7, 7, 7, 7, 7, 7, 7, 7 };

static const char *board = "7i43u";
static const char wou_id = 0;
static wou_param_t w_param;
static int pending_cnt;
static int normal_move_flag[MAX_CHAN] = {0, 0, 0, 0, 0, 0, 0, 0};


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
    uint32_t    prev_in0;
    uint32_t    prev_in1;

    hal_bit_t *sync_in_trigger;
    hal_u32_t *sync_in;		//
    hal_u32_t *wait_type;
    hal_float_t *timeout;
    int num_gpio_in;
    
    uint32_t    bp_tick;    /* base-period tick obtained from fetchmail */
    uint32_t    prev_bp;    /* previous base-period tick */

    /* sync output pins (output from motmod) */
    hal_bit_t *out[64];
    int num_gpio_out;
    uint64_t prev_out;		//ON or OFF
    hal_bit_t *position_compensation_en_trigger;
    hal_bit_t *position_compensation_en;
    hal_u32_t *position_compensation_ref;
    int8_t position_compensation_en_flag;

    // velocity control
    hal_float_t *requested_vel;
    hal_float_t *current_vel;
    double fp_original_requested_vel;
    double fp_requested_vel;
    double fp_current_vel;
    int32_t vel_sync;

    /* probe */
    hal_bit_t *probe_trigger;
    hal_u32_t *probe_type;
    hal_bit_t *probe_input;
    uint8_t prev_probe_input;
    int8_t probe_state;

    /* spindle */
//obsolete:    hal_bit_t *spindle_enable;
//obsolete:    hal_float_t *spindle_vel_cmd;
//obsolete:    hal_float_t *spindle_vel_fb;
    hal_bit_t *spindle_index_enable;    // TODO: move spindle-index sync into motion
    hal_bit_t *spindle_at_speed;
    hal_float_t *spindle_revs;
    double  last_spindle_index_pos;
    int32_t spindle_enc_count;          // predicted spindle-encoder count
    double  prev_spindle_irevs;         // to calculate index position
    
    /* MPG */
    hal_s32_t *mpg_count;

} machine_control_t;

/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t *stepgen_array;
static gpio_t *gpio;
static analog_t *analog;
static machine_control_t *machine_control;
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
static double recip_dt;		/* recprocal of period, avoids divides */

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
static void fetchmail(const uint8_t *buf_head)
{
    int         i;
    uint16_t    mail_tag;
    uint32_t    *p, din[1];
    // obsolete: double      pos_scale;
    stepgen_t   *stepgen;
    uint32_t    bp_tick;    // served as previous-bp-tick
    uint32_t    ferror_flag;
#if (MBOX_LOG)
    char        dmsg[1024];
    int         dsize;
    static      uint32_t prev_din0;
#endif

    memcpy(&mail_tag, (buf_head + 2), sizeof(uint16_t));
    switch(mail_tag)
    {
    case MT_MOTION_STATUS:

        /* for PLASMA with ADC_SPI */

        // BP_TICK
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
        machine_control->bp_tick = bp_tick;

        stepgen = stepgen_array;
        for (i=0; i<num_chan; i++) {
            // PULSE_POS
            p += 1;
            *(stepgen->pulse_pos) = *p;
            *(stepgen->pid_cmd) = (*(stepgen->pulse_pos))*(stepgen->scale_recip);
            // enc counter
            p += 1;
            *(stepgen->enc_pos) = *p;
            // pid output
            p +=1;
            *(stepgen->pid_output) = ((int32_t)*p)*(stepgen->scale_recip);
            // cmd error
            p += 1;
            *(stepgen->cmd_error) = ((int32_t)*p)*(stepgen->scale_recip);
            stepgen += 1;   // point to next joint
        }

        // digital inpout
        p += 1;
        din[0] = *p;
        //obsolete: din[0] &= gpio_mask_in0;

        // update gpio_in[31:0]
        // compare if there's any GPIO.DIN bit got toggled
        if (memcmp (&(machine_control->prev_in0), &din[0], sizeof(uint32_t))) {
            // avoid for-loop to save CPU cycles
            memcpy(&(machine_control->prev_in0), &din[0], sizeof(uint32_t));
            for (i = 0; i < 32; i++) {
                *(machine_control->in[i]) = ((machine_control->prev_in0) >> i) & 0x01;
            }
        }
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

        // probe state
        p += 1;
        machine_control->probe_state = *p;
        if(machine_control->probe_state == 5) {
            *machine_control->probe_input = 1;
        } else {
            *machine_control->probe_input = 0;
        }
        
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

#if (MBOX_LOG)
        if (din[0] != prev_din0) {
            prev_din0 = din[0];
            fprintf (stderr, "DIN: 0x%08X\n", din[0]);
        }
        dsize = sprintf (dmsg, "%10d  ", bp_tick);
        // fprintf (mbox_fp, "%10d  ", bp_tick);
        stepgen = stepgen_array;
        for (i=0; i<num_chan; i++) {
            dsize += sprintf (dmsg + dsize, "%10d  %10d %10f %10f  ",
                              *(stepgen->pulse_pos),
                              *(stepgen->enc_pos),
                              *(stepgen->pid_output),
                              *(stepgen->cmd_error)
                             );
            stepgen += 1;   // point to next joint
        }
        dsize += sprintf (dmsg + dsize, "0x%04X %10d",
                          din[0],
                          machine_control->probe_state);
        dsize += sprintf (dmsg + dsize, "  %10d 0x%04X", *(analog->in[0]), ferror_flag);
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
#if (MBOX_LOG)
//        if(*p < 100) {
            fprintf(mbox_fp, "# error occure with code(%d) bp_tick(%d)\n",*p, bp_tick);
//        }
//        fprintf(mbox_fp, "# error occure with code(%d)\n",bp_tick);
#endif
        break;
    default:
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
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
        // wou_flush(&w_param);
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ADDR(addr) | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);
    wou_flush(&w_param);

    return;
}



/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval, j, i;

    uint8_t data[MAX_DSIZE];
    int32_t immediate_data;
    char str[50];
    double max_vel, max_accel, pos_scale, thc_vel, value, max_following_error;
    int32_t bitn, vel_bit, accel_bit, accel_recip_bit, param_bit;
    int msg;

    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_ALL);

#if (TRACE!=0)
    // initialize file handle for logging wou steps
    dptrace = fopen("wou_stepgen.log", "w");
    /* prepare header for gnuplot */
    DPS("#%10s  %15s%15s%7s  %15s%15s%7s  %15s%15s%7s  %15s%15s%7s %15s\n",
         "1.dt",
         "2.prev_pos_cmd[0]", "3.pos_fb[0]",
         "4.home[0]",
         "5.prev_pos_cmd[1]", "6.pos_fb[1]",
         "7.home[1]",
         "8.prev_pos_cmd[2]", "9.pos_fb[2]",
         "10.home[2]",
         "11.prev_pos_cmd[3]", "12.pos_fb[3]",
         "13.home[3]",
         "14.spindle_revs");
#endif

    pending_cnt = 0;

    /* test for bitfile string: bits */
    if ((bits == 0) || (bits[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"WOU: ERROR: no fpga bitfile string: bits\n");
	return -1;
    } else {
	// initialize FPGA with bitfile(bits)
	wou_init(&w_param, board, wou_id, bits);
	if (wou_connect(&w_param) == -1) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "WOU: ERROR: Connection failed\n");
	    return -1;
	}
    }

    /* test for risc image file string: bins */
    if ((bins == 0) || (bins[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"WOU: ERROR: no risc binfile string: bins\n");
	return -1;
    } else {
	// programming risc with binfile(bins)
        wou_prog_risc(&w_param, bins);
#if (MBOX_LOG)
        mbox_fp = fopen ("./mbox.log", "w");
#endif
        // set mailbox callback function
        wou_set_mbox_cb (&w_param, fetchmail);
    }

    if(pulse_type != -1) {
        data[0] = pulse_type;
        wou_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE),
                (uint8_t) 1, data);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no pulse type: pulse_type\n");
        return -1;
    }

    if(enc_type != -1) {
        data[0] = enc_type;
        wou_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE),
                (uint8_t) 1, data);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no encoder type: enc_type\n");
        return -1;
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
    wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    wou_flush(&w_param);
    
    data[0] = 0x00;
    wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    wou_flush(&w_param);

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

    if (gpio_alm_out0 != -1) {
       data[0] = (uint8_t) gpio_alm_out0;
       wou_cmd(&w_param, WB_WR_CMD, GPIO_BASE | GPIO_ALM_OUT0, 1, data);
       wou_flush(&w_param);
    }
    
    if (gpio_alm_out1 != -1) {
       data[0] = (uint8_t) gpio_alm_out1;
       wou_cmd(&w_param, WB_WR_CMD, GPIO_BASE | GPIO_ALM_OUT1, 1, data);
       wou_flush(&w_param);
    }
    
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
	wou_cmd(&w_param,
		WB_WR_CMD, (SSIF_BASE | SSIF_MAX_PWM), num_chan, data);
    }

    wou_flush(&w_param);

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
    // pid_str[4] = j4_pid_str;
    // pid_str[5] = j5_pid_str;
    // pid_str[6] = j6_pid_str;
    // pid_str[7] = j7_pid_str;

    /* configure motion parameters for risc*/
    for(n=0; n<num_chan; n++) {
        // const char **pid_str;

        /* compute fraction bits */
        // compute proper fraction bit for command
        // compute fraction bit for velocity
        // accurate 0.0001 mm
        pos_scale = atof(pos_scale_str[n]);


// bitn fix at FRACTION_BITS bits:         value = (pos_scale);
// bitn fix at FRACTION_BITS bits:         value = value > 0? value:-value;
// bitn fix at FRACTION_BITS bits:         value = (1/value)/0.00001;
// bitn fix at FRACTION_BITS bits:         bitn = 0;
// bitn fix at FRACTION_BITS bits:         while(((int32_t)value>>bitn)>0) {
// bitn fix at FRACTION_BITS bits:             bitn++;
// bitn fix at FRACTION_BITS bits:         }
        bitn = FRACTION_BITS;
        pulse_fraction_bit[n] = bitn;  // cmd fraction bit never modified

// param fraction bit fix at FRACTION_BITS bits:         // fraction bit for vel
        max_vel = atof(max_vel_str[n]);
// param fraction bit fix at FRACTION_BITS bits:         value = (1.0/(max_vel*pos_scale*dt));
// param fraction bit fix at FRACTION_BITS bits:         value = value > 0? value:-value;
// param fraction bit fix at FRACTION_BITS bits:         vel_bit = 0;
// param fraction bit fix at FRACTION_BITS bits:         while(((int32_t)value>>vel_bit)>0) {
// param fraction bit fix at FRACTION_BITS bits:             vel_bit++;
// param fraction bit fix at FRACTION_BITS bits:         }
// param fraction bit fix at FRACTION_BITS bits:         // fraction bit for accel
        max_accel = atof(max_accel_str[n]);
// param fraction bit fix at FRACTION_BITS bits:         value = (1.0/(max_accel*pos_scale*dt*dt)); // accurate 1
// param fraction bit fix at FRACTION_BITS bits:         value = value > 0? value:-value;
// param fraction bit fix at FRACTION_BITS bits:         accel_bit = 0;
// param fraction bit fix at FRACTION_BITS bits:         while(((int32_t)value>>accel_bit)>0) {
// param fraction bit fix at FRACTION_BITS bits:             accel_bit++;
// param fraction bit fix at FRACTION_BITS bits:         }
// param fraction bit fix at FRACTION_BITS bits:         // fraction bit for accel_recip
// param fraction bit fix at FRACTION_BITS bits:         value = ((max_accel*pos_scale*dt*dt))/1; // accurate 1
// param fraction bit fix at FRACTION_BITS bits:         value = value > 0? value:-value;
// param fraction bit fix at FRACTION_BITS bits:         accel_recip_bit = 0;
// param fraction bit fix at FRACTION_BITS bits:         while(((int32_t)value>>accel_recip_bit)>0) {
// param fraction bit fix at FRACTION_BITS bits:             accel_recip_bit++;
// param fraction bit fix at FRACTION_BITS bits:         }
// param fraction bit fix at FRACTION_BITS bits:         param_bit = 0;
// param fraction bit fix at FRACTION_BITS bits:         param_bit = param_bit > bitn? param_bit:bitn;//max(param_bit, bitn);
// param fraction bit fix at FRACTION_BITS bits:         param_bit = param_bit > vel_bit? param_bit:vel_bit;//max(param_bit, vel_bit);
// param fraction bit fix at FRACTION_BITS bits:         param_bit = param_bit > accel_bit? param_bit:accel_bit;//max(param_bit, accel_bit);
// param fraction bit fix at FRACTION_BITS bits:         param_bit = param_bit > accel_recip_bit? param_bit:accel_recip_bit;//max(param_bit, accel_recip_bit);
        param_bit = FRACTION_BITS; 

        param_fraction_bit[n] = param_bit;
        vel_bit = param_bit;
        accel_bit = param_bit;
        accel_recip_bit = param_bit;
        rtapi_print_msg(RTAPI_MSG_DBG,"cmd_fract(%d) param_fract(%d)", bitn, param_bit);

        /* config fraction bit of pulse command */
        bitn = param_bit;
        immediate_data = bitn;
        write_mot_param (n, (CMD_FRACT_BIT), immediate_data);

        /* config fraction bit of param */
        immediate_data = param_bit;
        write_mot_param (n, (PARAM_FRACT_BIT), immediate_data);
        /* config velocity */
        immediate_data = (uint32_t)(ceil(max_vel*pos_scale*dt)*(1 << vel_bit));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;
        immediate_data += 1;
        rtapi_print_msg(RTAPI_MSG_DBG,
                " max_vel= %f*%f*%f*(2^%d) = (%d) ", max_vel, pos_scale, dt, vel_bit, immediate_data);
        assert(immediate_data>0);
        write_mot_param (n, (MAX_VELOCITY), immediate_data);

        /* config acceleration */
        immediate_data = (uint32_t)(ceil((max_accel*pos_scale*dt*
                                        dt)*(1 << accel_bit)));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;
        immediate_data += 1;
        rtapi_print_msg(RTAPI_MSG_DBG,"max_accel=%f*%f*(%f^2)*(2^%d) = (%d) ",
                 max_accel, pos_scale, dt, accel_bit, immediate_data);

        assert(immediate_data>0);
        write_mot_param (n, (MAX_ACCEL), immediate_data);

        /* config acceleration recip */
        immediate_data = (uint32_t)(ceil((1/(max_accel*pos_scale*dt*
                                        dt))*(1 << accel_recip_bit)));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;
        rtapi_print_msg(RTAPI_MSG_DBG, "(1/(max_accel*scale)=(1/(%f*%f*(%f^2)))*(2^%d) = (%d) ",
                max_accel, pos_scale, dt, accel_recip_bit, immediate_data);
        assert(immediate_data>0);

        write_mot_param (n, (MAX_ACCEL_RECIP), immediate_data);

        /* config max following error */
        // following error send with unit pulse
        max_following_error = atof(ferror_str[n]);
        immediate_data = (uint32_t)(ceil(max_following_error * pos_scale));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;


        rtapi_print_msg(RTAPI_MSG_DBG, "(max ferror) = (%d) (%d) ",
                       ((uint32_t) (immediate_data/pos_scale)),(immediate_data));
        write_mot_param (n, (MAXFOLLWING_ERR), immediate_data);

        /* config move type */
        for(j = 0; home_use_index_str[j]; j++) {
            str[j] = toupper(home_use_index_str[n][j]);
        }
        str[j] = '\0';

        if((strcmp(str,"YES") == 0)) {
            home_use_index[n] = 1;
            rtapi_print_msg(RTAPI_MSG_DBG, "use_index = yes\n");
        } else {
            rtapi_print_msg(RTAPI_MSG_DBG, "use_index = no\n");
            home_use_index[n] = 0;
        }

        // set move type as normal by default
        immediate_data = NORMAL_MOVE;
        write_mot_param (n, (MOTION_TYPE), immediate_data);

        // test valid PID parameter for joint[n]
        if (pid_str[n][0] != NULL) {
            rtapi_print_msg(RTAPI_MSG_INFO, "J%d_PID: ", n);
            rtapi_print_msg(RTAPI_MSG_INFO,"#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO 14:PE 15:PB\n");
            for (i=0; i < (FF2-P_GAIN+1); i++) {
                // all gain varies from 0(0%) to 65535(100%)
                value = atof(pid_str[n][i]);
                immediate_data = (int32_t) (value);
                write_mot_param (n, (P_GAIN + i), immediate_data);
                rtapi_print_msg(RTAPI_MSG_INFO, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }
            for (; i < (PROBE_BACK_OFF-P_GAIN); i++) {
                // parameter use parameter fraction, parameter unit: pulse
                value = atof(pid_str[n][i]);
                immediate_data = (int32_t) (value) * (1 << param_fraction_bit[n]);
                write_mot_param (n, (P_GAIN + i), immediate_data);
                rtapi_print_msg(RTAPI_MSG_INFO, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }

            value = 0;
            immediate_data = (int32_t) (value);
            write_mot_param (n, (ENABLE), immediate_data);
            rtapi_print_msg(RTAPI_MSG_INFO, "\n");
        }

     }
    // for gantry parameter
//    if(gantry_str[0] != NULL) {
//        rtapi_print_msg(RTAPI_MSG_INFO, "GANTRY: ");
//        rtapi_print_msg(RTAPI_MSG_INFO,"#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO\n");
//        for (i=0; i < (FF2-P_GAIN+1); i++) {
//            // all gain varies from 0(0%) to 65535(100%)
//            value = atof(gantry_str[i]);
//            immediate_data = (int32_t) (value);
//            write_mot_param (4, (P_GAIN + i), immediate_data);
//            rtapi_print_msg(RTAPI_MSG_INFO, "gantry(%d) = %s (%d)\n", i, gantry_str[i], immediate_data);
//        }
//        for (; i < (PROBE_BACK_OFF-P_GAIN+1); i++) {
//                        // parameter use parameter fraction, parameter unit: pulse
//            value = atof(gantry_str[i]);
//            immediate_data = (int32_t) (value) * (1 << param_fraction_bit[1]);
//            write_mot_param (4, (P_GAIN + i), immediate_data);
//            rtapi_print_msg(RTAPI_MSG_INFO, "gantgry(%d) = %s (%d)\n",i , gantry_str[i], immediate_data);
//        }
//
//        value = 0;
//        immediate_data = (int32_t) (value);
//        write_mot_param (4, (ENABLE), immediate_data);
//        rtapi_print_msg(RTAPI_MSG_INFO, "\n");
//    }

    /* to send position compensation velocity  of Z*/
    thc_vel = atof(thc_velocity);
    pos_scale = atof(pos_scale_str[2]);
    immediate_data = (uint32_t)(thc_vel*pos_scale*dt*(1 << param_fraction_bit[2]));
    immediate_data = immediate_data > 0? immediate_data:-immediate_data;
    write_mot_param (2, (COMP_VEL), immediate_data);

    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo/stepper interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    // TODO: RTL: remove SSIF_EN (always enable SSIF)
    // FIXME: WORKAROUND: always enable SSIF_EN by SW
    // SSIF_EN = 1
    data[0] = 2;
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
    // RISC ON
    data[0] = 1;        
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, data);
    wou_flush(&w_param);
    /* have good config info, connect to the HAL */
    comp_id = hal_init("wou");
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
    double physical_maxvel;	// max vel supported by current step timings & position-scale
    double maxvel;		// actual max vel to use this time

    uint16_t sync_cmd;
    int wou_pos_cmd, integer_pos_cmd;
    int j;
    // ret, data[]: for wou_cmd()
    uint8_t data[MAX_DSIZE];
    uint64_t sync_io_data;
    // int     ret;

    // for homing:
    uint8_t r_load_pos;
    uint8_t r_switch_en;
    uint8_t r_index_en;
    uint8_t r_index_lock;
    static uint8_t prev_r_index_en = 0;
    static uint8_t prev_r_index_lock = 0;
    int32_t immediate_data = 0;
    double fp_req_vel, fp_cur_vel, fp_diff;
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

//    DP("before wou_update()\n");
    wou_update(&w_param);

    // read SSIF_INDEX_LOCK
    memcpy(&r_index_lock,
	   wou_reg_ptr(&w_param, SSIF_BASE + SSIF_INDEX_LOCK), 1);
    if (r_index_lock != prev_r_index_lock) {
	rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: index_lock(0x%02X) prev_index_lock(0x%02X)\n",
	                                r_index_lock, prev_r_index_lock);
	prev_r_index_lock = r_index_lock;
    }

    /* begin: process position compensation enable */
    if((*(machine_control->position_compensation_en_trigger) != 0)) {

        if(*(machine_control->thc_enbable)) {
            immediate_data = (uint32_t)(*(machine_control->position_compensation_ref));
            fprintf(stderr,"position compensation triggered(%d) ref(%d)\n",
                            *(machine_control->position_compensation_en),immediate_data);

            for(j=0; j<sizeof(uint32_t); j++) {
                sync_cmd = SYNC_DATA | ((uint8_t *)&immediate_data)[j];
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                        sizeof(uint16_t), data);
            }
            sync_cmd = SYNC_PC |  SYNC_COMP_EN(*(machine_control->position_compensation_en));
            memcpy(data, &sync_cmd, sizeof(uint16_t));
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                    sizeof(uint16_t), data);

            machine_control->position_compensation_en_flag = *machine_control->position_compensation_en;
            machine_control->fp_original_requested_vel = (*machine_control->requested_vel);
            fprintf(stderr,"original_requested_vel(%f)\n", machine_control->fp_original_requested_vel);
        }
        *(machine_control->position_compensation_en_trigger) = 0;
    }
    /* end: process position compensation enable */

    /* begin: process motion synchronized input */
    if (*(machine_control->sync_in_trigger) != 0) {
        assert(*(machine_control->sync_in) >= 0);
        assert(*(machine_control->sync_in) < num_gpio_in);

       // begin: setup sync timeout
        immediate_data = (uint32_t)(*(machine_control->timeout)/(servo_period_ns * 0.000000001)); // ?? sec timeout / one tick interval
        //immediate_data = 1000; // ticks about 1000 * 0.00065536 sec
        // transmit immediate data
        fprintf(stderr,"set risc timeout(%u) type (%d)\n",immediate_data, *(machine_control->wait_type));
        for(j=0; j<sizeof(uint32_t); j++) {
            sync_cmd = SYNC_DATA | ((uint8_t *)&immediate_data)[j];
            memcpy(data, &sync_cmd, sizeof(uint16_t));
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                    sizeof(uint16_t), data);
        }
        sync_cmd = SYNC_ST;
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), data);
        // end: setup sync timeout
        // begin: trigger sync in and wait timeout 
        sync_cmd = SYNC_DIN | PACK_IO_ID(*(machine_control->sync_in)) |
                                           PACK_DI_TYPE(*(machine_control->wait_type));
        wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
                                      sizeof(uint16_t), (uint8_t *) &sync_cmd);
        // end: trigger sync in and wait timeout
        *(machine_control->sync_in_trigger) = 0;
    }
    /* end: process motion synchronized input */

    /* begin: process motion synchronized output */
    sync_io_data = 0;
    j = 0;
    for (i = 0; i < machine_control->num_gpio_out; i++) {
        if(((machine_control->prev_out >> i) & 0x01) !=
                ((*(machine_control->out[i]) & 1))) {
            //TODO: replace plasma-on with general purpose enable bit
            if(i==1 /* plasma on bit */ && *(machine_control->plasma_enable)) {
//                fprintf(stderr,"plasma_switch(%d)\n",
//                        *(machine_control->sync_out[i]));
                fprintf(stderr,"gpio_%2d => (%d)\n",
                                        i,*(machine_control->out[i]));
                sync_cmd = SYNC_DOUT | PACK_IO_ID(i) | PACK_DO_VAL(*(machine_control->out[i]));
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);
            } else {
                fprintf(stderr,"gpio_%02d => (%d)\n",
                        i,*(machine_control->out[i]));
                sync_cmd = SYNC_DOUT | PACK_IO_ID(i) | PACK_DO_VAL(*(machine_control->out[i]));
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);
            }
            j ++;
        }

	sync_io_data |= ((*(machine_control->out[i]) & 1) << i);
       // write a wou frame for sync input into command FIFO
    }

    if (j > 0) {        //
        machine_control->prev_out = sync_io_data;
    }
    /* end: process motion synchronized output */

    /* begin: probe */
    if(*machine_control->probe_trigger != 0 ) {

        *machine_control->probe_trigger = 0;
        sync_cmd = SYNC_PROBE | ((uint16_t)*machine_control->probe_type);
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), data);
        wou_flush(&w_param);
        fprintf(stderr,"probe_trigger(%d) \n",*machine_control->probe_type);

    }
    if((machine_control->prev_probe_input != *machine_control->probe_input) &&
            machine_control->prev_probe_input) {
        /* disable probe motion in risc */
        sync_cmd = SYNC_PROBE;
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), data);
        wou_flush(&w_param);

    }
    machine_control->prev_probe_input = *machine_control->probe_input;
    /* end: probe */

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

    // DP("before checking home_state...\n");

    r_load_pos = 0;
    r_switch_en = 0;
    r_index_en = prev_r_index_en;
    /* begin: homing logic */
    for (n = 0; n < num_chan; n++) {
	if ((*stepgen->home_state != HOME_IDLE) && stepgen->pos_mode) {
	    static hal_s32_t prev_switch_pos;
	    hal_s32_t switch_pos_tmp;
	    hal_s32_t index_pos_tmp;
	    /* update home switch and motor index position while homing */
	    //TODO: to get switch pos and index pos from risc
	    memcpy((void *) &switch_pos_tmp,
		   wou_reg_ptr(&w_param,
			       SSIF_BASE + SSIF_SWITCH_POS + n * 4), 4);
	    memcpy((void *) &index_pos_tmp,
		   wou_reg_ptr(&w_param,
			       SSIF_BASE + SSIF_INDEX_POS + n * 4), 4);

	    *(stepgen->switch_pos) = switch_pos_tmp * stepgen->scale_recip;
	    *(stepgen->index_pos) = index_pos_tmp * stepgen->scale_recip;
	    if(prev_switch_pos != switch_pos_tmp) {
//                fprintf(stderr, "wou: switch_pos(0x%04X)\n",switch_pos_tmp);
                prev_switch_pos = switch_pos_tmp;
	    }

	    /* check if we should wait for HOME Switch Toggle */
	    if ((*stepgen->home_state == HOME_INITIAL_BACKOFF_WAIT) ||
		(*stepgen->home_state == HOME_INITIAL_SEARCH_WAIT) ||
		(*stepgen->home_state == HOME_FINAL_BACKOFF_WAIT) ||
		(*stepgen->home_state == HOME_RISE_SEARCH_WAIT) ||
		(*stepgen->home_state == HOME_FALL_SEARCH_WAIT) ||
		(*stepgen->home_state == HOME_INDEX_SEARCH_WAIT)) {
		if (stepgen->prev_home_state != *stepgen->home_state) {
		    // set r_switch_en to locate SWITCH_POS
		    // r_switch_en is reset by HW
		    r_switch_en |= (1 << n);

		    if((*stepgen->home_state == HOME_INITIAL_SEARCH_WAIT)) {
                        immediate_data = SEARCH_HOME_HIGH;
                    } else if((*stepgen->home_state == HOME_FINAL_BACKOFF_WAIT)) {
                        immediate_data = SEARCH_HOME_LOW;
                    } else if((*stepgen->home_state == HOME_INITIAL_BACKOFF_WAIT)) {
                        immediate_data = SEARCH_HOME_LOW;
                    } else if((*stepgen->home_state == HOME_RISE_SEARCH_WAIT)) {
                        immediate_data = SEARCH_HOME_HIGH;
                    } else if ((*stepgen->home_state == HOME_FALL_SEARCH_WAIT)) {
                        immediate_data = SEARCH_HOME_LOW;
                    } else if(*stepgen->home_state == HOME_INDEX_SEARCH_WAIT){
                        immediate_data = SEARCH_INDEX;

                    }
                    write_mot_param (n, (MOTION_TYPE), immediate_data);

		}

	    } else if ((*stepgen->home_state == HOME_SET_SWITCH_POSITION) ||
	                (*stepgen->home_state == HOME_SET_INDEX_POSITION)) {
                if(normal_move_flag[n] == 1) {
                    immediate_data = NORMAL_MOVE;
                    write_mot_param (n, (MOTION_TYPE), immediate_data);
                    normal_move_flag[n] = 0;
                }

                (stepgen->prev_pos_cmd) = *(stepgen->pos_fb);
                (*stepgen->pos_cmd) = *(stepgen->pos_fb);
            } else if(*stepgen->home_state == HOME_START) {
                normal_move_flag[n] = 1;
            }

	    /* check if we should wait for Motor Index Toggle */
	    if (*stepgen->home_state == HOME_INDEX_SEARCH_WAIT) {
		if (stepgen->prev_home_state != *stepgen->home_state) {
		    // set r_index_en while getting into HOME_INDEX_SEARCH_WAIT state
		    r_index_en |= (1 << n);
		} else if (r_index_lock & (1 << n)) {
		    // the motor index is locked
		    // reset r_index_en by SW
		    r_index_en &= (~(1 << n));	// reset index_en[n]
		    *(stepgen->index_enable) = 0;
//		    rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: J[%d] index_en(0x%02X) prev_r_index_en(0x%02X)\n",
//		                                    n, r_index_en, prev_r_index_en);
//		    rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: J[%d] index_pos(%f) INDEX_POS(0x%08X)\n",
//		                                    n, *(stepgen->index_pos), index_pos_tmp);
//		    rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: J[%d] switch_pos(%f) SWITCH_POS(0x%08X)\n",
//		                                    n, *(stepgen->switch_pos), switch_pos_tmp);
		}
//		stepgen->prev_pos_cmd = *stepgen->pos_cmd;
	    }

	    if (stepgen->prev_home_state == HOME_IDLE) {
                /**
                 * r_load_pos: set to ONE to load PULSE_POS, SWITCH_POS, and
                 * INDEX_POS with enc_counter at beginning of homing
                 * (HOME_START state)
                 *
		 * reset to ZERO one cycle after setting this register
                 **/
		r_load_pos |= (1 << n);
                if (*(stepgen->enc_pos) != *(stepgen->pulse_pos)) {
                    /* accumulator gets a half step offset, so it will step half
                       way between integer positions, not at the integer positions */
                    stepgen->rawcount = *(stepgen->enc_pos);
//                    (stepgen->prev_pos_cmd) = (double) (stepgen->rawcount) * stepgen->scale_recip;
                }
                fprintf(stderr, "j[%d] enc_counter(%d) pulse_pos(%d)\n",
                        n/*, stepgen->accum*/, *(stepgen->enc_pos), *(stepgen->pulse_pos));
	    }
	}
        stepgen->prev_home_state = *stepgen->home_state;
	/* move on to next channel */
	stepgen++;
    }
    /* enc: homing logic */
    /* check if we should update SWITCH/INDEX positions for HOMING */
    if (r_load_pos != 0) {
	// issue a WOU_WRITE
	wou_cmd(&w_param,
		WB_WR_CMD, SSIF_BASE | SSIF_LOAD_POS, 1, &r_load_pos);
	fprintf(stderr, "wou: r_load_pos(0x%x)\n", r_load_pos);
	wou_flush(&w_param);
    }

    /* check if we should enable HOME Switch Detection */
    if (r_switch_en != 0) {
	// issue a WOU_WRITE
	wou_cmd(&w_param,
		WB_WR_CMD, SSIF_BASE | SSIF_SWITCH_EN, 1, &r_switch_en);
	// fprintf(stderr, "wou: r_switch_en(0x%x)\n", r_switch_en);
	wou_flush(&w_param);
    }

    /* check if we should enable MOTOR Index Detection */
    if (r_index_en != prev_r_index_en) {
	// issue a WOU_WRITE
	wou_cmd(&w_param,
		WB_WR_CMD, SSIF_BASE | SSIF_INDEX_EN, 1, &r_index_en);
	fprintf(stderr, "wou: r_index_en(0x%x)\n", r_index_en);
	wou_flush(&w_param);
	prev_r_index_en = r_index_en;
    }

    pending_cnt += 1;
    if (pending_cnt == JNT_PER_WOF) {
        pending_cnt = 0;

        // send WB_RD_CMD to read registers back
        /* TODO: replace switch pos and index pos with mailbox siwtch pos and index pos.
         *       Also clean index lock in risc.
        */
        wou_cmd (&w_param,
                WB_RD_CMD,
                (SSIF_BASE | SSIF_INDEX_LOCK),
                1,
                data);

        wou_cmd (&w_param,
                WB_RD_CMD,
                (SSIF_BASE | SSIF_SWITCH_POS),
                16,
                data);

        wou_cmd (&w_param,
                WB_RD_CMD,
                (SSIF_BASE | SSIF_INDEX_POS),
                16,
                data);

        wou_flush(&w_param);

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
                write_mot_param (n, (MOTION_TYPE), immediate_data);
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

	*(stepgen->pos_fb) = (*stepgen->enc_pos) * stepgen->scale_recip;
        if (*stepgen->enc_pos != stepgen->prev_enc_pos) {
            // update velocity-feedback only after encoder movement
            *(stepgen->vel_fb) = ((*stepgen->pos_fb - stepgen->prev_pos_fb) * recip_dt
                                  / (machine_control->bp_tick - machine_control->prev_bp) 
                                 );
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
                machine_control->prev_bp = machine_control->bp_tick;
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
                wou_cmd(&w_param,
                        WB_WR_CMD, SSIF_BASE | SSIF_LOAD_POS, 1, &r_load_pos);
//                fprintf(stderr, "j[%d] enc_counter(%d) pulse_pos(%d)\n",
//                        n/*, stepgen->accum*/, *(stepgen->enc_pos), *(stepgen->pulse_pos));
//                fprintf(stderr, "j[%d] prev_pos_cmd(%f) pos_cmd(%f) rawcount(%lld)\n",
//                        n, (stepgen->prev_pos_cmd), *(stepgen->pos_cmd), stepgen->rawcount);
            }

            assert (i == n); // confirm the JCMD_SYNC_CMD is packed with all joints
            i += 1;
            wou_flush(&w_param);
            wou_pos_cmd = 0;
            sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);

            memcpy(data + 2*n* sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));
            sync_cmd = 0;

            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd,
                               sizeof(uint16_t));
            if (n == (num_chan - 1)) {
                // send to WOU when all axes commands are generated
                wou_cmd(&w_param,
                        WB_WR_CMD,
                        (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_chan, data);
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
	    /* begin: velocity and acceleration check */
            
//TODO:MBOX:            if (n == (num_chan - 1)) {
//TODO:MBOX:                rtapi_print_msg(RTAPI_MSG_WARN,
//TODO:MBOX:                                "WOU:A: j[%d] pos_mode(%d) pos_cmd(%f) maxvel(%f) maxaccel(%f) vel_cmd(%f) accel_cmd(%f)\n",
//TODO:MBOX:                                n,
//TODO:MBOX:                                stepgen->pos_mode,
//TODO:MBOX:                                *stepgen->pos_cmd,
//TODO:MBOX:                                stepgen->maxvel,
//TODO:MBOX:                                stepgen->maxaccel,
//TODO:MBOX:                                stepgen->vel_cmd,
//TODO:MBOX:                                stepgen->accel_cmd
//TODO:MBOX:                               );
//TODO:MBOX:            }

	    maxvel = stepgen->maxvel;   /* unit/s */
	    if (stepgen->vel_cmd > maxvel) {
	        stepgen->vel_cmd = maxvel;
	    } else if(stepgen->vel_cmd < -maxvel){
	        stepgen->vel_cmd = -maxvel;
	    }
	    stepgen->accel_cmd = (stepgen->vel_cmd - stepgen->prev_vel_cmd) * recip_dt; /* unit/s^2 */
//TODO:MBOX:            if (n == (num_chan - 1)) {
//TODO:MBOX:                rtapi_print_msg(RTAPI_MSG_WARN,
//TODO:MBOX:                                "WOU:B: j[%d] prev_vel_cmd(%f) accel_cmd(%f)\n",
//TODO:MBOX:                                n,
//TODO:MBOX:                                stepgen->prev_vel_cmd,
//TODO:MBOX:                                stepgen->accel_cmd
//TODO:MBOX:                               );
//TODO:MBOX:            }
	    if (stepgen->accel_cmd > stepgen->maxaccel) {
	        stepgen->accel_cmd = stepgen->maxaccel;
	        stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd * dt;
	    } else if (stepgen->accel_cmd < -(stepgen->maxaccel)) {
	        stepgen->accel_cmd = -(stepgen->maxaccel);
	        stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd * dt;
	    }
	    // stepgen->vel_cmd = stepgen->prev_vel_cmd + stepgen->accel_cmd;
            
//TODO:MBOX:            if (n == (num_chan - 1)) {
//TODO:MBOX:                rtapi_print_msg(RTAPI_MSG_WARN,
//TODO:MBOX:                                "WOU:C: j[%d] vel_cmd(%f) accel_cmd(%f)\n",
//TODO:MBOX:                                n,
//TODO:MBOX:                                stepgen->vel_cmd,
//TODO:MBOX:                                stepgen->accel_cmd
//TODO:MBOX:                               );
//TODO:MBOX:            }

	    /* end: velocity and acceleration check */
            //wou_pos_cmd = (int32_t)((stepgen->vel_cmd * dt *(stepgen->pos_scale)) * (1 << pulse_fraction_bit[n]));
            integer_pos_cmd = (int32_t)((stepgen->vel_cmd * dt *(stepgen->pos_scale)) * (1 << pulse_fraction_bit[n]));

            /* extract integer part of command */
            wou_pos_cmd = integer_pos_cmd >> pulse_fraction_bit[n];
            
            if(wou_pos_cmd > 8192 || wou_pos_cmd < -8192) {
                fprintf(stderr,"j(%d) pos_cmd(%f) prev_pos_cmd(%f) home_state(%d) vel_cmd(%f)\n",n ,
                        (*stepgen->pos_cmd), (stepgen->prev_pos_cmd), *stepgen->home_state, stepgen->vel_cmd);
                fprintf(stderr,"wou_stepgen.c: wou_pos_cmd(%d) too large\n", wou_pos_cmd);
                assert(0);
            }
            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            
            if (wou_pos_cmd >= 0) {
                sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);
            } else {
                wou_pos_cmd *= -1;
                sync_cmd = SYNC_JNT | DIR_N | (POS_MASK & wou_pos_cmd);
            }
            memcpy(data + 2*n * sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));
            
            wou_pos_cmd = integer_pos_cmd & FRACTION_MASK; 

            /* packing fraction part */
            sync_cmd = (uint16_t) wou_pos_cmd;
            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));

            (stepgen->prev_pos_cmd) += (double) ((integer_pos_cmd * stepgen->scale_recip)/(1<<pulse_fraction_bit[n]));
            stepgen->prev_vel_cmd = stepgen->vel_cmd;

            if (stepgen->pos_mode == 0) {
                    // TODO: let TRAJ-PLANNER judge the index/revolution
                    // TODO: remove this section from wou_stepgen.c
                    double delta;
                    double spindle_pos;
                    double spindle_irevs;
                    double pos_scale;
                    pos_scale = stepgen->pos_scale;
                    machine_control->spindle_enc_count += wou_pos_cmd;

                    spindle_pos = (double) (machine_control->spindle_enc_count) / pos_scale;

                    spindle_irevs =
                        (fmod((double) (machine_control->spindle_enc_count), pos_scale) / pos_scale);
                    delta = spindle_irevs - machine_control->prev_spindle_irevs;
                    machine_control->prev_spindle_irevs = spindle_irevs;

                    if ((*machine_control->spindle_index_enable == 1) && (*machine_control->spindle_at_speed)) {

                        if (delta < -0.5) {
                            // ex.: 0.9 -> 0.1 (forward)
                            machine_control->last_spindle_index_pos = floor(spindle_pos);
                            *machine_control->spindle_index_enable = 0;
                            // fprintf (stderr, "DEBUG: got spindle_index_enable request\n");
                            // fprintf (stderr, "DEBUG: spindle: delta(%f)\n", delta);
                        } else if (delta > 0.5) {
                            // ex.: 0.1 -> 0.9 (backward)
                            machine_control->last_spindle_index_pos = ceil (spindle_pos);
                            *machine_control->spindle_index_enable = 0;
                            // fprintf (stderr, "DEBUG: got spindle_index_enable request\n");
                            // fprintf (stderr, "DEBUG: spindle: delta(%f)\n", delta);
                        }
                    }

                    *machine_control->spindle_revs = spindle_pos - machine_control->last_spindle_index_pos;
            }

	}

        if (n == (num_chan - 1)) {
            // send to WOU when all axes commands are generated
            wou_cmd(&w_param,
                    WB_WR_CMD,
                    (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_chan, data);
        }
	DPS("%15.7f%15.7f%7d",
	    (stepgen->prev_pos_cmd), *stepgen->pos_fb,
             *stepgen->home_state);

	/* move on to next channel */
	stepgen++;
    }

    DPS("%15.7f", *machine_control->spindle_revs);
    // send velocity status to RISC
    if(machine_control->position_compensation_en_flag == 1) {
        fp_req_vel = ((machine_control->fp_original_requested_vel));
        fp_cur_vel = ((*machine_control->current_vel));
        if (fp_req_vel != machine_control->fp_requested_vel) {
            // tell RISC that we got a new request velocity (while changing Feedrate)
            // forward requested velocity
            machine_control->fp_requested_vel = fp_req_vel;
            // forward current velocity
            machine_control->fp_current_vel = fp_cur_vel;
            sync_cmd = SYNC_VEL;
            memcpy(data, &sync_cmd, sizeof(uint16_t));
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                    sizeof(uint16_t), data);
            machine_control->vel_sync = 0;
        }
        fp_diff = fabs (fp_cur_vel - machine_control->fp_original_requested_vel);
        // fp_diff = fp_diff > 0 ? fp_diff:-fp_diff;
        // 0.03: the coefficient of NEAR_VEL_SYNC
        // TODO: set K_NEAR_VEL_SYNC as a parameter for wou_stepgen
        if (((fp_diff) <= ((machine_control->fp_original_requested_vel)*0.03)) &&   //  120 mm/min
                (fp_cur_vel != machine_control->fp_current_vel) &&  // TODO: confirm why do we need this equation?
                (machine_control->vel_sync == 0)) {
            // forward current velocity
            if(machine_control->fp_original_requested_vel == machine_control->fp_requested_vel) {
                machine_control->vel_sync = 1;
                sync_cmd = (SYNC_VEL | (((int32_t)fp_cur_vel) << 1)) | 0x0001;
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                        sizeof(uint16_t), data);
                fprintf(stderr,"vel(%f) synced with original_vel(%f) requested(%f)\n",
                        machine_control->fp_current_vel,
                        machine_control->fp_original_requested_vel,
                        machine_control->fp_requested_vel);
            } else {
                fprintf(stderr,"vel synced but not original vel requested\n");
            }
        } else if(machine_control->vel_sync == 1 && (fp_diff >
                            ((machine_control->fp_original_requested_vel)*0.03))){
            machine_control->vel_sync = 0;
            sync_cmd = (SYNC_VEL|(((int32_t)fp_cur_vel) << 1)) & 0xFFFE;
            memcpy(data, &sync_cmd, sizeof(uint16_t));
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                   sizeof(uint16_t), data);
        }
        machine_control->fp_current_vel = fp_cur_vel;
    }

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

 /*   // export Digital IN
    for (i = 0; i < 16; i++) {
	retval = hal_pin_bit_newf(HAL_OUT, &(addr->in[i]), comp_id,
				  "wou.gpio.in.%02d", i);
	if (retval != 0) {
	    return retval;
	}
	*(addr->in[i]) = 0;
    }
*/
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
			      "wou.stepgen.%d.index_enable", num);
    if (retval != 0) {
	return retval;
    }

    /* export parameter for position scaling */
    retval = hal_param_float_newf(HAL_RW, &(addr->pos_scale), comp_id,
				  "wou.stepgen.%d.position-scale", num);
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

    //obsolete: if (step_type == 0) {
    //obsolete:     /* step/dir is the only one that uses dirsetup and dirhold */
    //obsolete:     retval = hal_param_u32_newf(HAL_RW, &(addr->dir_setup),
    //obsolete:     			    comp_id, "wou.stepgen.%d.dirsetup",
    //obsolete:     			    num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete:     retval = hal_param_u32_newf(HAL_RW, &(addr->dir_hold_dly),
    //obsolete:     			    comp_id, "wou.stepgen.%d.dirhold",
    //obsolete:     			    num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete: } else {
    //obsolete:     /* the others use dirdelay */
    //obsolete:     retval = hal_param_u32_newf(HAL_RW, &(addr->dir_hold_dly),
    //obsolete:     			    comp_id, "wou.stepgen.%d.dirdelay",
    //obsolete:     			    num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete: }
    //obsolete: /* export output pins */
    //obsolete: if (step_type == 0) {
    //obsolete:     /* step and direction */
    //obsolete:     retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[STEP_PIN]),
    //obsolete:     			  comp_id, "wou.stepgen.%d.step", num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete:     *(addr->phase[STEP_PIN]) = 0;
    //obsolete:     retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[DIR_PIN]),
    //obsolete:     			  comp_id, "wou.stepgen.%d.dir", num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete:     *(addr->phase[DIR_PIN]) = 0;
    //obsolete: } else if (step_type == 1) {
    //obsolete:     /* up and down */
    //obsolete:     retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[UP_PIN]),
    //obsolete:     			  comp_id, "wou.stepgen.%d.up", num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete:     *(addr->phase[UP_PIN]) = 0;
    //obsolete:     retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[DOWN_PIN]),
    //obsolete:     			  comp_id, "wou.stepgen.%d.down", num);
    //obsolete:     if (retval != 0) {
    //obsolete:         return retval;
    //obsolete:     }
    //obsolete:     *(addr->phase[DOWN_PIN]) = 0;
    //obsolete: } else {
    //obsolete:     /* stepping types 2 and higher use a varying number of phase pins */
    //obsolete:     addr->num_phases = num_phases_lut[step_type - 2];
    //obsolete:     for (n = 0; n < addr->num_phases; n++) {
    //obsolete:         retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[n]),
    //obsolete:     			      comp_id, "wou.stepgen.%d.phase-%c",
    //obsolete:     			      num, n + 'A');
    //obsolete:         if (retval != 0) {
    //obsolete:     	return retval;
    //obsolete:         }
    //obsolete:         *(addr->phase[n]) = 0;
    //obsolete:     }
    //obsolete: }
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
     }

    retval =
	hal_pin_bit_newf(HAL_IO, &(machine_control->sync_in_trigger), comp_id,
			 "wou.sync.in.trigger");
    *(machine_control->sync_in_trigger) = 0;	// pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    retval =
	hal_pin_u32_newf(HAL_IN, &(machine_control->sync_in), comp_id,
			 "wou.sync.in.index");
    *(machine_control->sync_in) = 0;	// pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->wait_type), comp_id,
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
            hal_pin_bit_newf(HAL_IN, &(machine_control->position_compensation_en_trigger), comp_id,
                            "wou.pos.comp.en.trigger");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->position_compensation_en_trigger) = 0;

    retval =
            hal_pin_bit_newf(HAL_IN, &(machine_control->position_compensation_en), comp_id,
                            "wou.pos.comp.en");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->position_compensation_en) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->position_compensation_ref), comp_id,
                            "wou.pos.comp.ref");
    if (retval != 0) {
            return retval;
    }
    *(machine_control->position_compensation_ref) = 0;
    retval =
            hal_pin_float_newf(HAL_IN, &(machine_control->requested_vel), comp_id,
                                "wou.requested-vel");
    *(machine_control->requested_vel) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    retval =
            hal_pin_float_newf(HAL_IN, &(machine_control->current_vel), comp_id,
                             "wou.current-vel");
    *(machine_control->current_vel) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }
    /* for plasma control */

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->thc_enbable), comp_id,
                                "wou.thc_enable");
    if (retval != 0) {
        return retval;
    }retval =
            hal_pin_bit_newf(HAL_IN, &(machine_control->plasma_enable), comp_id,
                            "wou.plasma_enable");
    if (retval != 0) {
        return retval;
    } else {
        *machine_control->plasma_enable = 1;
    }

    /* for probe */
    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->probe_trigger), comp_id,
                         "wou.probe.trigger");
    *(machine_control->probe_trigger) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_u32_newf(HAL_IO, &(machine_control->probe_type), comp_id,
                             "wou.probe.type");
    *(machine_control->probe_type) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->probe_input), comp_id,
                             "wou.probe.input");
    *(machine_control->probe_input) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    /* for spindle */
//obsolete:    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->spindle_enable), comp_id,
//obsolete:                             "wou.spindle.enable");
//obsolete:    *(machine_control->spindle_enable) = 0;
//obsolete:    if (retval != 0) {
//obsolete:        return retval;
//obsolete:    }
//obsolete:    retval = hal_pin_float_newf(HAL_IN, &(machine_control->spindle_vel_cmd), comp_id,
//obsolete:                                 "wou.spindle.vel_cmd");
//obsolete:    *(machine_control->spindle_vel_cmd) = 0;
//obsolete:    if (retval != 0) {
//obsolete:        return retval;
//obsolete:    }
//obsolete:    retval = hal_pin_float_newf(HAL_OUT, &(machine_control->spindle_vel_fb), comp_id,
//obsolete:                                     "wou.spindle.vel_cmd_fb");
//obsolete:    *(machine_control->spindle_vel_cmd) = 0;
//obsolete:    if (retval != 0) {
//obsolete:        return retval;
//obsolete:    }

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

    machine_control->prev_out = 0;
    machine_control->fp_current_vel = 0;
    machine_control->fp_requested_vel = 0;

    machine_control->position_compensation_en_flag = 0;

    machine_control->last_spindle_index_pos = 0;
    machine_control->prev_spindle_irevs = 0;
//obsolete:    machine_control->prev_spindle_pos = 0;

/*   restore saved message level*/
    rtapi_set_msg_level(msg);
    return 0;
}				// export_gpio ()


// vim:sw=4:sts=4:et:
