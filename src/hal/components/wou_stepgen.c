/********************************************************************
* Description:  stepgen.c
*               This file, 'stepgen.c', is a HAL component that 
*               provides software based step pulse generation.
*
* Author: John Kasunich
* License: GPL Version 2
*    
* Copyright (c) 2003-2007 All rights reserved.
*
* Last change: 
********************************************************************/
/** This file, 'stepgen.c', is a HAL component that provides software
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
    exported to HAL.  The values of these parameters are in nano-seconds,
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

    This module supports a number of stepping types, as follows:

    Type 0:  Step and Direction
               _____         _____               _____
    STEP  ____/     \_______/     \_____________/     \______
              |     |       |     |             |     |
    Time      |-(1)-|--(2)--|-(1)-|--(3)--|-(4)-|-(1)-|
                                          |__________________
    DIR   ________________________________/

    There are two output pins, STEP and DIR.  Step pulses appear on
    STEP.  A positive frequency command results in DIR low, negative
    frequency command means DIR high.  The minimum period of the
    step pulses is 'steplen' + 'stepspace', and the frequency
    command is clamped to avoid exceeding these limits.  'steplen'
    and 'stepspace' must both be non-zero.  'dirsetup' or 'dirhold'
    may be zero, but their sum must be non-zero, to ensure non-zero
    low time between the last up step and the first down step.

    Type 1:  Up/Down
             _____       _____
    UP    __/     \_____/     \________________________________
            |     |     |     |         |
    Time    |-(1)-|-(2)-|-(1)-|---(5)---|-(1)-|-(2)-|-(1)-|
                                        |_____|     |_____|
    DOWN  ______________________________/     \_____/     \____


    There are two output pins, UP and DOWN.  A positive frequency
    command results in pulses on UP, negative frequency command
    results in pulses on DOWN.  The minimum period of the step
    pulses is 'steplen' + 'stepspace', and the frequency command
    is clamped to avoid exceeding these limits.  'steplen',
    'stepspace', and 'dirdelay' must all be non-zero.

    Types 2 and higher:  State Patterns

    STATE   |---1---|---2---|---3---|----4----|---3---|---2---|
            |       |       |       |         |       |       |
    Time    |--(1)--|--(1)--|--(1)--|--(1+5)--|--(1)--|--(1)--|

    All the remaining stepping types are simply different repeating
    patterns on two to five output pins.  When a step occurs, the
    output pins change to the next (or previous) pattern in the
    state listings that follow.  The output pins are called 'PhaseA'
    thru 'PhaseE'.  Timing constraints are obeyed as indicated
    in the drawing above.  'steplen' must be non-zero.  'dirdelay'
    may be zero.  Because stepspace is not used, state based
    stepping types can run faster than types 0 and 1.

    Type 2:  Quadrature (aka Gray/Grey code)

    State   Phase A   Phase B
      0        1        0
      1        1        1
      2        0        1
      3        0        0
      0        1        0

    Type 3:  Three Wire

    State   Phase A   Phase B   Phase C
      0        1        0         0
      1        0        1         0
      2        0        0         1
      0        1        0         0

    Type 4:  Three Wire HalfStep

    State   Phase A   Phase B   Phase C
      0        1        0         0
      1        1        1         0
      2        0        1         0
      3        0        1         1
      4        0        0         1
      5        1        0         1
      0        1        0         0

    Type 5:  Unipolar Full Step (one winding on)

    State   Phase A   Phase B   Phase C   Phase D
      0        1        0         0         0
      1        0        1         0         0
      2        0        0         1         0
      3        0        0         0         1
      0        1        0         0         0

    Type 6:  Unipolar Full Step (two windings on)

    State   Phase A   Phase B   Phase C   Phase D
      0        1        1         0         0
      1        0        1         1         0
      2        0        0         1         1
      3        1        0         0         1
      0        1        1         0         0

    Type 7:  Bipolar Full Step (one winding on)

    State   Phase A   Phase B   Phase C   Phase D
      0        1        0         0         0
      1        1        1         1         0
      2        0        1         1         1
      3        0        0         0         1
      0        1        0         0         0

    Type 8:  Bipolar Full Step (two windings on)

    State   Phase A   Phase B   Phase C   Phase D
      0        1        0         1         0
      1        0        1         1         0
      2        0        1         0         1
      3        1        0         0         1
      0        1        0         1         0

    Type 9:  Unipolar Half Step

    State   Phase A   Phase B   Phase C   Phase D
      0        1        0         0         0
      1        1        1         0         0
      2        0        1         0         0
      3        0        1         1         0
      4        0        0         1         0
      5        0        0         1         1
      6        0        0         0         1
      7        1        0         0         1
      0        1        0         0         0

    Type 10:  Bipolar Half Step

    State   Phase A   Phase B   Phase C   Phase D
      0        1        0         0         0
      1        1        0         1         0
      2        1        1         1         0
      3        0        1         1         0
      4        0        1         1         1
      5        0        1         0         1
      6        0        0         0         1
      7        1        0         0         1
      0        1        0         0         0

    Type 11:  Five Wire Unipolar

    State   Phase A   Phase B   Phase C   Phase D  Phase E
      0        1        0         0         0        0
      1        0        1         0         0        0
      2        0        0         1         0        0
      3        0        0         0         1        0
      4        0        0         0         0        1
      0        1        0         0         0        0

    Type 12:  Five Wire Wave

    State   Phase A   Phase B   Phase C   Phase D  Phase E
      0        1        1         0         0        0
      1        0        1         1         0        0
      2        0        0         1         1        0
      3        0        0         0         1        1
      4        1        0         0         0        1
      0        1        1         0         0        0

    Type 13:  Five Wire Unipolar HalfStep

    State   Phase A   Phase B   Phase C   Phase D  Phase E
      0        1        0         0         0        0
      1        1        1         0         0        0
      2        0        1         0         0        0
      3        0        1         1         0        0
      4        0        0         1         0        0
      5        0        0         1         1        0
      6        0        0         0         1        0
      7        0        0         0         1        1
      8        0        0         0         0        1
      9        1        0         0         0        1
      0        1        0         0         0        0

    Type 14:  Five Wire Wave HalfStep

    State   Phase A   Phase B   Phase C   Phase D  Phase E
      0        1        1         0         0        0
      1        1        1         1         0        0
      2        0        1         1         0        0
      3        0        1         1         1        0
      4        0        0         1         1        0
      5        0        0         1         1        1
      6        0        0         0         1        1
      7        1        0         0         1        1
      8        1        0         0         0        1
      9        1        1         0         0        1
      0        1        1         0         0        0

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

#include <float.h>
#include <assert.h>
#include <stdio.h>
#include "rtapi_math.h"

#include <stdint.h>
#include <wou.h>
#include <wb_regs.h>

#define MAX_CHAN 8

// to disable DP(): #define TRACE 0
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
// FILE *dptrace = fopen("dptrace.log","w");
static FILE *dptrace;
#endif

/* module information */
MODULE_AUTHOR("John Kasunich");
MODULE_DESCRIPTION("Step Pulse Generator for EMC HAL");
MODULE_LICENSE("GPL");
int step_type[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
RTAPI_MP_ARRAY_INT(step_type,MAX_CHAN,"stepping types for up to 8 channels");
const char *ctrl_type[MAX_CHAN] = { "p", "p", "p", "p", "p", "p", "p", "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type,MAX_CHAN,"control type (pos or vel) for up to 8 channels");

static const char *board = "7i43u";
static const char wou_id = 0;
static const char *bitfile = "./fpga_top.bit";
static wou_param_t w_param;


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/** This structure contains the runtime data for a single generator. */

/* structure members are ordered to optimize caching for makepulses,
   which runs in the fastest thread */

typedef struct {
    /* stuff that is both read and written by makepulses */
    unsigned int timer1;	/* times out when step pulse should end */
    unsigned int timer2;	/* times out when safe to change dir */
    unsigned int timer3;	/* times out when safe to step in new dir */
    int hold_dds;		/* prevents accumulator from updating */
    long addval;		/* actual frequency generator add value */
    volatile long long accum;	/* frequency generator accumulator */
    hal_s32_t rawcount;		/* param: position feedback in counts */
    int curr_dir;		/* current direction */
    int state;			/* current position in state table */
    /* stuff that is read but not written by makepulses */
    hal_bit_t *enable;		/* pin for enable stepgen */
    long target_addval;		/* desired freq generator add value */
    long deltalim;		/* max allowed change per period */
    hal_u32_t step_len;		/* parameter: step pulse length */
    hal_u32_t dir_hold_dly;	/* param: direction hold time or delay */
    hal_u32_t dir_setup;	/* param: direction setup time */
    int step_type;		/* stepping type - see list above */
    int cycle_max;		/* cycle length for step types 2 and up */
    int num_phases;		/* number of phases for types 2 and up */
    hal_bit_t *phase[5];	/* pins for output signals */
    const unsigned char *lut;	/* pointer to state lookup table */
    /* stuff that is not accessed by makepulses */
    int pos_mode;		/* 1 = position mode, 0 = velocity mode */
    hal_u32_t step_space;	/* parameter: min step pulse spacing */
    // double old_pos_cmd;		/* previous position command (counts) */
    hal_s32_t *count;		/* pin: captured feedback in counts */
    hal_float_t pos_scale;	/* param: steps per position unit */
    double old_scale;		/* stored scale value */
    double scale_recip;		/* reciprocal value used for scaling */
    hal_float_t *vel_cmd;	/* pin: velocity command (pos units/sec) */
    hal_float_t *pos_cmd;	/* pin: position command (position units) */
    hal_float_t *pos_fb;	/* pin: position feedback (position units) */
    hal_float_t pos_accum;	/* accumulated sub-step cmd */
    hal_float_t cur_pos;	/* current position */
    hal_float_t freq;		/* param: frequency command */
    hal_float_t maxvel;		/* param: max velocity, (pos units/sec) */
    hal_float_t maxaccel;	/* param: max accel (pos units/sec^2) */
    hal_u32_t old_step_len;	/* used to detect parameter changes */
    hal_u32_t old_step_space;
    hal_u32_t old_dir_hold_dly;
    hal_u32_t old_dir_setup;
    int printed_error;		/* flag to avoid repeated printing */
} stepgen_t;

/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t *stepgen_array;

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

#define PICKOFF		28	/* bit location in DDS accum */



/* other globals */
static int comp_id;		/* component ID */
static int num_chan = 0;	/* number of step generators configured */
static long periodns;		/* makepulses function period in nanosec */
static long old_periodns;	/* used to detect changes in periodns */
static double periodfp;		/* makepulses function period in seconds */
static double freqscale;	/* conv. factor from Hz to addval counts */
static double accelscale;	/* conv. Hz/sec to addval cnts/period */
static long old_dtns;		/* update_freq funct period in nsec */
static double dt;		/* update_freq period in seconds */
static double recip_dt;		/* recprocal of period, avoids divides */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_stepgen(int num, stepgen_t * addr, int step_type, int pos_mode);
static void update_freq(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval;

    uint8_t data[MAX_DSIZE];
    int ret;

#if (TRACE!=0)
    // initialize file handle for logging wou steps
    dptrace = fopen("wou_steps.log", "w");
    /* prepare header for gnuplot */
    DPS ("#%10s  %15s%12s  %15s%12s  %15s%12s  %15s%12s\n", 
          "dt",  "pos_cmd[0]", "pos_fb[0]", "pos_cmd[1]", "pos_fb[1]",
                 "pos_cmd[2]", "pos_fb[2]", "pos_cmd[3]", "pos_fb[3]");

#endif
    
    wou_init(&w_param, board, wou_id, bitfile);
    if (wou_connect(&w_param) == -1) {  
	    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR Connection failed\n");
	    return -1;
    }
    // forward SERVO pulses to LEDs
    data[0] = 1;
    ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_AI_MODE),
                   GPIO_LEDS_SEL,
                   1,
                   data);
    assert (ret==0);
    // JCMD_TBASE: 0: sif base_period is "32768" ticks
    data[0] = 0; 
    ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_AI_MODE),
                   (JCMD_BASE | JCMD_TBASE),
                   1,
                   data);
    assert (ret==0);
    // JCMD_CTRL: 
    //  [bit-0]: (1)JCMD in PLAY mode, forward JCMD_POS_W to JCMD_FIFO
    //  [bit-1]: SIF_EN, servo interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    data[0] = 3;
    ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_AI_MODE),
                   (JCMD_BASE | JCMD_CTRL),
                   1,
                   data);
    assert (ret==0);
    wou_flush(&w_param);
    
    for (n = 0; n < MAX_CHAN && step_type[n] != -1 ; n++) {
	if ((step_type[n] > MAX_STEP_TYPE) || (step_type[n] < 0)) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "STEPGEN: ERROR: bad stepping type '%i', axis %i\n",
			    step_type[n], n);
	    return -1;
	}
	if ((ctrl_type[n][0] == 'p' ) || (ctrl_type[n][0] == 'P')) {
	    ctrl_type[n] = "p";
	} else if ((ctrl_type[n][0] == 'v' ) || (ctrl_type[n][0] == 'V')) {
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

    /* periodns will be set to the proper value when 'make_pulses()' runs for 
       the first time.  We load a default value here to avoid glitches at
       startup, but all these 'constants' are recomputed inside
       'update_freq()' using the real period. */
    old_periodns = periodns = 50000;
    old_dtns = 1310720;
    /* precompute some constants */
    periodfp = periodns * 0.000000001;
    freqscale = (1L << PICKOFF) * periodfp;
    accelscale = freqscale * periodfp;
    dt = old_dtns * 0.000000001;
    recip_dt = 1.0 / dt;
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
    /* export all the variables for each pulse generator */
    for (n = 0; n < num_chan; n++) {
	/* export all vars */
	retval = export_stepgen(n, &(stepgen_array[n]),
	    step_type[n], (ctrl_type[n][0] == 'p'));
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"STEPGEN: ERROR: stepgen %d var export failed\n", n);
	    hal_exit(comp_id);
	    return -1;
	}
    }
    retval = hal_export_funct("wou.stepgen.update-freq", update_freq,
	stepgen_array, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "STEPGEN: ERROR: freq update funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,
	"STEPGEN: installed %d step pulse generators\n", num_chan);
    
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

static void update_freq(void *arg, long period)
{
    long     min_step_period;
    double   max_freq;
    stepgen_t *stepgen;
    int n;
    double pos_cmd, vel_cmd, curr_pos, curr_vel, max_ac;
    double match_ac, match_time, new_vel, desired_freq;
    double dp, dv, est_out, est_cmd, est_err, avg_v;
    int wou_pos_cmd;
    // ret, data[]: for wou_cmd()
    uint8_t data[MAX_DSIZE];
    int ret;
#if (TRACE!=0)
    static uint32_t _dt = 0;
#endif
    /*! \todo FIXME - while this code works just fine, there are a bunch of
       internal variables, many of which hold intermediate results that
       don't really need their own variables.  They are used either for
       clarity, or because that's how the code evolved.  This algorithm
       could use some cleanup and optimization. */
    /* this periodns stuff is a little convoluted because we need to
       calculate some constants here in this relatively slow thread but the
       constants are based on the period of the much faster 'make_pulses()'
       thread. */
    
    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
int msg;
msg = rtapi_get_msg_level();
// rtapi_set_msg_level(RTAPI_MSG_WARN);
rtapi_set_msg_level(RTAPI_MSG_ALL);
// rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: enter %s\n", __FUNCTION__);
// long long int now;
// now = rtapi_get_time();
// rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: enter %s @(%lld)\n",
//                                __FUNCTION__, now);

    /* point at stepgen data */
    stepgen = arg;
   
#if (TRACE!=0)
    if (*stepgen->enable) {
      DPS("%11u", _dt);
      _dt ++;
    }
#endif

    // num_chan: 4, calculated from step_type;
    /* loop thru generators */
	    
    for (n = 0; n < num_chan; n++) {
	/* test for disabled stepgen */
	if (*stepgen->enable == 0) {
	    /* AXIS not enable */
            /* keep updating parameters for better performance */
            stepgen->scale_recip = 1.0 / stepgen->pos_scale;
            
	    /* set velocity to zero */
	    stepgen->freq = 0;
	    stepgen->addval = 0;
	    stepgen->target_addval = 0;
	    /* and skip to next one */
	    stepgen++;
	    continue;
	}

	/* calculate frequency limit */
        // TODO: calculate limits once at initialization
        // each joint may have different STEPLEN setting
        // if (step_type[n] < 2) {
        //   min_step_period = stepgen->step_len + stepgen->step_space;
        // } else {
        min_step_period = stepgen->step_len + stepgen->dir_hold_dly;
        // }
	max_freq = 1.0 / (min_step_period * 0.000000001);
	/* check for user specified frequency limit parameter */
	if (stepgen->maxvel <= 0.0) {
	    /* set to zero if negative */
	    stepgen->maxvel = 0.0;
	} else {
	    /* parameter is non-zero, compare to max_freq */
	    desired_freq = stepgen->maxvel * fabs(stepgen->pos_scale);

// rtapi_print_msg(RTAPI_MSG_DBG, "%s: min_step_period(%ld), max_freq(%f), desired_freq(%f)\n", 
//                               __FUNCTION__, min_step_period, max_freq, desired_freq);
// rtapi_print_msg(RTAPI_MSG_DBG, "%s: maxvel(%f), pos_scale(%f)\n", 
//                               __FUNCTION__, stepgen->maxvel, stepgen->pos_scale);

	    if (desired_freq > max_freq) {
		/* parameter is too high, complain about it */
		if(!stepgen->printed_error) {
		    rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: Channel %d: The requested maximum velocity of %d steps/sec is too high.\n",
			n, (int)desired_freq);
		    rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: Channel %d: The maximum possible frequency is %d steps/second\n",
			n, (int)max_freq);
		    stepgen->printed_error = 1;
		}
		/* parameter is too high, limit it */
		stepgen->maxvel = max_freq / fabs(stepgen->pos_scale);
	    } else {
		/* lower max_freq to match parameter */
		// max_freq = stepgen->maxvel * fabs(stepgen->pos_scale);
		max_freq = desired_freq;
	    }
	}
	/* set internal accel limit to its absolute max, which is
	   zero to full speed in one thread period */
        // @500KHz servo pulse, 1.31ms period, max_ac is 381M p/s^2
	max_ac = max_freq * recip_dt;
	/* check for user specified accel limit parameter */
	if (stepgen->maxaccel <= 0.0) {
	    /* set to zero if negative */
	    stepgen->maxaccel = 0.0;
	} else {
	    /* parameter is non-zero, compare to max_ac */
	    if ((stepgen->maxaccel * fabs(stepgen->pos_scale)) > max_ac) {
		/* parameter is too high, lower it */
		stepgen->maxaccel = max_ac / fabs(stepgen->pos_scale);
	    } else {
		/* lower limit to match parameter */
		max_ac = stepgen->maxaccel * fabs(stepgen->pos_scale);
	    }
	}
	/* at this point, all scaling, limits, and other parameter
	   changes have been handled - time for the main control */
	if ( stepgen->pos_mode ) {
            DPS ("  %15.7f%12.7f", *stepgen->pos_cmd, *stepgen->pos_fb);
	    /* calculate position command in counts */
            pos_cmd = (*stepgen->pos_cmd) * stepgen->pos_scale;
	    curr_pos = stepgen->accum;
	    /* calculate velocity command in counts/sec */
	    vel_cmd = (pos_cmd - curr_pos) * recip_dt;
	    /* get velocity in counts/sec */
	    curr_vel = stepgen->freq;
            /* unit for the followings are counts or counts/sec */
            // fprintf (wou_fh, "\tJ%d: pos_cmd(%f) curr_pos(%f) vel_cmd(%f) curr_vel(%f)\n",
            //                  n, pos_cmd, curr_pos, vel_cmd, curr_vel);

	    /* At this point we have good values for pos_cmd, curr_pos,
	       vel_cmd, curr_vel, max_freq and max_ac, all in counts,
	       counts/sec, or counts/sec^2.  Now we just have to do
	       something useful with them. */
	    /* determine which way we need to ramp to match velocity */
	    if (vel_cmd > curr_vel) {
	        match_ac = max_ac;
	    } else {
	        match_ac = -max_ac;
	    }
	    /* determine how long the match would take */
	    match_time = (vel_cmd - curr_vel) / match_ac;
	    /* calc output position at the end of the match */
	    avg_v = (vel_cmd + curr_vel) * 0.5;
	    est_out = curr_pos + avg_v * match_time;
	    /* calculate the expected command position at that time */
	    est_cmd = pos_cmd + vel_cmd * (match_time - 1.5 * dt);
	    /* calculate error at that time */
	    est_err = est_out - est_cmd;
	    if (match_time < dt) {
	      /* we can match velocity in one period */
              if (fabs(est_err) < 0.0001) {
                /* after match the position error will be acceptable */
                /* so we just do the velocity match */
                new_vel = vel_cmd;
              } else {
                /* try to correct position error */
                new_vel = vel_cmd - 0.5 * est_err * recip_dt;
                /* apply accel limits */
                if (new_vel > (curr_vel + max_ac * dt)) {
                  new_vel = curr_vel + max_ac * dt;
                } else if (new_vel < (curr_vel - max_ac * dt)) {
                  new_vel = curr_vel - max_ac * dt;
                }
              }
	    } else {
              /* calculate change in final position if we ramp in the
		 opposite direction for one period */
	      dv = -2.0 * match_ac * dt;
	      dp = dv * match_time;
	      /* decide which way to ramp */
	      if (fabs(est_err + dp * 2.0) < fabs(est_err)) {
	          match_ac = -match_ac;
	      }
	      /* and do it */
	      new_vel = curr_vel + match_ac * dt;
	    }

	    /* apply frequency limit */
	    if (new_vel > max_freq) {
	      new_vel = max_freq;
              //do happens: rtapi_print_msg(RTAPI_MSG_ERR, "STEPGEN[%d]: over max_freq\n", n);
	    } else if (new_vel < -max_freq) {
	      new_vel = -max_freq;
              //do happens: rtapi_print_msg(RTAPI_MSG_ERR, "STEPGEN[%d]: over max_freq\n", n);
	    }
	    /* end of position mode */
	} else {
            // do not support velocity mode yet
            assert(0);
	    /* end of velocity mode */
	}
	stepgen->freq = new_vel;
        
        
        // TODO: calculate WOU commands
        // each AXIS cycle is 1638400ns, 8192 ticks of 200ns(5MHz) clocks
        // each AXIS cycle is 1310720ns, 32768 ticks of 40ns(25MHz) clocks
        // pos_cmd = new_vel * dt;
        // fprintf (wou_fh, "\tJ%d: new_vel(%f), pos_cmd(%f)\n", n, new_vel, pos_cmd);
        // wou_pos_cmd = (int) pos_cmd;
        wou_pos_cmd = (int) new_vel * dt;
        stepgen->accum = curr_pos + wou_pos_cmd;

        // fprintf (wou_fh, "\tJ%d: wou_pos_cmd(%d)\n", n, wou_pos_cmd);
        assert (wou_pos_cmd < 8192);
        assert (wou_pos_cmd > -8192);
        
        {
          if (wou_pos_cmd >= 0) {
            // data[0]: MSB {dir, pos[12:8]}, dir(1): positive
            data[2*n    ] = (uint8_t) (((wou_pos_cmd >> 8) & 0x1F) | 0x20); 
            data[2*n + 1] = (uint8_t) (wou_pos_cmd & 0xFF);
          } else {
            // data[0]: MSB {dir, pos[12:8]}, dir(0): negative
            wou_pos_cmd *= -1;
            data[2*n    ] = (uint8_t) ((wou_pos_cmd >> 8) & 0x1F); 
            data[2*n + 1] = (uint8_t) (wou_pos_cmd & 0xFF);
          }
          // fprintf (wou_fh, "\tJ%d: data[]: <0x%02x><0x%02x>\n", n, data[0], data[1]);
          if (n == (num_chan - 1)) {
            // send to WOU when all axes commands are generated
            ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_FIFO_MODE),
                   (JCMD_BASE | JCMD_POS_W),
                   2*num_chan,
                   data);
            assert (ret==0);
          }
        }

        // stepgen->cur_pos = curr_pos + wou_pos_cmd;
        // stepgen->pos_accum = pos_cmd - wou_pos_cmd;
        // fprintf (wou_fh, "\tJ%d: pos_cmd(%f), accum(%f)\n", n, pos_cmd, stepgen->pos_accum);
        
        // *(stepgen->pos_fb) = stepgen->accum / stepgen->pos_scale;
        *(stepgen->pos_fb) = stepgen->accum * stepgen->scale_recip;

	/* move on to next channel */
	stepgen++;
    }

#if (TRACE!=0)
    if (*(stepgen-1)->enable) {
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

static int export_stepgen(int num, stepgen_t * addr, int step_type, int pos_mode)
{
    int n, retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_WARN);
    rtapi_set_msg_level(RTAPI_MSG_ALL);

    /* export param variable for raw counts */
    retval = hal_param_s32_newf(HAL_RO, &(addr->rawcount), comp_id,
	"wou.stepgen.%d.rawcounts", num);
    if (retval != 0) { return retval; }
    /* export pin for counts captured by update() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->count), comp_id,
	"wou.stepgen.%d.counts", num);
    if (retval != 0) { return retval; }
    /* export parameter for position scaling */
    retval = hal_param_float_newf(HAL_RW, &(addr->pos_scale), comp_id,
	"wou.stepgen.%d.position-scale", num);
    if (retval != 0) { return retval; }
    /* export pin for command */
    if ( pos_mode ) {
	retval = hal_pin_float_newf(HAL_IN, &(addr->pos_cmd), comp_id,
	    "wou.stepgen.%d.position-cmd", num);
    } else {
	retval = hal_pin_float_newf(HAL_IN, &(addr->vel_cmd), comp_id,
	    "wou.stepgen.%d.velocity-cmd", num);
    }
    if (retval != 0) { return retval; }
    /* export pin for enable command */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->enable), comp_id,
	"wou.stepgen.%d.enable", num);
    if (retval != 0) { return retval; }
    /* export pin for scaled position captured by update() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos_fb), comp_id,
	"wou.stepgen.%d.position-fb", num);
    if (retval != 0) { return retval; }
    /* export param for scaled velocity (frequency in Hz) */
    retval = hal_param_float_newf(HAL_RO, &(addr->freq), comp_id,
	"wou.stepgen.%d.frequency", num);
    if (retval != 0) { return retval; }
    /* export parameter for max frequency */
    retval = hal_param_float_newf(HAL_RW, &(addr->maxvel), comp_id,
	"wou.stepgen.%d.maxvel", num);
    if (retval != 0) { return retval; }
    /* export parameter for max accel/decel */
    retval = hal_param_float_newf(HAL_RW, &(addr->maxaccel), comp_id,
	"wou.stepgen.%d.maxaccel", num);
    if (retval != 0) { return retval; }
    /* every step type uses steplen */
    retval = hal_param_u32_newf(HAL_RW, &(addr->step_len), comp_id,
	"wou.stepgen.%d.steplen", num);
    if (retval != 0) { return retval; }
    if (step_type < 2) {
	/* step/dir and up/down use 'stepspace' */
	retval = hal_param_u32_newf(HAL_RW, &(addr->step_space),
	    comp_id, "wou.stepgen.%d.stepspace", num);
	if (retval != 0) { return retval; }
    }
    if ( step_type == 0 ) {
	/* step/dir is the only one that uses dirsetup and dirhold */
	retval = hal_param_u32_newf(HAL_RW, &(addr->dir_setup),
	    comp_id, "wou.stepgen.%d.dirsetup", num);
	if (retval != 0) { return retval; }
	retval = hal_param_u32_newf(HAL_RW, &(addr->dir_hold_dly),
	    comp_id, "wou.stepgen.%d.dirhold", num);
	if (retval != 0) { return retval; }
    } else {
	/* the others use dirdelay */
	retval = hal_param_u32_newf(HAL_RW, &(addr->dir_hold_dly),
	    comp_id, "wou.stepgen.%d.dirdelay", num);
	if (retval != 0) { return retval; }
    }
    /* export output pins */
    if ( step_type == 0 ) {
	/* step and direction */
	retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[STEP_PIN]),
	    comp_id, "wou.stepgen.%d.step", num);
	if (retval != 0) { return retval; }
	*(addr->phase[STEP_PIN]) = 0;
	retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[DIR_PIN]),
	    comp_id, "wou.stepgen.%d.dir", num);
	if (retval != 0) { return retval; }
	*(addr->phase[DIR_PIN]) = 0;
    } else if (step_type == 1) {
	/* up and down */
	retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[UP_PIN]),
	    comp_id, "wou.stepgen.%d.up", num);
	if (retval != 0) { return retval; }
	*(addr->phase[UP_PIN]) = 0;
	retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[DOWN_PIN]),
	    comp_id, "wou.stepgen.%d.down", num);
	if (retval != 0) { return retval; }
	*(addr->phase[DOWN_PIN]) = 0;
    } else {
	/* stepping types 2 and higher use a varying number of phase pins */
	addr->num_phases = num_phases_lut[step_type - 2];
	for (n = 0; n < addr->num_phases; n++) {
	    retval = hal_pin_bit_newf(HAL_OUT, &(addr->phase[n]),
		comp_id, "wou.stepgen.%d.phase-%c", num, n + 'A');
	    if (retval != 0) { return retval; }
	    *(addr->phase[n]) = 0;
	}
    }
    /* set default parameter values */
    addr->pos_scale = 1.0;
    addr->old_scale = 0.0;
    addr->scale_recip = 0.0;
    addr->freq = 0.0;
    addr->maxvel = 0.0;
    addr->maxaccel = 0.0;
    addr->step_type = step_type;
    addr->pos_mode = pos_mode;
    /* timing parameter defaults depend on step type */
    addr->step_len = 1;
    if ( step_type < 2 ) {
	addr->step_space = 1;
    } else {
	addr->step_space = 0;
    }
    if ( step_type == 0 ) {
	addr->dir_hold_dly = 1;
	addr->dir_setup = 1;
    } else {
	addr->dir_hold_dly = 1;
	addr->dir_setup = 0;
    }
    /* set 'old' values to make update_freq validate the timing params */
    addr->old_step_len = ~0;
    addr->old_step_space = ~0;
    addr->old_dir_hold_dly = ~0;
    addr->old_dir_setup = ~0;
    if ( step_type >= 2 ) {
	/* init output stuff */
	addr->cycle_max = cycle_len_lut[step_type - 2] - 1;
	addr->lut = &(master_lut[step_type - 2][0]);
    }
    /* init the step generator core to zero output */
    addr->cur_pos = 0.0;
    addr->pos_accum = 0.0;
    addr->timer1 = 0;
    addr->timer2 = 0;
    addr->timer3 = 0;
    addr->hold_dds = 0;
    addr->addval = 0;
    addr->accum = 0;
    addr->rawcount = 0;
    addr->curr_dir = 0;
    addr->state = 0;
    *(addr->enable) = 0;
    addr->target_addval = 0;
    addr->deltalim = 0;
    /* other init */
    addr->printed_error = 0;
    // addr->old_pos_cmd = 0.0;
    /* set initial pin values */
    *(addr->count) = 0;
    *(addr->pos_fb) = 0.0;
    if ( pos_mode ) {
	*(addr->pos_cmd) = 0.0;
    } else {
	*(addr->vel_cmd) = 0.0;
    }
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}
