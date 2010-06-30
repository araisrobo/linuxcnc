/********************************************************************
* Description:  wou_stepgen.c
*               This file, 'wou_stepgen.c', is a HAL component that 
*               provides pulse/dir commands for MESA 7i43U through USB.
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

#include <string.h>
#include <float.h>
#include <assert.h>
#include <stdio.h>
#include "rtapi_math.h"
#include "motion.h"

#include <stdint.h>
#include <wou.h>
#include <wb_regs.h>

#define MAX_CHAN 8
#define MAX_STEP_CUR 255

// to disable DP(): #define TRACE 0
#define TRACE 1
#include "dptrace.h"
#if (TRACE!=0)
// FILE *dptrace = fopen("dptrace.log","w");
static FILE *dptrace;
#endif

/* module information */
MODULE_AUTHOR("Yi-Shin Li");
MODULE_DESCRIPTION("Wishbone Over USB for EMC HAL");
MODULE_LICENSE("GPL");
int step_type[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
RTAPI_MP_ARRAY_INT(step_type,MAX_CHAN,"stepping types for up to 8 channels");
const char *ctrl_type[MAX_CHAN] = { "p", "p", "p", "p", "p", "p", "p", "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type,MAX_CHAN,"control type (pos or vel) for up to 8 channels");
const char *bits = "\0";
RTAPI_MP_STRING(bits, "FPGA bitfile");
int gpio_mask_in0 = -1;
RTAPI_MP_INT(gpio_mask_in0, "WOU Register Value for GPIO_MASK_IN0");
int gpio_mask_in1 = -1;
RTAPI_MP_INT(gpio_mask_in1, "WOU Register Value for GPIO_MASK_IN1");
int gpio_leds_sel = -1;
RTAPI_MP_INT(gpio_leds_sel, "WOU Register Value for GPIO_LEDS_SEL");
int jcmd_dir_pol = -1;
RTAPI_MP_INT(jcmd_dir_pol, "WOU Register Value for JCMD_DIR_POL");
int step_cur[MAX_CHAN] = { -1, -1, -1, -1, -1, -1, -1, -1 };
RTAPI_MP_ARRAY_INT(step_cur, MAX_CHAN, "current limit for up to 8 channel of stepping drivers");
int num_sync_in = 16;
RTAPI_MP_INT(num_sync_in, "Number of WOU HAL PINs for sync input");
int num_sync_out = 8;
RTAPI_MP_INT(num_sync_out, "Number of WOU HAL PINs for sync output");



static const char *board = "7i43u";
static const char wou_id = 0;
// static const char *bitfile = "./fpga_top.bit";
static wou_param_t w_param;


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/** This structure contains the runtime data for a single generator. */

/* structure members are ordered to optimize caching for makepulses,
   which runs in the fastest thread */

typedef struct {
    // hal_pin_*_newf: variable has to be pointer
    // hal_param_*_newf: varaiable not necessary to be pointer

    /* stuff that is both read and written by makepulses */
    volatile long long accum;	/* frequency generator accumulator */
    /* stuff that is read but not written by makepulses */
    hal_bit_t *enable;		/* pin for enable stepgen */
    hal_u32_t step_len;		/* parameter: step pulse length */
    hal_u32_t dir_hold_dly;	/* param: direction hold time or delay */
    hal_u32_t dir_setup;	/* param: direction setup time */
    int step_type;		/* stepping type - see list above */
    int num_phases;		/* number of phases for types 2 and up */
    hal_bit_t *phase[5];	/* pins for output signals */
    /* stuff that is not accessed by makepulses */
    int pos_mode;		/* 1 = position mode, 0 = velocity mode */
    hal_u32_t step_space;	/* parameter: min step pulse spacing */
    hal_s32_t *pulse_pos;	/* pin: pulse_pos to servo drive, captured from FPGA */
    hal_s32_t *enc_pos;	        /* pin: encoder position from servo drive, captured from FPGA */
    hal_float_t *switch_pos;	/* pin: scaled home switch position in absolute motor position */
    hal_float_t *index_pos;	/* pin: scaled index position in absolute motor position */
    hal_bit_t *index_enable;	/* pin for index_enable */
    hal_float_t pos_scale;	/* param: steps per position unit */
    double scale_recip;		/* reciprocal value used for scaling */
    hal_float_t *vel_cmd;	/* pin: velocity command (pos units/sec) */
    hal_float_t *pos_cmd;	/* pin: position command (position units) */
    hal_float_t *pos_fb;	/* pin: position feedback (position units) */
    hal_float_t cur_pos;	/* current position (position units) */
    hal_float_t freq;		/* param: frequency command */
    hal_float_t maxvel;		/* param: max velocity, (pos units/sec) */
    hal_float_t maxaccel;	/* param: max accel (pos units/sec^2) */
    int printed_error;		/* flag to avoid repeated printing */

    hal_s32_t *home_state;      /* pin: home_state from homing.c */
    hal_s32_t prev_home_state;  /* param: previous home_state for homing */

    double vel_fb;
    double prev_pos_cmd;        /* prev pos_cmd in counts */
    double prev_pos;            /* prev position in counts */
    double sum_err_0;
    double sum_err_1;
} stepgen_t;

typedef struct {
  // 16in 8out
  hal_bit_t *in[16];
  hal_bit_t *out[8];
  int       num_in;
  int       num_out;
  uint8_t   prev_out;
  uint16_t  prev_in;
} gpio_t;

typedef struct {
    /* sync input pins (input to motmod)*/
//    hal_bit_t   *sync_in[64]; //replace with pin index
    hal_bit_t   *sync_in_trigger;
    hal_u32_t   *sync_in; //
    hal_u32_t   *wait_type;
    hal_float_t *timeout;
    int         num_sync_in;
//    uint64_t    prev_in;
    /* sync output pins (output from motmod) */
    hal_bit_t   *sync_out[64];
    int         num_sync_out;
    uint64_t    prev_out;       //ON or OFF
    /* immeidate_pos pins*/
/*    hal_float_t *immeidate_pos[9];
    hal_u32_t   *
    int         num_immediate_pos;*/

} m_control_t;




/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t  *stepgen_array;
static gpio_t     *gpio;
static m_control_t *m_control;
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
static long old_dtns;		/* update_freq funct period in nsec */
static double dt;		/* update_freq period in seconds */
static double recip_dt;		/* recprocal of period, avoids divides */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_stepgen(int num, stepgen_t * addr, int step_type, int pos_mode);
static int export_gpio(gpio_t *addr);
static int export_m_control(m_control_t *m_control);
static void update_freq(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval;

    uint8_t data[MAX_DSIZE];
//    int ret;

#if (TRACE!=0)
    // initialize file handle for logging wou steps
    dptrace = fopen("wou_stepgen.log", "w");
    /* prepare header for gnuplot */
    DPS ("#%10s  %17s%15s%15s%15s%15s%7s  %17s%15s%15s%15s%15s%7s  %17s%15s%15s%15s%15s%7s  %17s%15s%15s%15s%15s%7s\n", 
          "dt",  "pos_cmd[0]", "cur_pos[0]", "pos_fb[0]", "match_ac[0]", "curr_vel[0]", "home[0]",
                 "pos_cmd[1]", "cur_pos[1]", "pos_fb[1]", "match_ac[1]", "curr_vel[1]", "home[1]",
                 "pos_cmd[2]", "cur_pos[2]", "pos_fb[2]", "match_ac[2]", "curr_vel[2]", "home[2]",
                 "pos_cmd[3]", "cur_pos[3]", "pos_fb[3]", "match_ac[3]", "curr_vel[3]", "home[3]");
#endif
    
    /* test for bitfile string: bits */
    if ((bits == 0) || (bits[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: no fpga bitfile string: bits\n");
	return -1;
    } else {
        // initialize FPGA with bitfile(bits)
        wou_init(&w_param, board, wou_id, bits);
        if (wou_connect(&w_param) == -1) {  
                rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: Connection failed\n");
                return -1;
        }
    }

    /* test for GPIO_MASK_IN0: gpio_mask_in0 */
    if ((gpio_mask_in0 == -1)) {
	rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: no value for GPIO_MASK_IN0: gpio_mask_in0\n");
	return -1;
    } else {
        // un-mask HOME-SWITCH inputs (bits_i[5:2])
        data[0] = (uint8_t) gpio_mask_in0;
        wou_cmd (&w_param,
                WB_WR_CMD,
                GPIO_BASE | GPIO_MASK_IN0,
                1,
                data);
       /* if (ret) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: writing GPIO_LEDS_SEL\n");
            return -1;
        }*/
    }
    
    /* test for GPIO_MASK_IN1: gpio_mask_in1 */
    if ((gpio_mask_in1 == -1)) {
	rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: no value for GPIO_MASK_IN1: gpio_mask_in1\n");
	return -1;
    } else {
        // un-mask HOME-SWITCH inputs (bits_i[5:2])
        data[0] = (uint8_t) gpio_mask_in1;
        wou_cmd (&w_param,
                       WB_WR_CMD,
                       GPIO_BASE | GPIO_MASK_IN1,
                       1,
                       data);
/*        if (ret) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: writing GPIO_MASK_IN1\n");
            return -1;
        }*/
    }


    /* test for GPIO_LEDS_SEL: gpio_leds_sel */
    if ((gpio_leds_sel == -1)) {
	rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: no value for GPIO_LEDS_SEL: gpio_leds_sel\n");
	return -1;
    } else {
        // select signals for LEDs
        data[0] = (uint8_t) gpio_leds_sel;
        wou_cmd (&w_param,
                       WB_WR_CMD,
                       GPIO_BASE | GPIO_LEDS_SEL,
                       1,
                       data);
       /* if (ret) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: writing GPIO_LEDS_SEL\n");
            return -1;
        }*/
    }
    
 /*   // test for JCMD_DIR_POL: jcmd_dir_pol
    if ((jcmd_dir_pol == -1)) {
	rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: no value for JCMD_DIR_POL: jcmd_dir_pol\n");
	return -1;
    } else {
        // JCMD_DIR_POL: Direction Polarity to compensate mechanical direction
        data[0] = (uint8_t) jcmd_dir_pol;
        ret = wou_cmd (&w_param,
                       WB_WR_CMD,
                       JCMD_BASE | JCMD_DIR_POL,
                       1,
                       data);
        // rtapi_print_msg(RTAPI_MSG_ERR, "WOU: DEBUG: JCMD_DIR_POL=%d\n", jcmd_dir_pol);
        if (ret) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: writing JCMD_DIR_POL\n");
            return -1;
        }
    }
*/
    /* test for stepping motor current limit: step_cur*/
    num_chan = 0;
    for (n = 0; n < MAX_CHAN && step_cur[n] != -1 ; n++) {
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
                      WB_WR_CMD,
                      (SSIF_BASE | SSIF_MAX_PWM), 
                      num_chan, 
                      data);
    /*    if (ret) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: writing SSIF_MAX_PWM\n");
            return -1;
        }*/
    }

    wou_flush(&w_param);
    
    num_chan = 0;
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
    gpio = hal_malloc(sizeof(gpio_t));
    if (gpio == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"GPIO: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
    m_control = hal_malloc(sizeof(m_control_t));
        if (m_control == 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "M_CONTROL: ERROR: hal_malloc() failed\n");
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
    retval = export_gpio(gpio);  // 16-in, 8-out
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "GPIO: ERROR: gpio var export failed\n");
        hal_exit(comp_id);
        return -1;
    }
/* put export m_control below */
   // static int export_m_control (m_control_t *m_control)
    retval = export_m_control(m_control);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "GPIO: ERROR:  m_control var export failed\n");
        hal_exit(comp_id);
        return -1;
    }
/* put export m_control above */
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

// This function was invented by Jeff Epler.
// It forces a floating-point variable to be degraded from native register
// size (80 bits on x86) to C double size (64 bits).
static double force_precision(double d) __attribute__((__noinline__));
static double force_precision(double d) {
    return d;
}

static void update_freq(void *arg, long period)
{
  long    min_step_period;
  double  max_freq;
  stepgen_t *stepgen;
  int n, i;
  double match_ac, new_vel, end_vel, desired_freq;
  // double dp, dv, est_out, est_cmd, est_err, match_time, vel_cmd, avg_v;
  double est_err;

  double ff_vel;
  double velocity_error;
  double match_accel;
  double seconds_to_vel_match;
  double position_at_match;
  double position_cmd_at_match;
  double error_at_match;
  double velocity_cmd;
  
  double physical_maxvel;  // max vel supported by current step timings & position-scale
  double maxvel;           // actual max vel to use this time

    int wou_pos_cmd;
    // ret, data[]: for wou_cmd()
    uint8_t data[MAX_DSIZE];
    uint64_t sync_io_data;
    // int     ret;
    
    // for homing:
    uint8_t r_rst_pos;
    uint8_t r_switch_en;
    uint8_t r_index_en;
    uint8_t r_index_lock;
    static uint8_t prev_r_switch_en = 0;
    static uint8_t prev_r_index_en = 0;
    static uint8_t prev_r_index_lock = 0;

#if (TRACE!=0)
  static uint32_t _dt = 0;
#endif
  /* FIXME - while this code works just fine, there are a bunch of
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
  rtapi_set_msg_level(RTAPI_MSG_ALL);

  /* check and update WOU Registers */
  wou_update(&w_param);

    // copy GPIO.IN ports if it differs from previous value
    if (memcmp(&(gpio->prev_in), wou_reg_ptr(&w_param, SSIF_BASE + SSIF_SWITCH_IN), 2)) {
        // update prev_in from WOU_REGISTER
        memcpy(&(gpio->prev_in), wou_reg_ptr(&w_param, SSIF_BASE + SSIF_SWITCH_IN), 2);
        // rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: switch_in(0x%04X)\n", gpio->prev_in);
        for (i=0; i < gpio->num_in; i++) {
          *(gpio->in[i]) = ((gpio->prev_in) >> i) & 0x01;
        }
    }
 
    // read SSIF_SWITCH_EN
    if (memcmp(&prev_r_switch_en, wou_reg_ptr(&w_param, SSIF_BASE + SSIF_SWITCH_EN), 1)) {
        memcpy(&r_switch_en, wou_reg_ptr(&w_param, SSIF_BASE + SSIF_SWITCH_EN), 1);
        // rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: switch_en(0x%02X) prev_switch_en(0x%02X)\n", 
        //                               r_switch_en, prev_r_switch_en);
        prev_r_switch_en = r_switch_en;
    }

    // read SSIF_INDEX_LOCK
    memcpy(&r_index_lock, wou_reg_ptr(&w_param, SSIF_BASE + SSIF_INDEX_LOCK), 1);
    if (r_index_lock != prev_r_index_lock) {
        // rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: index_lock(0x%02X) prev_index_lock(0x%02X)\n", 
        //                                 r_index_lock, prev_r_index_lock);
        prev_r_index_lock = r_index_lock;
    }

  // process GPIO.OUT
  data[0] = 0;
  for (i=0; i < gpio->num_out; i++) {
    data[0] |= ((*(gpio->out[i]) & 1) << i);
  }
  if (data[0] != gpio->prev_out) {
    gpio->prev_out = data[0];
    // issue a WOU_WRITE while got new GPIO.OUT value
    wou_cmd (&w_param,
                   WB_WR_CMD,
                   GPIO_BASE | GPIO_OUT,
                   1,
                   data);
//    assert (ret==0);

    /* wou.gpio.out.00 is mapped to SVO-ON */
    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SIF_EN, servo interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    if (*(gpio->out[0])) {
        data[0] = 3;    // SVO-ON
    } else {
        data[0] = 4;    // SVO-OFF
    }
    wou_cmd (&w_param,
                   WB_WR_CMD,
                   (JCMD_BASE | JCMD_CTRL),
                   1,
                   data);
    /*if (ret) {
        rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: writing JCMD_CTRL\n");
        return;
    }*/

    wou_flush(&w_param);
  }

  /* begin: process motion synchronized input */
/* handle with pin index
 * sync_io_data = 0;
  for (i=0; i < m_control->num_sync_in; i++) {
      sync_io_data |= ((*(m_control->sync_in[i]) & 1) << i);
        if (sync_io_data != m_control->prev_in) {
      m_control->prev_in = sync_io_data
  }*/
  if(*(m_control->sync_in_trigger) != 0) {
      printf("sync_input detected pin(%d) wait_type(%d) timeout(%f)\n",*(m_control->sync_in),
              *(m_control->wait_type),*(m_control->timeout));


      // write a wou frame for sync output into command FIFO
    /*
         ret = wou_cmd (&w_param,
                        (WB_WR_CMD | WB_AI_MODE),
                        GPIO_BASE | GPIO_OUT,
                        1,
                        data);
      assert (ret==0);
      wou_flush(&w_param);*/
      *(m_control->sync_in_trigger) = 0;

  }

  /* end: process motion synchronized input */

  /* begin: process motion synchronized output */
  sync_io_data = 0;
  for (i=0; i < m_control->num_sync_out; i++) {
      sync_io_data |= ((*(m_control->sync_out[i]) & 1) << i);
  }

  if (sync_io_data != m_control->prev_out) {
      m_control->prev_out = sync_io_data;

    // write a wou frame for sync input into command FIFO
/*
    ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_AI_MODE),
                   GPIO_BASE | GPIO_OUT,
                   1,
                   data);
    assert (ret==0);

    wou_flush(&w_param);
*/
  }

  /* end: process motion synchronized output */
  /* point at stepgen data */
  stepgen = arg;
 
#if (TRACE!=0)
  if (*stepgen->enable) {
    DPS("%11u", _dt); // %11u: '#' + %10s
    _dt ++;
  }
#endif

  // num_chan: 4, calculated from step_type;
  /* loop thru generators */
  
    r_rst_pos = 0;
    r_switch_en = 0;
    r_index_en = prev_r_index_en;
    for (n = 0; n < num_chan; n++) {
      if (*stepgen->home_state != HOME_IDLE)  {
          hal_s32_t switch_pos_tmp;
          hal_s32_t index_pos_tmp;
          /* update home switch and motor index position while homing */
          memcpy ((void *)&switch_pos_tmp, 
                  wou_reg_ptr(&w_param, SSIF_BASE + SSIF_SWITCH_POS + n*4), 
                  4);
          memcpy ((void *)&index_pos_tmp, 
                  wou_reg_ptr(&w_param, SSIF_BASE + SSIF_INDEX_POS + n*4), 
                  4);
          
          wou_cmd (&w_param,
                   WB_RD_CMD,
                   (SSIF_BASE + SSIF_SWITCH_POS + n*4),
                   4,
                   data);

          wou_cmd (&w_param,
                   WB_RD_CMD,
                   (SSIF_BASE + SSIF_INDEX_POS + n*4),
                   4,
                   data);


          *(stepgen->switch_pos) = switch_pos_tmp * stepgen->scale_recip;
          *(stepgen->index_pos) = index_pos_tmp * stepgen->scale_recip;
          
          //debug: rtapi_print_msg(RTAPI_MSG_DBG, "WOU: j[%d] switch_pos_tmp(0x%08X) switch_pos(%f)\n", 
          //debug:                                 n, switch_pos_tmp, *(stepgen->switch_pos));

          /* check if we should wait for HOME Switch Toggle */
          if ((*stepgen->home_state == HOME_INITIAL_BACKOFF_WAIT) ||
              (*stepgen->home_state == HOME_INITIAL_SEARCH_WAIT) ||
              (*stepgen->home_state == HOME_FINAL_BACKOFF_WAIT) ||
              (*stepgen->home_state == HOME_RISE_SEARCH_WAIT) ||
              (*stepgen->home_state == HOME_FALL_SEARCH_WAIT)) 
          {
              if (stepgen->prev_home_state != *stepgen->home_state) {
                  // set r_switch_en to locate SWITCH_POS
                  // r_switch_en is reset by HW 
                  r_switch_en |= (1<<n);
              }
          }
          
          /* check if we should wait for Motor Index Toggle */
          if (*stepgen->home_state == HOME_INDEX_SEARCH_WAIT) {
              if (stepgen->prev_home_state != *stepgen->home_state) {
                  // set r_index_en while getting into HOME_INDEX_SEARCH_WAIT state
                  r_index_en |= (1<<n);
              } else if (r_index_lock & (1<<n)) {
                  // the motor index is locked
                  // reset r_index_en by SW
                  r_index_en &= (~(1<<n));    // reset index_en[n]
                  *(stepgen->index_enable) = 0;
                  // rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: index_en(0x%02X) prev_r_index_en(0x%02X)\n", 
                  //                                 r_index_en, prev_r_index_en);
                  // rtapi_print_msg(RTAPI_MSG_DBG, "STEPGEN: index_pos(%f)\n", 
                  //                                 *(stepgen->index_pos));
              }
          }
          
          if (stepgen->prev_home_state == HOME_IDLE) {
              // * set to clear PULSE_POS, ENC_POS, SWITCH_POS, INDEX_POS at
              //   beginning of homing
              // * reset one cycle after setting this register
              r_rst_pos |= (1<<n);
          }
          
          stepgen->prev_home_state = *stepgen->home_state;
      }
      /* move on to next channel */
      stepgen++;
    }
    
  /* check if we should clear SWITCH/INDEX positions for HOMING */
  if (r_rst_pos != 0) {
    // issue a WOU_WRITE 
    wou_cmd (&w_param,
                   WB_WR_CMD,
                   SSIF_BASE | SSIF_RST_POS,
                   1,
                   &r_rst_pos);
    // fprintf(stderr, "wou: r_rst_pos(0x%x)\n", r_rst_pos);
//    assert (ret==0);
    wou_flush(&w_param);
  }
  
  /* check if we should enable HOME Switch Detection */
  if (r_switch_en != 0) {
    // issue a WOU_WRITE 
    wou_cmd (&w_param,
                   WB_WR_CMD,
                   SSIF_BASE | SSIF_SWITCH_EN,
                   1,
                   &r_switch_en);
    // fprintf(stderr, "wou: r_switch_en(0x%x)\n", r_switch_en);
//    assert (ret==0);
    wou_flush(&w_param);
  }
  
  /* check if we should enable MOTOR Index Detection */
  if (r_index_en != prev_r_index_en) {
    // issue a WOU_WRITE 
    wou_cmd (&w_param,
                   WB_WR_CMD,
                   SSIF_BASE | SSIF_INDEX_EN,
                   1,
                   &r_index_en);
    // fprintf(stderr, "wou: r_index_en(0x%x)\n", r_index_en);
//    assert (ret==0);
    wou_flush(&w_param);
    prev_r_index_en = r_index_en;
  }

    //bug: // replace "bp_reg_update"
    //bug: // send WB_RD_CMD to read registers back
    //bug: wou_cmd (&w_param,
    //bug:        WB_RD_CMD,
    //bug:        (SSIF_BASE + SSIF_PULSE_POS),
    //bug:        34,  // SSIF_PULSE_POS, SSIF_ENC_POS, SSIF_SWITCH_IN
    //bug:        data);
    //bug: wou_flush(&w_param);
    

    stepgen = arg;
    for (n = 0; n < num_chan; n++) {
        /* update registers from FPGA */
        memcpy ((void *)stepgen->pulse_pos, 
                wou_reg_ptr(&w_param, SSIF_BASE + SSIF_PULSE_POS + n*4), 
                4);
        memcpy ((void *)stepgen->enc_pos, 
                wou_reg_ptr(&w_param, SSIF_BASE + SSIF_ENC_POS + n*4),
                4);

    /**
     * Use pulse_pos because there's no enc_pos for stepping motor driver.
     * Also, there's no too much difference between pulse_pos and enc_pos
     * for servo drivers.
     **/
    *(stepgen->pos_fb) = *(stepgen->pulse_pos) * stepgen->scale_recip;

    /* test for disabled stepgen */
    if (*stepgen->enable == 0) {
        /* AXIS not PWR-ON */
        /* keep updating parameters for better performance */
        stepgen->scale_recip = 1.0 / stepgen->pos_scale;
        
        /* set velocity to zero */
        stepgen->freq = 0;
        /* and skip to next one */
        stepgen++;
        continue;
    }

    
    //
    // first sanity-check our maxaccel and maxvel params
    //

    // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds
    {
        double min_ns_per_step = stepgen->step_len + stepgen->dir_hold_dly;
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

        if (stepgen->maxvel == 0.0) {
            maxvel = physical_maxvel;
        } else {
            maxvel = stepgen->maxvel;
        }
    }

    /* at this point, all scaling, limits, and other parameter
       changes have been handled - time for the main control */
    if ( stepgen->pos_mode ) {
      DPS ("  %17.7f", *stepgen->pos_cmd);

// took from src/hal/drivers/mesa-hostmot2/stepgen.c:
// Here's the stepgen position controller.  It uses first-order
// feedforward and proportional error feedback.  This code is based
// on John Kasunich's software stepgen code.

      // calculate feed-forward velocity in machine units per second
      ff_vel = ((*stepgen->pos_cmd) - stepgen->prev_pos_cmd) * recip_dt;

      stepgen->prev_pos_cmd = (*stepgen->pos_cmd);

      velocity_error = (stepgen->vel_fb) - ff_vel;

      // Do we need to change speed to match the speed of position-cmd?
      // If maxaccel is 0, there's no accel limit: fix this velocity error
      // by the next servo period!  This leaves acceleration control up to
      // the trajectory planner.
      // If maxaccel is not zero, the user has specified a maxaccel and we
      // adhere to that.
      if (velocity_error > 0.0) {
          if (stepgen->maxaccel == 0) {
              match_accel = -velocity_error * recip_dt;
          } else {
              match_accel = -stepgen->maxaccel;
          }
      } else if (velocity_error < 0.0) {
          if (stepgen->maxaccel == 0) {
              match_accel = velocity_error * recip_dt;
          } else {
              match_accel = stepgen->maxaccel;
          }
      } else {
          match_accel = 0;
      }

      if (match_accel == 0) {
          // vel is just right, dont need to accelerate
          seconds_to_vel_match = 0.0;
      } else {
          seconds_to_vel_match = -velocity_error / match_accel;
      }

      // compute expected position at the time of velocity match
      // Note: this is "feedback position at the beginning of the servo period after we attain velocity match"
      {
          double avg_v;
          avg_v = (ff_vel + stepgen->vel_fb) * 0.5;
          // position_at_match = *stepgen->pos_fb + (avg_v * (seconds_to_vel_match + dt));
          position_at_match = stepgen->cur_pos + (avg_v * (seconds_to_vel_match + dt));
      }

      // Note: this assumes that position-cmd keeps the current velocity
      position_cmd_at_match = *stepgen->pos_cmd + (ff_vel * seconds_to_vel_match);
      error_at_match = position_at_match - position_cmd_at_match;

      if (seconds_to_vel_match < dt) {
          // we can match velocity in one period
          // try to correct whatever position error we have
          // orig: velocity_cmd = ff_vel - (0.5 * error_at_match * recip_dt);
          // velocity_cmd = (*stepgen->pos_cmd - *stepgen->pos_fb) * recip_dt;
          velocity_cmd = (*stepgen->pos_cmd - stepgen->cur_pos) * recip_dt;

          // apply accel limits?
          if (stepgen->maxaccel > 0) {
              if (velocity_cmd > (stepgen->vel_fb + (stepgen->maxaccel * dt))) {
                  velocity_cmd = stepgen->vel_fb + (stepgen->maxaccel * dt);
              } else if (velocity_cmd < (stepgen->vel_fb - (stepgen->maxaccel * dt))) {
                  velocity_cmd = stepgen->vel_fb - (stepgen->maxaccel * dt);
              }
          }

      } else {
          // we're going to have to work for more than one period to match velocity
          // FIXME: I dont really get this part yet

          double dv;
          double dp;

          /* calculate change in final position if we ramp in the opposite direction for one period */
          dv = -2.0 * match_accel * dt;
          dp = dv * seconds_to_vel_match;

          /* decide which way to ramp */
          if (fabs(error_at_match + (dp * 2.0)) < fabs(error_at_match)) {
              match_accel = -match_accel;
          }

          /* and do it */
          velocity_cmd = stepgen->vel_fb + (match_accel * dt);
      }

      new_vel = velocity_cmd;
      /* apply frequency limit */      
      if (new_vel > maxvel) {
        new_vel = maxvel;
      } else if (new_vel < -maxvel) {
        new_vel = -maxvel;
      }
      stepgen->vel_fb = new_vel;
      
    } else {
        // do not support velocity mode yet
        assert(0);
        /* end of velocity mode */
    }
    
    // calculate WOU commands
    // each AXIS cycle is 1310720ns, 32768 ticks of 40ns(25MHz) clocks
    wou_pos_cmd = (int) (new_vel * stepgen->pos_scale * dt);
    stepgen->accum += wou_pos_cmd;
      
    assert (wou_pos_cmd < 8192);
    assert (wou_pos_cmd > -8192);
    
    {
        uint16_t sync_cmd;

        // SYNC_JNT: opcode for SYNC_JNT command
        // DIR_P: Direction, (positive(1), negative(0))
        // POS_MASK: relative position mask

      if (wou_pos_cmd >= 0) {
        // data[0]: MSB {dir, pos[12:8]}, dir(1): positive
          sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);
          // data[2*n    ] = (uint8_t) (((wou_pos_cmd >> 8) & 0x1F) | 0x20);
          // data[2*n + 1] = (uint8_t) (wou_pos_cmd & 0xFF);
      } else {
          // data[0]: MSB {dir, pos[12:8]}, dir(0): negative
          wou_pos_cmd *= -1;
          sync_cmd = SYNC_JNT | DIR_N | (POS_MASK & wou_pos_cmd);
          // data[2*n    ] = (uint8_t) ((wou_pos_cmd >> 8) & 0x1F);
          // data[2*n + 1] = (uint8_t) (wou_pos_cmd & 0xFF);
      }
          memcpy (data+n*sizeof(uint16_t), &sync_cmd, sizeof(uint16_t));


      // fprintf (wou_fh, "\tJ%d: data[]: <0x%02x><0x%02x>\n", n, data[0], data[1]);
      if (n == (num_chan - 1)) {
        // send to WOU when all axes commands are generated
        wou_cmd (&w_param,
                       WB_WR_CMD,
                       (JCMD_BASE | JCMD_SYNC_CMD),
                       2*num_chan,
                       data);
//        assert (ret==0);
      }
    }

    // *(stepgen->pos_fb) = stepgen->accum * stepgen->scale_recip;
    // DPS ("%15.7f%15.7f%15.7f", *stepgen->pos_fb, match_accel, new_vel);
    stepgen->cur_pos = stepgen->accum * stepgen->scale_recip;
    // *(stepgen->pos_fb) = stepgen->cur_pos;
    DPS ("%15.7f%15.7f%15.7f%15.7f%7d", 
          stepgen->cur_pos, 
          *stepgen->pos_fb, 
          match_accel,
          new_vel, 
          *stepgen->home_state);
  
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

static int export_gpio (gpio_t *addr)
{
  int i, retval, msg;

  /* This function exports a lot of stuff, which results in a lot of
     logging if msg_level is at INFO or ALL. So we save the current value
     of msg_level and restore it later.  If you actually need to log this
     function's actions, change the second line below */
  msg = rtapi_get_msg_level();
  // rtapi_set_msg_level(RTAPI_MSG_WARN);
  rtapi_set_msg_level(RTAPI_MSG_ALL);

  /* export pin for counts captured by wou_update() */
  for (i = 0; i < 16; i++) {
    retval = hal_pin_bit_newf(HAL_OUT, &(addr->in[i]), comp_id, 
                              "wou.gpio.in.%02d", i);
    if (retval != 0) { return retval; }
    *(addr->in[i]) = 0;
  }

  for (i = 0; i < 8; i++) {
    retval = hal_pin_bit_newf(HAL_IN, &(addr->out[i]), comp_id, 
                              "wou.gpio.out.%02d", i);
    if (retval != 0) {return retval;}
    *(addr->out[i]) = 0;
  }
  
  /* set default parameter values */
  addr->num_in = 16;
  addr->num_out = 8;
  addr->prev_out = 0;
  addr->prev_in = 1;
  // gpio.in[0] is SVO-ALM, which is low active;
  // set "prev_in[0]" as 1 also
  *(addr->in[0]) = 1;

  /* restore saved message level */
  rtapi_set_msg_level(msg);
  return 0;
} // export_gpio ()

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
//    retval = hal_param_s32_newf(HAL_RO, &(addr->rawcount), comp_id,
//	"wou.stepgen.%d.rawcounts", num);
//    if (retval != 0) { return retval; }
    
    /* export pin for counts captured by wou_update() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->pulse_pos), comp_id,
	"wou.stepgen.%d.pulse_pos", num);
    if (retval != 0) { return retval; }
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc_pos), comp_id,
	"wou.stepgen.%d.enc_pos", num);
    if (retval != 0) { return retval; }
    
    /* export pin for scaled switch position (absolute motor position) */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->switch_pos), comp_id,
	"wou.stepgen.%d.switch-pos", num);
    if (retval != 0) { return retval; }
    
    /* export pin for scaled index position (absolute motor position) */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->index_pos), comp_id,
	"wou.stepgen.%d.index-pos", num);
    if (retval != 0) { return retval; }
    
    /* export pin for index_enable */
    retval = hal_pin_bit_newf(HAL_IO, &(addr->index_enable), comp_id,
	"wou.stepgen.%d.index_enable", num);
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
    
    /* export parameter to obtain homing state */
    retval = hal_pin_s32_newf(HAL_IN, &(addr->home_state), comp_id,
        "wou.stepgen.%d.home-state", num);
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
    /* init the step generator core to zero output */
    addr->cur_pos = 0.0;
    addr->accum = 0;

    addr->vel_fb = 0;
    addr->prev_pos_cmd = 0;
    addr->prev_pos = 0;
    addr->sum_err_0 = 0;
    addr->sum_err_1 = 0;

    *(addr->enable) = 0;
    /* other init */
    addr->printed_error = 0;
    // addr->old_pos_cmd = 0.0;
    /* set initial pin values */
    *(addr->pulse_pos) = 0;
    *(addr->enc_pos) = 0;
    *(addr->pos_fb) = 0.0;
    *(addr->switch_pos) = 0.0;
    *(addr->index_pos) = 0.0;
    if ( pos_mode ) {
	*(addr->pos_cmd) = 0.0;
    } else {
	*(addr->vel_cmd) = 0.0;
    }
    *(addr->home_state) = HOME_IDLE;
    addr->prev_home_state = HOME_IDLE;

    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}


static int export_m_control (m_control_t *m_control)
{
  int i, retval, msg;
  /* This function exports a lot of stuff, which results in a lot of
     logging if msg_level is at INFO or ALL. So we save the current value
     of msg_level and restore it later.  If you actually need to log this
     function's actions, change the second line below
*/
  msg = rtapi_get_msg_level();
  // rtapi_set_msg_level(RTAPI_MSG_WARN);
  rtapi_set_msg_level(RTAPI_MSG_ALL);

/*   export pin for counts captured by wou_update()*/
/* replace with pin index
  for (i = 0; i < num_sync_in; i++) {
    retval = hal_pin_bit_newf(HAL_IN, &(m_control->sync_in[i]), comp_id,
                              "wou.sync.in.%02d", i);
    if (retval != 0) { return retval; }
    *(m_control->sync_in[i]) = 0;
  }
*/
  retval = hal_pin_bit_newf(HAL_IO,&(m_control->sync_in_trigger), comp_id, "wou.sync.in.trigger");
  *(m_control->sync_in_trigger) = 0;  // pin index must not beyond index

  retval = hal_pin_u32_newf(HAL_IN,&(m_control->sync_in), comp_id, "wou.sync.in.index");
  *(m_control->sync_in) = 0;  // pin index must not beyond index

  retval = hal_pin_u32_newf(HAL_IN,&(m_control->wait_type), comp_id,
                                  "wou.sync.in.wait_type");
  if (retval != 0) { return retval;}
  *(m_control->wait_type) = 0;
  retval = hal_pin_float_newf(HAL_IN,&(m_control->timeout), comp_id,
                                    "wou.sync.in.timeout");
  if (retval != 0) { return retval;}
  *(m_control->timeout) = 0.0;

  for (i = 0; i < num_sync_out; i++) {
    retval = hal_pin_bit_newf(HAL_IN, &(m_control->sync_out[i]), comp_id,
                              "wou.sync.out.%02d", i);
    if (retval != 0) { return retval; }
    *(m_control->sync_out[i]) = 0;
  }
  m_control->num_sync_in = num_sync_in;
  m_control->num_sync_out = num_sync_out;
  m_control->prev_out = 0;
//  m_control->prev_in = 0;

/*
  *(m_control->)
  for (i = 0; i < 9; i++) {
    retval = hal_pin_bit_newf(HAL_IN, &(addr->out[i]), comp_id,
                              "wou.gpio.out.%02d", i);
    if (retval != 0) {return retval;}
    *(addr->out[i]) = 0;
  }
*/

/*   restore saved message level*/
  rtapi_set_msg_level(msg);
  return 0;
} // export_gpio ()


// vim:sw=4:sts=4:et:
