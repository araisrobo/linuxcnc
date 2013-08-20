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

	insmod stepgen ctrl_type=p,p,v,p

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

#ifdef RTAPI
#warning "this for user space only"
#else
// SIM

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
#include <time.h>

#include <wou.h>
#include <wb_regs.h>
#include <mailtag.h>
#include "sync_cmd.h"

#define REQUEST_TICK_SYNC_AFTER 500 // after 500 tick count, request risc to sync tick count
#define MAX_CHAN 8
#define MAX_STEP_CUR 255
#define PLASMA_ON_BIT 0x02
#define FRACTION_BITS 16
#define FIXED_POINT_SCALE   65536.0     // (double (1 << FRACTION_BITS))
#define FRACTION_MASK 0x0000FFFF
#define PID_LOOP 8
#define SON_DELAY_TICK  1500

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

#undef  DEBUG

/* module information */
MODULE_AUTHOR("Yi-Shin Li");
MODULE_DESCRIPTION("Wishbone Over USB for EMC HAL");
MODULE_LICENSE("GPL");

const char *ctrl_type[MAX_CHAN] =
    { " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(ctrl_type, MAX_CHAN,
		      "control type (pos or vel) for up to 8 channels");

const char *pulse_type[MAX_CHAN] =
    { " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(pulse_type, MAX_CHAN,
		      "pulse type (AB-PHASE(a) or STEP-DIR(s)) for up to 8 channels");

const char *enc_type[MAX_CHAN] =
    { " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(enc_type, MAX_CHAN,
		      "encoder type (REAL(r) or LOOP-BACK(l)) for up to 8 channels");


const char *bits = "\0";
RTAPI_MP_STRING(bits, "FPGA bitfile");

const char *bins = "\0";
RTAPI_MP_STRING(bins, "RISC binfile");

int alarm_en = -1;
RTAPI_MP_INT(alarm_en, "hardware alarm dection mode");

// int pulse_type = -1;
// RTAPI_MP_INT(pulse_type, "WOU Register Value for pulse type");
// 
// int enc_type = -1;
// RTAPI_MP_INT(enc_type, "WOU Register Value for encoder type");

int servo_period_ns = -1;   // init to '-1' for testing valid parameter value
RTAPI_MP_INT(servo_period_ns, "used for calculating new velocity command, unit: ns");

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
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j4_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[4]");

const char *j5_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j5_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[5]");

const char *j6_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j6_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[6]");

const char *j7_pid_str[NUM_PID_PARAMS] =
        { "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
RTAPI_MP_ARRAY_STRING(j7_pid_str, NUM_PID_PARAMS,
                      "pid parameters for joint[7]");


const char *max_vel_str[MAX_CHAN] =
    { "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" };
RTAPI_MP_ARRAY_STRING(max_vel_str, MAX_CHAN,
                      "max velocity value for up to 8 channels");

const char *max_accel_str[MAX_CHAN] =
    { "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" };
RTAPI_MP_ARRAY_STRING(max_accel_str, MAX_CHAN,
                      "max acceleration value for up to 8 channels");

const char *max_jerk_str[MAX_CHAN] =
    { "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" };
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

const char *pattern_type_str ="NO_TEST"; // ANALOG_0: analog input0
RTAPI_MP_STRING(pattern_type_str,
                "indicate test pattern type");

// const char *probe_pin_type="DIGITAL_PIN"; // ANALOG_IN
// RTAPI_MP_STRING(probe_pin_type,
//                 "indicate probing type");

const char *probe_config= "0x00010000";         // probing input channel
RTAPI_MP_STRING(probe_config,
                "probe config for RISC");

//const char *probe_decel_cmd= "0";         // deceleration command for probing in user-unit/s
//RTAPI_MP_STRING(probe_decel_cmd,
//                "deceleration for probing");

const char *probe_analog_ref_level= "2048";
RTAPI_MP_STRING(probe_analog_ref_level,
                "indicate probing level used by analog probing");

// int alr_output = 0x00000000;
// RTAPI_MP_INT(alr_output, "Digital Output when E-Stop presents");
const char *alr_output= "0";
RTAPI_MP_STRING(alr_output,
                "Digital Output when E-Stop presents");

static int test_pattern_type = 0;  // use dbg_pat_str to update dbg_pat_type

// static const char *board = "7i43u";
// static const char wou_id = 0;
// static wou_param_t w_param;

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
    int64_t     rawcount;       /* precision: 64.16; accumulated pulse sent to FPGA */
    hal_s32_t   *rawcount32;	/* 32-bit integer part of rawcount */
    hal_bit_t   prev_enable;
    uint32_t    son_delay;      /* eanble delay tick for svo-on of ac-servo drivers */
    hal_bit_t   *enable;		/* pin for enable stepgen */
    hal_u32_t   step_len;		/* parameter: step pulse length */
    int pos_mode;		/* 1 = position command mode, 0 = velocity command mode */
    hal_s32_t *pulse_pos;	/* pin: pulse_pos to servo drive, captured from FPGA */
    hal_s32_t *enc_pos;		/* pin: encoder position from servo drive, captured from FPGA */
    hal_float_t *rpm;	        /* pin: velocity command (pos units/sec) */
    hal_float_t pulse_per_rev;	/* param: number of pulse per revolution */
    

    hal_float_t *switch_pos;	/* pin: scaled home switch position in absolute motor position */
    int32_t   switch_pos_i;
    hal_float_t *index_pos;	/* pin: scaled index position in absolute motor position */
    hal_bit_t *index_enable;	/* pin for index_enable */
    hal_float_t pos_scale;	/* param: steps per position unit */
    double scale_recip;		/* reciprocal value used for scaling */
    hal_float_t *vel_cmd;	/* pin: velocity command (pos units/sec) */
    double prev_vel_cmd;        /* prev vel cmd: previous velocity command */
    double      pos_cmd_s;	/* saved pos_cmd at rising edge of usb_busy */
    hal_float_t *pos_cmd;	/* pin: position command (position units) */
    double prev_pos_cmd;        /* prev pos_cmd: previous position command */
    hal_float_t *probed_pos;
    hal_bit_t *align_pos_cmd;
    hal_float_t *pos_fb;	/* pin: position feedback (position units) */
    hal_float_t *vel_fb;        /* pin: velocity feedback */
    double      prev_pos_fb;    /* previous position feedback for calculating vel_fb */
    hal_bit_t *ferror_flag;     /* following error flag from risc */
    hal_float_t freq;		/* param: frequency command */
    hal_float_t maxvel;		/* param: max velocity, (pos units/sec) */
    hal_float_t maxaccel;	/* param: max accel (pos units/sec^2) */
    int printed_error;		/* flag to avoid repeated printing */
    uint32_t    pulse_maxv;     /* max-vel in pulse */
    uint32_t    pulse_maxa;     /* max-accel in pulse */
    uint32_t    pulse_maxj;     /* max-jerk  in pulse */
    int32_t     pulse_vel;      /* velocity in pulse */
    int32_t     pulse_accel;    /* accel in pulse */
    int32_t     pulse_jerk;     /* jerk in pulse */

    /* motion type be set */
    int32_t motion_type;          /* motion type wrote to risc */
    
    hal_s32_t     *cmd_fbs;     /* position command retained by RISC (unit: pulse) */
    hal_float_t   *jog_scale;   /* for risc jogging */
    hal_float_t   *jog_vel;

    hal_bit_t     *jog_enable;
    int8_t        prev_jog_enable;
    double       prev_jog_vel;
} stepgen_t;
// #pragma pack(pop)   /* restore original alignment from stack */

typedef struct {
    // Analog input: 0~4.096VDC, up to 16 channel
    hal_s32_t *in[16];
    // TODO: may add *out[] here
} analog_t;

// machine_control_t:
typedef struct {
    hal_bit_t   *usb_busy;
    hal_bit_t   usb_busy_s;
    hal_bit_t   *ignore_ahc_limit;
    hal_bit_t   *align_pos_cmd;
    hal_bit_t   *ignore_host_cmd;
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
    hal_u32_t *sync_in;		//
    hal_u32_t *wait_type;
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
    /* test pattern  */
    hal_s32_t *test_pattern;
    /* MPG */
    hal_s32_t *mpg_count;
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

    hal_bit_t   *teleop_mode;
    hal_bit_t   *coord_mode;
//    uint8_t     motion_mode;
//    uint8_t     motion_mode_prev;
//    uint8_t     pid_enable;

    hal_bit_t   *update_pos_ack;
} machine_control_t;

/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t *stepgen_array;
static analog_t *analog;
static machine_control_t *machine_control;
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

static int export_stepgen(int num, stepgen_t * addr,
			  int pos_mode);
static int export_analog(analog_t * addr);
static int export_machine_control(machine_control_t * machine_control);
static void update_freq(void *arg, long period);
// static void update_rt_cmd(void);


void endian_swap(uint32_t  *x)
{
    *x = (*x>>24) | ((*x<<8) & 0x00FF0000) |((*x>>8) & 0x0000FF00) |(*x<<24);
}

static void write_mot_pos_cmd (uint32_t joint, int64_t mot_pos_cmd)
{
    uint16_t    sync_cmd;
    uint8_t     buf[sizeof(uint16_t)];
    int         j;

    for(j=0; j<sizeof(int64_t); j++) {
        // byte sequence for int64_t: "4, 5, 6, 7, 0, 1, 2, 3"
        sync_cmd = SYNC_DATA | ((uint8_t *)&mot_pos_cmd)[(4+j) & 0x07];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
    }

    sync_cmd = SYNC_MOT_POS_CMD | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));

    return;
}

static void write_mot_param (uint32_t joint, uint32_t addr, int32_t data)
{
    uint16_t    sync_cmd;
    uint8_t     buf[MAX_DSIZE];
    int         j;

    for(j=0; j<sizeof(int32_t); j++) {
        sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ADDR(addr) | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));

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
          }
      }
      /* write command */
        sync_cmd = SYNC_USB_CMD | *mc->usb_cmd; // TODO: set in control.c or do homing.c
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
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
      }
      data = (int32_t)(*mc->usb_cmd_param[1] * pos_scale * dt * FIXED_POINT_SCALE);
      fprintf(stderr,"get home command param1 (0x%0d) joint\n", data);
      for(j=0; j<sizeof(int32_t); j++) {
          sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
          memcpy(buf, &sync_cmd, sizeof(uint16_t));
      }
      data = (int32_t)(*mc->usb_cmd_param[2] * pos_scale * dt * FIXED_POINT_SCALE);
      fprintf(stderr,"get home command param2 (0x%0d) joint\n", data);
      for(j=0; j<sizeof(int32_t); j++) {
          sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
          memcpy(buf, &sync_cmd, sizeof(uint16_t));
      }
      /* write command */
        sync_cmd = SYNC_USB_CMD | *mc->usb_cmd; // TODO: set in control.c or do homing.c
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
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
            }
      }
      /* write command */
        sync_cmd = SYNC_USB_CMD | *mc->usb_cmd; // TODO: set in control.c or do homing.c
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
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
    }

    sync_cmd = SYNC_MACH_PARAM | PACK_MACH_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));


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

    /* test for bitfile string: bits */
    if ((bits == 0) || (bits[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"WOU: ERROR: no fpga bitfile string: bits\n");
	return -1;
    } else {
	// initialize FPGA with bitfile(bits)
    }

    /* test for risc image file string: bins */
    if ((bins == 0) || (bins[0] == '\0')) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"WOU: ERROR: no risc binfile string: bins\n");
	return -1;
    } else {
	// programming risc with binfile(bins)
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
        
        // set crc counter callback function
        
        // set rt_cmd callback function
    }

    if(alarm_en != -1) {
        if (alarm_en == 1) {
            data[0] = GPIO_ALARM_EN;
        } else if (alarm_en == 0) {
            data[0] = 0;
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "WOU: ERROR: unknown alarm_en value: %d\n", alarm_en);
            return -1;
        }
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOU: ERROR: no alarm_en\n");
        return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "ALARM_EN: 0x%02X\n", data[0]);
	

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
    for (n = 0; n < MAX_CHAN && (ctrl_type[n][0] != ' ') ; n++) {
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
    
    DPS("num_joints: %d\n", num_joints);
    
    assert (num_joints <=6);  // support up to 6 joints for USB/7i43 
    if (num_joints == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: no channels configured\n");
	return -1;
    }

    /* to clear PULSE/ENC/SWITCH/INDEX positions for all joints*/
    // issue a WOU_WRITE to RESET SSIF position registers
    data[0] = (1 << num_joints) - 1;  // bit-map-for-num_joints
    // issue a WOU_WRITE to clear SSIF_RST_POS register
    data[0] = 0x00;
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

    // configure NUM_JOINTS after all joint parameters are set

    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo/stepper interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    // TODO: RTL: remove SSIF_EN (always enable SSIF)
    // FIXME: WORKAROUND: always enable SSIF_EN by SW
    // SSIF_EN = 1
    data[0] = 2;
    // RISC ON
    data[0] = 1;        
    /* have good config info, connect to the HAL */
    comp_id = hal_init("wou_sim");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"STEPGEN: ERROR: hal_init(wou_sim) failed\n");
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
    
    /* configure motion parameters */
    for(n=0; n<num_joints; n++) {
        /* compute fraction bits */
        // compute proper fraction bit for command
        // compute fraction bit for velocity
        // accurate 0.0001 mm
        pos_scale   = fabs(atof(pos_scale_str[n]));
        max_vel     = atof(max_vel_str[n]);
        max_accel   = atof(max_accel_str[n]);
        max_jerk    = atof(max_jerk_str[n]);
        assert (max_vel > 0);
        assert (max_accel > 0);
        assert (max_jerk > 0);
        
        /* config MAX velocity */
        // * 1.05 : add 5% head room
        immediate_data = (uint32_t)((max_vel * pos_scale * dt * FIXED_POINT_SCALE) * 1.05);    
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_vel(%d) = %f*%f*%f*%f\n", 
                        n, immediate_data, max_vel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data>0);
        write_mot_param (n, (MAX_VELOCITY), immediate_data);
        stepgen_array[n].pulse_maxv = immediate_data;
        
        /* config acceleration */
        // * 1.05 : add 5% head room
        immediate_data = (uint32_t)((max_accel * pos_scale * dt * FIXED_POINT_SCALE * dt) * 1.05 );
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_accel(%d) = %f*%f*(%f^2)*(%f)\n",
                        n, immediate_data, max_accel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_ACCEL), immediate_data);
        stepgen_array[n].pulse_maxa = immediate_data;

        /* config acceleration recip */
        immediate_data = (uint32_t)(FIXED_POINT_SCALE / (max_accel * pos_scale * dt * dt * 1.05));
        rtapi_print_msg(RTAPI_MSG_DBG, 
                        "j[%d] max_accel_recip(%d) = (%f/(%f*%f*(%f^2)))\n",
                        n, immediate_data, FIXED_POINT_SCALE, max_accel, pos_scale, dt);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_ACCEL_RECIP), immediate_data);

        /* config max jerk */
        /* in tp.c, the tc->jerk is ~2.17x of ini-jerk */
        immediate_data = (uint32_t)(3.0 * (max_jerk * pos_scale * FIXED_POINT_SCALE * dt * dt * dt));
        rtapi_print_msg(RTAPI_MSG_DBG,
                        "j[%d] max_jerk(%d) = (%f * %f * %f * %f^3)))\n",
                        n, immediate_data, FIXED_POINT_SCALE, max_jerk, pos_scale, dt);
        assert(immediate_data != 0);
        write_mot_param (n, (MAX_JERK), immediate_data);
        stepgen_array[n].pulse_maxj = immediate_data;

        /* config max following error */
        // following error send with unit pulse
        max_following_error = atof(ferror_str[n]);
        immediate_data = (uint32_t)(ceil(max_following_error * pos_scale));
        rtapi_print_msg(RTAPI_MSG_DBG, "max ferror(%d)\n", immediate_data);
        write_mot_param (n, (MAXFOLLWING_ERR), immediate_data);

    }

    // config PID parameter
    pid_str[0] = j0_pid_str;
    pid_str[1] = j1_pid_str;
    pid_str[2] = j2_pid_str;
    pid_str[3] = j3_pid_str;
    pid_str[4] = j4_pid_str;
    pid_str[5] = j5_pid_str;
    pid_str[6] = j6_pid_str;
    pid_str[7] = j7_pid_str;
    for (n=0; n < PID_LOOP; n++) {
        if (pid_str[n][0] != NULL) {
            rtapi_print_msg(RTAPI_MSG_INFO, "J%d_PID: ", n);
            rtapi_print_msg(RTAPI_MSG_INFO,"#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO\n");
            // all gains (P, I, D, FF0, FF1, FF2) varie from 0(0%) to 65535(100%)
            // all the others units are '1 pulse'
            for (i=0; i < NUM_PID_PARAMS; i++) {
                value = atof(pid_str[n][i]);
                immediate_data = (int32_t) (value);
                // P_GAIN: the mot_param index for P_GAIN value
                write_mot_param (n, (P_GAIN + i), immediate_data);
                rtapi_print_msg(RTAPI_MSG_INFO, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }

            value = 0;
            immediate_data = (int32_t) (value);
            write_mot_param (n, (ENABLE), immediate_data);
            rtapi_print_msg(RTAPI_MSG_INFO, "\n");
        }
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
    
    DPS("rtapi_app_main: end\n");

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
    int n, i;
    double physical_maxvel;	// max vel supported by current step timings & position-scal

    uint16_t sync_cmd;
    int32_t wou_pos_cmd, integer_pos_cmd;
    uint8_t data[MAX_DSIZE];    // data[]: for wou_cmd()
    uint32_t sync_out_data;

    int32_t immediate_data = 0;
#if (TRACE!=0)
    static uint32_t _dt = 0;
#endif

#ifdef DEBUG
    static int debug_msg_tick = 0;
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
        
    // TODO: confirm trajecotry planning thread is always ahead of wou
        *(machine_control->usb_busy) = 0;
        if (machine_control->usb_busy_s == 1) {
            // DP("usb_busy: end\n");
            // reload saved traj-planning command
            stepgen = arg;
            for (n = 0; n < num_joints; n++) {
                *(stepgen->pos_cmd) = stepgen->pos_cmd_s;
                stepgen ++;
            }
        }
        machine_control->usb_busy_s = 0;

    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_ALL);
    rtapi_set_msg_level(RTAPI_MSG_WARN);
    

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
        // begin: trigger sync in and wait timeout 
        sync_cmd = SYNC_DIN | PACK_IO_ID((uint32_t)*(machine_control->sync_in)) |
                                           PACK_DI_TYPE((uint32_t)*(machine_control->wait_type));
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

    // in[0] == 1 (ESTOP released)
    *(machine_control->in[0]) = 1;
    if (*(machine_control->in[0]) == 0) {
        // force updating prev_out and ignore out[] if ESTOP is pressed
        machine_control->prev_out =  *machine_control->dout0;
    }

    i = 0;
    stepgen = arg;
    for (n = 0; n < num_joints; n++) {

        if (*stepgen->enable != stepgen->prev_enable) {
            if ((stepgen->son_delay > SON_DELAY_TICK)
                 || (*stepgen->enable == 0)) {
                stepgen->son_delay = 0;
                stepgen->prev_enable = *stepgen->enable;
                stepgen->rawcount = (((int64_t) *(stepgen->enc_pos)) << FRACTION_BITS);
                write_mot_pos_cmd(n, stepgen->rawcount << (32 - FRACTION_BITS));
                write_mot_param (n, (ENABLE), (int32_t) *stepgen->enable);
            } else {
                stepgen->son_delay ++;
            }
        }


        *(stepgen->rawcount32) = (int32_t) (stepgen->rawcount >> FRACTION_BITS);


        *(stepgen->pos_fb) = (*stepgen->enc_pos) * stepgen->scale_recip;
        *(stepgen->switch_pos) = stepgen->switch_pos_i * stepgen->scale_recip;

        // update velocity-feedback only after encoder movement
        if ((*machine_control->bp_tick - machine_control->prev_bp) > 0/* ((int32_t)VEL_UPDATE_BP) */) {
            *(stepgen->vel_fb) = ((*stepgen->pos_fb - stepgen->prev_pos_fb) * recip_dt
                                  / (*machine_control->bp_tick - machine_control->prev_bp)
                                 );
            stepgen->prev_pos_fb = *stepgen->pos_fb;
            if (n == (num_joints - 1)) {
                // update bp_tick for the last joint
                machine_control->prev_bp = *machine_control->bp_tick;
                *machine_control->bp_tick += 1; // for sim, always increase bp_tick
            }
        }

	/* test for disabled stepgen */
	if (stepgen->prev_enable == 0) {
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
            stepgen->pulse_vel = 0;
            stepgen->pulse_accel = 0;
            stepgen->pulse_jerk = 0;

#ifdef DEBUG
            debug_msg_tick = 0;
#endif

            assert (i == n); // confirm the JCMD_SYNC_CMD is packed with all joints
            i += 1;
            wou_pos_cmd = 0;
            sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);

            memcpy(data + 2*n* sizeof(uint16_t), &sync_cmd,
                   sizeof(uint16_t));
            sync_cmd = 0;

            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd,
                               sizeof(uint16_t));
            if (n == (num_joints - 1)) {
                // send to WOU when all axes commands are generated
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
        if (*machine_control->update_pos_ack == 1)
        {
            (stepgen->prev_pos_cmd) = (*stepgen->pos_cmd);
            stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE * stepgen->pos_scale;
        }
	    if (*machine_control->align_pos_cmd == 1 /* || *machine_control->ignore_host_cmd */ ){
	        (stepgen->prev_pos_cmd) = (*stepgen->pos_cmd);
                stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE * stepgen->pos_scale;
                write_mot_pos_cmd(n, stepgen->rawcount << (32 - FRACTION_BITS));
                stepgen->pulse_vel = 0;
                stepgen->pulse_accel = 0;
                stepgen->pulse_jerk = 0;
	    }
	    *stepgen->vel_cmd = ((*stepgen->pos_cmd) - (stepgen->prev_pos_cmd));
	} else {
	    /* velocity command mode */
            /* NB: has to wire *pos_cmd-pin to velocity command input */
            if (fabs(*stepgen->pos_cmd) > stepgen->maxvel) {
                *stepgen->vel_cmd = (stepgen->maxvel) * dt ; 
                if (*stepgen->pos_cmd < 0) {
                    *stepgen->vel_cmd *= -1.0;
                }
            } else {
                *stepgen->vel_cmd = (*stepgen->pos_cmd) * dt ; // notice: has to wire *pos_cmd-pin to velocity command input
            }
	}
	
        {
            int32_t pulse_accel; 
            int32_t pulse_jerk; 

            integer_pos_cmd = (int32_t)(*stepgen->vel_cmd * (stepgen->pos_scale) * FIXED_POINT_SCALE);
            
            // integer_pos_cmd is indeed pulse_vel (velocity in pulse)
            if (abs(integer_pos_cmd) > stepgen->pulse_maxv) {
                pulse_accel = integer_pos_cmd - stepgen->pulse_vel;
                pulse_jerk = pulse_accel - stepgen->pulse_accel;
                printf("j[%d], vel_cmd(%f)\n",
                        n, *stepgen->vel_cmd);
                printf("j[%d], pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
                        n, integer_pos_cmd, pulse_accel, pulse_jerk);
                printf("j[%d], PREV pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
                        n, stepgen->pulse_vel, stepgen->pulse_accel, stepgen->pulse_jerk);
                printf("j[%d], pulse_maxv(%d), pulse_maxa(%d), pulse_maxj(%d)\n",
                        n, stepgen->pulse_maxv, stepgen->pulse_maxa, stepgen->pulse_maxj);
            }
            assert (abs(integer_pos_cmd) <= stepgen->pulse_maxv);
            pulse_accel = integer_pos_cmd - stepgen->pulse_vel;
            /* TODO: there's S-CURVE decel bug in tp.c; enable the
             * pulse_maxa assertion after resolving that bug */
            // TODO: assert (abs(pulse_accel) <= stepgen->pulse_maxa);
            pulse_jerk = pulse_accel - stepgen->pulse_accel;
            //pulse_maxj: if (abs(pulse_jerk) > stepgen->pulse_maxj) {
            //pulse_maxj:     DP("j[%d], vel_cmd(%f)\n",
            //pulse_maxj:             n, *stepgen->vel_cmd);
            //pulse_maxj:     DP("j[%d], pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
            //pulse_maxj:             n, integer_pos_cmd, pulse_accel, pulse_jerk);
            //pulse_maxj:     DP("j[%d], PREV pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
            //pulse_maxj:             n, stepgen->pulse_vel, stepgen->pulse_accel, stepgen->pulse_jerk);
            //pulse_maxj:     DP("j[%d], pulse_maxv(%d), pulse_maxa(%d), pulse_maxj(%d)\n",
            //pulse_maxj:             n, stepgen->pulse_maxv, stepgen->pulse_maxa, stepgen->pulse_maxj);
            //pulse_maxj: }
            /* TODO: there's S-CURVE decel bug in tp.c; enable the
             * pulse_maxj assertion after resolving that bug */
            // TODO: assert (abs(pulse_jerk) <= stepgen->pulse_maxj);
            stepgen->pulse_vel = integer_pos_cmd;
            stepgen->pulse_accel = pulse_accel;
            stepgen->pulse_jerk = pulse_jerk;

            /* extract integer part of command */
            wou_pos_cmd = abs(integer_pos_cmd) >> FRACTION_BITS;
            
            if(wou_pos_cmd >= 8192) {
                fprintf(stderr,"j(%d) pos_cmd(%f) prev_pos_cmd(%f) vel_cmd(%f)\n",
                        n ,
                        (*stepgen->pos_cmd), 
                        (stepgen->prev_pos_cmd), 
                        *stepgen->vel_cmd);
                fprintf(stderr,"wou_stepgen.c: wou_pos_cmd(%d) too large\n", wou_pos_cmd);
                assert(0);
            }

            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            
            if (integer_pos_cmd >= 0) {
                sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);
            } else {
                sync_cmd = SYNC_JNT | DIR_N | (POS_MASK & wou_pos_cmd);
            }
            memcpy(data + (2 * n * sizeof(uint16_t)), &sync_cmd, sizeof(uint16_t));
            
            /* packing fraction part */
            wou_pos_cmd = (abs(integer_pos_cmd)) & FRACTION_MASK;
            sync_cmd = (uint16_t) wou_pos_cmd;
            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd, sizeof(uint16_t));

            stepgen->rawcount += (int64_t) integer_pos_cmd; // precision: 64.16
            stepgen->prev_pos_cmd = (((double)stepgen->rawcount * stepgen->scale_recip)/(FIXED_POINT_SCALE));
            stepgen->prev_vel_cmd = *stepgen->vel_cmd;
            *stepgen->rpm = *stepgen->vel_cmd 
                            * stepgen->pos_scale 
                            / stepgen->pulse_per_rev
                            * recip_dt * 60.0;

            // pos_mode(1): position mode: 
            if (stepgen->pos_mode == 1) {
                // for sim:
                *stepgen->enc_pos =  *stepgen->pos_cmd * stepgen->pos_scale;
                *stepgen->pulse_pos = *stepgen->enc_pos;
            }

            // pos_mode(0): velocity mode: 
            if (stepgen->pos_mode == 0) {
                // for sim:
                *stepgen->enc_pos += *stepgen->vel_cmd * stepgen->pos_scale;
                *stepgen->pulse_pos = *stepgen->enc_pos;
            }

	}

        if (n == (num_joints - 1)) {
            // send to WOU when all axes commands are generated
        }

	DPS("       0x%08X%15.7f%15.7f",
	    integer_pos_cmd, 
            (stepgen->prev_pos_cmd), 
            *stepgen->pos_fb);
	
#ifdef DEBUG
        if (debug_msg_tick == 1700) {
            rtapi_print("j[%d]: rawcount.int(%lld) enc_pos(%d) cmd_fbs(%d) debug(%d) pulse_pos(%d) pos_cmd(%f) pos_fb(%f)\n",
                        n, 
                        (stepgen->rawcount >> FRACTION_BITS),
                        *(stepgen->enc_pos),
                        *(stepgen->cmd_fbs),
                        *(machine_control->debug[n]),
                        *(stepgen->pulse_pos),
                        *stepgen->pos_cmd,
                        *stepgen->pos_fb);
        }
#endif

	/* move on to next channel */
	stepgen++;
    }

#ifdef DEBUG
    debug_msg_tick ++;
    if (debug_msg_tick > 1700) {
        debug_msg_tick = 0;
    }
#endif

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
    }
    machine_control->prev_vel_sync = sync_cmd;
    machine_control->prev_motion_state = *machine_control->motion_state;

#if (TRACE!=0)
    stepgen = arg;
    if (*(stepgen->enable)) {
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


static int export_stepgen(int num, stepgen_t * addr,
			  int pos_mode)
{
    int retval, msg;

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

    /* export pin for counts captured by wou_update() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->pulse_pos), comp_id,
			      "wou.stepgen.%d.pulse_pos", num);
    if (retval != 0) {
	return retval;
    }

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->rawcount32), comp_id,
			      "wou.stepgen.%d.rawcount32", num);
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

    /* export pin for enable command */
    addr->prev_enable = 0;
    addr->son_delay = 0;
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

    retval = hal_pin_float_newf(HAL_IN, &(addr->jog_vel), comp_id,
                                    "wou.stepgen.%d.jog-vel", num);
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(addr->jog_enable), comp_id,
                                    "wou.stepgen.%d.jog-enable", num);
    if (retval != 0) {
        return retval;
    }
    
    retval = hal_pin_float_newf(HAL_OUT, &(addr->rpm), comp_id,
                                    "wou.stepgen.%d.rpm", num);
    if (retval != 0) {
        return retval;
    }

    /* export parameter for pulse_per_rev */
    retval = hal_param_float_newf(HAL_RW, &(addr->pulse_per_rev), comp_id,
				  "wou.stepgen.%d.pulse_per_rev", num);
    if (retval != 0) {
	return retval;
    }

    /* set default values */
    addr->pulse_per_rev = 1.0;
    *addr->rpm           = 0.0;

    addr->pos_scale = 1.0;
    addr->scale_recip = 0.0;
    addr->freq = 0.0;
    addr->maxvel = 0.0;
    addr->maxaccel = 0.0;
    addr->pos_mode = pos_mode;
    addr->pos_cmd_s = 0;
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
    *(addr->rawcount32) = 0;
    *(addr->enc_pos) = 0;
    *(addr->pos_fb) = 0.0;
    *(addr->vel_fb) = 0;
    *(addr->switch_pos) = 0.0;
    *(addr->index_pos) = 0.0;
    *(addr->pos_cmd) = 0.0;
    *(addr->vel_cmd) = 0.0;
    (addr->prev_vel_cmd) = 0.0;
    addr->pulse_vel = 0;
    addr->pulse_accel = 0;
    addr->pulse_jerk = 0;
    *(addr->jog_scale) = 1.0;

    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}

static int export_machine_control(machine_control_t * machine_control)
{
    int i, retval, msg;

    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);
    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->usb_busy), comp_id,
                                  "wou.usb-busy");
    if (retval != 0) {
        return retval;
    }

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
    *(machine_control->bp_tick) = 1; // for sim, force bp_tick to 1, and prev_bp to 0
    machine_control->prev_bp = 0;



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

    machine_control->prev_out = 0;
    machine_control->usb_busy_s = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->teleop_mode), comp_id,
                              "wou.motion.teleop-mode");
    if (retval != 0) { return retval; }
    *(machine_control->teleop_mode) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->coord_mode), comp_id,
                              "wou.motion.coord-mode");
    if (retval != 0) { return retval; }
    *(machine_control->coord_mode) = 0;

//    machine_control->motion_mode = 0;
//    machine_control->motion_mode_prev = 0;
//    machine_control->pid_enable = 0;

    /* restore saved message level*/
    rtapi_set_msg_level(msg);
    return 0;
}

#endif	// else-RTAPI


// vim:sw=4:sts=4:et:
