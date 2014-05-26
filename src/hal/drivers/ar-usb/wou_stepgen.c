/********************************************************************
 * Description:  wou_stepgen.c
 *               This file, 'wou_stepgen.c', is a HAL component that
 *               It was based on stepgen.c by John Kasunich.
 *
 * Author: Yishin Li
 * License: GPL Version 2
 *
 * Copyright (c) 2009-2010 All rights reserved.
 *
 * Last change:
 ********************************************************************/
#include "config.h"

#ifndef RTAPI_SIM
#error "this module for user space simulator only"
#else
// SIM
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */
#include "tc.h"                 /* motion state */
#include <math.h>
#include <string.h>
#include <float.h>
#include "rtapi_math.h"
#include "motion.h"

#include <wou.h>
#include <wb_regs.h>
#include <mailtag.h>
#include "sync_cmd.h"

#define REQUEST_TICK_SYNC_AFTER 500 // after 500 tick count, request risc to sync tick count
#define MAX_CHAN 8
#define MAX_STEP_CUR 255
#define PLASMA_ON_BIT 0x02
#define FRACTION_BITS 16
#define FIXED_POINT_SCALE   65536.0             // (double (1 << FRACTION_BITS))
#define FP_SCALE_RECIP      0.0000152587890625  // (1.0/65536.0)
#define FRACTION_MASK 0x0000FFFF
#define PID_LOOP 8
#define SON_DELAY_TICK  1500

// to disable DP(): #define TRACE 0
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
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
MODULE_AUTHOR("Yishin Li");
MODULE_DESCRIPTION("Wishbone Over USB for EMC HAL");
MODULE_LICENSE("GPL");

const char *pulse_type[MAX_CHAN] =
{ " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(pulse_type, MAX_CHAN,
        "pulse type (AB-PHASE(A) or STEP-DIR(S) or PWM-DIR(P)) for up to 8 channels");

const char *enc_type[MAX_CHAN] =
{ " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(enc_type, MAX_CHAN,
        "encoder type (REAL(r) or LOOP-BACK(l)) for up to 8 channels");

// enc_pol: encoder polarity, default to POSITIVE(p)
const char *enc_pol[MAX_CHAN] =
{ "p", "p", "p", "p", "p", "p", "p", "p" };
RTAPI_MP_ARRAY_STRING(enc_pol, MAX_CHAN,
        "encoder polarity (POSITIVE(p) or NEGATIVE(n)) for up to 8 channels");

// lsp_id: gpio pin id for limit-switch-positive(lsp)
const char *lsp_id[MAX_CHAN] =
{ " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(lsp_id, MAX_CHAN,
        "lsp_id: gpio pin id for limit-switch-positive(lsp) for up to 8 channels");

// lsn_id: gpio pin id for limit-switch-negative(lsn)
const char *lsn_id[MAX_CHAN] =
{ " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(lsn_id, MAX_CHAN,
        "lsn_id: gpio pin id for limit-switch-negative(lsn) for up to 8 channels");

// lsp_id: gpio pin id for ALARM siganl
const char *alr_id[MAX_CHAN] =
{ " ", " ", " ", " ", " ", " ", " ", " " };
RTAPI_MP_ARRAY_STRING(alr_id, MAX_CHAN,
        "alr_id: gpio pin id for ALARM signal for up to 8 channels");

const char *bits = "\0";
RTAPI_MP_STRING(bits, "FPGA bitfile");

const char *bins = "\0";
RTAPI_MP_STRING(bins, "RISC binfile");

int alarm_en = -1;
RTAPI_MP_INT(alarm_en, "hardware alarm dection mode");

int servo_period_ns = -1;   // init to '-1' for testing valid parameter value
RTAPI_MP_INT(servo_period_ns, "used for calculating new velocity command, unit: ns");

# define GPIO_IN_NUM    80
# define GPIO_OUT_NUM   32

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

const char *enc_scale_str[MAX_CHAN] =
{ "1.0", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0" };
RTAPI_MP_ARRAY_STRING(enc_scale_str, MAX_CHAN,
        "enc scale value for up to 8 channels");

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

const char *pattern_type_str ="NO_TEST"; // ANALOG_0: analog input0
RTAPI_MP_STRING(pattern_type_str,
        "indicate test pattern type");

const char *alr_output_0= "0";
RTAPI_MP_STRING(alr_output_0, "DOUT[31:0] while E-Stop is pressed");
const char *alr_output_1= "0";
RTAPI_MP_STRING(alr_output_1, "DOUT[63:32] while E-Stop is pressed");

int gantry_polarity = 0;
RTAPI_MP_INT(gantry_polarity, "gantry polarity");

int gantry_brake_gpio = -1;
RTAPI_MP_INT(gantry_brake_gpio, "gantry brake_gpio");

static int test_pattern_type = 0;  // use dbg_pat_str to update dbg_pat_type

static const char *board = "7i43u";
static const char wou_id = 0;
static wou_param_t w_param;

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
    hal_s32_t   *rawcount32;    /* 32-bit integer part of rawcount */
    hal_bit_t   prev_enable;
    uint32_t    son_delay;      /* eanble delay tick for svo-on of ac-servo drivers */
    hal_bit_t   *enable;        /* pin for enable stepgen */
    hal_u32_t   step_len;       /* parameter: step pulse length */
    char        pulse_type;     /* A(AB-PHASE), S(STEP-DIR), P(PWM) */
    hal_s32_t   *pulse_pos;     /* pin: pulse_pos to servo drive, captured from FPGA */
    hal_s32_t   *enc_pos;       /* pin: encoder position from servo drive, captured from FPGA */

    hal_float_t pos_scale;	/* param: steps per position unit */
    double scale_recip;		/* reciprocal value used for scaling */
    hal_float_t *vel_cmd;	/* pin: velocity command (pos units/cycle_time) */
    double prev_vel_cmd;        /* prev vel cmd: previous velocity command */
    double      pos_cmd_s;	/* saved pos_cmd at rising edge of usb_busy */
    hal_float_t *pos_cmd;	/* pin: motor_pos_cmd (position units) */
    double prev_pos_cmd;        /* prev pos_cmd: previous position command */
    hal_float_t *probed_pos;
    hal_float_t *pos_fb;	/* pin: position feedback (position units) */
    hal_float_t *risc_pos_cmd;  /* pin: position command issued by RISC (position units) */
    hal_float_t *vel_fb;        /* pin: velocity feedback */
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
    int32_t motion_type;        /* motion type wrote to risc */

    hal_s32_t     *cmd_fbs;     /* position command retained by RISC (unit: pulse) */
    hal_s32_t     *enc_vel_p;   /* encoder velocity in pulse per servo-period */

    hal_bit_t   *homing;
    hal_float_t *risc_probe_vel;
    hal_float_t *risc_probe_dist;
    hal_s32_t   *risc_probe_pin;
    hal_s32_t   *risc_probe_type;
    uint32_t    risc_probing;

    hal_float_t *uu_per_rev;
    hal_float_t prev_uu_per_rev;

    hal_bit_t   *bypass_lsp;
    hal_bit_t   *bypass_lsn;
    hal_bit_t   prev_bypass_lsp;
    hal_bit_t   prev_bypass_lsn;

} stepgen_t;
// #pragma pack(pop)   /* restore original alignment from stack */

typedef struct {
    // Analog input: 0~4.096VDC, up to 16 channel
    hal_s32_t *in[16];
    hal_s32_t *out[4];
    hal_s32_t prev_out[4];
} analog_t;

// machine_control_t:
typedef struct {
    hal_bit_t   *usb_busy;
    hal_bit_t   usb_busy_s;
    hal_bit_t   *ignore_ahc_limit;
    int32_t     prev_vel_sync;
    hal_float_t *vel_sync_scale;
    hal_float_t *current_vel;
    hal_float_t *requested_vel;
    hal_float_t *feed_scale;
    hal_bit_t   *vel_sync;  /* A pin to determine when (vel * feedscale) beyond (req_vel * vel_sync_scale) */
    hal_bit_t   *rt_abort;  // realtime abort to FPGA
    /* plasma control */
    hal_bit_t   *thc_enbable;
    //TODO: replace plasma enable with output enable for each pin.
    hal_bit_t   *plasma_enable;
    /* sync input pins (input to motmod) */
    hal_bit_t   *in[80];
    hal_bit_t   *in_n[80];
    uint32_t    prev_in0;
    uint32_t    prev_in1;
    uint32_t    prev_in2;
    hal_float_t *analog_ref_level;
    double prev_analog_ref_level;
    hal_bit_t *sync_in_trigger;
    hal_u32_t *sync_in_index;		//
    hal_u32_t *wait_type;
    hal_float_t *timeout;
    double prev_timeout;

    hal_u32_t   *bp_tick;       /* base-period tick obtained from fetchmail */
    hal_u32_t   *dout0;         /* the DOUT value obtained from fetchmail */
    hal_u32_t   *crc_error_counter;

    /* sync output pins (output from motmod) */
    hal_bit_t   *out[32];
    uint32_t    prev_out;		//ON or OFF

    uint32_t     prev_ahc_state;
    hal_bit_t    *ahc_state;     // 0: disable 1:enable
    hal_float_t  *ahc_level;
    double      prev_ahc_level;
    /* motion state tracker */
    hal_s32_t    *motion_state;
    hal_u32_t    *jog_sel;
    hal_s32_t    *jog_vel_scale;

    /* command channel for emc2 */
    hal_u32_t *wou_cmd;
    uint32_t prev_wou_cmd;
    uint32_t a_cmd_on_going;
    /* test pattern  */
    hal_s32_t *test_pattern;
    /* MPG */
    hal_s32_t *mpg_count;
    hal_s32_t *debug[32];

    hal_bit_t   *teleop_mode;
    hal_bit_t   *coord_mode;
    hal_bit_t   *homing;

    uint32_t	prev_machine_ctrl;	// num_joints is not included
    hal_bit_t	*machine_on;

    hal_bit_t   *update_pos_req;
    hal_bit_t   *update_pos_ack;
    hal_u32_t   *rcmd_seq_num_req;
    hal_u32_t   *rcmd_seq_num_ack;
    hal_u32_t   *rcmd_state;

    hal_u32_t   *max_tick_time;
    hal_bit_t   *probe_result;
    hal_bit_t   *machine_moving;
    hal_bit_t   *ahc_doing;

    hal_u32_t   *spindle_joint_id;

    hal_bit_t   *gantry_en;
    hal_bit_t   *gantry_lock;
    uint32_t    gantry_ctrl;
    uint32_t    prev_gantry_ctrl;

    hal_u32_t    *trigger_din;
    hal_u32_t    *trigger_ain;
    hal_u32_t    *trigger_type;
    hal_bit_t    *trigger_cond;
    hal_u32_t    *trigger_level;
    hal_bit_t   *trigger_enable;     // 0: disable 1:enable
    hal_bit_t   *probing;
    hal_bit_t   *trigger_result;     // 0: disable 1:enable

    hal_bit_t     prev_trigger_enable;
    hal_bit_t     prev_probing;
    double      prev_trigger_level;

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

static int export_stepgen(int num, stepgen_t * addr, char pulse_type);
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
    uint32_t    *p, din[3]; //, dout[1];
    uint8_t     *buf;
    stepgen_t   *stepgen;
    uint32_t    bp_tick;    // served as previous-bp-tick
    uint32_t    machine_status;
    uint32_t    joints_vel;
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
    *machine_control->bp_tick = bp_tick;

    switch(mail_tag)
    {
    case MT_MOTION_STATUS:
        /* for PLASMA with ADC_SPI */
        //redundant: p = (uint32_t *) (buf_head + 4); // BP_TICK
        stepgen = stepgen_array;
        joints_vel = 0;
        for (i=0; i<num_joints; i++) {
            p += 1;
            *(stepgen->pulse_pos) = (int32_t)*p;
            p += 1;
            *(stepgen->enc_pos) = (int32_t)*p;
            p += 1;
            *(stepgen->cmd_fbs) = (int32_t)*p;
            p += 1;
            *(stepgen->enc_vel_p)  = (int32_t)*p; // encoder velocity in pulses per servo-period
            joints_vel |= *(stepgen->enc_vel_p);
            stepgen += 1;   // point to next joint
        }
        *machine_control->machine_moving = (joints_vel != 0);

        // digital input
        p += 1;
        din[0] = *p;
        p += 1;
        din[1] = *p;
        p += 1;
        din[2] = *p;
        // digital output
        p += 1;
        *machine_control->dout0 = *p;

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

        // update gpio_in[79:64]
        // compare if there's any GPIO.DIN bit got toggled
        if (machine_control->prev_in2 != din[2]) {
            // avoid for-loop to save CPU cycles
            machine_control->prev_in2 = din[2];
            for (i = 64; i < 80; i++) {
                *(machine_control->in[i]) = ((machine_control->prev_in2) >> (i-64)) & 0x01;
                *(machine_control->in_n[i]) = (~(*(machine_control->in[i]))) & 0x01;
            }
        }

        // copy 16 channel of 16-bit ADC value
        p += 1;
        buf = (uint8_t*)p;
        for (i=0; i<8; i++) {
            *(analog->in[i*2]) = *(((uint16_t*)buf)+i*2+1);
            *(analog->in[i*2+1]) = *(((uint16_t*)buf)+i*2);
        }

        // MPG
        p += 8; // skip 16ch of 16-bit ADC value
        *(machine_control->mpg_count) = *p;
        // the MPG on my hand is 1-click for a full-AB-phase-wave.
        // therefore the mpg_count will increase by 4.
        // divide it by 4 for smooth jogging.
        // otherwise, there will be 4 units of motions for every MPG click.
        *(machine_control->mpg_count) >>= 2;
        p += 1;
        machine_status = *p;
        stepgen = stepgen_array;
        for (i=0; i<num_joints; i++) {
            *stepgen->ferror_flag = machine_status & (1 << i);
            stepgen += 1;   // point to next joint
        }
        *machine_control->probe_result = (machine_status >> PROBE_RESULT_BIT) & 1;
        *machine_control->ahc_doing = (machine_status >> AHC_DOING_BIT) & 1;

        p += 1;
        *(machine_control->max_tick_time) = *p;

        p += 1;
        *machine_control->rcmd_state = *p;

#if (MBOX_LOG)
        dsize = sprintf (dmsg, "%10d  ", bp_tick);  // #0
        // fprintf (mbox_fp, "%10d  ", bp_tick);
        stepgen = stepgen_array;
        for (i=0; i<num_joints; i++) {
            dsize += sprintf (dmsg + dsize, "%10d %10d %10d  ",
                    *(stepgen->pulse_pos),
                    *(stepgen->enc_pos),
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
//        printf ("MT_ERROR_CODE: code(%d) bp_tick(%d) \n", *p, bp_tick);
        break;

    case MT_DEBUG:
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
        for (i=0; i<8; i++) {
            p += 1;
            *machine_control->debug[i] = *p;
        }
        break;

    case MT_RISC_CMD:
        p = (uint32_t *) (buf_head + 8);
        *machine_control->rcmd_state = *p;
        if (*p == RCMD_UPDATE_POS_REQ)
        {
            // SET update_pos_req here
            // will RESET it after sending a RCMD_UPDATE_POS_ACK packet
            *machine_control->update_pos_req = 1;
            p += 1;
            *machine_control->rcmd_seq_num_req = *p;
        }
        DP("update_pos_req(%d) rcmd_seq_num_req(%d) rcmd_state(%d)\n", *machine_control->update_pos_req, *machine_control->rcmd_seq_num_req, *machine_control->rcmd_state);
        break;

    case MT_PROBED_POS:
        stepgen = stepgen_array;
        p = (uint32_t *) (buf_head + 4);
        for (i=0; i<num_joints; i++) {
            p += 1;
            *(stepgen->probed_pos) = (double) ((int32_t)*p) * (stepgen->scale_recip);
//            printf("joint[%d] probed_pos(%f)\n", i, *(stepgen->probed_pos));
            stepgen += 1;   // point to next joint
        }
        p += 1;
        *machine_control->trigger_result = ((uint8_t)*p);
        break;

    default:
        fprintf(stderr, "ERROR: wou_stepgen.c unknown mail tag (%d)\n", mail_tag);
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
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ADDR(addr) | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);
    while(wou_flush(&w_param) == -1);
    DP ("end of SYNC_MOT_PARAM_CMD\n");

    return;
}

static void send_sync_cmd (uint16_t sync_cmd, uint32_t *data, uint32_t size)
{
    uint16_t    buf;
    int         i, j;

    // TODO: pack whole data into a single WB_WR_CMD
    for (i=0; i<size; i++)
    {
        for(j=0; j<sizeof(int32_t); j++) {
            buf = SYNC_DATA | ((uint8_t *)data)[j];
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), sizeof(uint16_t), (const uint8_t *)&buf);
        }
        data++;
    }

    wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), sizeof(uint16_t), (const uint8_t *)&sync_cmd);
    while(wou_flush(&w_param) == -1);   // wait until all those WB_WR_CMDs are accepted by WOU

    DP ("end of send_sync_cmd\n");
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
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
    }
    sync_cmd = SYNC_MACH_PARAM | PACK_MACH_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);

    while(wou_flush(&w_param) == -1);
    DP ("end of write_machine_param(%u)\n", addr);     // addr(32): MACHINE_CTRL ... updated when accel_state changes
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
    double max_vel, max_accel, pos_scale, value, max_following_error;
    double enc_scale;
    double max_jerk;
    int msg;
    int lsp, lsn, alr;
    uint16_t sync_cmd;
    uint32_t dac_ctrl_reg;

    msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_ALL);
    // rtapi_set_msg_level(RTAPI_MSG_INFO);
    rtapi_set_msg_level(RTAPI_MSG_DBG);

    machine_control = NULL;

    /* test for bitfile string: bits */
    if ((bits == 0) || (bits[0] == '\0')) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOU: ERROR: no fpga bitfile string: bits\n");
        return -1;
    } else {
        // initialize FPGA with bitfile(bits)
        wou_init(&w_param, board, wou_id, bits);
        if (wou_connect(&w_param) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOU: ERROR: Connection failed\n");
            return -1;
        }
    }

#if (TRACE!=0)
    // initialize file handle for logging wou steps
    dptrace = fopen("wou_stepgen.log", "w");
#endif

    /* test for risc image file string: bins */
    if ((bins == 0) || (bins[0] == '\0')) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOU: ERROR: no risc binfile string: bins\n");
        return -1;
    } else {
        // programming risc with binfile(bins)
        if (wou_prog_risc(&w_param, bins) != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "WOU: ERROR: load RISC program filed\n");
            return -1;
        }

#if (MBOX_LOG)
        mbox_fp = fopen ("./mbox.log", "w");
        fprintf (mbox_fp, "%10s  ", "bp_tick");
        for (i=0; i<4; i++) {
            fprintf (mbox_fp, "%9s%d  %9s%d %9s%d  ",
                    "pls_pos-", i,
                    "enc_pos-", i,
                    "jnt_cmd-", i
            );
        }
        fprintf (mbox_fp, "\n");
#endif
#if (DEBUG_LOG)
        debug_fp = fopen ("./debug.log", "w");
#endif
        // set mailbox callback function
        wou_set_mbox_cb (&w_param, fetchmail);

        // set crc counter callback function
        wou_set_crc_error_cb (&w_param, get_crc_error_counter);

        // set rt_cmd callback function
        wou_set_rt_cmd_cb (&w_param, update_rt_cmd);
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
        wou_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                (uint8_t) 1, data);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOU: ERROR: no alarm_en\n");
        return -1;
    }

    if (gantry_polarity != 0) {
        // gantry_polarity should either be 1 or -1
        write_machine_param(GANTRY_POLARITY, gantry_polarity);
    }
    while(wou_flush(&w_param) == -1);

    // "pulse type (AB-PHASE(a) or STEP-DIR(s) or PWM-DIR(p)) for up to 8 channels")
    data[0] = 0;
    data[1] = 0;
    num_joints = 0;
    for (n = 0; n < MAX_CHAN && (pulse_type[n][0] != ' ') ; n++) {
        if (toupper(pulse_type[n][0]) == 'A') {
            // PULSE_TYPE(0): ab-phase
        } else if (toupper(pulse_type[n][0]) == 'S') {
            // PULSE_TYPE(1): step-dir
            if (n < 4) {
                data[0] |= (1 << (n * 2));
            } else {
                data[1] |= (1 << ((n - 4) * 2));
            }
        } else if (toupper(pulse_type[n][0]) == 'P') {
            // PULSE_TYPE(1): pwm-dir
            if (n < 4) {
                data[0] |= (3 << (n * 2));
            } else {
                data[1] |= (3 << ((n - 4) * 2));
            }
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "STEPGEN: ERROR: bad pulse_type '%s' for joint %i (must be 'A' or 'S' or 'P')\n",
                    pulse_type[n], n);
            return -1;
        }
        num_joints++;
    }
    if(n > 0) {
        wou_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE),
                (uint8_t) 2, data);
        rtapi_print_msg(RTAPI_MSG_INFO,
                "STEPGEN: PULSE_TYPE[J3:J0]: 0x%02X\n", data[0]);
        rtapi_print_msg(RTAPI_MSG_INFO,
                "STEPGEN: PULSE_TYPE[J7:J4]: 0x%02X\n", data[1]);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOU: ERROR: no pulse_type defined\n");
        return -1;
    }

    // "encoder type: (REAL(r) or LOOP-BACK(l)) for up to 8 channels"
    data[0] = 0;
    for (n = 0; n < MAX_CHAN && (enc_type[n][0] != ' ') ; n++) {
        if ((enc_type[n][0] == 'l') || (enc_type[n][0] == 'L')) {
            // ENC_TYPE(0): fake ENCODER counts (loop PULSE_CMD to ENCODER)
        } else if ((enc_type[n][0] == 'r') || (enc_type[n][0] == 'R')) {
            // ENC_TYPE(1): real ENCODER counts
            data[0] |= (1 << n);
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "STEPGEN: ERROR: bad enc_type '%s' for joint %i (must be 'r' or 'l')\n",
                    enc_type[n], n);
            return -1;
        }
    }
    if(n > 0) {
        wou_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE),
                (uint8_t) 1, data);
        rtapi_print_msg(RTAPI_MSG_INFO,
                "STEPGEN: ENC_TYPE: 0x%02X\n", data[0]);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOU: ERROR: no enc_type defined\n");
        return -1;
    }

    // "encoder polarity (POSITIVE(p) or NEGATIVE(n)) for up to 8 channels"
    data[0] = 0;
    for (n = 0; n < MAX_CHAN; n++) {
        if ((enc_pol[n][0] == 'p') || (enc_pol[n][0] == 'P')) {
            // ENC_POL(0): POSITIVE ENCODER POLARITY (default)
        } else if ((enc_pol[n][0] == 'n') || (enc_pol[n][0] == 'N')) {
            // ENC_POL(1): NEGATIVE ENCODER POLARITY
            data[0] |= (1 << n);
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "STEPGEN: ERROR: bad enc_pol '%s' for joint %i (must be 'p' or 'n')\n",
                    enc_pol[n], n);
            return -1;
        }
    }
    wou_cmd (&w_param, WB_WR_CMD,
            (uint16_t) (SSIF_BASE | SSIF_ENC_POL),
            (uint8_t) 1, data);

    // "set LSP_ID/LSN_ID for up to 8 channels"
    for (n = 0; n < MAX_CHAN && (lsp_id[n][0] != ' ') ; n++) {
        pos_scale = atof(pos_scale_str[n]);
        lsp = atoi(lsp_id[n]);
        lsn = atoi(lsn_id[n]);
        if(pos_scale >= 0) {
            immediate_data = (n << 16) | (lsp << 8) | (lsn);
        } else {
            immediate_data = (n << 16) | (lsn << 8) | (lsp);
        }
        write_machine_param(JOINT_LSP_LSN, immediate_data);
        while(wou_flush(&w_param) == -1);

    }

    // "set ALR_ID for up to 8 channels"
    immediate_data = 0; // reset ALR_EN_BITS to 0
    for (n = 0; n < MAX_CHAN && (alr_id[n][0] != ' ') ; n++) {
        alr = atoi(alr_id[n]);
        if(alr != 255) {
            immediate_data |= (1 << alr);
            assert (alr < 7);    // AR08: ALARM maps to GPIO[6:1]
            assert (alr > 0);
        }
    }
    write_machine_param(ALR_EN_BITS, immediate_data);
    while(wou_flush(&w_param) == -1);

    // configure alarm output (for E-Stop)
    write_machine_param(ALR_OUTPUT_0, (uint32_t) strtoul(alr_output_0, NULL, 16));
    while(wou_flush(&w_param) == -1);
    fprintf(stderr, "ALR_OUTPUT_0(%08X)",(uint32_t) strtoul(alr_output_0, NULL, 16));

    write_machine_param(ALR_OUTPUT_1, (uint32_t) strtoul(alr_output_1, NULL, 16));
    while(wou_flush(&w_param) == -1);
    fprintf(stderr, "ALR_OUTPUT_1(%08X)",(uint32_t) strtoul(alr_output_1, NULL, 16));

    for (i = 0; i < 4; i++) {
        sync_cmd = SYNC_DAC | (i << 8) | (0x55);    /* DAC, ID:i, ADDR: 0x55(Control Register) */
        printf ("TODO: write DAC_CTRL_REG through config/ini\n");
        dac_ctrl_reg = 0x1001;
        send_sync_cmd (sync_cmd, &dac_ctrl_reg, 1);
    }

    // config auto height control behavior
    immediate_data = atoi(ahc_ch_str);
    write_machine_param(AHC_ANALOG_CH, immediate_data);
    while(wou_flush(&w_param) == -1);
    immediate_data = atoi(ahc_joint_str);
    pos_scale = atof(pos_scale_str[immediate_data]);
    write_machine_param(AHC_JNT, immediate_data);
    while(wou_flush(&w_param) == -1);

    if (strcmp(ahc_polarity, "POSITIVE") == 0) {
        if (pos_scale >=0) {
            // set risc positive
            write_machine_param(AHC_POLARITY, 1);

        } else {
            // set risc negative
            write_machine_param(AHC_POLARITY, -1);
        }

    } else if (strcmp(ahc_polarity, "NEGATIVE") == 0) {
        if (pos_scale >= 0) {
            // set risc negative
            write_machine_param(AHC_POLARITY, -1);
        } else {
            // set risc positive
            write_machine_param(AHC_POLARITY, 1);
        }

    } else {
        fprintf(stderr, "wou_stepgen.c: non-supported ahc polarity config\n");
        assert(0);
    }
    while(wou_flush(&w_param) == -1);


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
    while(wou_flush(&w_param) == -1);

    // Update num_joints while checking pulse_type[]
    assert (num_joints <= 6);  // support up to 6 joints for USB/7i43
    if (num_joints == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "STEPGEN: ERROR: no channels configured\n");
        return -1;
    }

    /* to clear PULSE/ENC/SWITCH/INDEX positions for all joints*/
    // issue a WOU_WRITE to RESET SSIF position registers
    data[0] = (1 << num_joints) - 1;  // bit-map-for-num_joints
    wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    while(wou_flush(&w_param) == -1);

#if (TRACE!=0)
// initialize file handle for logging wou steps
//     dptrace = fopen("wou_stepgen.log", "w");
    /* prepare header for gnuplot */
//    DPS("#%10s  %15s%15s%15s%15s  %15s%15s%15s%15s  %15s%15s%15s%15s  %15s%15s%15s%15s\n",
//            "dt",
//            "int_pcmd[0]", "prev_pcmd[0]", "pos_fb[0]", "risc_pos_cmd[0]",
//            "int_pcmd[1]", "prev_pcmd[1]", "pos_fb[1]", "risc_pos_cmd[1]",
//            "int_pcmd[2]", "prev_pcmd[2]", "pos_fb[2]", "risc_pos_cmd[2]",
//            "int_pcmd[3]", "prev_pcmd[3]", "pos_fb[3]", "risc_pos_cmd[3]"
//    );

    DPS("#%10s", "dt");
    for (n = 0; n < num_joints; n++)
    {
        DPS("      int_pcmd[%d]", n);
        DPS("     prev_pcmd[%d]", n);
        DPS("        pos_fb[%d]", n);
        DPS("  risc_pos_cmd[%d]", n);
    }
    DPS("\n");
#endif

    // issue a WOU_WRITE to clear SSIF_RST_POS register
    data[0] = 0x00;
    wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
    while(wou_flush(&w_param) == -1);

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

    // MACHINE_CTRL,   // [31:24]  RESERVED
    //                 // [23:16]  NUM_JOINTS
    //                 // [15: 8]  WORLD(1)/JOINT(0) mode
    //                 // [ 7: 0]  PID_ENABLE
    // configure NUM_JOINTS after all joint parameters are set
    immediate_data = (num_joints << 16); // assume motion_mode(0) and pid_enable(0)
    write_machine_param(MACHINE_CTRL, (uint32_t) immediate_data);
    while(wou_flush(&w_param) == -1);

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
    while(wou_flush(&w_param) == -1);
    /* have good config info, connect to the HAL */
    comp_id = hal_init("wou");
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

    /* configure motion parameters */
    for(n=0; n<num_joints; n++) {
        /* compute fraction bits */
        // compute proper fraction bit for command
        // compute fraction bit for velocity
        // accurate 0.0001 mm
        pos_scale   = atof(pos_scale_str[n]);
        max_vel     = atof(max_vel_str[n]);
        max_accel   = atof(max_accel_str[n]);
        max_jerk    = atof(max_jerk_str[n]);
        assert (max_vel > 0);
        assert (max_accel > 0);
        assert (max_jerk > 0);

        /* config encoder scale parameter */
        enc_scale   = atof(enc_scale_str[n]);
        assert (enc_scale > 0);
        immediate_data = (uint32_t)(enc_scale * FIXED_POINT_SCALE);
        write_mot_param (n, (ENC_SCALE), immediate_data);
        while(wou_flush(&w_param) == -1);

        /* unit_pulse_scale per servo_period */
        immediate_data = (uint32_t)(FIXED_POINT_SCALE * pos_scale * dt);
        rtapi_print_msg(RTAPI_MSG_DBG, "j[%d] scale(0x%08X)\n", n, immediate_data);
        assert(immediate_data != 0);
        write_mot_param (n, (SCALE), immediate_data);
        while(wou_flush(&w_param) == -1);
        pos_scale = fabs(pos_scale);    // absolute pos_scale for MAX_VEL/ACCEL calculation

        /* config MAX velocity */
        // * 1.05 : add 5% head room
        immediate_data = (uint32_t)((max_vel * pos_scale * dt * FIXED_POINT_SCALE) * 1.05);    
        rtapi_print_msg(RTAPI_MSG_DBG,
                "j[%d] max_vel(%d) = %f*%f*%f*%f\n",
                n, immediate_data, max_vel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data>0);
        write_mot_param (n, (MAX_VELOCITY), immediate_data);
        while(wou_flush(&w_param) == -1);
        stepgen_array[n].pulse_maxv = immediate_data;

        /* config acceleration */
        // * 1.05 : add 5% head room
        immediate_data = (uint32_t)((max_accel * pos_scale * dt * FIXED_POINT_SCALE * dt) * 1.05 );
        rtapi_print_msg(RTAPI_MSG_DBG,
                "j[%d] max_accel(%d) = %f*%f*(%f^2)*(%f)\n",
                n, immediate_data, max_accel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data > 0);
        write_mot_param (n, (MAX_ACCEL), immediate_data);
        while(wou_flush(&w_param) == -1);
        stepgen_array[n].pulse_maxa = immediate_data;

//        /* config acceleration recip */
//        immediate_data = (uint32_t)(FIXED_POINT_SCALE / (max_accel * pos_scale * dt * dt * 1.05));
//        rtapi_print_msg(RTAPI_MSG_DBG,
//                "j[%d] max_accel_recip(%d) = (%f/(%f*%f*(%f^2)))\n",
//                n, immediate_data, FIXED_POINT_SCALE, max_accel, pos_scale, dt);
//        assert(immediate_data > 0);
//        write_mot_param (n, (MAX_ACCEL_RECIP), immediate_data);
//        while(wou_flush(&w_param) == -1);



        /* config max jerk */
        /* TODO: confirm the "2.17x" of jerk:
         *        in tp.c, the tc->jerk is ~2.17x of ini-jerk */
        immediate_data = (uint32_t)(3.0 * (max_jerk * pos_scale * FIXED_POINT_SCALE * dt * dt * dt));
        rtapi_print_msg(RTAPI_MSG_DBG,
                "j[%d] max_jerk(%d) = (%f * %f * %f * %f^3)))\n",
                n, immediate_data, FIXED_POINT_SCALE, max_jerk, pos_scale, dt);
        assert(immediate_data != 0);
        write_mot_param (n, (MAX_JERK), immediate_data);
        while(wou_flush(&w_param) == -1);
        stepgen_array[n].pulse_maxj = immediate_data;

        /* config max following error */
        // following error send with unit pulse
        max_following_error = atof(ferror_str[n]);
        immediate_data = (uint32_t)(ceil(max_following_error * pos_scale));
        rtapi_print_msg(RTAPI_MSG_DBG, "max ferror(%d)\n", immediate_data);
        write_mot_param (n, (MAXFOLLWING_ERR), immediate_data);
        while(wou_flush(&w_param) == -1);
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
                while(wou_flush(&w_param) == -1);
                rtapi_print_msg(RTAPI_MSG_INFO, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }

//            value = 0;
//            immediate_data = (int32_t) (value);
//            write_mot_param (n, (ENABLE), immediate_data);
//            while(wou_flush(&w_param) == -1);
//            rtapi_print_msg(RTAPI_MSG_INFO, "\n");
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
    machine_control->prev_machine_ctrl = 0;	// num_joints is not included
    machine_control->prev_gantry_ctrl = 0;           // gantry_brake_gpio is not included

    /* export all the variables for each pulse generator */
    for (n = 0; n < num_joints; n++) {
        /* export all vars */
        retval = export_stepgen(n, &(stepgen_array[n]), toupper(pulse_type[n][0]));
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

static void update_rt_cmd(void)
{
    uint8_t data[MAX_DSIZE];    // data[]: for wou_cmd()
    int32_t immediate_data = 0;
    if (machine_control) {
        if (*machine_control->rt_abort == 1) {
            immediate_data = RT_ABORT;
            memcpy(data, &immediate_data, sizeof(uint32_t));
            rt_wou_cmd (&w_param, 
                    WB_WR_CMD,
                    (uint16_t) (JCMD_BASE | OR32_RT_CMD),
                    sizeof(uint32_t),
                    data);
            rt_wou_flush(&w_param);
        }
    }
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
    uint32_t tmp;
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

    // TODO: confirm trajecotry planning thread is always ahead of wou
    if (wou_flush(&w_param) == -1) {
        // struct timespec time;

        // raise flag to pause trajectory planning
        *(machine_control->usb_busy) = 1;
        if (machine_control->usb_busy_s == 0) {
            // DP("usb_busy: begin\n");
            // store current traj-planning command
            stepgen = arg;
            for (n = 0; n < num_joints; n++) {
                stepgen->pos_cmd_s = *(stepgen->pos_cmd);
                stepgen ++;
            }
        }
        machine_control->usb_busy_s = 1;
        // printf ("usb is busy\n");
        // time.tv_sec = 0;
        // time.tv_nsec = 300000;      // 0.3ms
        // nanosleep(&time, NULL);     // sleep 0.3ms to prevent busy loop
        // sleep(1);
        // usleep(10000);  // usleep for 10ms will suspend too long to keep usb-link alive
        // usleep(1000);  // suspend for 1ms
        // usleep(10);  // suspend for 0.01ms
        return;
    } else {
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
    }

    msg = rtapi_get_msg_level();
    //     rtapi_set_msg_level(RTAPI_MSG_ALL);
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    // wou_status (&w_param); // print usb bandwidth utilization
    wou_update(&w_param);   // link to wou_recv()

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

    /* begin motion_mode */
    /**
     *  MACHINE_CTRL,   // [31:28]  JOG_VEL_SCALE
     *                  // [27:24]  SPINDLE_JOINT_ID
     *                  // [23:16]  NUM_JOINTS
     *                  // [l5: 8]  JOG_SEL
     *                                  [15]: MPG(1), CONT(0)
     *                                  [14]: RESERVED
     *                                  [13:8]: J[5:0], EN(1)
     *                  // [ 7: 4]  ACCEL_STATE
     *                  // [ 3: 1]  MOTION_MODE:
     *                                  FREE    (0)
     *                                  TELEOP  (1)
     *                                  COORD   (2)
     *                                  HOMING  (4)
     *                  // [    0]  MACHINE_ON
     **/

    assert(abs(*machine_control->jog_vel_scale) < 8);
    tmp = (*machine_control->jog_vel_scale << 28)
          | (*machine_control->jog_sel << 8)
          | (*machine_control->motion_state << 4)
          | (*machine_control->homing << 3)
          | (*machine_control->coord_mode << 2)
          | (*machine_control->teleop_mode << 1)
          | (*machine_control->machine_on);
    if (tmp != machine_control->prev_machine_ctrl) {
        machine_control->prev_machine_ctrl = tmp;
        immediate_data = (num_joints << 16) | tmp;
        immediate_data = ((*machine_control->spindle_joint_id) << 24) | immediate_data ;
        write_machine_param(MACHINE_CTRL, (uint32_t) immediate_data);
    }

    tmp = (*machine_control->gantry_en << 31)
          | (*machine_control->gantry_lock << 30);
    if (tmp != machine_control->prev_gantry_ctrl) {
        machine_control->prev_gantry_ctrl = tmp;
        if (*machine_control->machine_on)
        {   // only Lock/Release gantry brake after servo-on to prevent dropping
            immediate_data = tmp | (gantry_brake_gpio & GCTRL_BRAKE_GPIO_MASK);
            write_machine_param(GANTRY_CTRL, (uint32_t) immediate_data);
        }
    }
    /* end: */

    if (*machine_control->update_pos_ack)
    {
        uint32_t dbuf[2];
        dbuf[0] = RCMD_UPDATE_POS_ACK;
        dbuf[1] = *machine_control->rcmd_seq_num_req;
        DP("update_pos_ack");
        send_sync_cmd ((SYNC_USB_CMD | RISC_CMD_TYPE), dbuf, 2);
        // Reset update_pos_req after sending a RCMD_UPDATE_POS_ACK packet
        *machine_control->update_pos_req = 0;
        DP("update_pos_ack(%d) rcmd_seq_num_ack(%d)\n", *machine_control->update_pos_ack, dbuf[1]);
    }

    /* begin: handle AHC state, AHC level */
    if (*machine_control->ahc_level != machine_control->prev_ahc_level) {
        immediate_data = (uint32_t)(*(machine_control->ahc_level));
        immediate_data *= 65536; //coonvert from 32.0 to 16.16
        write_machine_param(AHC_LEVEL, immediate_data);
        machine_control->prev_ahc_level = *(machine_control->ahc_level);
        fprintf(stderr,"wou_stepgen.c: ahc_level(%d)\n",
                (uint32_t) *(machine_control->ahc_level));
        machine_control->prev_ahc_level = *(machine_control->ahc_level);
    }

    if ((*machine_control->ahc_state) !=
            (machine_control->prev_ahc_state)) {
        immediate_data = (*(machine_control->ahc_state));
        write_machine_param(AHC_STATE, immediate_data);
        fprintf(stderr,"wou_stepgen.c: ahc_state(%d)\n",
                *(machine_control->ahc_state));
        machine_control->prev_ahc_state = *machine_control->ahc_state;
    }
    /* end: handle AHC state, AHC level */

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
    /* for M200, ARTEK's SYNC_INPUT */
    /* for G33 and G33.1, to synchronize spindle index */
    if (*(machine_control->sync_in_trigger) != 0) {
        assert(*(machine_control->sync_in_index) >= 0);
        assert(*(machine_control->sync_in_index) < GPIO_IN_NUM);
        DP("wou_stepgen.c: risc singal wait trigged(input(%d) type (%d))\n",
                (uint32_t)*machine_control->sync_in_index,
                (uint32_t)*(machine_control->wait_type));
        // begin: trigger sync in and wait timeout
        sync_cmd = SYNC_DIN |
                   PACK_IO_ID((uint32_t)*(machine_control->sync_in_index)) |
                   PACK_DI_TYPE((uint32_t)*(machine_control->wait_type));
        wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD), sizeof(uint16_t), (uint8_t *) &sync_cmd);
        // end: trigger sync in and wait timeout
        *(machine_control->sync_in_trigger) = 0;
    }
    /* end: process motion synchronized input */

    /* begin: process motion synchronized output */
    sync_out_data = 0;
    for (i = 0; i < GPIO_OUT_NUM; i++) {
        if(((machine_control->prev_out >> i) & 0x01) !=
                (*(machine_control->out[i]))) {
            {
                // write a wou frame for sync output into command FIFO
//                fprintf(stderr, "wou_stepgen.c: gpio_%02d => (%d)\n", i,
//                        *(machine_control->out[i]));
                sync_cmd = SYNC_DOUT | PACK_IO_ID(i) | PACK_DO_VAL(*(machine_control->out[i]));
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);
            }
        }
        sync_out_data |= ((*(machine_control->out[i])) << i);
    }
    machine_control->prev_out = sync_out_data;
    /* end: process motion synchronized output */

    /* DAC: fixed as 4 dac channels */
    for (i = 0; i < 4; i++) {
        if(analog->prev_out[i] != *(analog->out[i]))
        {
            analog->prev_out[i] = *(analog->out[i]);
            sync_cmd = SYNC_DAC | (i << 8) | (0x01);    /* DAC, ID:i, ADDR: 0x01 */
            send_sync_cmd (sync_cmd, (uint32_t *)analog->out[i], 1);
            // printf("analog->out[%d]\n", *(analog->out[i]));
        }
    }

    /* point at stepgen data */
    stepgen = arg;

#if (TRACE!=0)
    _dt++;
    if (*stepgen->enable) {
        DPS("%11u", _dt);	// %11u: '#' + %10s
    }
#endif

    // in[0] == 1 (ESTOP released)
    // in[0] == 0 (ESTOP pressed)
    if (*(machine_control->in[0]) == 0) {
        // force updating prev_out and ignore "out[] for RISC" if ESTOP is pressed
        // The dout0 is forced by ALARM_OUT when ESTOP is pressed
        machine_control->prev_out =  *machine_control->dout0;
    }

    i = 0;
    stepgen = arg;
    for (n = 0; n < num_joints; n++) {

        if (*stepgen->enable != stepgen->prev_enable) {
            stepgen->prev_enable = *stepgen->enable;
        }

        *(stepgen->rawcount32) = (int32_t) (stepgen->rawcount >> FRACTION_BITS);

        if (*stepgen->uu_per_rev != stepgen->prev_uu_per_rev)
        {   // pass compensation scale
            /* set spindle_sync_motion scale parameter */
            immediate_data = (int32_t)(*stepgen->uu_per_rev
                                      * FIXED_POINT_SCALE
                                      * (stepgen->pos_scale)
                                      / stepgen_array[(*machine_control->spindle_joint_id)].pos_scale);
            write_mot_param (n, (SSYNC_SCALE), immediate_data); // format: 16.16
            stepgen->prev_uu_per_rev = *stepgen->uu_per_rev;
            DP("j[%d].SSYNC_SCALE: 0x%08X\n", n, immediate_data);
            DP("spindle_joint_id(%d) uu_per_rev(%f) prev_uu_per_rev(%f)\n", *machine_control->spindle_joint_id, *stepgen->uu_per_rev, stepgen->prev_uu_per_rev);
        }

        if (*stepgen->bypass_lsp != stepgen->prev_bypass_lsp)
        {   // bypass limit-switch-positive
            int lsp, lsn;

            if (*stepgen->bypass_lsp)
            {   /* 0 -> 1 */
                lsp = 255;
            }
            else
            {   /* 1 -> 0 */
                lsp = atoi(lsp_id[n]);
            }
            lsn = atoi(lsn_id[n]);
            if(stepgen->pos_scale >= 0) {
                immediate_data = (n << 16) | (lsp << 8) | (lsn);
            } else {
                immediate_data = (n << 16) | (lsn << 8) | (lsp);
            }
            write_machine_param(JOINT_LSP_LSN, immediate_data);
            while(wou_flush(&w_param) == -1);
            stepgen->prev_bypass_lsp = *stepgen->bypass_lsp;
        }

        if (*stepgen->bypass_lsn != stepgen->prev_bypass_lsn)
        {   // bypass limit-switch-negative
            int lsp, lsn;

            if (*stepgen->bypass_lsn)
            {   /* 0 -> 1 */
                lsn = 255;
            }
            else
            {   /* 1 -> 0 */
                lsn = atoi(lsn_id[n]);
            }
            lsp = atoi(lsp_id[n]);
            if(stepgen->pos_scale >= 0) {
                immediate_data = (n << 16) | (lsp << 8) | (lsn);
            } else {
                immediate_data = (n << 16) | (lsn << 8) | (lsp);
            }
            write_machine_param(JOINT_LSP_LSN, immediate_data);
            while(wou_flush(&w_param) == -1);
            stepgen->prev_bypass_lsn = *stepgen->bypass_lsn;
        }

        *(stepgen->pos_fb) = (*stepgen->enc_pos) * stepgen->scale_recip;
        *(stepgen->risc_pos_cmd) = (*stepgen->cmd_fbs) * stepgen->scale_recip;

        // update velocity-feedback based on RISC-reported encoder-velocity
        // enc_vel_p is in 16.16 pulse per servo-period format
        *(stepgen->vel_fb) = *(stepgen->enc_vel_p) * stepgen->scale_recip * recip_dt * FP_SCALE_RECIP;

        /* test for disabled stepgen */
        if (stepgen->prev_enable == 0) {
            /* AXIS not PWR-ON */
            /* keep updating parameters for better performance */
            stepgen->scale_recip = 1.0 / stepgen->pos_scale;

            /* set velocity to zero */
            stepgen->freq = 0;

            /* to prevent position drift while toggeling "PWR-ON" switch */
            (stepgen->prev_pos_cmd) = *stepgen->pos_cmd;
            stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE * stepgen->pos_scale;

            /* clear vel status when enable = 0 */
            *stepgen->vel_cmd = 0;
            stepgen->prev_vel_cmd = 0;
            stepgen->pulse_vel = 0;
            stepgen->pulse_accel = 0;
            stepgen->pulse_jerk = 0;

            /* in HAL, 若任何一軸的 *stepgen->enable 訊號忘了接，就會造成這個 assertion */
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
            if (n == (num_joints - 1)) {
                // send to WOU when all axes commands are generated
                wou_cmd(&w_param,
                        WB_WR_CMD,
                        (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_joints, data);
            }

            // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds
            {
                if (stepgen->pulse_type != 'P') {
                    /* AB-PHASE or STEP-DIR */
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
                } else {
                    /* PWM-DIR */
                    if (stepgen->pulse_type == 'P') {
                        stepgen->maxvel = 100.0 * fabs(stepgen->pos_scale) * FIXED_POINT_SCALE;
                    }
                }

            }

            /* and skip to next one */
            stepgen++;
            continue;
        }

        if(*stepgen->risc_probe_vel == 0)
            stepgen->risc_probing = 0;

        if((*stepgen->homing) &&
           (*stepgen->risc_probe_vel != 0) &&
           (stepgen->risc_probing == 0) &&
           (*machine_control->rcmd_state == RCMD_IDLE))
        {
            // do RISC_PROBE
            uint32_t dbuf[4];
            dbuf[0] = RCMD_PROBE_REQ;
            dbuf[1] = n |   // joint_num
                        (*stepgen->risc_probe_type << 8) |
                        (*stepgen->risc_probe_pin << 16);
            dbuf[2] = *stepgen->risc_probe_vel * stepgen->pos_scale * dt * FIXED_POINT_SCALE;       // fixed-point 16.16
            dbuf[3] = *stepgen->risc_probe_dist * stepgen->pos_scale;                               // distance in pulse
            send_sync_cmd ((SYNC_USB_CMD | RISC_CMD_TYPE), dbuf, 4);
            assert(*stepgen->risc_probe_pin < 64);
            assert(dbuf[2] != 0);
            stepgen->risc_probing = 1;
            DP ("j[%d]: do risc-probing type(%d) pin(%d)\n", n, *stepgen->risc_probe_type, *stepgen->risc_probe_pin);
        }

//        if((*machine_control->trigger_enable != machine_control->prev_trigger_enable) &&
//           (*machine_control->rcmd_state == RCMD_IDLE))
        if((*machine_control->probing != machine_control->prev_probing))
//        		&&
//                   (*machine_control->rcmd_state == RCMD_IDLE))
        {
            // do GMCODE_PROBE
            uint32_t dbuf[4];
            dbuf[0] = RCMD_GMCODE_PROBE;
            dbuf[1] = (*machine_control->trigger_level & 0x3FFF) |
                        ((*machine_control->trigger_din & 0xFF) << 14) |
                        ((*machine_control->trigger_ain & 0x3F) << 22) |
                        ((*machine_control->trigger_type & 0x3) << 28) |
                        ((*machine_control->trigger_cond & 0x1) << 30) |
                        ((*machine_control->probing & 0x1) << 31);
                            // distance in pulse
//            printf("level(%d) din(%d) ain(%d)\n type(%d) cond(%d) enable(%d)",
//            		*machine_control->trigger_level, *machine_control->trigger_din,
//            		*machine_control->trigger_ain, *machine_control->trigger_type,
//            		*machine_control->trigger_cond, *machine_control->trigger_enable);
            send_sync_cmd ((SYNC_USB_CMD | RISC_CMD_TYPE), dbuf, 2);
            machine_control->prev_probing = *machine_control->probing;
        }

        //
        // first sanity-check our maxaccel and maxvel params
        //

        if (stepgen->pulse_type != 'P') {
            /* pulse_type is either A(AB-PHASE) or S(STEP-DIR) */
            int32_t pulse_accel;
            int32_t pulse_jerk;

            if (*(machine_control->update_pos_ack))
            {
                (stepgen->prev_pos_cmd) = (*stepgen->pos_cmd);
                stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE * stepgen->pos_scale;
                DP ("update_pos_ack(%d)\n", *machine_control->update_pos_ack);
                DP ("j[%d], pos_cmd(%f) prev_pos_cmd(%f)\n",
                        n, (*stepgen->pos_cmd), (stepgen->prev_pos_cmd));
            }
            *stepgen->vel_cmd = ((*stepgen->pos_cmd) - (stepgen->prev_pos_cmd));

            integer_pos_cmd = (int32_t)(*stepgen->vel_cmd * (stepgen->pos_scale) * FIXED_POINT_SCALE);

            // integer_pos_cmd is indeed pulse_vel (velocity in pulse)
            if (abs(integer_pos_cmd) > stepgen->pulse_maxv) {
                pulse_accel = integer_pos_cmd - stepgen->pulse_vel;
                pulse_jerk = pulse_accel - stepgen->pulse_accel;
                printf("j[%d], pos_fb(%f) \n", n, (*stepgen->pos_fb));
                printf("j[%d], vel_cmd(%f) pos_cmd(%f) prev_pos_cmd(%f)\n",
                        n, *stepgen->vel_cmd, (*stepgen->pos_cmd), (stepgen->prev_pos_cmd));
                printf("j[%d], pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
                        n, integer_pos_cmd, pulse_accel, pulse_jerk);
                printf("j[%d], PREV pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
                        n, stepgen->pulse_vel, stepgen->pulse_accel, stepgen->pulse_jerk);
                printf("j[%d], pulse_maxv(%d), pulse_maxa(%d), pulse_maxj(%d)\n",
                        n, stepgen->pulse_maxv, stepgen->pulse_maxa, stepgen->pulse_maxj);
            }
            assert (abs(integer_pos_cmd) <= stepgen->pulse_maxv);
            pulse_accel = integer_pos_cmd - stepgen->pulse_vel;
            pulse_jerk = pulse_accel - stepgen->pulse_accel;
            /* TODO: there's S-CURVE decel bug in tp.c; enable maxa, maxj assertions after resolving it */
            // TODO: assert (abs(pulse_accel) <= stepgen->pulse_maxa);
            // TODO: assert (abs(pulse_jerk) <= stepgen->pulse_maxj);
            stepgen->pulse_vel = integer_pos_cmd;
            stepgen->pulse_accel = pulse_accel;
            stepgen->pulse_jerk = pulse_jerk;
        } else
        {
            /* pulse_type is P(PWM) */
            /* pos_cmd is PWM duty ratio, +/- 0~100 */
            integer_pos_cmd = (int32_t)((*stepgen->pos_cmd * (stepgen->pos_scale)) * FIXED_POINT_SCALE); // 16.16 precision
            if (abs(integer_pos_cmd) > ((int32_t) stepgen->maxvel))
            {
                if (integer_pos_cmd > 0)
                {
                    integer_pos_cmd = (int32_t) stepgen->maxvel;
                } else
                {
                    integer_pos_cmd = (int32_t) -stepgen->maxvel;
                }
            }
        }

        {
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

            // TODO: pack sync_cmd into single-32-bit-word for each joint

            /* packing integer part of position command (16-bit) */
            if (integer_pos_cmd >= 0) {
                sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wou_pos_cmd);
            } else {
                sync_cmd = SYNC_JNT | DIR_N | (POS_MASK & wou_pos_cmd);
            }
            memcpy(data + (2 * n * sizeof(uint16_t)), &sync_cmd, sizeof(uint16_t));

            /* packing fraction part (16-bit) */
            wou_pos_cmd = (abs(integer_pos_cmd)) & FRACTION_MASK;
            sync_cmd = (uint16_t) wou_pos_cmd;
            memcpy(data + (2*n+1) * sizeof(uint16_t), &sync_cmd, sizeof(uint16_t));

            stepgen->rawcount += (int64_t) integer_pos_cmd; // precision: 64.16
            stepgen->prev_pos_cmd = (((double)stepgen->rawcount * stepgen->scale_recip)/(FIXED_POINT_SCALE));
            stepgen->prev_vel_cmd = *stepgen->vel_cmd;
        }

        if (n == (num_joints - 1)) {
            // send to WOU when all axes commands are generated
            wou_cmd(&w_param,
                    WB_WR_CMD,
                    (JCMD_BASE | JCMD_SYNC_CMD), 4 * num_joints, data);
        }

        DPS("       0x%08X%15.7f%15.7f%15.7f",
                integer_pos_cmd,
                (stepgen->prev_pos_cmd),
                *stepgen->pos_fb,
                *stepgen->risc_pos_cmd);

        /* move on to next channel */
        stepgen++;
    }

    sync_cmd = SYNC_EOF;
    memcpy(data, &sync_cmd, sizeof(uint16_t));
    wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), sizeof(uint16_t), data);

#if (TRACE!=0)
    stepgen = arg;
    if (*(stepgen->enable)) {
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

    // export Analog OUT
    for (i = 0; i < 4; i++) {
        retval = hal_pin_s32_newf(HAL_IN, &(addr->out[i]), comp_id,
                "wou.analog.out.%02d", i);
        if (retval != 0) {
            return retval;
        }
        *(addr->out[i]) = 0;
        addr->prev_out[i] = 0;
    }

    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
} // export_analog ()


static int export_stepgen(int num, stepgen_t * addr, char pulse_type)
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

    /* export parameter for position scaling */
    retval = hal_param_float_newf(HAL_RW, &(addr->pos_scale), comp_id, "wou.stepgen.%d.position-scale", num);
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

    retval = hal_pin_float_newf(HAL_OUT, &(addr->risc_pos_cmd), comp_id,
            "wou.stepgen.%d.risc-pos-cmd", num);
    if (retval != 0) {
        return retval;
    }

    /* export pin for velocity feedback (unit/sec) */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel_fb), comp_id,
            "wou.stepgen.%d.vel-fb", num);
    if (retval != 0) {
        return retval;
    }

    /* export pin for velocity feedback (pulse) */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc_vel_p), comp_id,
            "wou.stepgen.%d.enc-vel", num);
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

    retval = hal_pin_bit_newf(HAL_IN, &(addr->homing), comp_id, "wou.stepgen.%d.homing", num);
    if (retval != 0) { return retval; }
    *addr->homing = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(addr->bypass_lsp), comp_id, "wou.stepgen.%d.bypass_lsp", num);
    if (retval != 0) { return retval; }
    *addr->bypass_lsp = 0;
    addr->prev_bypass_lsp = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(addr->bypass_lsn), comp_id, "wou.stepgen.%d.bypass_lsn", num);
    if (retval != 0) { return retval; }
    *addr->bypass_lsn = 0;
    addr->prev_bypass_lsn = 0;

    retval = hal_pin_float_newf(HAL_IN, &(addr->risc_probe_vel), comp_id, "wou.stepgen.%d.risc-probe-vel", num);
    if (retval != 0) { return retval; }
    *addr->risc_probe_vel = 0;

    retval = hal_pin_float_newf(HAL_IN, &(addr->risc_probe_dist), comp_id, "wou.stepgen.%d.risc-probe-dist", num);
    if (retval != 0) { return retval; }
    *addr->risc_probe_dist = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(addr->risc_probe_pin), comp_id, "wou.stepgen.%d.risc-probe-pin", num);
    if (retval != 0) { return retval; }
    *addr->risc_probe_pin = -1;

    retval = hal_pin_s32_newf(HAL_IN, &(addr->risc_probe_type), comp_id, "wou.stepgen.%d.risc-probe-type", num);
    if (retval != 0) { return retval; }
    *addr->risc_probe_type = -1;

    retval = hal_pin_float_newf(HAL_IN, &(addr->uu_per_rev), comp_id, "wou.stepgen.%d.uu-per-rev", num);
    if (retval != 0) {
        return retval;
    }
    *(addr->uu_per_rev) = 0;
    (addr->prev_uu_per_rev) = 0;

    addr->pos_scale = 1.0;
    addr->scale_recip = 0.0;
    addr->freq = 0.0;
    addr->maxvel = 0.0;
    addr->maxaccel = 0.0;
    addr->pulse_type = pulse_type;
    addr->pos_cmd_s = 0;
    /* timing parameter defaults depend on step type */
    addr->step_len = 1;
    /* init the step generator core to zero output */
    /* accumulator gets a half step offset, so it will step half
       way between integer positions, not at the integer positions */
    addr->rawcount = 0;
    addr->prev_pos_cmd = 0;

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
    *(addr->pos_cmd) = 0.0;
    *(addr->vel_cmd) = 0.0;
    (addr->prev_vel_cmd) = 0.0;
    addr->pulse_vel = 0;
    addr->pulse_accel = 0;
    addr->pulse_jerk = 0;

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

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->machine_on), comp_id, "wou.machine-on");
    if (retval != 0) { return retval; }
    *(machine_control->machine_on) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->gantry_en), comp_id, "wou.gantry-en");
    if (retval != 0) { return retval; }
    *(machine_control->gantry_en) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->gantry_lock), comp_id, "wou.gantry-lock");
    if (retval != 0) { return retval; }
    *(machine_control->gantry_lock) = 0;

    // rt_abort: realtime abort command to FPGA
    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->rt_abort), comp_id, "wou.rt.abort");
    if (retval != 0) { return retval; }
    *(machine_control->rt_abort) = 0;

    // export input status pin
    for (i = 0; i < GPIO_IN_NUM; i++) {
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
            hal_pin_u32_newf(HAL_IN, &(machine_control->sync_in_index), comp_id, "wou.sync.in.index");
    *(machine_control->sync_in_index) = 0;	// pin index must not beyond index
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

    for (i = 0; i < GPIO_OUT_NUM; i++) {
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
            hal_pin_bit_newf(HAL_IN, &(machine_control->ahc_state), comp_id, "wou.ahc.state");
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

    /* wou command */
    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->wou_cmd), comp_id,
            "wou.motion.cmd");
    *(machine_control->wou_cmd) = 0;    // pin index must not beyond index
    machine_control->prev_wou_cmd = 0;
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->motion_state), comp_id, "wou.motion-state");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->motion_state) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->jog_sel), comp_id, "wou.motion.jog-sel");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->jog_sel) = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->jog_vel_scale), comp_id, "wou.motion.jog-vel-scale");
    if (retval != 0) {
        return retval;
    }
    *(machine_control->jog_vel_scale) = 0;

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

    for (i=0; i<32; i++) {
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

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->bp_tick), comp_id, "wou.bp-tick");
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

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->homing), comp_id,
            "wou.motion.homing");
    if (retval != 0) { return retval; }
    *(machine_control->homing) = 0;

    // for RISC_CMD REQ and ACK
    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->update_pos_req), comp_id,
            "wou.motion.update-pos-req");
    if (retval != 0) { return retval; }
    *(machine_control->update_pos_req) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->rcmd_seq_num_req), comp_id,
            "wou.motion.rcmd-seq-num-req");
    if (retval != 0) { return retval; }
    *(machine_control->rcmd_seq_num_req) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->rcmd_state), comp_id,
            "wou.motion.rcmd-state");
    if (retval != 0) { return retval; }
    *(machine_control->rcmd_state) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->update_pos_ack), comp_id,
            "wou.motion.update-pos-ack");
    if (retval != 0) { return retval; }
    *(machine_control->update_pos_ack) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->rcmd_seq_num_ack), comp_id,
            "wou.motion.rcmd-seq-num-ack");
    if (retval != 0) { return retval; }
    *(machine_control->rcmd_seq_num_ack) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->max_tick_time), comp_id, "wou.max_tick_time");
    *(machine_control->max_tick_time) = 0;    // pin index must not beyond index
    if (retval != 0) {
        return retval;
    }

    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->probe_result), comp_id, "wou.motion.probe-result");
    if (retval != 0) { return retval; }
    *(machine_control->probe_result) = 0;

    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->machine_moving), comp_id, "wou.motion.machine-is-moving");
    if (retval != 0) { return retval; }
    *(machine_control->machine_moving) = 0;

    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->ahc_doing), comp_id, "wou.ahc.doing");
    if (retval != 0) { return retval; }
    *(machine_control->ahc_doing) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->spindle_joint_id), comp_id, "wou.motion.spindle-joint-id");
    if (retval != 0) { return retval; }
    *(machine_control->spindle_joint_id) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_din), comp_id, "wou.trigger.din");
    if (retval != 0) { return retval; }
    *(machine_control->trigger_din) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_ain), comp_id, "wou.trigger.ain");
    if (retval != 0) { return retval; }
    *(machine_control->trigger_ain) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_type), comp_id, "wou.trigger.type");
    if (retval != 0) { return retval; }
    *(machine_control->trigger_type) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->trigger_cond), comp_id, "wou.trigger.cond");
    if (retval != 0) { return retval; }
    *(machine_control->trigger_cond) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_level), comp_id, "wou.trigger.level");
    if (retval != 0) { return retval; }
    *(machine_control->trigger_level) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->trigger_enable), comp_id, "wou.trigger.enable");
    if (retval != 0) { return retval; }
    *(machine_control->trigger_enable) = 0;
    (machine_control->prev_trigger_enable) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->probing), comp_id, "wou.motion.probing");
    if (retval != 0) { return retval; }
    *(machine_control->probing) = 0;
    (machine_control->prev_probing) = 0;

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->trigger_result), comp_id, "wou.trigger.result");
    if (retval != 0) {
        return retval;
    }

    /* restore saved message level*/
    rtapi_set_msg_level(msg);
    return 0;
}

#endif	// RTAPI_SIM

// vim:sw=4:sts=4:et:
