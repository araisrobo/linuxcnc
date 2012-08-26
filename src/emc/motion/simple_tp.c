/********************************************************************
* Description: simple_tp.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Author: jmkasunich
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>

#include "simple_tp.h"
#include "rtapi_math.h"
#include "posemath.h"

// to disable DP(): #define TRACE 0
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
// FILE *dptrace = fopen("dptrace.log","w");
static FILE *dptrace = 0;
static uint32_t dt = 0;
#endif

/*
 Continuous form
 PT = P0 + V0T + 1/2A0T2 + 1/6JT3
 VT = V0 + A0T + 1/2 JT2
 AT = A0 + JT

 Discrete time form (讓 T 成為 1, 那麼 T^2 == 1, T^3 == 1)
 PT = PT + VT + 1/2AT + 1/6J
 VT = VT + AT + 1/2JT
 AT = AT + JT
 */

void simple_tp_update(simple_tp_t *tp, double period)
{
    double max_dv, tiny_dp, pos_err, vel_req;
    double max_da;
    double dv_upper, dv_lower;
    double acc_req;
    double vel_err;
    double tiny_dv;

#if (TRACE!=0)
    double vel_req_cmd;

    if (dptrace == 0) {
        dptrace = fopen("simple_tp.log","w");
        dt = 0;
        DPS("%11s  %15s%15s%15s%15s%15s%15s%15s%15s%7s%7s ",
             "#dt",  "pos_cmd", "curr_pos", "curr_vel", "curr_acc", "max_dv", "vel_req", "max_vel", "max_acc", "enable", "active"
           );
    }
#endif

    tp->active = 0;
    /* compute max change in velocity per servo period */
    max_dv = tp->max_acc * period;                  // unit: dist/sec
    max_da = tp->max_jerk * period;                 // max_da: unit: dist/sec^2
    if (tp->position_mode) {
        dv_upper = max_dv;
        dv_lower = -max_dv;
    } else {
        // s-curve for non-positioning motion only
        // this max_dv is buggy; it's just for T-curve vel_req calc
        dv_upper = (tp->curr_acc + max_da) * period;    // dv_upper: unit: dist/sec
        dv_lower = (tp->curr_acc - max_da) * period;
        if (dv_upper > max_dv) {
            dv_upper = max_dv;
        }
        if (dv_lower < -max_dv) {
            dv_lower = -max_dv;
        }
    }
        
    /* compute a tiny position range, to be treated as zero */
    /* calculate desired velocity */
    if (tp->enable) {
        tiny_dp = max_dv * period * 0.001;
        /* accurately positioning at "tp->pos_cmd" */
        /* planner enabled, request a velocity that tends to drive
           pos_err to zero, but allows for stopping without position
           overshoot */
        pos_err = tp->pos_cmd - tp->curr_pos;
        /* positive and negative errors require some sign flipping to
           avoid sqrt(negative) */
        if (pos_err > tiny_dp) {
            vel_req = -max_dv +
                       sqrt(2.0 * tp->max_acc * pos_err + max_dv * max_dv);
            /* mark planner as active */
            tp->active = 1;
        } else if (pos_err < -tiny_dp) {
            vel_req =  max_dv -
                       sqrt(-2.0 * tp->max_acc * pos_err + max_dv * max_dv);
            /* mark planner as active */
            tp->active = 1;
        } else {
            /* within 'tiny_dp' of desired pos, no need to move */
            vel_req = 0.0;
        }
    } else {
	/* planner disabled, request zero velocity */
	vel_req = 0.0;
	/* and set command to present position to avoid movement when
	   next enabled */
	tp->pos_cmd = tp->curr_pos;
    }

#if (TRACE!=0)
    vel_req_cmd = vel_req;
#endif

    if (tp->position_mode == 1) {
        // accurate T-curve positioning
        /* ramp velocity toward request at accel limit */
        if (vel_req > tp->curr_vel + dv_upper) {
            vel_req = tp->curr_vel + dv_upper;
        } else if (vel_req < tp->curr_vel + dv_lower) {
            vel_req = tp->curr_vel + dv_lower;
        }
        /* limit velocity request */
        if (vel_req > tp->max_vel) {
            vel_req = tp->max_vel;
        } else if (vel_req < -tp->max_vel) {
            vel_req = -tp->max_vel;
        }
        tp->curr_acc = (vel_req - tp->curr_vel) / period;
        tp->curr_vel = vel_req;
    } else {
        // s-curve for non-positioning motion only
        /* compute a tiny velocity range, to be treated as zero */
        tiny_dv = max_da * period * 0.001;
        vel_err = vel_req - tp->curr_vel;
        if (vel_err > tiny_dv) {
            acc_req = -max_da +
                       sqrt(2.0 * tp->max_jerk * vel_err + max_da * max_da);
        } else if (vel_err < -tiny_dv) {
            // vel_req <= 0
            acc_req =  max_da -
                       sqrt(-2.0 * tp->max_jerk * vel_err + max_da * max_da);
        } else {
            acc_req = 0;  // acc_req is within tiny_dv
        }

#if (TRACE!=0)
        if ((dt % 5) == 0) {        // check for j0 of 5-joints
            DP("\nvel_err(%f) acc_req(%f)\n", vel_err, acc_req);
        }
#endif

        /* limit accel toward request at jerk limit */
        if (acc_req > tp->curr_acc + max_da) {
            acc_req = tp->curr_acc + max_da;
        } else if (acc_req < tp->curr_acc - max_da) {
            acc_req = tp->curr_acc - max_da;
        }

        /* limit accel request */
        if (acc_req > tp->max_acc) {
            tp->curr_acc = tp->max_acc;
        } else if (acc_req < -tp->max_acc) {
            tp->curr_acc = -tp->max_acc;
        } else {
            tp->curr_acc = acc_req;
        }

        tp->curr_vel += (tp->curr_acc * period);

        /* check for still moving */
        if (fabs(tp->curr_vel) <= tiny_dv) {
            tp->curr_vel = 0;
        }
    }

    /* check for still moving */
    if (tp->curr_vel != 0.0) {
	/* yes, mark planner active */
	tp->active = 1;
    }

    /* integrate velocity to get new position */
    tp->curr_pos += tp->curr_vel * period;

#if (TRACE!=0)
    if ((dt % 5) == 0) {        // check for j0 of 5-joints
        DPS("\n%11u ", dt);
        DPS(" %15.7f%15.7f%15.7f%15.7f%15.7f%15.7f%15.7f%15.7f%7d%7d",
              tp->pos_cmd, tp->curr_pos, tp->curr_vel, tp->curr_acc, max_dv, vel_req_cmd, tp->max_vel, tp->max_acc, tp->enable, tp->active
           );
    }
    dt += 1;
#endif

}
