/********************************************************************
* Description: align_gantry_kins.c
*   Simple example kinematics for thita alignment in software
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Yishin Li, ARAIS ROBOT TECHNOLOGY
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2011 All rights reserved.
*
********************************************************************/

#include "rtapi_math.h"
#include "kinematics.h"		/* these decls */

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

// to disable DP():
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
// FILE *dptrace = fopen("dptrace.log","w");
static FILE *dptrace;
#endif

typedef struct {
    hal_float_t *theta; // unit: rad
    double prev_theta;
    double x_cent, prev_x_cent, y_cent, prev_y_cent;
    double x_offset, prev_x_offset, y_offset, prev_y_offset;
    hal_bit_t * touch_off_cent;
} align_pins_t;

static align_pins_t *align_pins;

#define THETA           (*(align_pins->theta))
#define PREV_THETA      (align_pins->prev_theta)
#define PREV_X_OFFSET   (align_pins->prev_x_offset)
#define PREV_Y_OFFSET   (align_pins->prev_y_offset)
#define PREV_X_CENT     (align_pins->prev_y_cent)
#define PREV_Y_CENT     (align_pins->prev_x_cent)
#define X_OFFSET        (align_pins->x_offset)
#define Y_OFFSET        (align_pins->y_offset)
#define X_CENT          (align_pins->x_cent)
#define Y_CENT          (align_pins->y_cent)
#define TOUCH_OFF_CENT	(*(align_pins->touch_off_cent))
void cord_change_handler(EmcPose * pos, double  * joints) {
    // update XYZ position but keep joint position unchanged 
    if (TOUCH_OFF_CENT == 1) {
        X_CENT = joints[0];
        Y_CENT = joints[1];
    }
    if (THETA != PREV_THETA ||
	TOUCH_OFF_CENT == 1) { 
	if (joints[0] == 0 && joints[1] == 0) {
	    return;
	}
        // fprintf(stderr, "THETA(%f)\n", THETA);
        // fprintf(stderr, "x-cent(%f) y-cent(%f) x-offset(%f) y-offset(%f)\n", X_CENT, Y_CENT, X_OFFSET, Y_OFFSET);
        // fprintf(stderr, "1:x(%f) y(%f) \nj0(%f) j1(%f)\n", pos->tran.x, pos->tran.y,
        //                joints[0], joints[1]);
        pos->tran.x  = (joints[0] - X_CENT - X_OFFSET) * cos(THETA) +
                       (joints[1] - Y_CENT - Y_OFFSET) * sin(THETA) + X_OFFSET + X_CENT;
        pos->tran.y  = -(joints[0] - X_CENT - X_OFFSET)* sin(THETA) +
                        (joints[1] - Y_CENT - Y_OFFSET) * cos(THETA) + Y_OFFSET + Y_CENT;
        // fprintf(stderr, "2:x(%f) y(%f) \nj0(%f) j1(%f)\n", pos->tran.x, pos->tran.y,
        //         joints[0], joints[1]);
        PREV_THETA    = THETA;
        PREV_X_OFFSET = X_OFFSET; 
        PREV_Y_OFFSET = Y_OFFSET; 
        PREV_X_CENT   = X_CENT;   
        PREV_Y_CENT   = Y_CENT;   
    }
    TOUCH_OFF_CENT = 0;
}

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    // double c_rad = -joints[5]*M_PI/180;
    // fprintf(stderr,"kF j0(%f) j1(%f)\n",joints[0], joints[1]);
    cord_change_handler((EmcPose *)pos, (double*)joints);
    pos->tran.x  = (joints[0] - X_CENT - X_OFFSET) * cos(THETA) +
                   (joints[1] - Y_CENT - Y_OFFSET) * sin(THETA) + X_OFFSET + X_CENT;
    pos->tran.y  = -(joints[0] - X_CENT - X_OFFSET)* sin(THETA) +
                    (joints[1] - Y_CENT - Y_OFFSET) * cos(THETA) + Y_OFFSET + Y_CENT;
//    pos->tran.x  = (joints[0] - X_OFFSET) * cos(THETA) + (joints[1] - Y_OFFSET) * sin(THETA) ;
//    pos->tran.y  = -(joints[0] - X_OFFSET)* sin(THETA) + (joints[1] - Y_OFFSET) * cos(THETA);
//    pos->tran.x = joints[0] * cos(THETA) - joints[1] * sin(THETA);
//    pos->tran.y =joints[0] * sin(THETA) + joints[1] * cos(THETA);
    pos->tran.z = joints[3];
    pos->a = joints[4];
    pos->b = joints[5];
    pos->c = joints[6];
    // pos->u = joints[6];
    // pos->v = joints[7];
    // pos->w = joints[8];
    // fprintf(stderr,"kF x(%f) y(%f)\n",pos->tran.x, pos->tran.y);
    DP("kFWD: theta(%f), j0(%f), j1(%f), x(%f), y(%f)\n",
       THETA, joints[0], joints[1], pos->tran.x, pos->tran.y);

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    // double c_rad = pos->c*M_PI/180;
    double rad, x_cent, y_cent;
    // fprintf(stderr,"kI j0(%f) j1(%f) THETA(%f) X_CENT(%f) Y_CENT(%f)\n",joints[0], joints[1],
    // THETA, X_CENT, Y_CENT);
    // fprintf(stderr,"KI x(%f) y(%f)\n", pos->tran.x, pos->tran.y);
    if (THETA == PREV_THETA) {
	rad = THETA;
    } else {
        rad = PREV_THETA;
    }
    if (X_CENT == PREV_X_CENT) {
	x_cent = X_CENT;
    } else {
        x_cent = PREV_X_CENT;
    }
    if (Y_CENT == PREV_Y_CENT) {
	y_cent = Y_CENT;
    } else {
        y_cent = PREV_Y_CENT;
    }
    joints[0] = (pos->tran.x - x_cent - X_OFFSET) * cos(rad) -
                (pos->tran.y - y_cent - Y_OFFSET) * sin(rad) + X_OFFSET + x_cent;

    joints[1] = (pos->tran.x - x_cent - X_OFFSET) * sin(rad) +
                (pos->tran.y - y_cent - Y_OFFSET) * cos(rad) + Y_OFFSET + y_cent;
    joints[2] = joints[1];      // joint2: YY
    joints[3] = pos->tran.z;
    joints[4] = pos->a;
    joints[5] = pos->b;
    joints[6] = pos->c;
    // joints[6] = pos->u;
    // joints[7] = pos->v;
    // joints[8] = pos->w;
    // fprintf(stderr,"kI j0(%f) j1(%f)\n",joints[0], joints[1]);
    DP("kINV: theta(%f), j0(%f), j1(%f), x(%f), y(%f)\n",
       THETA, joints[0], joints[1], pos->tran.x, pos->tran.y);

    return 0;
}

/* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void) 
{
    int res = 0;

#if (TRACE!=0)
    dptrace = fopen("kins.log","w");
#endif
    
    DP("begin\n");
    comp_id = hal_init("align_gantry_kins");
    if (comp_id < 0) {
        // ERROR
        DP("ABORT\n");
        return comp_id;
    }
    
    align_pins = hal_malloc(sizeof(align_pins_t));
    if (!align_pins) goto error;
    if ((res = hal_pin_float_new("align-gantry-kins.theta", HAL_IN, &(align_pins->theta), comp_id)) < 0) goto error;
    THETA = 0;
    if ((res = hal_pin_bit_new("align-gantry-kins.touch-off-cent", HAL_IO, &(align_pins->touch_off_cent), comp_id)) < 0) goto error;
    TOUCH_OFF_CENT = 0;
    // align_pins->theta = 0;
    // align_pins->theta = 0.78539815;   // 45 degree

    hal_ready(comp_id);
    DP ("success\n");
    return 0;
    
error:
    DP("ERROR\n");
    hal_exit(comp_id);
#if (TRACE!=0)
    fclose(dptrace);
#endif
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
