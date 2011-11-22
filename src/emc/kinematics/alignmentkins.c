/********************************************************************
* Description: alignmentkins.c
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
#define TRACE 1
#include "dptrace.h"
#if (TRACE!=0)
// FILE *dptrace = fopen("dptrace.log","w");
static FILE *dptrace;
#endif

typedef struct {
    hal_float_t theta; // unit: rad
} align_params_t;

static align_params_t *align_params;

#define THETA (align_params->theta)

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    // double c_rad = -joints[5]*M_PI/180;
    pos->tran.x = joints[0] * cos(THETA) - joints[1] * sin(THETA);
    pos->tran.y = joints[0] * sin(THETA) + joints[1] * cos(THETA);
    pos->tran.z = joints[2];
    pos->a = joints[3];
    pos->b = joints[4];
    pos->c = joints[5];
    // pos->u = joints[6];
    // pos->v = joints[7];
    // pos->w = joints[8];

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
    joints[0] = pos->tran.x * cos(THETA) - pos->tran.y * sin(THETA);
    joints[1] = pos->tran.x * sin(THETA) + pos->tran.y * cos(THETA);
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    // joints[6] = pos->u;
    // joints[7] = pos->v;
    // joints[8] = pos->w;

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
    dptrace = fopen("alignmentkins.log","w");
#endif

    comp_id = hal_init("alignmentkins");
    if (comp_id < 0) {
        // ERROR
        return comp_id;
    }
    
    align_params = hal_malloc(sizeof(align_params_t));
    if (!align_params) goto error;
    if ((res = hal_param_float_new("alignmentkins.theta", HAL_RW, &(align_params->theta), comp_id)) < 0) goto error;
    
    // align_params->theta = 0;
    align_params->theta = 0.78539815;   // 45 degree

    hal_ready(comp_id);
    DP ("success\n");
    return 0;
    
error:
    hal_exit(comp_id);
#if (TRACE!=0)
    fclose(dptrace);
#endif
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
