/********************************************************************
* Description: ar_tap_kins.c
*   trivial kins for tapping machine
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

#include "string.h"
#include "assert.h"
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

const char *machine_type = "";
RTAPI_MP_STRING(machine_type, "Tapping Machine Type");

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{

    pos->tran.x = joints[0];
    pos->tran.y = joints[1];
    pos->tran.z = joints[3];
    pos->v = joints[2];
    pos->w = joints[4];
    pos->s = joints[5];

    DP("kFWD: x(%f), y(%f), j0(%f), j1(%f), j2(%f)\n",
        pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2]);
    DP("kFWD: s(%f), j5(%f)\n", pos->s, joints[5]);

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->v;  // YY
    joints[3] = pos->tran.z;
    joints[4] = pos->w;
    joints[5] = pos->s;
    // fprintf(stderr,"kI j0(%f) j1(%f)\n",joints[0], joints[1]);
    DP("kINV: x(%f), y(%f), j0(%f), j1(%f), j2(%f)\n",
       pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2]);
    DP("kINV: s(%f), j5(%f)\n", pos->s, joints[5]);

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


int comp_id;
int rtapi_app_main(void) 
{
#if (TRACE!=0)
    dptrace = fopen("kins.log","w");
#endif
    
    DP("begin\n");
    comp_id = hal_init("ar_tap_kins");
    if (comp_id < 0) {
        // ERROR
        DP("ABORT\n");
        return comp_id;
    }
    
    /* export param for scaled velocity (frequency in Hz) */
    hal_ready(comp_id);
    DP ("success\n");
    return 0;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
