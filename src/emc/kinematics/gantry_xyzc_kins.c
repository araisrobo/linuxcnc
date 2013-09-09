/********************************************************************
* Description: gantry_xyzc_kins.c
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
static FILE *dptrace;
#endif

typedef struct {
    hal_float_t *gantry_polarity;
    hal_float_t *yy_offset;
} align_pins_t;

static align_pins_t *align_pins;

#define GANTRY_POLARITY (*(align_pins->gantry_polarity))
#define YY_OFFSET       (*(align_pins->yy_offset))

const char *machine_type = "";
RTAPI_MP_STRING(machine_type, "Gantry Machine Type");

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
    pos->c = joints[4];
    pos->s = joints[5];

    DP("kFWD: x(%f), y(%f), j0(%f), j1(%f), j2(%f), yy_offset(%f),POLARITY(%f)\n",
        pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2], YY_OFFSET, GANTRY_POLARITY);
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
    joints[2] = pos->tran.y - (YY_OFFSET * GANTRY_POLARITY);  // YY
    joints[3] = pos->tran.z;
    joints[4] = pos->c;
    joints[5] = pos->s;
    DP("kINV: x(%f), y(%f), j0(%f), j1(%f), j2(%f), yy_offset(%f)\n",
       pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2], YY_OFFSET);
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
    int res = 0;

#if (TRACE!=0)
    dptrace = fopen("kins.log","w");
#endif
    
    DP("begin\n");
    comp_id = hal_init("gantry_xyzc_kins");
    if (comp_id < 0) {
        // ERROR
        DP("ABORT\n");
        return comp_id;
    }
    
    align_pins = hal_malloc(sizeof(align_pins_t));
    if (!align_pins) goto error;
    if ((res = hal_pin_float_new("gantry-xyzc-kins.yy-offset", HAL_IN, &(align_pins->yy_offset), comp_id)) < 0) goto error;
    YY_OFFSET = 0;

    /* export param for scaled velocity (frequency in Hz) */
    res = hal_pin_float_new("gantry-xyzc-kins.gantry-polarity", HAL_IN, &(align_pins->gantry_polarity), comp_id);
    if (res != 0) {
        goto error;
    }
    GANTRY_POLARITY = 1.0;

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
