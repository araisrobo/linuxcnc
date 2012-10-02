/********************************************************************
* Description: yyzzkins.c
*   Simple kinematics for gantry type machine with double Y and Z
*
* Author: Yishin Li, ARAIS ROBOT TECHNOLOGY
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2012 All rights reserved.
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

typedef struct yyzz_pins {
    hal_float_t *yy_offset;
    hal_float_t *zz_offset;
    hal_float_t *gantry_polarity_y;
    hal_float_t *gantry_polarity_z;
} yyzz_pins_t;

static yyzz_pins_t *yyzz_pins;

#define YY_OFFSET (*(yyzz_pins->yy_offset))
#define ZZ_OFFSET (*(yyzz_pins->zz_offset))
#define GANTRY_POLARITY_Y (*(yyzz_pins->gantry_polarity_y))
#define GANTRY_POLARITY_Z (*(yyzz_pins->gantry_polarity_z))

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    pos->tran.y = joints[0];
    pos->tran.z = joints[2];

    DP("kFWD: y(%f), z(%f), j0(%f), j1(%f), j2(%f), j3(%f)\n",
        pos->tran.y, pos->tran.z, joints[0], joints[1], joints[2], joints[3]);

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    joints[0] = pos->tran.y;
    joints[1] = (pos->tran.y + YY_OFFSET) * GANTRY_POLARITY_Y;

    joints[2] = pos->tran.z;
    joints[3] = (pos->tran.z + ZZ_OFFSET) * GANTRY_POLARITY_Z;

    DP("kINV: j0(%f), j1(%f), j2(%f), j3(%f), y(%f), z(%f)\n",
              joints[0], joints[1], joints[2], joints[3], pos->tran.y, pos->tran.z);

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
    comp_id = hal_init("yyzzkins");
    if (comp_id < 0) {
        // ERROR
        DP("ABORT\n");
        return comp_id;
    }
    
    yyzz_pins = hal_malloc(sizeof(yyzz_pins_t));
    if (!yyzz_pins) goto error;
    if ((res = hal_pin_float_new("yyzzkins.yy_offset", HAL_IN, &(yyzz_pins->yy_offset), comp_id)) < 0) goto error;
    if ((res = hal_pin_float_new("yyzzkins.zz_offset", HAL_IN, &(yyzz_pins->zz_offset), comp_id)) < 0) goto error;
    if ((res = hal_pin_float_new("yyzzkins.gantry-polarity-y", HAL_IN, &(yyzz_pins->gantry_polarity_y), comp_id)) < 0) goto error;
    if ((res = hal_pin_float_new("yyzzkins.gantry-polarity-z", HAL_IN, &(yyzz_pins->gantry_polarity_z), comp_id)) < 0) goto error;

    hal_ready(comp_id);
    DP ("done\n");
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
