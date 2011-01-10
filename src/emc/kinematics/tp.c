/********************************************************************
 * Description: tp.c
 *   Trajectory planner based on TC elements
 *
 *   Derived from a work by Fred Proctor & Will Shackleford
 *
 * Author:
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2004 All rights reserved.
 ********************************************************************/

#include "rtapi.h"		/* rtapi_print_msg */
#include "rtapi_string.h"       /* NULL */
#include "posemath.h"
#include "tc.h"
#include "tp.h"
#include "rtapi_math.h"
#include "../motion/motion.h"
#include "hal.h"
#include "../motion/mot_priv.h"
#include "motion_debug.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#define STATE_DEBUG 0  // for state machine debug
// to disable DP(): #define TRACE 0
#define TRACE 0
#include <stdint.h>
#include "dptrace.h"
#if (TRACE!=0)
static FILE* dptrace = 0;
static uint32_t _dt = 0;
#endif
#define VELOCITY_EPSTHON 1e-1
#define EPSTHON 1e-6

extern emcmot_status_t *emcmotStatus;
extern emcmot_debug_t *emcmotDebug;
static int immediate_state;
int output_chan = 0;
syncdio_t syncdio; //record tpSetDout's here
pos_comp_en_t pos_comp_en;

int tpCreate(TP_STRUCT * tp, int _queueSize, TC_STRUCT * tcSpace) {
    if (0 == tp) {
        return -1;
    }

    if (_queueSize <= 0) {
        tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->queueSize = _queueSize;
    }

    /* create the queue */
    if (-1 == tcqCreate(&tp->queue, tp->queueSize, tcSpace)) {
        return -1;
    }

#if (TRACE!=0)
    if (!dptrace) {
        dptrace = fopen("tp.log", "w");
        /* prepare header for gnuplot */
        DPS ("%11s%15s%15s%15s%15s%15s%15s%15s\n",
                "#dt", "newaccel", "newvel", "cur_vel", "progress", "target", "dist_to_go", "tolerance");
    }
    _dt += 1;
#endif

    /* init the rest of our data */
    return tpInit(tp);
}

// this clears any potential DIO toggles
// anychanged signals if any DIOs need to be changed
// dios[i] = 1, DIO needs to get turned on, -1 = off
int tpClearDIOs() {
    int i;
    syncdio.anychanged = 0;
    syncdio.sync_input_triggered = 0;
    for (i = 0; i < emcmotConfig->numDIO; i++)
        syncdio.dios[i] = 0;
    // also clean sync_input status
    /* for (i = 0; i < emcmotConfig->numSyncIn; i++)
     syncdio.sync_in[i] = 0;*/
    syncdio.sync_in = 255;
    syncdio.wait_type = 0;
    syncdio.timeout = 0.0;
    return 0;
}

int tpClearPosCompEn() {
    pos_comp_en.pos_comp_en_triggered = 0;
    pos_comp_en.en_flag = 0;
    pos_comp_en.pos_comp_ref = 0;
    return 0;
}
/*
 tpClear() is a "soft init" in the sense that the TP_STRUCT configuration
 parameters (cycleTime, vMax, and aMax) are left alone, but the queue is
 cleared, and the flags are set to an empty, ready queue. The currentPos
 is left alone, and goalPos is set to this position.

 This function is intended to put the motion queue in the state it would
 be if all queued motions finished at the current position.
 */
int tpClear(TP_STRUCT * tp) {
    tcqInit(&tp->queue);
    tp->queueSize = 0;
    tp->goalPos = tp->currentPos;
    tp->nextId = 0;
    //    DP("tp->nextId(%d)\n", tp->nextId);
    tp->execId = 0;
    tp->motionType = 0;
    tp->termCond = TC_TERM_COND_BLEND;
    tp->tolerance = 0.0;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->pausing = 0;
    tp->vScale = emcmotStatus->net_feed_scale;
    tp->synchronized = 0;
    tp->velocity_mode = 0;
    tp->uu_per_rev = 0.0;
    emcmotStatus->spindleSync = 0;
    emcmotStatus->current_vel = 0.0;
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->distance_to_go = 0.0;
    ZERO_EMC_POSE(emcmotStatus->dtg);
    tpClearPosCompEn();
    return tpClearDIOs();
}

int tpInit(TP_STRUCT * tp) {
    tp->cycleTime = 0.0;
    tp->vLimit = 0.0;
    tp->vScale = 1.0;
    tp->aMax = 0.0;
    tp->vMax = 0.0;
    tp->ini_maxvel = 0.0;
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;

    ZERO_EMC_POSE(tp->currentPos);

    return tpClear(tp);
}

int tpSetCycleTime(TP_STRUCT * tp, double secs) {
    if (0 == tp || secs <= 0.0) {
        return -1;
    }

    tp->cycleTime = secs;

    return 0;
}

// This is called before adding lines or circles, specifying
// vMax (the velocity requested by the F word) and
// ini_maxvel, the max velocity possible before meeting
// a machine constraint caused by an AXIS's max velocity.
// (the TP is allowed to go up to this high when feed 
// override >100% is requested)  These settings apply to
// subsequent moves until changed.

int tpSetVmax(TP_STRUCT * tp, double vMax, double ini_maxvel) {
    if (0 == tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return -1;
    }

    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;

    return 0;
}

// I think this is the [TRAJ] max velocity.  This should
// be the max velocity of the TOOL TIP, not necessarily
// any particular axis.  This applies to subsequent moves
// until changed.

int tpSetVlimit(TP_STRUCT * tp, double vLimit) {
    if (!tp)
        return -1;

    if (vLimit < 0.)
        tp->vLimit = 0.;
    else
        tp->vLimit = vLimit;

    return 0;
}

// Set max accel

int tpSetAmax(TP_STRUCT * tp, double aMax) {
    if (0 == tp || aMax <= 0.0) {
        return -1;
    }

    tp->aMax = aMax;

    return 0;
}

/*
 tpSetId() sets the id that will be used for the next appended motions.
 nextId is incremented so that the next time a motion is appended its id
 will be one more than the previous one, modulo a signed int. If
 you want your own ids for each motion, call this before each motion
 you append and stick what you want in here.
 */
int tpSetId(TP_STRUCT * tp, int id) {
    if (0 == tp) {
        return -1;
    }

    tp->nextId = id;
    //    DP("tp->nextId(%d) emcmotCommand->id(%d)\n", tp->nextId, emcmotCommand->id);

    return 0;
}

/*
 tpGetExecId() returns the id of the last motion that is currently
 executing.
 */
int tpGetExecId(TP_STRUCT * tp) {
    if (0 == tp) {
        return -1;
    }

    return tp->execId;
}

/*
 tpSetTermCond(tp, cond) sets the termination condition for all subsequent
 queued moves. If cond is TC_TERM_STOP, motion comes to a stop before
 a subsequent move begins. If cond is TC_TERM_BLEND, the following move
 is begun when the current move decelerates.
 */
int tpSetTermCond(TP_STRUCT * tp, int cond, double tolerance) {
    if (0 == tp) {
        return -1;
    }

    if (cond != TC_TERM_COND_STOP && cond != TC_TERM_COND_BLEND) {
        return -1;
    }

    tp->termCond = cond;
    tp->tolerance = tolerance;

    return 0;
}

// Used to tell the tp the initial position.  It sets
// the current position AND the goal position to be the same.  
// Used only at TP initialization and when switching modes.

int tpSetPos(TP_STRUCT * tp, EmcPose pos) {
    if (0 == tp) {
        return -1;
    }

    tp->currentPos = pos;
    tp->goalPos = pos;

    return 0;
}

int tpAddRigidTap(TP_STRUCT *tp, EmcPose end, double vel, double ini_maxvel,
        double acc, unsigned char enables) {
    TC_STRUCT tc;
    PmLine line_xyz;
    PmPose start_xyz, end_xyz;
    PmCartesian abc, uvw;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
    rtapi_print_msg(RTAPI_MSG_ERR, "TODO: add jerk infomation\n");
    assert(0);
    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return -1;
    }

    start_xyz.tran = tp->goalPos.tran;
    end_xyz.tran = end.tran;

    start_xyz.rot = identity_quat;
    end_xyz.rot = identity_quat;

    // abc cannot move
    abc.x = tp->goalPos.a;
    abc.y = tp->goalPos.b;
    abc.z = tp->goalPos.c;

    uvw.x = tp->goalPos.u;
    uvw.y = tp->goalPos.v;
    uvw.z = tp->goalPos.w;

    pmLineInit(&line_xyz, start_xyz, end_xyz);

    tc.sync_accel = 0;
    tc.cycle_time = tp->cycleTime;
    tc.coords.rigidtap.reversal_target = line_xyz.tmag;

    // allow 10 turns of the spindle to stop - we don't want to just go on forever
    tc.target = line_xyz.tmag + 10. * tp->uu_per_rev;

    tc.progress = 0.0;
    tc.accel_state = ACCEL_S7;
    tc.distance_to_go = tc.target;
    tc.accel_time = 0.0;
    tc.reqvel = vel;
    tc.maxaccel = acc;
    // FIXME: the accel-increase-rate(jerk) is set as tc->maxaccel/sec
    // TODO: define accel-increase-rate(jerk) at CONFIG-FILE
    // tc.jerk = 9.0 * acc;
    DP("TODO: fix tc.jerk\n");
    assert(0);
    tc.feed_override = 0.0;
    tc.maxvel = ini_maxvel;
    tc.id = tp->nextId;
    tc.active = 0;
    tc.atspeed = 1;

    tc.cur_accel = 0.0;
    tc.currentvel = 0.0;
    // tc.blending = 0;
    // tc.blend_vel = 0.0;
    // tc.nexttc_vel= 0.0;

    tc.coords.rigidtap.xyz = line_xyz;
    tc.coords.rigidtap.abc = abc;
    tc.coords.rigidtap.uvw = uvw;
    tc.coords.rigidtap.state = TAPPING;
    tc.motion_type = TC_RIGIDTAP;

    tc.canon_motion_type = 0;
    tc.blend_with_next = 0;
    tc.tolerance = tp->tolerance;

    if (!tp->synchronized) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Cannot add unsynchronized rigid tap move.\n");
        return -1;
    }
    tc.synchronized = tp->synchronized;

    tc.uu_per_rev = tp->uu_per_rev;
    tc.velocity_mode = tp->velocity_mode;
    tc.enables = enables;

    if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
        tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
    } else {
        tc.syncdio.anychanged = 0;
        tc.syncdio.sync_input_triggered = 0;
    }

    if (tc.pos_comp_en.pos_comp_en_triggered != 0) {
        tc.pos_comp_en = pos_comp_en;
        tpClearPosCompEn();
    } else {
        tc.pos_comp_en.en_flag = 0;
        tc.pos_comp_en.pos_comp_en_triggered = 0;
        tc.pos_comp_en.pos_comp_ref = 0;
    }

    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return -1;
    }

    // do not change tp->goalPos here,
    // since this move will end just where it started

    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;
    //   DP("tp->nextId(%d)\n", tp->nextId);

    return 0;
}

// Add a straight line to the tc queue.  This is a coordinated
// move in any or all of the six axes.  it goes from the end
// of the previous move to the new end specified here at the
// currently-active accel and vel settings from the tp struct.
// EMC_MOTION_TYPE_FEED
int tpAddLine(TP_STRUCT * tp, EmcPose end, int type, double vel,
        double ini_maxvel, double acc, double ini_maxjerk,
        unsigned char enables, char atspeed) {
    TC_STRUCT tc;
    PmLine line_xyz, line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
    //    fprintf(stderr,"tpAddline(): ini_maxjerk(%f) req_vel(%f) req_acc(%f) ini_maxvel(%f)\n",
    //            ini_maxjerk, vel, acc, ini_maxvel);
    if (ini_maxjerk == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "jerk is not provided or jerk is 0\n");
        assert(ini_maxjerk > 0);
    }
    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return -1;
    }

    start_xyz.tran = tp->goalPos.tran;
    end_xyz.tran = end.tran;

    start_uvw.tran.x = tp->goalPos.u;
    start_uvw.tran.y = tp->goalPos.v;
    start_uvw.tran.z = tp->goalPos.w;
    end_uvw.tran.x = end.u;
    end_uvw.tran.y = end.v;
    end_uvw.tran.z = end.w;

    start_abc.tran.x = tp->goalPos.a;
    start_abc.tran.y = tp->goalPos.b;
    start_abc.tran.z = tp->goalPos.c;
    end_abc.tran.x = end.a;
    end_abc.tran.y = end.b;
    end_abc.tran.z = end.c;

    start_xyz.rot = identity_quat;
    end_xyz.rot = identity_quat;
    start_uvw.rot = identity_quat;
    end_uvw.rot = identity_quat;
    start_abc.rot = identity_quat;
    end_abc.rot = identity_quat;

    pmLineInit(&line_xyz, start_xyz, end_xyz);
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);

    tc.sync_accel = 0;
    tc.cycle_time = tp->cycleTime;

    if (!line_xyz.tmag_zero)
        tc.target = line_xyz.tmag;
    else if (!line_uvw.tmag_zero)
        tc.target = line_uvw.tmag;
    else
        tc.target = line_abc.tmag;

    tc.progress = 0.0;
    tc.accel_state = ACCEL_S7;
    tc.distance_to_go = tc.target;
    tc.accel_time = 0.0;
    tc.reqvel = vel;
    tc.maxaccel = acc;
    // FIXME: the accel-increase-rate(jerk) is set as tc->maxaccel/sec
    // TODO: define accel-increase-rate(jerk) at CONFIG-FILE
    tc.jerk = ini_maxjerk;
    tc.feed_override = 0.0;
    tc.maxvel = ini_maxvel;
    tc.id = tp->nextId;
    tc.active = 0;
    tc.atspeed = atspeed;

    tc.cur_accel = 0.0;
    tc.currentvel = 0.0;
    // tc.blending = 0;
    // tc.blend_vel = 0.0;
    // tc.nexttc_vel= 0.0;

    tc.coords.line.xyz = line_xyz;
    tc.coords.line.uvw = line_uvw;
    tc.coords.line.abc = line_abc;
    tc.motion_type = TC_LINEAR;

    tc.canon_motion_type = type;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
    tc.velocity_mode = tp->velocity_mode;
    tc.uu_per_rev = tp->uu_per_rev;
    tc.enables = enables;

    if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
        tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
    } else {
        tc.syncdio.anychanged = 0;
        tc.syncdio.sync_input_triggered = 0;
    }
    if (pos_comp_en.pos_comp_en_triggered != 0) {
        tc.pos_comp_en = pos_comp_en;
        tpClearPosCompEn();
    } else {
        tc.pos_comp_en.en_flag = 0;
        tc.pos_comp_en.pos_comp_en_triggered = 0;
        tc.pos_comp_en.pos_comp_ref = 0;
    }
    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return -1;
    }

    tp->goalPos = end; // remember the end of this move, as it's
    // the start of the next one.
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;
    //    DP("tp->nextId(%d)\n", tp->nextId);

    return 0;
}

// likewise, this adds a circular (circle, arc, helix) move from
// the end of the last move to this new position.  end is the
// xyzabc of the destination, center/normal/turn specify the arc
// in a way that makes sense to pmCircleInit (we don't care about
// the details here.)  Note that degenerate arcs/circles are not
// allowed; we are guaranteed to have a move in xyz so target is
// always the circle/arc/helical length.

int tpAddCircle(TP_STRUCT * tp, EmcPose end, PmCartesian center,
        PmCartesian normal, int turn, int type, double vel, double ini_maxvel,
        double acc, double ini_maxjerk, unsigned char enables, char atspeed) {
    TC_STRUCT tc;
    PmCircle circle;
    PmLine line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    double helix_z_component; // z of the helix's cylindrical coord system
    double helix_length;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };

    //    fprintf(stderr,"tpAddCircle(): ini_maxjerk(%f) req_vel(%f) req_acc(%f) ini_maxvel(%f)\n",
    //                ini_maxjerk, vel, acc, ini_maxvel);

    if (ini_maxjerk == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "jerk is not provided or jerk is 0\n");
        assert(ini_maxjerk > 0);
    }
    if (!tp || tp->aborting)
        return -1;

    start_xyz.tran = tp->goalPos.tran;
    end_xyz.tran = end.tran;

    start_abc.tran.x = tp->goalPos.a;
    start_abc.tran.y = tp->goalPos.b;
    start_abc.tran.z = tp->goalPos.c;
    end_abc.tran.x = end.a;
    end_abc.tran.y = end.b;
    end_abc.tran.z = end.c;

    start_uvw.tran.x = tp->goalPos.u;
    start_uvw.tran.y = tp->goalPos.v;
    start_uvw.tran.z = tp->goalPos.w;
    end_uvw.tran.x = end.u;
    end_uvw.tran.y = end.v;
    end_uvw.tran.z = end.w;

    start_xyz.rot = identity_quat;
    end_xyz.rot = identity_quat;
    start_uvw.rot = identity_quat;
    end_uvw.rot = identity_quat;
    start_abc.rot = identity_quat;
    end_abc.rot = identity_quat;

    pmCircleInit(&circle, start_xyz, end_xyz, center, normal, turn);
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);

    // find helix length
    pmCartMag(circle.rHelix, &helix_z_component);
    helix_length = pmSqrt(pmSq(circle.angle * circle.radius)
            + pmSq(helix_z_component));

    tc.sync_accel = 0;
    tc.cycle_time = tp->cycleTime;
    tc.target = helix_length;
    tc.progress = 0.0;
    tc.accel_state = ACCEL_S7;
    tc.distance_to_go = tc.target;
    tc.accel_time = 0.0;
    tc.reqvel = vel;
    tc.maxaccel = acc;
    tc.jerk = ini_maxjerk;
    tc.feed_override = 0.0;
    tc.maxvel = ini_maxvel;
    tc.id = tp->nextId;
    tc.active = 0;
    tc.atspeed = atspeed;

    tc.cur_accel = 0.0;
    tc.currentvel = 0.0;
    // tc.blending = 0;
    // tc.blend_vel = 0.0;
    // tc.nexttc_vel= 0.0;

    tc.coords.circle.xyz = circle;
    tc.coords.circle.uvw = line_uvw;
    tc.coords.circle.abc = line_abc;

    tc.motion_type = TC_CIRCULAR;
    tc.canon_motion_type = type;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
    tc.velocity_mode = tp->velocity_mode;
    tc.uu_per_rev = tp->uu_per_rev;
    tc.enables = enables;

    if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
        tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
    } else {
        tc.syncdio.anychanged = 0;
        tc.syncdio.sync_input_triggered = 0;
    }
    if (pos_comp_en.pos_comp_en_triggered != 0) {
        tc.pos_comp_en = pos_comp_en;
        tpClearPosCompEn();
    } else {
        tc.pos_comp_en.en_flag = 0;
        tc.pos_comp_en.pos_comp_en_triggered = 0;
        tc.pos_comp_en.pos_comp_ref = 0;
    }
    if (tcqPut(&tp->queue, tc) == -1) {
        return -1;
    }

    tp->goalPos = end;
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;
    //    DP("tp->nextId(%d)\n", tp->nextId);

    return 0;
}

// likewise, this adds a NURBS circular (circle, arc, helix) move from
// the end of the last move to this new position.  end is the
// xyzabc of the destination, center/normal/turn specify the arc
// in a way that makes sense to pmCircleInit (we don't care about
// the details here.)  Note that degenerate arcs/circles are not
// allowed; we are guaranteed to have a move in xyz so target is
// always the circle/arc/helical length.


int tpAddNURBS(TP_STRUCT *tp, int type, nurbs_block_t nurbs_block, EmcPose pos,
        unsigned char enables, double vel, double ini_maxvel,
        double ini_maxacc, double ini_maxjerk) {
    static TC_STRUCT tc;
    static uint32_t knots_todo = 0, order = 0,
            nr_of_ctrl_pts = 0, nr_of_knots = 0;
    nurbs_block_t *nurbs_to_tc = &tc.nurbs_block;//EmcPose* control_points;
    if (ini_maxjerk == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "jerk is not provided or jerk is 0\n");
        assert(ini_maxjerk > 0);
    }
    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return -1;
    }

    if (0 == knots_todo) {
        //        fprintf(stderr,"TC_NURBS PARAM INIT\n");
        knots_todo = nurbs_block.nr_of_knots;
        order = nurbs_block.order;

        nr_of_ctrl_pts = nurbs_block.nr_of_ctrl_pts;
        nr_of_knots = nurbs_block.nr_of_knots;


        nurbs_to_tc->ctrl_pts_ptr = (CONTROL_POINT*) malloc(
                sizeof(CONTROL_POINT) * nurbs_block.nr_of_ctrl_pts);
        nurbs_to_tc->knots_ptr = (double*) malloc(sizeof(double)
                * nurbs_block.nr_of_knots);
        nurbs_to_tc->N = (double*) malloc(sizeof(double) * (order + 1));
        nurbs_to_tc->axis_mask = nurbs_block.axis_mask;

    }
    if (knots_todo > 0) {
        if (knots_todo > (nr_of_knots - nr_of_ctrl_pts)) { // this part add ctrl pts and knots
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].X = pos.tran.x;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].Y = pos.tran.y;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].Z = pos.tran.z;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].A = pos.a;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].B = pos.b;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].C = pos.c;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].U = pos.u;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].V = pos.v;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].W = pos.w;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].R
                    = nurbs_block.weight;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].F = vel; // requested feedrate for this cp
            nurbs_to_tc->knots_ptr[nr_of_knots - knots_todo] = nurbs_block.knot;

        } else {
            nurbs_to_tc->knots_ptr[nr_of_knots - knots_todo] = nurbs_block.knot;

        }
        knots_todo -= 1;
    }


    if ((0 == knots_todo)) {
        // knots == 0 means all NURBS collected
#if 0 //dump control points and knots info
        uint32_t i=0;
        //fprintf(stderr,"tp.c tpAddNURBS() \n");
        for(i=0;i<nr_of_ctrl_pts;i++) {
            fprintf(stderr,"index [%d] = knots=%f cp(%f,%f,%f %f) weight = %f \n",i,
                    nurbs_to_tc->knots_ptr[i],
                    nurbs_to_tc->ctrl_pts_ptr[i].X,
                    nurbs_to_tc->ctrl_pts_ptr[i].Y,
                    nurbs_to_tc->ctrl_pts_ptr[i].Z,
                    nurbs_to_tc->ctrl_pts_ptr[i].A,
                    nurbs_to_tc->ctrl_pts_ptr[i].R);
        }
        for(;i<nr_of_knots;i++) {
            fprintf(stderr,"index [%d] = knots=%f \n",i,nurbs_to_tc->knots_ptr[i]);
        }
#endif

        // process tc , tp


        tc.sync_accel = 0;
        tc.cycle_time = tp->cycleTime;

        tc.target = nurbs_block.curve_len;

        tc.progress = 0.0;
        tc.accel_state = ACCEL_S7;
        tc.distance_to_go = tc.target;
        tc.accel_time = 0.0;
        tc.reqvel = nurbs_to_tc->ctrl_pts_ptr[0].F;// the first feedrate for first cp for reqvel//vel;
        tc.maxaccel = ini_maxacc;

        tc.jerk = ini_maxjerk;
        tc.feed_override = 0.0;
        tc.maxvel = ini_maxvel;
        tc.id = tp->nextId;
        tc.active = 0;
        tc.atspeed = 0;//atspeed;  // FIXME-eric(L)

        tc.nurbs_block.curve_len = nurbs_block.curve_len;
        tc.nurbs_block.order = nurbs_block.order;
        tc.nurbs_block.nr_of_ctrl_pts = nurbs_block.nr_of_ctrl_pts;
        tc.nurbs_block.nr_of_knots = nurbs_block.nr_of_knots;

        tc.cur_accel = 0.0;
        tc.currentvel = 0.0;

        tc.motion_type = TC_NURBS;
        tc.canon_motion_type = type;
        tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
        tc.tolerance = tp->tolerance;

        tc.synchronized = tp->synchronized;
        tc.velocity_mode = tp->velocity_mode;
        tc.uu_per_rev = tp->uu_per_rev;
        tc.enables = enables;

        if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
            tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
            tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
        } else {
            tc.syncdio.anychanged = 0;
            tc.syncdio.sync_input_triggered = 0;
        }
        if (pos_comp_en.pos_comp_en_triggered != 0) {
            tc.pos_comp_en = pos_comp_en;
            tpClearPosCompEn();
        } else {
            tc.pos_comp_en.en_flag = 0;
            tc.pos_comp_en.pos_comp_en_triggered = 0;
            tc.pos_comp_en.pos_comp_ref = 0;
        }
        if (tcqPut(&tp->queue, tc) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
            return -1;
        }

        tp->goalPos = pos; // remember the end of this move, ie. last control point
        // the start of the next one.
        tp->done = 0;
        tp->depth = tcqLen(&tp->queue);
        tp->nextId++;
    }
    return 0;
}

//FIXME: eric affection of feed override for velocity is disabled
//       dynamic adjust of feed override affect S-curve speed decrease
#if STATE_DEBUG
/* state flag for debug */
static int s0 = 0,
s1 = 0,
s2 = 0,
s3 = 0,
s4 = 0,
s5 = 0,
s6 = 0,
s7 = 0,
s8 = 0, /*s0_8_3 = 0, s1_8_3 = 0,*/
s9 = 0, /*s4_9_3 = 0, s5_9_3 = 0, s6_9_3 = 0,*/
s10 = 0, s2_10 = 0, s4_10 = 0, s5_10 = 0, s6_10 = 0, reach_target = 0;

#define ENTER_STATE(s) {\
                         if(s == 0) { \
                            fprintf(stderr,"%s(bgn):vel(%.3f)accel(%.3f)jerk(%.3f)dtg(%.3f)prog(%.3f)reqvel(%.1f)\n",#s,\
                                    tc->currentvel,tc->cur_accel,tc->rt_jerk,tc->target-tc->progress,tc->progress, tc->reqvel);\
                            s = 1;\
                         }\
                     }
#define EXIT_STATE(s) {\
                        if(s == 1) { \
                            fprintf(stderr,"%s(end):vel(%.3f)accel(%.3f)jerk(%.3f)dtg(%.3f)prog(%.3f)reqvel(%.1f)\n",#s,\
                                    tc->currentvel,tc->cur_accel,tc->rt_jerk,tc->target-tc->progress,tc->progress, tc->reqvel);\
                           s = 0;\
                        }\
                     }
// state machine fprintf
#define SP(fmt, args...) {\
            fprintf(stderr, fmt, ##args); \
        }\

#else
#define ENTER_STATE(s) do{}while(0)
#define EXIT_STATE(s)  do{}while(0)
#define SP(fmt, args...) do {} while(0)
#endif

/*
 Continuous form
 PT = P0 + V0T + 1/2A0T2 + 1/6JT3
 VT = V0 + A0T + 1/2 JT2
 AT = A0 + JT
 */
void tcRunCycle(TP_STRUCT *tp, TC_STRUCT *tc, double *v, int *on_final_decel) {
    double newvel, newaccel, target_vel, accel_vel, t, t1, t2, decel_dist, a,
            v1, prog;
    do {
        immediate_state = 0;
        switch (tc->accel_state) {
        case ACCEL_S0:
            ENTER_STATE(s0);

            // AT = A0 + JT
            newaccel = tc->cur_accel + tc->rt_jerk * tc->cycle_time;
            a = newaccel + tc->rt_jerk * tc->cycle_time;
            // VT = V0 + A0T + 1/2 JT2
            newvel = tc->currentvel + tc->cur_accel * tc->cycle_time + 0.5
                    * tc->rt_jerk * pow(tc->cycle_time, 2);
            v1 = newvel + newaccel * tc->cycle_time + 0.5 * tc->rt_jerk * pow(
                    tc->cycle_time, 2);
            // add cycle_time here due to decision later
            tc->accel_time += tc->cycle_time; // accel_time acts as time here only. it acts as accel_vel other state
            // hit acceleration limit -> S1_8_3
            if (newaccel >= tc->maxaccel) {
                tc->rt_maxaccel = tc->cur_accel;
                tc->accel_state = ACCEL_S1;
                tc->rt_jerk = 0;
                tc->accel_time = tc->currentvel - tc->vel_from;
                SP(" Leave S0 due to Acceleration limit\n");
                immediate_state = 1;
                EXIT_STATE(s0);
                break;
            }
            // hit vel limitation -> S2
            if ((newvel - tc->vel_from + newvel) >= (tc->reqvel
                    * tc->feed_override)) {
                tc->rt_maxaccel = tc->cur_accel;
                tc->accel_state = ACCEL_S2;
                tc->rt_jerk = -tc->jerk;
                immediate_state = 1;
                SP(" Leave S0 due to velocity limit\n");
                EXIT_STATE(s0);
                break;
            }
            // hit progress limitation
            // 0) determine which kind of deceleration will occur
            //    S4->S6 or S4->S5->S6
            // 1) determine if deceleration progress exceed target
            accel_vel = v1 - tc->vel_from;
            target_vel = v1 + accel_vel; // possible S3 vel
            t = sqrt(target_vel / tc->jerk);

            if (tc->jerk * t > tc->maxaccel) {
                // consider deceleration progress include S5
                /* t = sqrt(2*accel_vel/tc->jerk);
                 decel_dist = tc->currentvel*t+0.5*tc->cur_accel*pow(t,2)-1.0/6.0*tc->jerk*pow(t,3);// S2
                 */
                t = a / tc->jerk;
                decel_dist = v1 * t + 0.5 * a * t * t - 1.0 / 6.0 * tc->jerk
                        * t * t * t; //S2
                t = tc->maxaccel / tc->jerk;
                t = floor(t / tc->cycle_time);
                t = t * tc->cycle_time;
                decel_dist += target_vel * t - 1.0 / 6.0 * tc->jerk * pow(t, 3); //S4
                decel_dist += accel_vel * t - 0.5 * tc->maxaccel * pow(t, 2)
                        + 1.0 / 6.0 * tc->jerk * pow(t, 3); //S6
                // VT = V0 + A0T + 1/2 JT2
                t = (target_vel - 2 * accel_vel) / tc->maxaccel;
                //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                decel_dist += (target_vel - accel_vel) * t - 0.5 * tc->maxaccel
                        * pow(t, 2);// accel_vel)*t*0.5; //S5
                //                SP("S0 EXPECTED S2+S4+S5+S6 decel(%f)\n", decel_dist);

            } else {
                // consider deceleration progress exclude S5
                //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                /*accel = tc->jerk*t;
                 decel_dist = target_vel*t +0.5*accel*pow(t,2)- 1.0/6.0*tc->jerk*pow(t,3); // S2*/
                t = a / tc->jerk;
                //                SP("S0 decel t(%f)\n", t);
                decel_dist = v1 * t + 0.5 * a * t * t - 1.0 / 6.0 * tc->jerk
                        * t * t * t; //S2
                //                SP("S0 EXPECTED S2 decel(%f)\n", decel_dist);
                t = sqrt(accel_vel / tc->jerk);
                decel_dist += target_vel * t - 1.0 / 6.0 * tc->jerk * t * t * t; // S4
                //                SP("S0 EXPECTED S2+S4 decel(%f)\n",decel_dist);
                decel_dist += 0.5 * target_vel * t - 0.5 * (tc->jerk * t)
                        * pow(t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3); //S6
                //                SP("S0 EXPECTED S2+S4+S6 decel(%f) dtg(%f)\n", decel_dist,tc->target-tc->progress);

            }
            if (decel_dist + (v1 * tc->cycle_time + 0.5 * a * pow(
                    tc->cycle_time, 2) + 1.0 / 6.0 * tc->jerk * pow(
                    tc->cycle_time, 3)) >= tc->target - tc->progress) {
                tc->accel_state = ACCEL_S2;
                tc->rt_jerk = -tc->jerk;
                tc->decel_dist = decel_dist;
                immediate_state = 1;
                SP(" Leave S0 due to progress limit decel_dist(%f)\n",decel_dist);
                //                tc->reqvel = (tc->currentvel + (tc->currentvel - tc->vel_from))/tc->feed_override;
                //                tc->ori_reqvel = tc->reqvel;
                EXIT_STATE(s0);
            }
            tc->decel_dist = decel_dist;
            // handle speed up / down / pause
            if ((tp->pausing == 1) || (tc->reqvel * tc->feed_override == 0)) {
                tc->accel_state = ACCEL_S10; // into pausing
                tc->prev_state = ACCEL_S0;
                immediate_state = 1;
                EXIT_STATE(s0);
                break;
            } else if (tc->on_feed_change == 0) {
                if ((tc->feed_override > tc->ori_feed_override) || (tc->reqvel
                        - tc->ori_reqvel > VELOCITY_EPSTHON)) {

                    // speed up
                    tc->accel_state = ACCEL_S8;
                    tc->prev_state = ACCEL_S0;
                    immediate_state = 1;
                    EXIT_STATE(s0);
                    break;
                } else if ((tc->feed_override < tc->ori_feed_override)
                        || (tc->reqvel - tc->ori_reqvel < -VELOCITY_EPSTHON)) {
                    // speed down
                    tc->accel_state = ACCEL_S9;
                    tc->prev_state = ACCEL_S0;
                    immediate_state = 1;
                    EXIT_STATE(s0);
                    break;
                }
            }
            assert(newvel >= 0);
            break;
        case ACCEL_S1:
            ENTER_STATE(s1);
            // AT = A0 + JT
            newaccel = tc->cur_accel + tc->rt_jerk * tc->cycle_time;
            // VT = V0 + A0T + 1/2 JT2
            newvel = tc->currentvel + tc->cur_accel * tc->cycle_time + 0.5
                    * tc->rt_jerk * pow(tc->cycle_time, 2);
            // hit velocity limitation
            if ((newvel + tc->accel_time) >= (tc->reqvel * tc->feed_override)) {
                tc->accel_state = ACCEL_S2;
                tc->rt_jerk = -tc->jerk;
                immediate_state = 1;
                EXIT_STATE(s1);
                break;
            }
            // hit progress limitation
            // determine if deceleration progress S2->S4->S5->S6 exceed target
            t = tc->cur_accel / tc->jerk; // time on S2
            accel_vel = 0.5 * tc->jerk * pow(t, 2); // init speed on S6
            target_vel = tc->currentvel + tc->rt_maxaccel * t - 0.5 * tc->jerk
                    * pow(t, 2); // possible S3 vel
            //   PT = P0 + V0T + 1/2A0T2 + 1/6JT3
            decel_dist = tc->currentvel * t + 0.5 * tc->rt_maxaccel * pow(t, 2)
                    - 1.0 / 6.0 * tc->jerk * pow(t, 3); // d(S2)
            t = tc->rt_maxaccel / tc->jerk; // time on S4
            decel_dist += target_vel * t - 1.0 / 6.0 * tc->jerk * pow(t, 3); //d(S4)
            /*decel_dist += decel_dist; //S4*/
            decel_dist += accel_vel * t - 0.5 * tc->rt_maxaccel * pow(t, 2)
                    + 1.0 / 6.0 * tc->jerk * pow(t, 3); // d(S6)
            t = (target_vel - 2 * accel_vel) / tc->rt_maxaccel;
            decel_dist += (target_vel - accel_vel) * t - 0.5 * tc->rt_maxaccel
                    * pow(t, 2); // d(S5)

            if (decel_dist >= tc->target - tc->progress - /* not allow S1 walk exceed decel_dist*/
            (tc->currentvel * tc->cycle_time + 0.5 * tc->cur_accel * pow(
                    tc->cycle_time, 2))) {

                /*fprintf(stderr,"decel_dist(%f) dtg(%f) last_decel_dist(%f)\n",
                 decel_dist, a, tc->target - tc->progress);*/
                tc->accel_state = ACCEL_S2;
                tc->rt_jerk = -tc->jerk;
                immediate_state = 1;
                //                tc->reqvel = (tc->currentvel + accel_vel)/tc->feed_override;
                //                tc->ori_reqvel = tc->reqvel;

                EXIT_STATE(s1);
            }
            a = decel_dist;
            tc->decel_dist = decel_dist;
            // handle speed up / down

            if ((tp->pausing == 1) || (tc->reqvel * tc->feed_override == 0)) {
                tc->accel_state = ACCEL_S10; // into pausing
                tc->prev_state = ACCEL_S1;
                immediate_state = 1;
                EXIT_STATE(s1);
                break;
            } else if (tc->on_feed_change == 0) {
                if ((tc->feed_override > tc->ori_feed_override) || (tc->reqvel
                        - tc->ori_reqvel > VELOCITY_EPSTHON)) {

                    // speed up
                    tc->accel_state = ACCEL_S8;
                    tc->prev_state = ACCEL_S1;
                    immediate_state = 1;
                    EXIT_STATE(s1);
                    break;
                } else if ((tc->feed_override < tc->ori_feed_override)
                        || (tc->reqvel - tc->ori_reqvel < -VELOCITY_EPSTHON)) {
                    // speed down
                    tc->accel_state = ACCEL_S9;
                    tc->prev_state = ACCEL_S1;
                    immediate_state = 1;
                    EXIT_STATE(s1);
                    break;
                }
            }
            assert(newvel >= 0);
            break;
        case ACCEL_S2:
            ENTER_STATE(s2);

            // AT = A0 + JT
            newaccel = tc->cur_accel + tc->rt_jerk * tc->cycle_time;
            // VT = V0 + A0T + 1/2 JT2
            newvel = tc->currentvel + tc->cur_accel * tc->cycle_time + 0.5
                    * tc->rt_jerk * pow(tc->cycle_time, 2);

            t = sqrt(tc->currentvel / tc->jerk); // time for deceleration to 0.5 currentvel
            target_vel = tc->currentvel;//tc->reqvel*tc->feed_override;

            if (tc->jerk * t > tc->maxaccel) {
                // S4+S5+S6
                t = tc->maxaccel / tc->jerk;
                t = floor(t / tc->cycle_time) * tc->cycle_time;
                tc->rt_maxaccel = tc->jerk * t;
                //                SP("rt_maxaccel(%f)\n",tc->rt_maxaccel);
                accel_vel = 0.5 * tc->jerk * pow(t, 2);
                target_vel = tc->currentvel; // possible S3 vel
                //                SP("t(%f)  \n",t);
                //   PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                decel_dist = target_vel * t - 1.0 / 6.0 * tc->jerk * pow(t, 3); // S4
                //                SP("EXPECTED d(s4)(%f) ",decel_dist);
                decel_dist += accel_vel * t - 0.5 * tc->rt_maxaccel * pow(t, 2)
                        + 1.0 / 6.0 * tc->jerk * pow(t, 3); // S6
                //                SP("d(s6)(%f) ",accel_vel*t-0.5*tc->rt_maxaccel*pow(t,2)+1.0/6.0*tc->jerk*pow(t,3));
                t = (target_vel - 2 * accel_vel) / tc->rt_maxaccel;
                decel_dist += (target_vel - accel_vel) * t - 0.5
                        * tc->rt_maxaccel * pow(t, 2);
                //                SP("d(s5)(%f) ",(target_vel - accel_vel)*t -0.5*tc->rt_maxaccel*pow(t,2));

            } else {
                // consider deceleration progress exclude S5
                //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                decel_dist = target_vel * t - 1.0 / 6.0 * tc->jerk * pow(t, 3);
                //              SP("EXPECTED d(s4)(%f) target_vel(%f)", decel_dist, target_vel);
                decel_dist += 0.5 * target_vel * t - 0.5 * (tc->jerk * t)
                        * pow(t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3);
                //              SP("EXPECTED d(s4)+d(s6) = (%f) ",decel_dist);

            }
            if (newaccel <= 0) {
                tc->decel_dist = decel_dist;
                SP("decel_dist(%f) dtg(%f)\n",tc->decel_dist, tc->target-tc->progress);
                tc->accel_state = ACCEL_S3;
                tc->rt_jerk = 0;
                tc->cur_accel = 0;
                tc->on_feed_change = 0;
                immediate_state = 1;
                EXIT_STATE(s2);
                break;
            }
            // progress limit
            if (decel_dist + newvel * 2 * tc->cycle_time + 0.5 * newaccel
                    * pow(2 * tc->cycle_time, 2) + 1.0 / 6.0 * tc->rt_jerk
                    * pow(2 * tc->cycle_time, 3) >= tc->target - tc->progress) {
                tc->decel_dist = decel_dist;
                SP("decel_dist(%f) dtg(%f)\n",tc->decel_dist, tc->target-tc->progress);
                tc->accel_state = ACCEL_S3;
                tc->rt_jerk = 0;
                tc->cur_accel = 0;
                tc->on_feed_change = 0;
                immediate_state = 1;
                EXIT_STATE(s2);
                break;
            }
            // handle speed up / down
            if ((tp->pausing == 1) || (tc->reqvel * tc->feed_override == 0)) {
                tc->accel_state = ACCEL_S10; // into pausing
                tc->prev_state = ACCEL_S2;
                immediate_state = 1;
                EXIT_STATE(s2);
                break;
            } else if (tc->on_feed_change == 0) {
                if ((tc->feed_override > tc->ori_feed_override) || (tc->reqvel
                        - tc->ori_reqvel > VELOCITY_EPSTHON)) {

                    // speed up
                    tc->accel_state = ACCEL_S8;
                    tc->prev_state = ACCEL_S2;
                    immediate_state = 1;
                    EXIT_STATE(s2);
                    break;
                } else if ((tc->feed_override < tc->ori_feed_override)
                        || (tc->reqvel - tc->ori_reqvel < -VELOCITY_EPSTHON)) {
                    // speed down
                    tc->accel_state = ACCEL_S9;
                    tc->prev_state = ACCEL_S2;
                    immediate_state = 1;
                    EXIT_STATE(s2);
                    break;
                }
            }
            assert(newvel >= 0);
            break;
        case ACCEL_S3:
            ENTER_STATE(s3);

            // AT = A0 + JT
            newaccel = 0;//tc->cur_accel + tc->rt_jerk * tc->cycle_time;
            // VT = V0 + A0T + 1/2 JT2
            newvel = /*tc->reqvel*tc->feed_override;*/tc->currentvel;//+ tc->cur_accel * tc->cycle_time + 0.5 * tc->rt_jerk * pow(tc->cycle_time,2);
            if (tc->decel_dist + (tc->currentvel) * tc->cycle_time > tc->target
                    - tc->progress) {
                SP("tc->decel_dist(%f)\n", tc->decel_dist);
                tc->accel_state = ACCEL_S4;
                tc->accel_time = tc->currentvel; // save currentvel to accel_time
                tc->target_vel = 0;
                tc->rt_jerk = -tc->jerk;
                immediate_state = 1;
                EXIT_STATE(s3);
                break;
            }
            // handle speed up / down
            if ((tp->pausing == 1) || (tc->reqvel * tc->feed_override == 0)) {
                tc->accel_state = ACCEL_S10; // into pausing
                tc->prev_state = ACCEL_S3;
                immediate_state = 1;
                EXIT_STATE(s3);
                break;
            } else if (tc->on_feed_change == 0) {
                if ((tc->feed_override > tc->ori_feed_override) || (tc->reqvel
                        - tc->ori_reqvel > VELOCITY_EPSTHON)) {
                    // speed up
                    tc->accel_state = ACCEL_S8;
                    tc->prev_state = ACCEL_S3;
                    immediate_state = 1;
                    EXIT_STATE(s3);
                    break;
                } else if ((tc->feed_override < tc->ori_feed_override)
                        || (tc->reqvel - tc->ori_reqvel < -VELOCITY_EPSTHON)) {
                    // speed down
                    tc->accel_state = ACCEL_S9;
                    tc->prev_state = ACCEL_S3;
                    immediate_state = 1;
                    EXIT_STATE(s3);
                    break;
                }
            }
            break;
            assert(newvel >= 0);
        case ACCEL_S4:
            ENTER_STATE(s4);
            /* not necessary to process speed modification */
            // AT = A0 + JT
            newaccel = tc->cur_accel + tc->rt_jerk * tc->cycle_time;
            // VT = V0 + A0T + 1/2 JT2
            newvel = tc->currentvel + tc->cur_accel * tc->cycle_time + 0.5
                    * tc->rt_jerk * pow(tc->cycle_time, 2);

            if ((newaccel <= -tc->maxaccel)) {
                tc->accel_state = ACCEL_S5;
                tc->rt_jerk = 0;
                //                tc->time_leave_s4 = sqrt(2*(tc->accel_time - tc->currentvel)/tc->jerk);
                tc->accel_time = tc->accel_time - tc->currentvel/*newvel*/; // record the delta-V

                immediate_state = 1;
                EXIT_STATE(s4);
                break;
            }
            t = -tc->cur_accel / tc->jerk;
            decel_dist = tc->currentvel * t + 0.5 * tc->cur_accel * t * t + 1.0
                    / 6.0 * tc->jerk * t * t * t;
            if (newvel <= 0.5 * (tc->accel_time + tc->target_vel)
                    && (decel_dist < (tc->target - tc->progress))/*from S3*/) { // accel_time stores the initial-velocity of deceleration
                if (tc->target_vel == 0) {
                    SP("vel(%f) accel(%f) dtg(%f) decel_dist(%f)\n",
                            tc->currentvel, tc->cur_accel, tc->target-tc->progress, decel_dist);
                    {
                        double a, b, c, d;
                        a = 0.5 * tc->jerk;
                        b = tc->cur_accel;
                        c = tc->currentvel;
                        SP("d(%f)\n",b*b-4*a*c);
                        if ((b * b - 4 * a * c < 0)) {
                            c = b * b / (4 * a);
                            tc->currentvel = c;
                            t = -tc->cur_accel / tc->jerk;
                            decel_dist = tc->currentvel * t + 0.5
                                    * tc->cur_accel * t * t + 1.0 / 6.0
                                    * tc->jerk * t * t * t;
                            d = b * b - 4 * a * c + EPSTHON;
                            assert(d >=0);
                            t1 = (-b + sqrt(d)) / (2 * a);
                            t2 = (-b - sqrt(d)) / (2 * a);
                            t = t1 < t2 ? t1 : t2;
                            t = ceil(t / tc->cycle_time);
                            tc->dist_comp = (t >= 1 ? (tc->target
                                    - tc->progress - decel_dist) / (t) : 0);
                        } else {
                            d = b * b - 4 * a * c + EPSTHON;
                            assert(d >=0);
                            t1 = (-b + sqrt(d)) / (2 * a);
                            t2 = (-b - sqrt(d)) / (2 * a);
                            t = t1 < t2 ? t1 : t2;
                            t = ceil(t / tc->cycle_time);

                            t1 = t * tc->cycle_time;
                            decel_dist = tc->currentvel * t1 + 0.5
                                    * tc->cur_accel * t1 * t1 + 1.0 / 6.0
                                    * tc->jerk * t1 * t1 * t1;

                            tc->dist_comp = (t >= 1 ? (tc->target
                                    - tc->progress - decel_dist) / (t) : 0);
                        }

                    }

                    SP("t1(%f) t2(%f) t(%f) diff(%f) decel_dist(%f) dist_comp(%f) cycles(%f)\n",
                            t1,t2,t*tc->cycle_time,tc->target - tc->progress - decel_dist,decel_dist, tc->dist_comp,t);
                } else {
                    tc->dist_comp = 0;
                }
                tc->accel_time = tc->currentvel;
                tc->accel_state = ACCEL_S6;
                tc->rt_jerk = tc->jerk;
                immediate_state = 1;

                EXIT_STATE(s4);
                break;
            }
            assert(newvel >= 0);
            break;//TODO-eric: confirm if S8 and S9 interrupt by continue speed up and down
        case ACCEL_S5:
            ENTER_STATE(s5);
            /*  not necessary process speed modification */
            newaccel = tc->cur_accel;
            newvel = tc->currentvel + newaccel * tc->cycle_time;

            t = -tc->cur_accel / tc->jerk;
            decel_dist = tc->currentvel * t + 0.5 * tc->cur_accel * t * t + 1.0
                    / 6.0 * tc->jerk * t * t * t;
            /* this comparison will meet the first solution of time
             * and cause a error
             */
            /*if(decel_dist > tc->target -tc->progress) {
             SP("dtg(%f) decel_dist(%f)\n",tc->target - tc->progress, decel_dist);
             tc->accel_state = ACCEL_S6;
             tc->accel_time = tc->currentvel;
             tc->rt_jerk = tc->jerk;
             immediate_state = 1;
             EXIT_STATE(s5);
             break;
             }*/
            if (newvel <= tc->accel_time + tc->target_vel && (decel_dist
                    < (tc->target - tc->progress))) { // accel_time stores the delta-V for ACCEL_S4
                SP("vel(%f) accel(%f) dtg(%f) decel_dist(%f)\n",
                        tc->currentvel, tc->cur_accel, tc->target-tc->progress, decel_dist);
                if (tc->target_vel == 0) {
                    t = -tc->cur_accel / tc->jerk;
                    t = (t / tc->cycle_time);
                    {
                        double a, b, c, d;
                        a = 0.5 * tc->jerk;
                        b = tc->cur_accel;
                        c = tc->currentvel;
                        SP("d(%f)\n",b*b-4*a*c);
                        if ((b * b - 4 * a * c < 0)) {
                            c = b * b / (4 * a);
                            tc->currentvel = c;
                            t = -tc->cur_accel / tc->jerk;
                            decel_dist = tc->currentvel * t + 0.5
                                    * tc->cur_accel * t * t + 1.0 / 6.0
                                    * tc->jerk * t * t * t;
                            d = b * b - 4 * a * c + EPSTHON;
                            assert(d >=0);
                            t1 = (-b + sqrt(d)) / (2 * a);
                            t2 = (-b - sqrt(d)) / (2 * a);
                            t = t1 < t2 ? t1 : t2;
                            t = ceil(t / tc->cycle_time);
                            tc->dist_comp = (t >= 1 ? (tc->target
                                    - tc->progress - decel_dist) / (t) : 0);

                        } else {
                            d = b * b - 4 * a * c + EPSTHON;
                            assert(d >=0);
                            t1 = (-b + sqrt(d)) / (2 * a);
                            t2 = (-b - sqrt(d)) / (2 * a);
                            t = t1 < t2 ? t1 : t2;
                            t = ceil(t / tc->cycle_time);
                            t1 = t * tc->cycle_time;
                            decel_dist = tc->currentvel * t1 + 0.5
                                    * tc->cur_accel * t1 * t1 + 1.0 / 6.0
                                    * tc->jerk * t1 * t1 * t1;
                            tc->dist_comp = (t >= 1 ? (tc->target
                                    - tc->progress - decel_dist) / (t) : 0);
                        }

                    }

                    SP("t1(%f) t2(%f) t(%f) diff(%f) decel_dist(%f) dist_comp(%f) cycles(%f)\n",
                            t1,t2,t*tc->cycle_time,tc->target - tc->progress - decel_dist,decel_dist, tc->dist_comp,t);
                } else {
                    tc->dist_comp = 0;
                }
                tc->accel_state = ACCEL_S6;
                tc->rt_jerk = tc->jerk;
                tc->accel_time = tc->currentvel;

                immediate_state = 1;
                EXIT_STATE(s5);
                break;
            }

            assert(newvel >= 0);
            break;
        case ACCEL_S6:
            ENTER_STATE(s6);
            // AT = A0 + JT
            newaccel = tc->cur_accel + tc->rt_jerk * tc->cycle_time;
            // VT = V0 + A0T + 1/2 JT2
            newvel = tc->currentvel + tc->cur_accel * tc->cycle_time + 0.5
                    * tc->rt_jerk * pow(tc->cycle_time, 2);

            if (tc->target_vel == 0 && newvel < tc->target_vel) {
                newvel = tc->currentvel;
            }
            if (tc->target_vel != 0 && newaccel > 0/*(newvel - EPSTHON < tc->target_vel) */) { //raise torrence for velocity check

                /* estimates decel_dist for S3*/
                t = sqrt(tc->currentvel / tc->jerk); // time for deceleration to 0.5 currentvel
                target_vel = tc->currentvel;//tc->reqvel*tc->feed_override;;
                if (tc->jerk * t > tc->rt_maxaccel) {
                    // S4+S5+S6
                    t = tc->rt_maxaccel / tc->jerk;
                    accel_vel = 0.5 * tc->jerk * pow(t, 2);
                    target_vel = tc->currentvel; // possible S3 vel

                    //   PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    decel_dist = target_vel * t - 1.0 / 6.0 * tc->jerk * pow(t,
                            3); // S4
                    SP("expected d(s4)(%f) ",decel_dist);
                    decel_dist += accel_vel * t - 0.5 * tc->rt_maxaccel * pow(
                            t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3); // S6
                    SP("d(s6)(%f) ",accel_vel*t-0.5*tc->rt_maxaccel*pow(t,2)+1.0/6.0*tc->jerk*pow(t,3));
                    t = (target_vel - 2 * accel_vel) / tc->rt_maxaccel;
                    decel_dist += (target_vel - accel_vel) * t - 0.5
                            * tc->rt_maxaccel * pow(t, 2);
                    SP("d(s5)(%f) ",(target_vel - accel_vel)*t -0.5*tc->rt_maxaccel*pow(t,2));

                } else {
                    // consider deceleration progress exclude S5
                    //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    decel_dist = target_vel * t - 1.0 / 6.0 * tc->jerk * pow(t,
                            3);
                    decel_dist += 0.5 * target_vel * t - 0.5 * (tc->jerk * t)
                            * pow(t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3);
                    //                  SP("exclude S5");
                }
                newvel = /*tc->reqvel*tc->feed_override*/tc->currentvel;
                newaccel = 0/*tc->cur_accel*/;
                tc->decel_dist = decel_dist;
                SP("decel_dist(%f) dtg(%f)\n",tc->decel_dist, tc->target-tc->progress);
                tc->on_feed_change = 0;
                tc->accel_state = ACCEL_S3;
                tc->rt_jerk = 0;
                tc->cur_accel = 0;
                immediate_state = 1;
                EXIT_STATE(s6);
                break;
            }

            /* if(newvel < 0 || newaccel >0) SP("newvel(%f) curvel(%f) newaccel(%f)  cur_accel(%f)\n",
             newvel, tc->currentvel, newaccel, tc->cur_accel);*/
            /*assert(tc->currentvel >= 0);
             assert(tc->cur_accel <= 0);*/
            break;
        case ACCEL_S7:// pause wait resume or initial state
            newvel = 0;
            newaccel = 0;
            ENTER_STATE(s7);
            if (tp->pausing == 0 && (tc->reqvel * tc->feed_override > 0)) {
                tc->accel_state = ACCEL_S0;
                tc->motion_target = tc->target - tc->progress;
                tc->motion_progress = 0;
                tc->motion_distance_to_go = tc->motion_target;
                tc->accel_time = 0;
                tc->currentvel = 0;
                tc->cur_accel = 0;
                tc->ori_feed_override = tc->feed_override;
                tc->ori_reqvel = tc->reqvel;
                tc->vel_from = 0;
                tc->rt_jerk = tc->jerk;
                tc->target_vel = 0;
                tc->dist_comp = 0;
                tc->on_feed_change = 0;
                immediate_state = 0;
                EXIT_STATE(s7);
            }
            assert(newvel >= 0);
            break;
        case ACCEL_S8: // speed up
            ENTER_STATE(s8);
            tc->ori_feed_override = tc->feed_override;
            tc->ori_reqvel = tc->reqvel;
            tc->vel_from = tc->currentvel;

            if (ACCEL_S3 != tc->prev_state) {
                tc->accel_state = tc->prev_state;
                tc->on_feed_change = 0;
                // immediate_state = 1;
                EXIT_STATE(s8);
                break;
            } else {
                // prediction distant when speed up
                /* estimates decel_dist for S3->S4->S5->S6->S3 */
                target_vel = (tc->currentvel + tc->reqvel * tc->feed_override)
                        / 2;
                t = sqrt(2 * (target_vel - tc->currentvel) / tc->jerk); // time for deceleration to 0.5 currentvel
                t = floor(t / tc->cycle_time) * tc->cycle_time;
                SP("time_for_accel(%f)\n",t);
                if (tc->jerk * t > tc->rt_maxaccel) {
                    // S0+S1+S2
                    t = tc->rt_maxaccel / tc->jerk; // time would align to cycle_time already
                    accel_vel = 0.5 * tc->jerk * pow(t, 2);

                    // accure predict S2 final speed
                    //                    target_vel = tc->currentvel + 2*accel_vel;
                    target_vel = tc->reqvel * tc->feed_override - accel_vel
                            - (tc->currentvel + accel_vel);
                    t1 = floor((target_vel / tc->rt_maxaccel) / tc->cycle_time)
                            * tc->cycle_time;
                    // possible vel before leaving S2
                    target_vel = tc->currentvel + accel_vel + tc->rt_maxaccel
                            * t1 + accel_vel;

                    //   PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    decel_dist = tc->currentvel * t + 1.0 / 6.0 * tc->jerk
                            * pow(t, 3); // S0
                    SP("expected d(s0)(%f) ",decel_dist);
                    decel_dist += (target_vel - accel_vel) * t + 0.5
                            * tc->rt_maxaccel * pow(t, 2) - 1.0 / 6.0
                            * tc->jerk * pow(t, 3); // S2
                    SP("d(s2)(%f) ",accel_vel*t-0.5*tc->rt_maxaccel*pow(t,2)+1.0/6.0*tc->jerk*pow(t,3));
                    t = t1;//(target_vel - 2*accel_vel)/tc->rt_maxaccel;
                    decel_dist += (tc->currentvel + accel_vel) * t + 0.5
                            * tc->rt_maxaccel * pow(t, 2); // S1
                    SP("d(s1)(%f) ",(target_vel - accel_vel)*t -0.5*tc->rt_maxaccel*pow(t,2));

                } else {
                    // S0+S2
                    //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    t = floor(t / tc->cycle_time) * tc->cycle_time;
                    decel_dist = tc->currentvel * t + 1.0 / 6.0 * tc->jerk
                            * pow(t, 3);
                    accel_vel = 0.5 * tc->jerk * pow(t, 2);
                    target_vel = tc->currentvel + accel_vel;
                    decel_dist += target_vel * t + 0.5 * (tc->jerk * t) * pow(
                            t, 2) - 1.0 / 6.0 * tc->jerk * pow(t, 3);
                }

                // in speed up case, we should consider decel_dist inlcude S4,S5,S6
                // copy from S2
                t = sqrt(tc->currentvel / tc->jerk); // time for deceleration to 0.5 currentvel
                target_vel = tc->currentvel;//tc->reqvel*tc->feed_override;;
                if (tc->jerk * t > tc->rt_maxaccel) {
                    // S4+S5+S6
                    t = tc->rt_maxaccel / tc->jerk;
                    accel_vel = 0.5 * tc->jerk * pow(t, 2);
                    target_vel = tc->currentvel; // possible S3 vel

                    //   PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    decel_dist += target_vel * t - 1.0 / 6.0 * tc->jerk * pow(
                            t, 3); // S4
                    SP("expected d(s4)(%f) ",decel_dist);
                    decel_dist += accel_vel * t - 0.5 * tc->rt_maxaccel * pow(
                            t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3); // S6
                    SP("d(s6)(%f) ",accel_vel*t-0.5*tc->rt_maxaccel*pow(t,2)+1.0/6.0*tc->jerk*pow(t,3));
                    t = (target_vel - 2 * accel_vel) / tc->rt_maxaccel;
                    decel_dist += (target_vel - accel_vel) * t - 0.5
                            * tc->rt_maxaccel * pow(t, 2);
                    SP("d(s5)(%f) ",(target_vel - accel_vel)*t -0.5*tc->rt_maxaccel*pow(t,2));

                } else {
                    // consider deceleration progress exclude S5
                    //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    decel_dist += target_vel * t - 1.0 / 6.0 * tc->jerk * pow(
                            t, 3);
                    decel_dist += 0.5 * target_vel * t - 0.5 * (tc->jerk * t)
                            * pow(t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3);
                    SP("exclude S5");
                }
                // copy from S2

                if (tc->decel_dist + decel_dist < tc->target - tc->progress) { // *3 to prevent not enough dist to decel

                    tc->accel_state = ACCEL_S0;
                    tc->rt_jerk = tc->jerk;
                    tc->accel_time = 0;
                    tc->accel_dist = 0;
                    tc->on_feed_change = 1;
                    immediate_state = 1;
                    EXIT_STATE(s8);
                    break;

                } else {
                    tc->accel_state = tc->prev_state;
                    tc->on_feed_change = 0;
                    // immediate_state = 1;
                    EXIT_STATE(s8);
                }
            }
            break;
        case ACCEL_S9: // speed down
            ENTER_STATE(s9);
            tc->ori_feed_override = tc->feed_override;
            tc->ori_reqvel = tc->reqvel;
            tc->vel_from = tc->currentvel;

            if (ACCEL_S3 != tc->prev_state) {
                tc->accel_state = tc->prev_state;
                tc->on_feed_change = 0;
                // immediate_state = 1;
                EXIT_STATE(s9);
                break;
            } else {
                // prediction distant when speed down
                /* estimates decel_dist for S3->S4->S5->S6->S3 */
                target_vel = (tc->currentvel + tc->reqvel * tc->feed_override)
                        / 2;
                t = sqrt(2 * (tc->currentvel - target_vel) / tc->jerk); // time for deceleration to 0.5 currentvel
                t = floor(t / tc->cycle_time) * tc->cycle_time;
                SP("time_for_decel(%f)\n",t);
                if (tc->jerk * t > tc->rt_maxaccel) {
                    // S4+S5+S6
                    t = tc->rt_maxaccel / tc->jerk;
                    accel_vel = 0.5 * tc->jerk * pow(t, 2);

                    // accure predict S6 final speed
                    target_vel = tc->currentvel - accel_vel - (tc->reqvel
                            * tc->feed_override + accel_vel);
                    t1 = floor((target_vel / tc->rt_maxaccel) / tc->cycle_time)
                            * tc->cycle_time;
                    target_vel = tc->currentvel - accel_vel - tc->rt_maxaccel
                            * t1 - accel_vel;// possible vel before leaving S6

                    //   PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    decel_dist = tc->currentvel * t - 1.0 / 6.0 * tc->jerk
                            * pow(t, 3); // S4
                    SP("expected d(s4)(%f) ",decel_dist);
                    decel_dist += (accel_vel + target_vel) * t - 0.5
                            * tc->rt_maxaccel * pow(t, 2) + 1.0 / 6.0
                            * tc->jerk * pow(t, 3); // S6
                    SP("d(s6)(%f) ",accel_vel*t-0.5*tc->rt_maxaccel*pow(t,2)+1.0/6.0*tc->jerk*pow(t,3));
                    t = t1;/*(target_vel - 2*accel_vel)/tc->rt_maxaccel;*/
                    decel_dist += (tc->currentvel - accel_vel) * t - 0.5
                            * tc->rt_maxaccel * pow(t, 2);
                    SP("d(s5)(%f) ",(target_vel - accel_vel)*t -0.5*tc->rt_maxaccel*pow(t,2));

                } else {
                    // consider deceleration progress exclude S5
                    //PT = P0 + V0T + 1/2A0T2 + 1/6JT3
                    t = floor(t / tc->cycle_time) * tc->cycle_time;
                    decel_dist = tc->currentvel * t - 1.0 / 6.0 * tc->jerk
                            * pow(t, 3);
                    accel_vel = 0.5 * tc->jerk * pow(t, 2);
                    target_vel = tc->currentvel - accel_vel;
                    decel_dist += target_vel * t - 0.5 * (tc->jerk * t) * pow(
                            t, 2) + 1.0 / 6.0 * tc->jerk * pow(t, 3);
                    //                  SP("exclude S5");
                }

                if (tc->decel_dist + decel_dist < tc->target - tc->progress) {
                    tc->accel_state = ACCEL_S4;
                    tc->accel_time = tc->currentvel;
                    tc->accel_dist = 0;
                    tc->target_vel = tc->reqvel * tc->feed_override;
                    tc->rt_jerk = -tc->jerk;
                    tc->on_feed_change = 1;
                    immediate_state = 1;
                    EXIT_STATE(s9);
                    break;
                } else {
                    tc->accel_state = tc->prev_state;
                    tc->on_feed_change = 0;
                    //  immediate_state = 1;
                    EXIT_STATE(s9);
                }
            }
            break;

        case ACCEL_S10: // pausing from S2
            ENTER_STATE(s10);
            tc->vel_from = tc->currentvel;
            if (tc->prev_state != ACCEL_S3) {
                tc->accel_state = ACCEL_S2_10;
                tc->rt_jerk = -tc->jerk;
            } else {
                tc->rt_jerk = -tc->jerk;
                tc->accel_state = ACCEL_S4_10;
            }
            immediate_state = 1;
            EXIT_STATE(s10);
            break;
        case ACCEL_S2_10:
            ENTER_STATE(s2_10);

            newaccel = tc->cur_accel - tc->jerk * tc->cycle_time;
            newvel = tc->currentvel + newaccel * tc->cycle_time;

            if (newaccel <= 0) {
                newaccel = 0;
                newvel = tc->currentvel + tc->cur_accel * tc->cycle_time;
                tc->accel_state = ACCEL_S4_10;
                tc->rt_jerk = -tc->jerk;
                tc->vel_from = newvel;
                EXIT_STATE(s2_10);
            }
            //   assert(newvel >= 0);
            break;
        case ACCEL_S4_10:
            ENTER_STATE(s4_10);
            newaccel = tc->cur_accel - tc->jerk * tc->cycle_time;
            newvel = tc->currentvel + newaccel * tc->cycle_time;
            if ((newaccel <= -tc->maxaccel)) {
                tc->accel_time = tc->vel_from - tc->currentvel;
                tc->accel_state = ACCEL_S5_10;
                tc->rt_jerk = 0;
                immediate_state = 1;
                EXIT_STATE(s4_10);
                break;
            }
            // hit velocity limit
            if (newvel <= 0.5 * (tc->vel_from)) { // velocity transaction point
                tc->rt_jerk = tc->jerk;
                tc->accel_state = ACCEL_S6_10;
                immediate_state = 1;
                EXIT_STATE(s4_10);
            }
            assert(newaccel <= 0);
            break;
        case ACCEL_S5_10:
            ENTER_STATE(s5_10);
            newaccel = tc->cur_accel;
            newvel = tc->currentvel + newaccel * tc->cycle_time;

            if (newvel <= tc->accel_time) { // accel_time stores the delta-V for ACCEL_S4
                tc->accel_state = ACCEL_S6_10;
                tc->rt_jerk = tc->jerk;
                tc->accel_time = tc->currentvel;
                immediate_state = 1;
                EXIT_STATE(s5_10);
            }
            break;
        case ACCEL_S6_10:
            ENTER_STATE(s6_10);
            newaccel = tc->cur_accel + tc->jerk * tc->cycle_time;
            newvel = tc->currentvel + newaccel * tc->cycle_time;
            if (newvel > tc->currentvel) {
                newvel = 0;
                newaccel = 0;
                tc->accel_state = ACCEL_S7;
                tc->rt_jerk = 0;
                tc->cur_accel = 0;
                tc->currentvel = 0;
                immediate_state = 1;
                EXIT_STATE(s6_10);
            }
            break;
        default:
            assert(0);
        } // switch (tc->accel_state)
    } while (immediate_state);
    //    PT = P0 + V0T + 1/2A0T2 + 1/6JT3
    prog = tc->currentvel * tc->cycle_time + 0.5 * tc->cur_accel * pow(
            tc->cycle_time, 2) + 1.0 / 6.0 * tc->rt_jerk * pow(tc->cycle_time,
            3) + tc->dist_comp;
    tc->motion_progress += prog;//(tc->currentvel + newvel) * 0.5 * tc->cycle_time;
    tc->progress += prog;//(tc->currentvel + newvel) * 0.5 * tc->cycle_time;
    if (tc->progress >= tc->target) {
        EXIT_STATE(s6);
        newvel = 0;
        newaccel = 0;
        tc->progress = tc->target;
#if STATE_DEBUG
        s0 = 0;
        s1 = 0;
        s2 = 0;
        s3 = 0;
        s4 = 0;
        s5 = 0;
        s6 = 0;
        s7 = 0;
        s8 = 0; /*s0_8_3 = 0; s1_8_3 = 0;*/
        s9 = 0;/* s4_9_3 = 0; s5_9_3 = 0; s6_9_3 = 0;*/
        s10 = 0; s2_10 = 0; s4_10 = 0; s5_10 = 0; s6_10 = 0; reach_target = 0;
#endif
    }
    DPS("%11u%15.5f%15.5f%15.5f%15.5f%15.5f%15.5f%15.5f\n",
            _dt, newaccel, newvel, tc->currentvel, tc->progress, tc->target, tc->distance_to_go, tc->tolerance);
#if (TRACE!=0)
    _dt += 1;
#endif
    tc->distance_to_go = tc->target - tc->progress;
    tc->motion_distance_to_go = tc->motion_target - tc->motion_progress;
    tc->cur_accel = newaccel;
    tc->currentvel = newvel;
    if (v)
        *v = newvel;
}
void tpToggleCompPosEn(TC_STRUCT * tc) {
    if (tc->pos_comp_en.pos_comp_en_triggered != 0) {
        emcmotPosCompWrite(tc->pos_comp_en.en_flag,
                tc->pos_comp_en.pos_comp_ref);
    }
    tc->pos_comp_en.pos_comp_en_triggered = 0;
}
void tpToggleDIOs(TC_STRUCT * tc) {
    int i = 0;
    if (tc->syncdio.anychanged != 0) { // we have DIO's to turn on or off
        for (i = 0; i < emcmotConfig->numDIO; i++) {
            if (tc->syncdio.dios[i] > 0) {
//                fprintf(stderr, "set DOUT(%d) %d\n", i, tc->syncdio.dios[i]);
                emcmotDioWrite(i, 1); // turn DIO[i] on
            }
            if (tc->syncdio.dios[i] < 0) {
//                fprintf(stderr, "set DOUT(%d) %d\n", i, tc->syncdio.dios[i]);
                emcmotDioWrite(i, 0); // turn DIO[i] off
            }
        }
        tc->syncdio.anychanged = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }
    if (tc->syncdio.sync_input_triggered != 0) {
        /* replace with pin index for sync in
         * for (i=0; i < emcmotConfig->numSyncIn; i++) {
         if (tc->syncdio.sync_in[i] > 0) emcmotSyncInputWrite(i, tc->syncdio.timeout, tc->syncdio.wait_type); //
         }*/
        emcmotSyncInputWrite(tc->syncdio.sync_in, tc->syncdio.timeout,
                tc->syncdio.wait_type);
        tc->syncdio.sync_input_triggered = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }
}

// This is the brains of the operation.  It's called every TRAJ period
// and is expected to set tp->currentPos to the new machine position.
// Lots of other tp fields (depth, done, etc) have to be twiddled to
// communicate the status; I think those are spelled out here correctly
// and I can't clean it up without breaking the API that the TP presents
// to motion.  It's not THAT bad and in the interest of not touching
// stuff outside this directory, I'm going to leave it for now.

int tpRunCycle(TP_STRUCT * tp, long period) {
    // vel = (new position - old position) / cycle time
    // (two position points required)
    //
    // acc = (new vel - old vel) / cycle time
    // (three position points required)

    TC_STRUCT *tc, *nexttc;
    double primary_vel;
    int on_final_decel;
    EmcPose primary_before, primary_after;
    EmcPose secondary_before, secondary_after;
    EmcPose primary_displacement, secondary_displacement;
    static double spindleoffset;
    static int waiting_for_index = 0;
    static int waiting_for_atspeed = 0;
    // double save_vel;
    static double revs;
    EmcPose target;

    emcmotStatus->tcqlen = tcqLen(&tp->queue);
    emcmotStatus->requested_vel = 0.0;
    tc = tcqItem(&tp->queue, 0, period);
    if (!tc) {
        // this means the motion queue is empty.  This can represent
        // the end of the program OR QUEUE STARVATION.  In either case,
        // I want to stop.  Some may not agree that's what it should do.
        tcqInit(&tp->queue);
        tp->goalPos = tp->currentPos;
        tp->done = 1;
        tp->depth = tp->activeDepth = 0;
        tp->aborting = 0;
        tp->execId = 0;
        tp->motionType = 0;
        tpResume(tp);
        // when not executing a move, use the current enable flags
        emcmotStatus->enables_queued = emcmotStatus->enables_new;
        return 0;
    }

    if (tc->target == tc->progress && waiting_for_atspeed != tc->id) {
        // if we're synced, and this move is ending, save the
        // spindle position so the next synced move can be in
        // the right place.
        if (tc->synchronized)
            spindleoffset += tc->target / tc->uu_per_rev;
        else
            spindleoffset = 0.0;

        // done with this move
        tcqRemove(&tp->queue, 1);

        // so get next move
        tc = tcqItem(&tp->queue, 0, period);
        if (!tc)
            return 0;
    }

    // now we have the active tc.  get the upcoming one, if there is one.
    // it's not an error if there isn't another one - we just don't
    // do blending.  This happens in MDI for instance.
    if (!emcmotDebug->stepping && tc->blend_with_next)
        nexttc = tcqItem(&tp->queue, 1, period);
    else
        nexttc = NULL;

    {
        int this_synch_pos = tc->synchronized && !tc->velocity_mode;
        int next_synch_pos = nexttc && nexttc->synchronized
                && !nexttc->velocity_mode;
        if (!this_synch_pos && next_synch_pos) {
            // we'll have to wait for spindle sync; might as well
            // stop at the right place (don't blend)
            tc->blend_with_next = 0;
            nexttc = NULL;
        }
    }

    if (nexttc && nexttc->atspeed) {
        // we'll have to wait for the spindle to be at-speed; might as well
        // stop at the right place (don't blend), like above
        tc->blend_with_next = 0;
        nexttc = NULL;
    }

    if (tp->aborting) {
        // an abort message has come
        if (waiting_for_index || waiting_for_atspeed || (tc->currentvel == 0.0
                && !nexttc) || (tc->currentvel == 0.0 && nexttc
                && nexttc->currentvel == 0.0)) {
            tcqInit(&tp->queue);
            tp->goalPos = tp->currentPos;
            tp->done = 1;
            tp->depth = tp->activeDepth = 0;
            tp->aborting = 0;
            tp->execId = 0;
            tp->motionType = 0;
            tp->synchronized = 0;
            waiting_for_index = 0;
            waiting_for_atspeed = 0;
            emcmotStatus->spindleSync = 0;
            tpResume(tp);
            return 0;
        } else {
            tc->reqvel = 0.0;
            if (nexttc)
                nexttc->reqvel = 0.0;
        }
    }

    // this is no longer the segment we were waiting_for_index for
    if (waiting_for_index && waiting_for_index != tc->id) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for index on motion id %d, but reached id %d\n",
                waiting_for_index, tc->id);
        waiting_for_index = 0;
    }
    if (waiting_for_atspeed && waiting_for_atspeed != tc->id) {

        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for atspeed on motion id %d, but reached id %d\n",
                waiting_for_atspeed, tc->id);
        waiting_for_atspeed = 0;
    }

    // check for at-speed before marking the tc active
    if (waiting_for_atspeed) {
        if (!emcmotStatus->spindle_is_atspeed) {
            /* spindle is still not at the right speed: wait */
            return 0;
        } else {
            waiting_for_atspeed = 0;
        }
    }

    if (tc->active == 0) {
        // this means this tc is being read for the first time.

        // wait for atspeed, if motion requested it.  also, force
        // atspeed check for the start of all spindle synchronized
        // moves.
        if ((tc->atspeed || (tc->synchronized && !tc->velocity_mode
                && !emcmotStatus->spindleSync))
                && !emcmotStatus->spindle_is_atspeed) {
            waiting_for_atspeed = tc->id;
            return 0;
        }

        tc->active = 1;
        tc->currentvel = 0;
        tp->depth = tp->activeDepth = 1;
        tp->motionType = tc->canon_motion_type;
        // tc->blending = 0;

        // honor accel constraint in case we happen to make an acute angle
        // with the next segment.
        if (tc->blend_with_next)
            tc->maxaccel /= 2.0;

        if (tc->synchronized) {
            if (!tc->velocity_mode && !emcmotStatus->spindleSync) {
                // if we aren't already synced, wait
                waiting_for_index = tc->id;
                // ask for an index reset
                emcmotStatus->spindle_index_enable = 1;
                spindleoffset = 0.0;
                // don't move: wait
                return 0;
            }
        }
    }

    if (waiting_for_index) {
        if (emcmotStatus->spindle_index_enable) {
            /* haven't passed index yet */
            return 0;
        } else {
            /* passed index, start the move */
            emcmotStatus->spindleSync = 1;
            waiting_for_index = 0;
            tc->sync_accel = 1;
            revs = 0;
        }
    }

    if (tc->motion_type == TC_RIGIDTAP) {
        static double old_spindlepos;
        double new_spindlepos = emcmotStatus->spindleRevs;

        switch (tc->coords.rigidtap.state) {
        case TAPPING:
            if (tc->progress >= tc->coords.rigidtap.reversal_target) {
                // command reversal
                emcmotStatus->spindle.speed *= -1;
                tc->coords.rigidtap.state = REVERSING;
            }
            break;
        case REVERSING:
            if (new_spindlepos < old_spindlepos) {
                PmPose start, end;
                PmLine *aux = &tc->coords.rigidtap.aux_xyz;
                // we've stopped, so set a new target at the original position
                tc->coords.rigidtap.spindlerevs_at_reversal = new_spindlepos
                        + spindleoffset;

                pmLinePoint(&tc->coords.rigidtap.xyz, tc->progress, &start);
                end = tc->coords.rigidtap.xyz.start;
                pmLineInit(aux, start, end);
                tc->coords.rigidtap.reversal_target = aux->tmag;
                tc->target = aux->tmag + 10. * tc->uu_per_rev;
                tc->progress = 0.0;

                tc->coords.rigidtap.state = RETRACTION;
            }
            old_spindlepos = new_spindlepos;
            break;
        case RETRACTION:
            if (tc->progress >= tc->coords.rigidtap.reversal_target) {
                emcmotStatus->spindle.speed *= -1;
                tc->coords.rigidtap.state = FINAL_REVERSAL;
            }
            break;
        case FINAL_REVERSAL:
            if (new_spindlepos > old_spindlepos) {
                PmPose start, end;
                PmLine *aux = &tc->coords.rigidtap.aux_xyz;
                pmLinePoint(aux, tc->progress, &start);
                end = tc->coords.rigidtap.xyz.start;
                pmLineInit(aux, start, end);
                tc->target = aux->tmag;
                tc->progress = 0.0;
                tc->synchronized = 0;
                tc->reqvel = tc->maxvel;

                tc->coords.rigidtap.state = FINAL_PLACEMENT;
            }
            old_spindlepos = new_spindlepos;
            break;
        case FINAL_PLACEMENT:
            // this is a regular move now, it'll stop at target above.
            break;
        }
    }

    if (!tc->synchronized)
        emcmotStatus->spindleSync = 0;

    if (nexttc && nexttc->active == 0) {
        // this means this tc is being read for the first time.

        nexttc->currentvel = 0;
        tp->depth = tp->activeDepth = 1;
        nexttc->active = 1;
        // nexttc->blending = 0;

        // honor accel constraint if we happen to make an acute angle with the
        // above segment or the following one
        if (tc->blend_with_next || nexttc->blend_with_next)
            nexttc->maxaccel /= 2.0;
    }

    if (tc->synchronized) {
        double pos_error;
        double oldrevs = revs;

        if (tc->velocity_mode) {
            pos_error = fabs(emcmotStatus->spindleSpeedIn) * tc->uu_per_rev;
            if (nexttc)
                pos_error -= nexttc->progress; /* ?? */
            if (!tp->aborting) {
                tc->feed_override = emcmotStatus->net_feed_scale;
                tc->reqvel = pos_error;
            }
        } else {
            double spindle_vel, target_vel;
            if (tc->motion_type == TC_RIGIDTAP && (tc->coords.rigidtap.state
                    == RETRACTION || tc->coords.rigidtap.state
                    == FINAL_REVERSAL))
                revs = tc->coords.rigidtap.spindlerevs_at_reversal
                        - emcmotStatus->spindleRevs;
            else
                revs = emcmotStatus->spindleRevs;

            pos_error = (revs - spindleoffset) * tc->uu_per_rev - tc->progress;
            if (nexttc)
                pos_error -= nexttc->progress;

            if (tc->sync_accel) {
                // ysli: TODO: study tc->sync_accel
                // detect when velocities match, and move the target accordingly.
                // acceleration will abruptly stop and we will be on our new target.
                spindle_vel = revs / (tc->cycle_time * tc->sync_accel++);
                target_vel = spindle_vel * tc->uu_per_rev;
                if (tc->currentvel >= target_vel) {
                    // move target so as to drive pos_error to 0 next cycle
                    spindleoffset = revs - tc->progress / tc->uu_per_rev;
                    tc->sync_accel = 0;
                    tc->reqvel = target_vel;
                } else {
                    // beginning of move and we are behind: accel as fast as we can
                    tc->reqvel = tc->maxvel;
                }
            } else {
                // we have synced the beginning of the move as best we can -
                // track position (minimize pos_error).
                double errorvel;
                spindle_vel = (revs - oldrevs) / tc->cycle_time;
                target_vel = spindle_vel * tc->uu_per_rev;
                errorvel = pmSqrt(fabs(pos_error) * tc->maxaccel);
                if (pos_error < 0)
                    errorvel = -errorvel;
                tc->reqvel = target_vel + errorvel;
            }
            tc->feed_override = 1.0;
        }
        if (tc->reqvel < 0.0)
            tc->reqvel = 0.0;
        if (nexttc) {
            if (nexttc->synchronized) {
                nexttc->reqvel = tc->reqvel;
                nexttc->feed_override = 1.0;
                if (nexttc->reqvel < 0.0)
                    nexttc->reqvel = 0.0;
            } else {
                nexttc->feed_override = emcmotStatus->net_feed_scale;
            }
        }
    } else {
        tc->feed_override = emcmotStatus->net_feed_scale;
        if (nexttc) {
            nexttc->feed_override = emcmotStatus->net_feed_scale;
        }
    }
    /* handle pausing */
    if (tp->pausing && (!tc->synchronized || tc->velocity_mode)) {
        //     tc->feed_override = 0.0;
        //     if(nexttc) {
        //	    nexttc->feed_override = 0.0;
        //	}
    }

    primary_before = tcGetPos(tc);
    tcRunCycle(tp, tc, &primary_vel, &on_final_decel);
    primary_after = tcGetPos(tc);
    pmCartCartSub(primary_after.tran, primary_before.tran,
            &primary_displacement.tran);
    primary_displacement.a = primary_after.a - primary_before.a;
    primary_displacement.b = primary_after.b - primary_before.b;
    primary_displacement.c = primary_after.c - primary_before.c;

    primary_displacement.u = primary_after.u - primary_before.u;
    primary_displacement.v = primary_after.v - primary_before.v;
    primary_displacement.w = primary_after.w - primary_before.w;

    // blend criteria
    if (nexttc && (tc->distance_to_go <= tc->tolerance)) {
        if (tc->currentvel > nexttc->currentvel) {

            target = tcGetEndpoint(tc);
            tp->motionType = tc->canon_motion_type;
            emcmotStatus->distance_to_go = tc->distance_to_go;
            emcmotStatus->enables_queued = tc->enables;
            // report our line number to the guis
            tp->execId = tc->id;
            emcmotStatus->requested_vel = tc->reqvel;
        } else {

            tpToggleDIOs(nexttc); //check and do DIO changes
            tpToggleCompPosEn(nexttc); // check if any position compensation flag enable or not
            target = tcGetEndpoint(nexttc);
            tp->motionType = nexttc->canon_motion_type;
            emcmotStatus->distance_to_go = nexttc->distance_to_go;
            emcmotStatus->enables_queued = nexttc->enables;
            // report our line number to the guis
            tp->execId = nexttc->id;
            emcmotStatus->requested_vel = nexttc->reqvel;
        }

        emcmotStatus->current_vel = tc->currentvel + nexttc->currentvel;

        secondary_before = tcGetPos(nexttc);
        tcRunCycle(tp, nexttc, NULL, NULL);

        secondary_after = tcGetPos(nexttc);
        pmCartCartSub(secondary_after.tran, secondary_before.tran,
                &secondary_displacement.tran);
        secondary_displacement.a = secondary_after.a - secondary_before.a;
        secondary_displacement.b = secondary_after.b - secondary_before.b;
        secondary_displacement.c = secondary_after.c - secondary_before.c;

        secondary_displacement.u = secondary_after.u - secondary_before.u;
        secondary_displacement.v = secondary_after.v - secondary_before.v;
        secondary_displacement.w = secondary_after.w - secondary_before.w;

        pmCartCartAdd(tp->currentPos.tran, primary_displacement.tran,
                &tp->currentPos.tran);
        pmCartCartAdd(tp->currentPos.tran, secondary_displacement.tran,
                &tp->currentPos.tran);
        tp->currentPos.a += primary_displacement.a + secondary_displacement.a;
        tp->currentPos.b += primary_displacement.b + secondary_displacement.b;
        tp->currentPos.c += primary_displacement.c + secondary_displacement.c;

        tp->currentPos.u += primary_displacement.u + secondary_displacement.u;
        tp->currentPos.v += primary_displacement.v + secondary_displacement.v;
        tp->currentPos.w += primary_displacement.w + secondary_displacement.w;
    } else { // if (nexttc && (tc->distance_to_go <= tc->tolerance))
        tpToggleDIOs(tc); //check and do DIO changes
        tpToggleCompPosEn(tc);

        target = tcGetEndpoint(tc);
        tp->motionType = tc->canon_motion_type;
        emcmotStatus->distance_to_go = tc->distance_to_go;
        tp->currentPos = primary_after;
        emcmotStatus->current_vel = tc->currentvel;
        emcmotStatus->requested_vel = tc->reqvel;
        emcmotStatus->enables_queued = tc->enables;
        // report our line number to the guis
        tp->execId = tc->id;
    }

    emcmotStatus->dtg.tran.x = target.tran.x - tp->currentPos.tran.x;
    emcmotStatus->dtg.tran.y = target.tran.y - tp->currentPos.tran.y;
    emcmotStatus->dtg.tran.z = target.tran.z - tp->currentPos.tran.z;
    emcmotStatus->dtg.a = target.a - tp->currentPos.a;
    emcmotStatus->dtg.b = target.b - tp->currentPos.b;
    emcmotStatus->dtg.c = target.c - tp->currentPos.c;
    emcmotStatus->dtg.u = target.u - tp->currentPos.u;
    emcmotStatus->dtg.v = target.v - tp->currentPos.v;
    emcmotStatus->dtg.w = target.w - tp->currentPos.w;

    return 0;
}

int tpSetSpindleSync(TP_STRUCT * tp, double sync, int mode) {
    if (sync) {
        tp->synchronized = 1;
        tp->uu_per_rev = sync;
        tp->velocity_mode = mode;
    } else
        tp->synchronized = 0;

    return 0;
}

int tpPause(TP_STRUCT * tp) {
    if (0 == tp) {
        return -1;
    }
    tp->pausing = 1;
    emcmotPosCompWrite(0, 0);
    return 0;
}

int tpResume(TP_STRUCT * tp) {
    if (0 == tp) {
        return -1;
    }
    tp->pausing = 0;
    return 0;
}

int tpAbort(TP_STRUCT * tp) {
    if (0 == tp) {
        return -1;
    }
    if (!tp->aborting) {
        /* to abort, signal a pause and set our abort flag */
        tp->aborting = 1;
    }
    tpClearPosCompEn();
    return tpClearDIOs(); //clears out any already cached DIOs
}

int tpGetMotionType(TP_STRUCT * tp) {
    return tp->motionType;
}

EmcPose tpGetPos(TP_STRUCT * tp) {
    EmcPose retval;

    if (0 == tp) {
        ZERO_EMC_POSE(retval);
        return retval;
    }

    return tp->currentPos;
}

int tpIsDone(TP_STRUCT * tp) {
    if (0 == tp) {
        return 0;
    }

    return tp->done;
}

int tpQueueDepth(TP_STRUCT * tp) {
    if (0 == tp) {
        return 0;
    }

    return tp->depth;
}

int tpActiveDepth(TP_STRUCT * tp) {
    if (0 == tp) {
        return 0;
    }

    return tp->activeDepth;
}

int tpSetAout(TP_STRUCT *tp, unsigned char index, double start, double end) {
    return 0;
}

int tpSetDout(TP_STRUCT *tp, int index, unsigned char start, unsigned char end) {
    if (0 == tp) {
        return -1;
    }
    syncdio.anychanged = 1; //something has changed
    if (start > 0)
        syncdio.dios[index] = 1; // the end value can't be set from canon currently, and has the same value as start
    else
        syncdio.dios[index] = -1;
    return 0;
}

int tpSetSyncInput(TP_STRUCT *tp, int index, double timeout, int wait_type) {
    // int i;
    if (0 == tp) {
        return -1;
    }
    if (index < 0 || index > EMCMOT_MAX_SYNC_INPUT) {
        return -1;
    }
    syncdio.sync_input_triggered = 1; //something has changed
    /*  replace with index
     for (i=0; i<EMCMOT_MAX_SYNC_INPUT; i++) {
     syncdio.sync_in[i] = -1;
     }
     */

    syncdio.sync_in = index; // the end value can't be set from canon currently, and has the same value as start
    syncdio.wait_type = wait_type;
    syncdio.timeout = timeout;
    return 0;
}
int tpSetPosCompEnWrite(TP_STRUCT *tp, int en_flag, int pos_comp_ref) {
    if (0 == tp) {
        return -1;
    }

    pos_comp_en.pos_comp_en_triggered = 1;
    pos_comp_en.en_flag = en_flag;
    pos_comp_en.pos_comp_ref = pos_comp_ref;

    return 0;
}
// vim:sw=4:sts=4:et:
