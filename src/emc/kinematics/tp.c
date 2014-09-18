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
#include "config.h"

#ifdef RTAPI_SIM
// SIM
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>

#else
#define assert(args...)		do {} while(0)
#endif

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

// #undef SMLBLND       // turn off seamless blending
#define SMLBLND         // to evaluate seamless blending

// to disable DP(): #define TRACE 0
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
static FILE* dptrace = 0;
static uint32_t _dt = 0;
#endif

#define CSS_TRACE 0
#if (CSS_TRACE!=0)
#if (TRACE==0)
static FILE* csstrace = 0;
static uint32_t _dt = 0;
#else
#undef CSS_TRACE // disable CSS_TRACE when TRACE is enabled
#endif
#endif

#define EPSTHON 1e-6

extern emcmot_status_t *emcmotStatus;
extern emcmot_debug_t *emcmotDebug;

static const double tiny = 1e-7;

int output_chan = 0;
static syncdio_t syncdio; //record tpSetDout's here
static pso_t pso;

int tpCreate(TP_STRUCT * tp, int _queueSize, TC_STRUCT * tcSpace)
{
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
        DPS ("%11s%6s%15s%15s%15s%15s%15s%15s%15s\n",
                "#dt", "state", "req_vel", "cur_accel", "cur_vel", "progress%", "target", "dist_to_go", "tc_target");
        _dt = 0;
    }
#endif

#if (CSS_TRACE!=0)
    if (!csstrace) {
        csstrace = fopen("tp_css.log", "w");
        _dt = 0;
    }
#endif
    /* init the rest of our data */
    return tpInit(tp);
}


// this clears any potential DIO toggles
// anychanged signals if any DIOs need to be changed
// dios[i] = 1, DIO needs to get turned on, -1 = off
int tpClearDIOs() {
    //XXX: All IO's will be flushed on next synced aio/dio! Is it ok?
    int i;
    syncdio.anychanged = 0;
    //    syncdio.dio_mask = 0;
    syncdio.aio_mask = 0;
    for (i = 0; i < emcmotConfig->numDIO; i++)
        syncdio.dios[i] = 0;
    for (i = 0; i < emcmotConfig->numAIO; i++)
        syncdio.aios[i] = 0;

    syncdio.sync_input_triggered = 0;
    syncdio.sync_in = 255;
    syncdio.wait_type = 0;
    syncdio.timeout = 0.0;
    return 0;
}

int tpClearPSO()
{
    pso.enable = 0;
    pso.mode = 0;
    pso.pitch = 0;
    pso.tick = 0;
    pso.next_progress = 0;
//    DP("tp.c, tpClearPSO(): TODO: resolve a method to terminate PSO at End Of Program\n");
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
int tpClear(TP_STRUCT * tp)
{
    tcqInit(&tp->queue);
    tp->queueSize = 0;
    tp->goalPos = tp->currentPos;
    tp->nextId = 0;
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

    return tpClearDIOs();
}


int tpInit(TP_STRUCT * tp)
{
    tp->cycleTime = 0.0;
    tp->vLimit = 0.0;
    tp->vScale = 1.0;
    tp->vMax = 0.0;
    tp->ini_maxvel = 0.0;
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;

    ZERO_EMC_POSE(tp->currentPos);

    return tpClear(tp);
}

int tpSetCycleTime(TP_STRUCT * tp, double secs)
{
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

int tpSetVmax(TP_STRUCT * tp, double vMax, double ini_maxvel)
{
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

int tpSetVlimit(TP_STRUCT * tp, double vLimit)
{
    if (!tp) return -1;

    if (vLimit < 0.) 
        tp->vLimit = 0.;
    else
        tp->vLimit = vLimit;

    return 0;
}


/*
  tpSetId() sets the id that will be used for the next appended motions.
  nextId is incremented so that the next time a motion is appended its id
  will be one more than the previous one, modulo a signed int. If
  you want your own ids for each motion, call this before each motion
  you append and stick what you want in here.
 */
int tpSetId(TP_STRUCT * tp, int id)
{

    if (!MOTION_ID_VALID(id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpSetId: invalid motion id %d\n", id);
        return -1;
    }

    if (0 == tp) {
        return -1;
    }

    tp->nextId = id;

    return 0;
}

/*
  tpGetExecId() returns the id of the last motion that is currently
  executing.
 */
int tpGetExecId(TP_STRUCT * tp)
{
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
int tpSetTermCond(TP_STRUCT * tp, int cond, double tolerance)
{
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

int tpSetPos(TP_STRUCT * tp, EmcPose pos)
{
    if (0 == tp) {
        return -1;
    }

    tp->currentPos = pos;
    tp->goalPos = pos;

    return 0;
}

/**
 * SPINDLE_SYNC_MOTION:
 *      -- RIGID_TAPPING(G33.1)
 *      -- CSS(G33 w/ G96)
 *      -- THREADING(G33 w/ G97)
 */
int tpAddSpindleSyncMotion(TP_STRUCT *tp, EmcPose end, double vel,
        double ini_maxvel, double acc, double jerk,
        int ssm_mode, unsigned char enables)
{
    TC_STRUCT tc;
    PmLine line_xyz;
    PmPose start_xyz, end_xyz;
    PmCartesian abc, uvw;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };

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

    // TODO: pass atspeed as parameter (G33.1/G33.2/G33.3 will not wait for atspeed, G33 needs)
    if (ssm_mode < 2){
        tc.atspeed = 0;
        pmLineInit(&line_xyz, start_xyz, end_xyz);
    }
    else if (ssm_mode == 2)
    {   // G33.2
        tc.atspeed = 1;
        pmLineInit(&line_xyz, start_xyz, start_xyz);      // prevent motion for xyz for G33.2
        tc.coords.spindle_sync.spindle_end_angle = end.s / 360.0;
    }


    tc.sync_accel = 0;
    tc.cycle_time = tp->cycleTime;

    // tc.target: set as revolutions of spindle
    tc.target = line_xyz.tmag / tp->uu_per_rev;
    DP("jerk(%f) req_vel(%f) req_acc(%f) ini_maxvel(%f)\n",
            jerk, vel, acc, ini_maxvel);
    DP("target(%f) uu_per_rev(%f)\n", tc.target, tp->uu_per_rev);

    tc.progress = 0.0;
    tc.accel_state = ACCEL_S3;
    tc.distance_to_go = tc.target;
    tc.reqvel = vel;
    tc.maxvel = ini_maxvel * tp->cycleTime;
    tc.maxaccel = acc * tp->cycleTime * tp->cycleTime;
    tc.jerk = jerk * tp->cycleTime * tp->cycleTime * tp->cycleTime;
    tc.feed_override = 0.0;
    tc.id = tp->nextId;
    tc.active = 0;




    tc.cur_accel = 0.0;
    tc.cur_vel = 0.0;
    tc.blending = 0;

    tc.coords.spindle_sync.xyz = line_xyz;
    tc.coords.spindle_sync.abc = abc;
    tc.coords.spindle_sync.uvw = uvw;
    // updated spindle speed constrain based on spindleSyncMotionMsg.vel of emccanon.cc
    tc.coords.spindle_sync.spindle_reqvel = vel;

    tc.motion_type = TC_SPINDLE_SYNC_MOTION;

    tc.canon_motion_type = 0;
    tc.blend_with_next = 0;
    tc.tolerance = tp->tolerance;
    tc.seamless_blend_mode = SMLBLND_DISABLE;
    tc.nexttc_target = 0;

    if(!tp->synchronized) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Cannot add unsynchronized spindle sync move.\n");
        return -1;
    }
    tc.synchronized = tp->synchronized;
    tc.uu_per_rev = tp->uu_per_rev;
    tc.css_progress_cmd = 0;
    tc.velocity_mode = tp->velocity_mode;
    tc.enables = enables;
    tc.indexrotary = -1;
    tc.coords.spindle_sync.spindle_start_pos_latch = 0;
    tc.coords.spindle_sync.spindle_start_pos = 0;
    tc.coords.spindle_sync.mode = ssm_mode;


    if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
        tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
    } else {
        tc.syncdio.anychanged = 0;
        tc.syncdio.sync_input_triggered = 0;
    }

    tc.pso = pso;

    if (vel > 0)        // vel is requested spindle velocity
    {
        tc.coords.spindle_sync.spindle_dir = 1.0;
    } else
    {
        tc.coords.spindle_sync.spindle_dir = -1.0;
    }

    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return -1;
    }
    if (ssm_mode == 1)  // for G33.1
    {   // REVERSING
        pmLineInit(&line_xyz, end_xyz, start_xyz);  // reverse the line direction
        tc.coords.spindle_sync.xyz = line_xyz;
        if (vel > 0)
        {
            tc.coords.spindle_sync.spindle_dir = -1.0;
        } else
        {
            tc.coords.spindle_sync.spindle_dir = 1.0;
        }
        if (tcqPut(&tp->queue, tc) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
            return -1;
        }
        // do not change tp->goalPos here,
        // since this move will end just where it started
    }
    else if (ssm_mode == 0)
    {   // for G33
        tp->goalPos = end;      // remember the end of this move, as it's
        // the start of the next one.
    }
    else if (ssm_mode == 2)
    {   // for G33.2
        // TODO: need to change tp->goalPos ?
    }


    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;

    return 0;
}

// Add a straight line to the tc queue.  This is a coordinated
// move in any or all of the six axes.  it goes from the end
// of the previous move to the new end specified here at the
// currently-active accel and vel settings from the tp struct.
// EMC_MOTION_TYPE_FEED
int tpAddLine(TP_STRUCT * tp, EmcPose end, int type, double vel, 
        double ini_maxvel, double acc,
        double ini_maxjerk, unsigned char enables,
        char atspeed, int indexrotary)
{
    TC_STRUCT tc;
    PmLine line_xyz, line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };

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
    tc.accel_state = ACCEL_S3;
    tc.distance_to_go = tc.target;
    tc.reqvel = vel;
    tc.maxvel = ini_maxvel * tp->cycleTime;
    tc.maxaccel = acc * tp->cycleTime * tp->cycleTime;
    tc.jerk = ini_maxjerk * tp->cycleTime * tp->cycleTime * tp->cycleTime;
    tc.feed_override = 0.0;
    tc.id = tp->nextId;
    tc.active = 0;
    tc.atspeed = atspeed;

    tc.cur_accel = 0.0;
    tc.cur_vel = 0.0;
    tc.blending = 0;

    tc.coords.line.xyz = line_xyz;
    tc.coords.line.uvw = line_uvw;
    tc.coords.line.abc = line_abc;
    tc.motion_type = TC_LINEAR;
    tc.canon_motion_type = type;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;
    tc.seamless_blend_mode = SMLBLND_INIT;
    tc.nexttc_target = 0;

    tc.synchronized = tp->synchronized;
    tc.velocity_mode = tp->velocity_mode;
    tc.uu_per_rev = tp->uu_per_rev;
    tc.css_progress_cmd = 0;
    tc.enables = enables;
    tc.indexrotary = indexrotary;

    if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0) ) {
        tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
    } else {
        tc.syncdio.anychanged = 0;
        tc.syncdio.sync_input_triggered = 0;
    }

    tc.pso = pso;

    tc.utvIn = line_xyz.uVec;
    tc.utvOut = line_xyz.uVec;

    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return -1;
    }

    tp->goalPos = end;      // remember the end of this move, as it's
    // the start of the next one.
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;

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
        double acc, double ini_maxjerk, unsigned char enables, char atspeed) 
{
    TC_STRUCT tc;
    PmCircle circle;
    PmLine line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    double helix_z_component;   // z of the helix's cylindrical coord system
    double helix_length;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };

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
    helix_length = pmSqrt(pmSq(circle.angle * circle.radius) +
            pmSq(helix_z_component));

    tc.sync_accel = 0;
    tc.cycle_time = tp->cycleTime;
    tc.target = helix_length;
    tc.progress = 0.0;
    tc.accel_state = ACCEL_S3;
    tc.distance_to_go = tc.target;
    tc.reqvel = vel;
    tc.maxvel = ini_maxvel * tp->cycleTime;
    tc.maxaccel = acc * tp->cycleTime * tp->cycleTime;
    tc.jerk = ini_maxjerk * tp->cycleTime * tp->cycleTime * tp->cycleTime;
    tc.feed_override = 0.0;
    tc.id = tp->nextId;
    tc.active = 0;
    tc.atspeed = atspeed;

    tc.cur_accel = 0.0;
    tc.cur_vel = 0.0;
    tc.blending = 0;

    tc.coords.circle.xyz = circle;
    tc.coords.circle.uvw = line_uvw;
    tc.coords.circle.abc = line_abc;
    tc.motion_type = TC_CIRCULAR;
    tc.canon_motion_type = type;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;
    tc.seamless_blend_mode = SMLBLND_INIT;
    tc.nexttc_target = 0;

    tc.synchronized = tp->synchronized;
    tc.velocity_mode = tp->velocity_mode;
    tc.uu_per_rev = tp->uu_per_rev;
    tc.css_progress_cmd = 0;
    tc.enables = enables;
    tc.indexrotary = -1;


    if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
        tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
    } else {
        tc.syncdio.anychanged = 0;
        tc.syncdio.sync_input_triggered = 0;
    }

    tc.pso = pso;

    tc.utvIn = circle.utvIn;
    tc.utvOut = circle.utvOut;

    if (tcqPut(&tp->queue, tc) == -1) {
        return -1;
    }

    tp->goalPos = end;
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;

    return 0;
}

// likewise, this adds a NURBS circular (circle, arc, helix) move from
// the end of the last move to this new position.  end is the
// xyzabc of the destination, center/normal/turn specify the arc
// in a way that makes sense to pmCircleInit (we don't care about
// the details here.)  Note that degenerate arcs/circles are not
// allowed; we are guaranteed to have a move in xyz so target is
// always the circle/arc/helical length.

#ifndef RTAPI_SIM
#warning "implement rtai_kmalloc() for nurbs"
static CONTROL_POINT ctrl_pts_array[8192];
static double knots_array[8192];
static double N_array[8192];
#endif

int tpAddNURBS(TP_STRUCT *tp, int type, nurbs_block_t nurbs_block, EmcPose pos,
        unsigned char enables, double vel, double ini_maxvel,
        double ini_maxacc, double ini_maxjerk) 
{
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

#ifndef RTAPI_SIM
#warning "implement rtai_kmalloc() for nurbs"
        nurbs_to_tc->ctrl_pts_ptr = ctrl_pts_array;
        nurbs_to_tc->knots_ptr = knots_array;
        nurbs_to_tc->N = N_array;
#else
        // SIM
        nurbs_to_tc->ctrl_pts_ptr = (CONTROL_POINT*) malloc(
                sizeof(CONTROL_POINT) * nurbs_block.nr_of_ctrl_pts);
        /* knots array: allocate 'order' of extra knots for THE-NURBS-BOOK-Algorithm */
        nurbs_to_tc->knots_ptr = (double*) malloc(sizeof(double)
                * (nurbs_block.nr_of_knots + nurbs_block.order));
        nurbs_to_tc->N = (double*) malloc(sizeof(double) * (order + 1));
#endif
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
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].R = nurbs_block.weight;
            nurbs_to_tc->ctrl_pts_ptr[nr_of_knots - knots_todo].D = nurbs_block.curvature;
            nurbs_to_tc->knots_ptr[nr_of_knots - knots_todo] = nurbs_block.knot;

        } else {
            nurbs_to_tc->knots_ptr[nr_of_knots - knots_todo] = nurbs_block.knot;

        }
        knots_todo -= 1;
    }


    if ((0 == knots_todo)) {
        int i;
        /* duplicate 'order' of extra knots for THE-NURBS-BOOK-Algorithm */
        for (i=0; i<nurbs_block.order; i++) {
            nurbs_to_tc->knots_ptr[nr_of_knots + i] = nurbs_block.knot;
        }

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
        tc.accel_state = ACCEL_S3;
        tc.distance_to_go = tc.target;
        tc.reqvel = vel;
        tc.maxvel = ini_maxvel * tp->cycleTime;
        tc.maxaccel = ini_maxacc * tp->cycleTime * tp->cycleTime;
        tc.jerk = ini_maxjerk * tp->cycleTime * tp->cycleTime * tp->cycleTime;

        tc.feed_override = 0.0;
        tc.id = tp->nextId;
        tc.active = 0;
        tc.atspeed = 0;//atspeed;  // FIXME-eric(L)

        tc.nurbs_block.curve_len = nurbs_block.curve_len;
        tc.nurbs_block.order = nurbs_block.order;
        tc.nurbs_block.nr_of_ctrl_pts = nurbs_block.nr_of_ctrl_pts;
        tc.nurbs_block.nr_of_knots = nurbs_block.nr_of_knots;
        tc.nurbs_block.reqvel = vel;

        tc.cur_accel = 0.0;
        tc.cur_vel = 0.0;

        tc.motion_type = TC_NURBS;
        tc.canon_motion_type = type;
        tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
        tc.tolerance = tp->tolerance;
        tc.seamless_blend_mode = SMLBLND_INIT;
        tc.nexttc_target = 0;

        tc.synchronized = tp->synchronized;
        tc.velocity_mode = tp->velocity_mode;
        tc.uu_per_rev = tp->uu_per_rev;
        tc.css_progress_cmd = 0;
        tc.enables = enables;
        tc.indexrotary = -1;

        if ((syncdio.anychanged != 0) || (syncdio.sync_input_triggered != 0)) {
            tc.syncdio = syncdio; //enqueue the list of DIOs that need toggling
            tpClearDIOs(); // clear out the list, in order to prepare for the next time we need to use it
        } else {
            tc.syncdio.anychanged = 0;
            tc.syncdio.sync_input_triggered = 0;
        }

        tc.pso = pso;

        //TODO: tc.utvIn = nurbs...;
        //TODO: tc.utvOut = nurbs...;

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

/*
 Continuous form
 PT = P0 + V0T + 1/2A0T2 + 1/6JT3
 VT = V0 + A0T + 1/2 JT2
 AT = A0 + JT

 Discrete time form (let T be 1, then T^2 == 1, T^3 == 1)
 PT = PT + VT + 1/2AT + 1/6J
 VT = VT + AT + 1/2JT
 AT = AT + JT
 */
/**
 * S-curve Velocity Profiler FSM
 * Yishin Li <ysli@araisrobo.com>
 * ARAIS ROBOT TECHNOLOGY, http://www.araisrobo.com/
 **/
void tcRunCycle(TP_STRUCT *tp, TC_STRUCT *tc) 
{
//    double t, t1, vel, acc, v1, dist, req_vel;
    double t, t1, vel, v1, dist, req_vel;

    static double ts, ti;
    static double k, s6_a, s6_v, s6_p, error_d, prev_s, prev_v;
    static double c1, c2, c3, c4, c5, c6;
    static double d1, d2, d3;

    double pi = 3.14159265359;
    int immediate_state;
    double tc_target;

    if(tc->seamless_blend_mode == SMLBLND_ENABLE) {
        tc_target = tc->target + tc->nexttc_target;
    } else {
        tc_target = tc->target;
    }

    immediate_state = 0;
    do {
        switch (tc->accel_state) {
        case ACCEL_S0:
            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_accel = tc->cur_accel + tc->jerk;
            tc->cur_vel = tc->cur_vel + tc->cur_accel + 0.5 * tc->jerk;
            tc->progress = tc->progress + tc->cur_vel + 0.5 * tc->cur_accel + 1.0/6.0 * tc->jerk;

            // check if we hit accel limit at next BP
            if ((tc->cur_accel + tc->jerk) >= tc->maxaccel) {
                tc->cur_accel = tc->maxaccel;
                tc->accel_state = ACCEL_S1;
                break;
            }

            // check if we will hit velocity limit; 
            // assume we start decel from here to "accel == 0"
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            t = ceil(tc->cur_accel / tc->jerk);
            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            vel = req_vel - tc->cur_accel * t + 0.5 * tc->jerk * t * t;
            if (tc->cur_vel >= vel) {
                tc->accel_state = ACCEL_S2;
                s6_v = tc->cur_vel;
                s6_a = fabs(tc->cur_accel);
                ts = floor((2*(req_vel - s6_v))/s6_a);
                k = s6_a*pi/(4*(req_vel - s6_v));
                prev_v = s6_v;
                d1 = s6_a/2;
                d2 = s6_a/(4*k);
                d3 = 2*k;
                ti = 1;
                break;
            }

            // check if we will hit progress limit
            // AT = AT + JT
            // VT = V0 + A0T + 1/2 JT^2
            // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
            // distance for S2
            t = ceil(tc->cur_accel/tc->jerk);
            dist = tc->progress + tc->cur_vel * t + 0.5 * tc->cur_accel * t * t
                    - 1.0/6.0 * tc->jerk * t * t * t;
            vel = tc->cur_vel + tc->cur_accel * t - 0.5 * tc->jerk * t * t;
            // distance for S3
            dist += (vel);

            /* 
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            t = ceil(sqrt(vel/tc->jerk));
            // AT = AT + JT
            t1 = ceil(tc->maxaccel / tc->jerk);   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist += t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist += t * vel;    // dist of (S4 + S6)
            }

            if (tc_target < dist) {
                tc->accel_state = ACCEL_S2;
                s6_v = tc->cur_vel;
                s6_a = fabs(tc->cur_accel);
                ts = floor((2*(req_vel - s6_v))/s6_a);
                k = s6_a*pi/(4*(req_vel - s6_v));
                prev_v = s6_v;
                d1 = s6_a/2;
                d2 = s6_a/(4*k);
                d3 = 2*k;
                ti = 1;
                break;
            }

            break;

        case ACCEL_S1:
            // jerk is 0 at this state
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_vel = tc->cur_vel + tc->cur_accel;
            tc->progress = tc->progress + tc->cur_vel + 0.5 * tc->cur_accel;

            // check if we will hit velocity limit; 
            // assume we start decel from here to "accel == 0"
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            // t = ceil(tc->cur_accel / tc->jerk);
            t = tc->cur_accel / tc->jerk;
            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            vel = req_vel - tc->cur_accel * t + 0.5 * tc->jerk * t * t;
            if (tc->cur_vel >= vel) {
                tc->accel_state = ACCEL_S2;
                s6_v = tc->cur_vel;
                s6_a = fabs(tc->cur_accel);
                ts = floor((2*(req_vel - s6_v))/s6_a);
                k = s6_a*pi/(4*(req_vel - s6_v));
                prev_v = s6_v;
                d1 = s6_a/2;
                d2 = s6_a/(4*k);
                d3 = 2*k;
                ti = 1;
                break;
            }

            // check if we will hit progress limit
            // distance for S2
            t = ceil(tc->cur_accel/tc->jerk);
            dist = tc->progress + tc->cur_vel * t + 0.5 * tc->cur_accel * t * t
                    - 1.0/6.0 * tc->jerk * t * t * t;
            vel = tc->cur_vel + tc->cur_accel * t - 0.5 * tc->jerk * t * t;
            // distance for S3
            dist += (vel);

            /* 
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            t = ceil(sqrt(vel/tc->jerk));
            // AT = AT + JT
            t1 = ceil(tc->maxaccel / tc->jerk);   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist += t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist += t * vel;    // dist of (S4 + S6)
            }

            if (tc_target < dist) {
                tc->accel_state = ACCEL_S2;
                s6_v = tc->cur_vel;
                s6_a = fabs(tc->cur_accel);
                ts = floor((2*(req_vel - s6_v))/s6_a);
                k = s6_a*pi/(4*(req_vel - s6_v));
                prev_v = s6_v;
                d1 = s6_a/2;
                d2 = s6_a/(4*k);
                d3 = 2*k;
                ti = 1;
                break;
            }
            break;


        case ACCEL_S2: 

            if(ti <= ts){
                tc->cur_vel = d1*ti+d2*sin(d3*ti)+s6_v;
                tc->cur_accel = tc->cur_vel - prev_v;
                tc->progress = tc->progress + tc->cur_vel + 0.5 * tc->cur_accel;
                prev_v = tc->cur_vel;
                ti = ti + 1;
            }
            else {
                tc->cur_accel = 0;
                tc->accel_state = ACCEL_S3;
            }

            // to DECELERATE to ACCEL==0

            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
//            tc->cur_accel = tc->cur_accel - tc->jerk;
//            tc->cur_vel = tc->cur_vel + tc->cur_accel - 0.5 * tc->jerk;
//            tc->progress = tc->progress + tc->cur_vel + 0.5 * tc->cur_accel - 1.0/6.0 * tc->jerk;
//
//            // check if (accel <= 0) at next BP
//            acc = tc->cur_accel - tc->jerk;
//            if (acc <= 0) {
//                tc->accel_state = ACCEL_S3;
//                break;
//            }
//
//            // check if we will hit velocity limit at next BP
//            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
//            if (req_vel > tc->maxvel) {
//                req_vel = tc->maxvel;
//            }
//            // vel: velocity at next BP
//            vel = tc->cur_vel + tc->cur_accel - 1.5 * tc->jerk;
//            if (vel > req_vel) {
//                tc->cur_vel = req_vel;
//                tc->accel_state = ACCEL_S3;
//                break;
//            }
//
//            // check if we will hit progress limit
//            // refer to 2011-10-17 ysli design note
//            // AT = AT + JT
//            // VT = V0 + A0T + 1/2 JT^2
//            // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
//            vel = tc->cur_vel;
//            // distance for S3
//            dist = tc->progress + (vel);
//
//            /*
//            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
//            t1 = sqrt(vel/j)
//             */
//            t = ceil(sqrt(vel/tc->jerk));
//            // AT = AT + JT
//            t1 = ceil(tc->maxaccel / tc->jerk);   // max time for S4
//            if (t > t1) {
//                // S4 -> S5 -> S6
//                dist += t1 * vel;    // dist of (S4 + S6)
//                // calc decel.dist for ACCEL_S5
//                // t: time for S5
//                t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
//                v1 = vel - 0.5 * tc->jerk * t1 * t1;
//                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
//                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
//            } else {
//                // S4 -> S6
//                dist += t * vel;    // dist of (S4 + S6)
//            }
//
//            if (tc_target < dist) {
//                tc->accel_state = ACCEL_S3;
//                break;
//            }

            break;

        case ACCEL_S3:
            // PT = PT + VT + 1/2AT + 1/6JT
            // , where (jerk == 0) and (accel == 0)
            tc->cur_accel = 0;
            tc->progress = tc->progress + tc->cur_vel;

            // check if we will hit progress limit
            // refer to 2011-10-17 ysli design note
            // AT = AT + JT
            // VT = V0 + A0T + 1/2 JT^2
            // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
            /* 
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            vel = tc->cur_vel;
            // t = floor(sqrt(vel/tc->jerk) - 0.5);
            t = sqrt(vel/tc->jerk);
            // t = sqrt(vel/tc->jerk);
            // AT = AT + JT
            // t1 = floor(tc->maxaccel / tc->jerk - 0.5);   // max time for S4
            t1 = tc->maxaccel / tc->jerk;   // max time for S4
            // t1 = tc->maxaccel / tc->jerk;   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist = tc->progress + t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                // t = floor((vel - tc->maxaccel * t1) / tc->maxaccel - 0.5);
                t = (vel - tc->maxaccel * t1) / tc->maxaccel - 0.5;
                // t = (vel - tc->maxaccel * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist = tc->progress + t * vel;    // dist of (S4 + S6)
            }

            // check if dist would be greater than tc_target at next cycle
            if (tc_target < (dist - vel)) {
                tc->accel_state = ACCEL_S4;
                DP("to ACCEL_S4\n");
                // blending at largest velocity for G64 w/o P<tolerance>
                if (!tc->tolerance) {
                    tc->tolerance = tc->target - tc->progress; // tc->distance_to_go
                } 
                break;
            }

            // check for changes of feed_override and request-velocity
            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            if ((tc->cur_vel + 1.5 * tc->jerk) < req_vel) {
                tc->accel_state = ACCEL_S0;
                break;
            } else if ((tc->cur_vel - 1.5 * tc->jerk) > req_vel) {
                tc->accel_state = ACCEL_S4;
                DP("to ACCEL_S4\n");
                break;
            }
            tc->cur_vel = req_vel;
            break;


        case ACCEL_S4:
            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_accel = tc->cur_accel - tc->jerk;
            tc->cur_vel = tc->cur_vel + tc->cur_accel - 0.5 * tc->jerk;
            if (tc->cur_vel <= 0) {
                tc->cur_vel = 0;
                tc->accel_state = ACCEL_S3;
                break;
            }
            tc->progress = tc->progress + tc->cur_vel + 0.5 * tc->cur_accel - 1.0/6.0 * tc->jerk;

            // (accel < 0) and (jerk < 0)
            assert (tc->cur_accel < 0);

            // check if we hit accel limit at next BP
            if ((tc->cur_accel - tc->jerk) <= -tc->maxaccel) {
                tc->cur_accel = -tc->maxaccel;
                tc->accel_state = ACCEL_S5;
                break;
            }

            // should we stay in S4 and keep decel?
            // calculate dist for S4 -> (maybe S5) -> S6
            t = - tc->cur_accel / tc->jerk;
            // target dist after switching to S6 (jerk is positive for S6)
            dist = tc->progress + tc->cur_vel * t 
                    + 0.5 * tc->cur_accel * t * t
                    + 1.0 / 6.0 * tc->jerk * t * t * t;
            // VT = V0 + A0T + 1/2JT2
            // obtain vel for S6 -> S3
            vel = tc->cur_vel + tc->cur_accel * t + 0.5 * tc->jerk * t * t;
            if (vel > 0) {
                /*
                0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
                t1 = sqrt(vel/j)
                 */
                t = ceil(sqrt(vel/tc->jerk));
                // AT = AT + JT
                t1 = ceil(tc->maxaccel / tc->jerk);   // max time for S4
                if (t > t1) {
                    // S4 -> S5 -> S6
                    dist += t1 * vel;    // dist of (S4 + S6)
                    // calc decel.dist for ACCEL_S5
                    // t: time for S5
                    t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                    v1 = vel - 0.5 * tc->jerk * t1 * t1;
                    // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                    dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
                } else {
                    // S4 -> S6
                    dist += t * vel;    // dist of (S4 + S6)
                }
            }

            if (tc_target < (dist - (tc->cur_vel + 1.5 * tc->cur_accel - 2.1666667 * tc->jerk))) {
                tc->accel_state = ACCEL_S4;
                DPS("should stay in S4 and keep decel\n");
                break;
            }

            // check if we will approaching requested velocity
            // vel should not be greater than "request velocity" after
            // starting acceleration to "accel == 0".
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            t = - tc->cur_accel / tc->jerk;
            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            if ((tc->cur_vel + tc->cur_accel * t + 0.5 * tc->jerk * t * t) <= req_vel) {
                if(tc->progress/tc->target < 0.9){
                    tc->accel_state = ACCEL_S6;
                    DPS("S4: hit velocity rule; move to S6\n");
                }
                else
                {
                    tc->accel_state = ACCEL_S7;
                    s6_v = tc->cur_vel;
                    s6_a = fabs(tc->cur_accel);
                    s6_p = tc->progress;
                    ts = floor((2*s6_v)/s6_a);
                    k = s6_a*pi/(4*s6_v);
                    error_d = tc->target - tc->progress - s6_v * s6_v / s6_a * (1-4/(pi*pi));
                    prev_s = 0;
                    prev_v = s6_v;
                    c1 = -s6_a/4;
                    c2 = s6_v+((error_d*s6_a)/(2*s6_v));
                    c3 = s6_a/(8*k*k);
                    c4 = 2*k;
                    c5 = -(error_d*s6_a)/(8*k*s6_v);
                    c6 = 4*k;
                    ti = 1;
                    DPS("S4: hit distance rule; move to S6\n");
                    break;
                }

            }

//            // check if dist would be greater than tc_target at next cycle
//            printf ("tc_target(%f) dist(%f) vel(%f) t(%f) t1(%f)\n", tc_target, dist, vel, t, t1);
////            if (tc_target > (dist - (tc->cur_vel + 1.5 * tc->cur_accel - 2.1666667 * tc->jerk)))
//            if (vel > 0)
//            {
//                // check if we will approaching requested velocity
//                // vel should not be greater than "request velocity" after
//                // starting acceleration to "accel == 0".
//                //
//                // AT = A0 + JT (let AT = 0 to calculate T)
//                // VT = V0 + A0T + 1/2JT2
//                req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
//                if (req_vel > tc->maxvel) {
//                    req_vel = tc->maxvel;
//                }
//                if (dist < tc_target) {
//                    if (vel <= req_vel) {
//                        tc->accel_state = ACCEL_S6;
//                        DPS("S4: hit velocity rule; move to S6 to decel to req_vel(%f)\n", req_vel);
//                        break;
//                    }
//                }
//            } else
//            {
//                tc->accel_state = ACCEL_S7;
//                s6_v = tc->cur_vel;
//                s6_a = fabs(tc->cur_accel);
//                s6_p = tc->progress;
//                ts = floor((2*s6_v)/s6_a);
//                k = s6_a*pi/(4*s6_v);
//                error_d = tc->target - tc->progress - s6_v * s6_v / s6_a * (1-4/(pi*pi));
//                prev_s = 0;
//                prev_v = s6_v;
//                c1 = -s6_a/4;
//                c2 = s6_v+((error_d*s6_a)/(2*s6_v));
//                c3 = s6_a/(8*k*k);
//                c4 = 2*k;
//                c5 = -(error_d*s6_a)/(8*k*s6_v);
//                c6 = 4*k;
//                ti = 1;
//                DPS("S4: hit distance rule; move to S6\n");
//                break;
//            }

            break;

        case ACCEL_S5:
            // jerk is 0 at this state
            // accel < 0
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_vel = tc->cur_vel + tc->cur_accel;
            tc->progress = tc->progress + tc->cur_vel + 0.5 * tc->cur_accel;

            // should we stay in S5 and keep decel?
            // calculate dist for S6 -> S4 -> (maybe S5) -> S6
            t = - tc->cur_accel / tc->jerk;
            // target dist after switching to S6 (jerk is positive for S6)
            dist = tc->progress + tc->cur_vel * t 
                    + 0.5 * tc->cur_accel * t * t
                    + 1.0 / 6.0 * tc->jerk * t * t * t;
            // VT = V0 + A0T + 1/2JT2
            // obtain vel for S6 -> S3
            vel = tc->cur_vel + tc->cur_accel * t + 0.5 * tc->jerk * t * t;

            if (vel > 0) { 
                /* S6 -> S3 -> S4 -> S5(maybe) -> S6 */
                /* 
                0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
                t1 = sqrt(vel/j)
                 */
                t = ceil(sqrt(vel/tc->jerk));
                // AT = AT + JT
                t1 = ceil(tc->maxaccel / tc->jerk);   // max time for S4
                if (t > t1) {
                    // S4 -> S5 -> S6
                    dist += t1 * vel;    // dist of (S4 + S6)
                    // calc decel.dist for ACCEL_S5
                    // t: time for S5
                    t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                    v1 = vel - 0.5 * tc->jerk * t1 * t1;
                    // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                    dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
                } else {
                    // S4 -> S6
                    dist += t * vel;    // dist of (S4 + S6)
                }
            }

            // check if dist would be greater than tc_target at next cycle
            if (tc_target < (dist - (tc->cur_vel + 1.5 * tc->cur_accel))) {
                tc->accel_state = ACCEL_S5;
                DPS("should stay in S5 and keep decel\n");
                break;
            }

            // check if we will approaching requested velocity
            // vel should not be greater than "request velocity" after
            // starting acceleration to "accel == 0".
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            // t: cycles for accel to decel to 0
            t = - tc->cur_accel / tc->jerk;
            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            if ((tc->cur_vel + tc->cur_accel * t + 0.5 * tc->jerk * t * t) <= req_vel) {
                tc->accel_state = ACCEL_S7;
                s6_v = tc->cur_vel;
                s6_a = fabs(tc->cur_accel);
                s6_p = tc->progress;
                ts = floor((2*s6_v)/s6_a);
                k = s6_a*pi/(4*s6_v);
                error_d = tc->target - tc->progress - s6_v * s6_v / s6_a * (1-4/(pi*pi));
                prev_s = 0;
                prev_v = s6_v;
                c1 = -s6_a/4;
                c2 = s6_v+((error_d*s6_a)/(2*s6_v));
                c3 = s6_a/(8*k*k);
                c4 = 2*k;
                c5 = -(error_d*s6_a)/(8*k*s6_v);
                c6 = 4*k;
                ti = 1;
                break;
            }

            break;

        case ACCEL_S6:
            // for approaching to req_vel
            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            req_vel = tc->reqvel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }

            tc->cur_accel = tc->cur_accel + tc->jerk;
            tc->cur_vel = tc->cur_vel + tc->cur_accel + 0.5 * tc->jerk;

            if (tc->cur_vel <= req_vel) {
                tc->accel_state = ACCEL_S3;
                if ((req_vel - tc->cur_vel) < 1.5*tc->jerk) {
                    // align to req_vel only when not changing feed_override
                    tc->cur_vel = req_vel;
                }
            }
            dist = tc->cur_vel + 0.5 * tc->cur_accel + 1.0/6.0 * tc->jerk;
            tc->progress = tc->progress + dist;

            if (tc->cur_accel >= 0) {
                tc->accel_state = ACCEL_S3;
            }

            break;

        case ACCEL_S7:
            // decel to target position based on Jofey's algorithm

            if(ti <= ts){
                dist = c1*ti*ti + c2*ti + c3*cos(c4*ti) + c5*cos(c6*ti-0.5*pi) - c3;
                tc->cur_vel = dist - prev_s;
                tc->cur_accel = tc->cur_vel - prev_v;
                prev_s = dist;
                prev_v = tc->cur_vel;
                tc->progress = s6_p + dist;
                ti = ti + 1;
            }
            else {
                tc->cur_vel = 0;
                tc->cur_accel = 0;
                tc->progress = tc->target;
                tc->accel_state = ACCEL_S3;
            }
            break;

        default:
            assert(0);
        } // switch (tc->accel_state)
    } while (immediate_state);

    if (tc->seamless_blend_mode != SMLBLND_ENABLE) {
        if (tc->progress >= tc->target) {
            // finished
            // DPS("hit target, cur_accel(%f), cur_vel(%f)\n", tc->cur_accel, tc->cur_vel);
            tc->progress = tc->target;
            tc->cur_accel = 0;
            tc->cur_vel = 0;
        }
    }

    if (tc->pso.enable)
    {
        if (tc->progress > tc->pso.next_progress)
        {
            double temp_progress;
            DP ("TODO: resolve blending two segments: may cause double entrance of tcRunCycle ()\n");
            DP ("tc->progress(%f) tc->pso.next_progress(%f)\n", tc->progress, tc->pso.next_progress);
            temp_progress = tc->progress;
            tc->progress = tc->pso.next_progress;
            emcmotStatus->pso_pos = tcGetPos(tc);
            tc->progress = temp_progress;
            tc->pso.next_progress += tc->pso.pitch;
            emcmotStatus->pso_mode = tc->pso.mode;
            emcmotStatus->pso_tick = tc->pso.tick;
            emcmotStatus->pso_req = 1;
            DP ("tp.c: pso_pos x(%f) y(%f)\n", emcmotStatus->pso_pos.tran.x, emcmotStatus->pso_pos.tran.y);
            DP ("tp.c: turn pso_req ON, pso_req(%d)\n", emcmotStatus->pso_req);
        }
        else
        {
            emcmotStatus->pso_req = 0;
        }
    }

    DPS("%11u%6d%15.5f%15.5f%15.5f%15.5f%15.6f%15.5f%15.5f\n",
            _dt, tc->accel_state, tc->reqvel * tc->feed_override * tc->cycle_time,
            tc->cur_accel, tc->cur_vel, tc->progress/tc->target, tc->progress,
            (tc->target - tc->progress), tc_target);
    tc->distance_to_go = tc->target - tc->progress;
    //TODO: this assert will be triggered with rockman.ini: 
    //      assert (tc->cur_vel >= 0);
}

void tpToggleDIOs(TC_STRUCT * tc) 
{
    int i = 0;
    if (tc->syncdio.anychanged != 0) { // we have DIO's to turn on or off
        for (i=0; i < emcmotConfig->numDIO; i++) {
            //            if (!(tc->syncdio.dio_mask & (1 << i))) continue;
            if (tc->syncdio.dios[i] > 0) emcmotDioWrite(i, 1); // turn DIO[i] on
            if (tc->syncdio.dios[i] < 0) emcmotDioWrite(i, 0); // turn DIO[i] off
        }
        for (i=0; i < emcmotConfig->numAIO; i++) {
            if (!(tc->syncdio.aio_mask & (1 << i))) continue;
            emcmotAioWrite(i, tc->syncdio.aios[i]); // set AIO[i]
        }
        tc->syncdio.anychanged = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }

    if (tc->syncdio.sync_input_triggered != 0) {
        emcmotSyncInputWrite(tc->syncdio.sync_in, tc->syncdio.timeout, tc->syncdio.wait_type);
        tc->syncdio.sync_input_triggered = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }
}

static void tpSetRotaryUnlock(int axis, int unlock) {
    emcmotSetRotaryUnlock(axis, unlock);
}

static int tpGetRotaryIsUnlocked(int axis) {
    return emcmotGetRotaryIsUnlocked(axis);
}


// This is the brains of the operation.  It's called every TRAJ period
// and is expected to set tp->currentPos to the new machine position.
// Lots of other tp fields (depth, done, etc) have to be twiddled to
// communicate the status; I think those are spelled out here correctly
// and I can't clean it up without breaking the API that the TP presents
// to motion.  It's not THAT bad and in the interest of not touching
// stuff outside this directory, I'm going to leave it for now.

int tpRunCycle(TP_STRUCT * tp, long period)
{
#if (TRACE!=0)
    _dt += 1;
#endif

    TC_STRUCT *tc, *nexttc;
    EmcPose primary_before, primary_after;
    EmcPose secondary_before, secondary_after;
    EmcPose primary_displacement, secondary_displacement;
    static int waiting_for_index = MOTION_INVALID_ID;
    static int waiting_for_atspeed = MOTION_INVALID_ID;
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
        // spindleSync maps to motion.spindle-velocity-mode
        emcmotStatus->spindleSync = 0;
        emcmotStatus->xuu_per_rev = 0;
        emcmotStatus->yuu_per_rev = 0;
        emcmotStatus->zuu_per_rev = 0;
        emcmotStatus->pso_req = 0;
        return 0;
    }


    if (tc->target == tc->progress && waiting_for_atspeed != tc->id) {
        if (emcmotStatus->probing && *(emcmot_hal_data->rtp_running))
        {
            // G38.X: 等待 RISC 處理完所有 TP 命令才結束
            return 0;
        }

        // if we're synced, and this move is ending, save the
        // spindle position so the next synced move can be in
        // the right place.
        emcmotStatus->xuu_per_rev = 0;
        emcmotStatus->yuu_per_rev = 0;
        emcmotStatus->zuu_per_rev = 0;

        if(tc->indexrotary != -1) {
            // this was an indexing move, so before we remove it we must
            // relock the axis
            tpSetRotaryUnlock(tc->indexrotary, 0);
            // if it is now locked, fall through and remove the finished move.
            // otherwise, just come back later and check again
            if(tpGetRotaryIsUnlocked(tc->indexrotary))
            {
                return 0;
            }
        }

        // done with this move
        tcqRemove(&tp->queue, 1);
        tp->depth = tcqLen(&tp->queue);

        // so get next move
        tc = tcqItem(&tp->queue, 0, period);
        if(!tc)
        {
            emcmotStatus->pso_req = 0;
            return 0;
        }
    }

    // now we have the active tc.  get the upcoming one, if there is one.
    // it's not an error if there isn't another one - we just don't
    // do blending.  This happens in MDI for instance.
    if(!emcmotDebug->stepping && tc->blend_with_next) 
        nexttc = tcqItem(&tp->queue, 1, period);
    else
        nexttc = NULL;

    {
        int this_synch_pos = tc->synchronized && !tc->velocity_mode;
        int next_synch_pos = nexttc && nexttc->synchronized && !nexttc->velocity_mode;
        if(!this_synch_pos && next_synch_pos) {
            // we'll have to wait for spindle sync; might as well
            // stop at the right place (don't blend)
            tc->blend_with_next = 0;
            nexttc = NULL;
        }
    }

    if(nexttc && nexttc->atspeed) {
        // we'll have to wait for the spindle to be at-speed; might as well
        // stop at the right place (don't blend), like above
        tc->blend_with_next = 0;
        nexttc = NULL;
    }

    if(tp->aborting) {
        // an abort message has come
        if( MOTION_ID_VALID(waiting_for_index) ||
                MOTION_ID_VALID(waiting_for_atspeed) ||
                (tc->cur_vel == 0.0 && !nexttc) ||
                (tc->cur_vel == 0.0 && nexttc && nexttc->cur_vel == 0.0) ) {
            tcqInit(&tp->queue);
            tp->goalPos = tp->currentPos;
            tp->done = 1;
            tp->depth = tp->activeDepth = 0;
            tp->aborting = 0;
            tp->execId = 0;
            tp->motionType = 0;
            tp->synchronized = 0;
            tc->accel_state = ACCEL_S3;
            waiting_for_index = MOTION_INVALID_ID;
            waiting_for_atspeed = MOTION_INVALID_ID;
            emcmotStatus->spindleSync = 0;
            emcmotStatus->xuu_per_rev = 0;
            emcmotStatus->yuu_per_rev = 0;
            emcmotStatus->zuu_per_rev = 0;
            tpResume(tp);
            return 0;
        } else {
            tc->reqvel = 0.0;
            if(nexttc) nexttc->reqvel = 0.0;
        }
    }

    // this is no longer the segment we were waiting_for_index for
    if (MOTION_ID_VALID(waiting_for_index) && waiting_for_index != tc->id) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for index on motion id %d, but reached id %d\n",
                waiting_for_index, tc->id);
        waiting_for_index = MOTION_INVALID_ID;
    }

    if (MOTION_ID_VALID(waiting_for_atspeed) && waiting_for_atspeed != tc->id)  
    {

        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for atspeed on motion id %d, but reached id %d\n",
                waiting_for_atspeed, tc->id);
        waiting_for_atspeed = MOTION_INVALID_ID;
    }

    // check for at-speed before marking the tc active
    if (MOTION_ID_VALID(waiting_for_atspeed)) {
        if((emcmotStatus->spindle.on) && (!emcmotStatus->spindle.at_speed)) {
            /* spindle is still not at the right speed: wait */
            return 0;
        } else {
            waiting_for_atspeed = MOTION_INVALID_ID;
        }
    }


    if(tc->active == 0) {
        // this means this tc is being read for the first time.

        if (tc->indexrotary != -1) {
            // request that the axis unlock
            tpSetRotaryUnlock(tc->indexrotary, 1);
            // if it is unlocked, fall through and start the move.
            // otherwise, just come back later and check again
            if (!tpGetRotaryIsUnlocked(tc->indexrotary))
                return 0;
        }

        tc->active = 1;
        tc->cur_vel = 0;
        //ysli: can't understand the usage of activeDepth.
        tp->activeDepth = 1;
        tp->motionType = tc->canon_motion_type;
        tc->blending = 0;

        // TODO: make sure the atspeed judgment is valid for G33 and G33.1
        if(tc->atspeed) {
            // force to wait 1 more cycle for updating emcmotStatus->spindle.at_speed
            waiting_for_atspeed = tc->id;
            emcmotStatus->xuu_per_rev = 0;
            emcmotStatus->yuu_per_rev = 0;
            emcmotStatus->zuu_per_rev = 0;
            return 0;
        }
    }


    emcmotStatus->spindleSync = tc->synchronized;
    if(tc->synchronized) {
        if (!tc->coords.spindle_sync.spindle_start_pos_latch)
        {
            if (tc->coords.spindle_sync.mode < 2)
            {   // G33, G33.1
                tc->coords.spindle_sync.spindle_start_pos_latch = 1;
                tc->coords.spindle_sync.spindle_start_pos = emcmotStatus->carte_pos_cmd.s;
                tc->cur_vel = fabs(emcmotStatus->spindle.curr_vel_rps) * tc->cycle_time;

                /* bitmap for rigid-tapping-AXIS_X */
                if (tc->coords.spindle_sync.xyz.uVec.x > tiny)
                {
                    emcmotStatus->xuu_per_rev = tc->uu_per_rev * tc->coords.spindle_sync.xyz.uVec.x;
                }
                /* bitmap for rigid-tapping-AXIS_Y */
                if (tc->coords.spindle_sync.xyz.uVec.y > tiny)
                {
                    emcmotStatus->yuu_per_rev = tc->uu_per_rev * tc->coords.spindle_sync.xyz.uVec.y;
                    printf("emcmotStatus->yuu_per_rev(%f)\n",emcmotStatus->yuu_per_rev);
                }
                /* bitmap for rigid-tapping-AXIS_Z */
                if (tc->coords.spindle_sync.xyz.uVec.z > tiny)
                {
                    emcmotStatus->zuu_per_rev = tc->uu_per_rev * tc->coords.spindle_sync.xyz.uVec.z;
                }
                return 0;   // for atspeed detection
            }
            else if (tc->coords.spindle_sync.mode == 2)
            {   // G33.2, wait for spindle to be stopped completely
                if (emcmotStatus->spindle.curr_vel_rps == 0)
                {
                    double start_angle; // unit: rev
                    tc->coords.spindle_sync.spindle_start_pos_latch = 1;
                    tc->coords.spindle_sync.spindle_start_pos = emcmotStatus->carte_pos_cmd.s;

                    tc->cur_vel = 0;

                    start_angle = emcmotStatus->carte_pos_cmd.s - floor(emcmotStatus->carte_pos_cmd.s);
                    tc->target = (tc->coords.spindle_sync.spindle_end_angle - start_angle) * tc->coords.spindle_sync.spindle_dir;
                    if (tc->target < 0)
                    {
                        tc->target += 1;        // move toward spindle_end_angle
                    }
                    DP("start_angle(%f)\n", start_angle);
                    DP("end_angle(%f)\n", tc->coords.spindle_sync.spindle_end_angle);
                    DP("emcmotStatus->spindle.direction(%d)\n", emcmotStatus->spindle.direction);
                    DP("tc->coords.spindle_sync.spindle_dir(%f)\n", tc->coords.spindle_sync.spindle_dir);
                    DP("target(%f)\n", tc->target);
                }
                return 0;   // for spindle stop detection
            }
        }

        if (tc->coords.spindle_sync.mode < 2)
        {   // G33, G33.1
            tc->reqvel = fabs(emcmotStatus->spindle.speed_req_rps) * emcmotStatus->net_spindle_scale;
        }
        else if (tc->coords.spindle_sync.mode == 2)
        {   // G33.2, unit for spindle_reqvel is RPS
            tc->reqvel = fabs(tc->coords.spindle_sync.spindle_reqvel) * emcmotStatus->net_spindle_scale;
        }

        if(tp->aborting) {
            tc->reqvel = 0;
        }

    }
    else
    {   // non spindle sync motion
        emcmotStatus->xuu_per_rev = 0;
        emcmotStatus->yuu_per_rev = 0;
        emcmotStatus->zuu_per_rev = 0;
    }

    if(nexttc && nexttc->active == 0) {
        // this means this tc is being read for the first time.

        nexttc->cur_vel = 0;
        tp->activeDepth = 1;
        nexttc->active = 1;
        nexttc->blending = 0;

    }

    tc->feed_override = emcmotStatus->net_feed_scale;
    if(nexttc) {
        nexttc->feed_override = emcmotStatus->net_feed_scale;
    }

    /* handle pausing */
    if(tp->pausing && (!tc->synchronized || tc->velocity_mode)) {
        tc->feed_override = 0.0;
        if(nexttc) {
            nexttc->feed_override = 0.0;
        }
    }

    // calculate the approximate peak velocity the nexttc will hit.
    // we know to start blending it in when the current tc goes below
    // this velocity...
    if(nexttc && nexttc->maxaccel) {

#ifdef SMLBLND
        /**
         * TODO: 
         * G64 Q[01] seamless blending:
         * where Q1 means "enable seamless blending";
         *       Q0 means "disable seamless blending"
         **/
        if (tc->seamless_blend_mode == SMLBLND_INIT) {
            double dot;
            double k;   /* curvature */
            double ca;  /* centripetal acceleration */
            double rv;  /* request velocity per cycleTime */
            rv = tc->reqvel * tp->cycleTime;
            if (rv > tc->maxvel) {
                rv = tc->maxvel;
            }
            // pmCartCartDisp(tc->utvOut, nexttc->utvIn, &k);
            pmCartCartDot(tc->utvOut, nexttc->utvIn, &dot);
            k = acos(dot)/rv;
            ca = k * rv * rv;
            // SMLBLND is for XYZ motion only
            if ((ca < tc->maxaccel) && (!tc->coords.line.xyz.tmag_zero) && (!nexttc->coords.line.xyz.tmag_zero)) {
                // allow seamless blending, SMLBLND
                // also, (nexttc->atspeed == 0)
                tc->seamless_blend_mode = SMLBLND_ENABLE;
                tc->nexttc_target = nexttc->target;
            } else {
                tc->seamless_blend_mode = SMLBLND_DISABLE;
            }
            DPS("tc->utvOut: x(%f) y(%f) z(%f)\n",
                    tc->utvOut.x,
                    tc->utvOut.y,
                    tc->utvOut.z);
            DPS("nexttc->utvIn: x(%f) y(%f) z(%f)\n",
                    nexttc->utvIn.x,
                    nexttc->utvIn.y,
                    nexttc->utvIn.z);
            DPS("k(%f) rv(%f) ca(%f) tc.maxaccel(%f) tc.jerk(%f)\n",
                    k,
                    rv * 1000000,
                    ca * 1000000,
                    tc->maxaccel * 1000000,
                    tc->jerk * 1000000);
            if (tc->seamless_blend_mode == SMLBLND_ENABLE) {
                DPS("SMLBLND_ENABLE\n");
            } else {
                DPS("SMLBLND_DISABLE\n");
            }
        }
#endif // SMLBLND
    }

    primary_before = tcGetPos(tc);
    tcRunCycle(tp, tc);

#ifdef SMLBLND
    if ((tc->seamless_blend_mode == SMLBLND_ENABLE) && 
            (tc->progress >= tc->target)) {
        // update tc with nexttc
        double next_vel;
        double next_accel;
        enum state_type next_accel_state;
        double next_progress;

        next_vel = tc->cur_vel;
        next_accel = tc->cur_accel;
        next_accel_state = tc->accel_state;
        next_progress = tc->progress - tc->target;

        tcqRemove(&tp->queue, 1);
        tp->depth = tcqLen(&tp->queue);

        // so get next move
        tc = tcqItem(&tp->queue, 0, period);

        assert(tc); // there must be nexttc in the tcq
        assert(tc->atspeed == 0);
        tc->active = 1;
        tc->cur_vel = next_vel;
        tc->cur_accel = next_accel;
        tc->accel_state = next_accel_state;
        tc->progress = next_progress;
        tp->activeDepth = 1;
        tp->motionType = tc->canon_motion_type;
        tc->blending = 0;
    }
#endif // SMLBLND

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
    if( (tc->blending && nexttc) || 
            (nexttc &&
                    (tc->seamless_blend_mode == SMLBLND_DISABLE) &&
                    (tc->distance_to_go <= tc->tolerance) &&
                    (nexttc->target >= (tc->distance_to_go * 2)))) {

        // make sure we continue to blend this segment even when its 
        // accel reaches 0 (at the very end)
        tc->blending = 1;

        // hack to show blends in axis
        // tp->motionType = 0;

        if(tc->cur_vel > nexttc->cur_vel) {
            target = tcGetEndpoint(tc);
            tp->motionType = tc->canon_motion_type;
            emcmotStatus->distance_to_go = tc->target - tc->progress;
            emcmotStatus->progress = tc->progress;
            emcmotStatus->enables_queued = tc->enables;
            // report our line number to the guis
            tp->execId = tc->id;
            emcmotStatus->requested_vel = tc->reqvel;
        } else {
            tpToggleDIOs(nexttc); //check and do DIO changes
            target = tcGetEndpoint(nexttc);
            tp->motionType = nexttc->canon_motion_type;
            emcmotStatus->distance_to_go = nexttc->target - nexttc->progress;
            emcmotStatus->progress = nexttc->progress;
            emcmotStatus->enables_queued = nexttc->enables;
            // report our line number to the guis
            tp->execId = nexttc->id;
            emcmotStatus->requested_vel = nexttc->reqvel;
        }

        emcmotStatus->current_vel = (tc->cur_vel + nexttc->cur_vel) / tc->cycle_time;

        secondary_before = tcGetPos(nexttc);
        tcRunCycle(tp, nexttc);

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
    } else {
        // not blending
        tpToggleDIOs(tc); //check and do DIO changes
        target = tcGetEndpoint(tc);
        tp->motionType = tc->canon_motion_type;
        emcmotStatus->distance_to_go = tc->target - tc->progress;
        emcmotStatus->progress = tc->progress;
        tp->currentPos = primary_after;
        emcmotStatus->current_vel = (tc->cur_vel) / tc->cycle_time;
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

    emcmotStatus->motion_type = tc->motion_type;
    emcmotStatus->motionState = tc->accel_state;

    return 0;
}

int tpSetSpindleSync(TP_STRUCT * tp, double uu_per_rev, int wait_for_index, int synchronized)
{
    // tp->synchronized is for updating tc->synchronized and emcmotStatus->spindleSync
    tp->uu_per_rev = uu_per_rev;
    tp->synchronized = synchronized;    // synchronized spindle motion
    tp->velocity_mode = wait_for_index; // TODO: replace velocity_mode as wait-for-index
    return 0;
}

int tpPause(TP_STRUCT * tp)
{
    if (0 == tp) {
        return -1;
    }
    tp->pausing = 1;
    return 0;
}

int tpResume(TP_STRUCT * tp)
{
    if (0 == tp) {
        return -1;
    }
    tp->pausing = 0;
    return 0;
}

int tpAbort(TP_STRUCT * tp)
{
    if (0 == tp) {
        return -1;
    }

    if (!tp->aborting) {
        /* to abort, signal a pause and set our abort flag */
        tpPause(tp);
        tp->aborting = 1;
    }

    tpClearPSO();
    tpClearDIOs(); //clears out any already cached DIOs
    return 0;
}

int tpGetMotionType(TP_STRUCT * tp)
{
    return tp->motionType;
}

EmcPose tpGetPos(TP_STRUCT * tp)
{
    EmcPose retval;

    if (0 == tp) {
        ZERO_EMC_POSE(retval);
        return retval;
    }

    return tp->currentPos;
}

int tpIsDone(TP_STRUCT * tp)
{
    if (0 == tp) {
        return 0;
    }

    return tp->done;
}

int tpQueueDepth(TP_STRUCT * tp)
{
    if (0 == tp) {
        return 0;
    }

    return tp->depth;
}

int tpActiveDepth(TP_STRUCT * tp)
{
    if (0 == tp) {
        return 0;
    }

    return tp->activeDepth;
}

int tpSetPSO(TP_STRUCT *tp, int enable, double pitch, int mode, int tick)
{
    if (0 == tp) {
        return -1;
    }
    pso.enable = enable;
    pso.pitch = pitch;
    pso.next_progress = 0;
    pso.mode = mode;
    pso.tick = tick;
    return 0;
}

int tpSetAout(TP_STRUCT *tp, unsigned char index, double start, double end) {
    if (0 == tp) {
        return -1;
    }
    syncdio.anychanged = 1; //something has changed
    syncdio.aio_mask |= (1 << index);
    syncdio.aios[index] = start;
    return 0;
}

int tpSetDout(TP_STRUCT *tp, int index, unsigned char start, unsigned char end) {
    if (0 == tp) {
        return -1;
    }
    syncdio.anychanged = 1; //something has changed
    //    syncdio.dio_mask |= (1 << index);
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
// vim:sw=4:sts=4:et:
