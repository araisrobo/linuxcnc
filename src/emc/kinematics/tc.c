/*!
********************************************************************
* Description: tc.c
*\brief Discriminate-based trajectory planning
*
*\author Derived from a work by Fred Proctor & Will Shackleford
*\author rewritten by Chris Radek
*
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/

/*
  FIXME-- should include <stdlib.h> for sizeof(), but conflicts with
  a bunch of <linux> headers
  */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "rtapi.h"		/* rtapi_print_msg */
#include "posemath.h"
#include "emcpos.h"
#include "tc.h"
#include "nurbs.h"
#include "../motion/motion.h"
//#include "hal.h"
//#include "../motion/mot_priv.h"
//#include "motion_debug.h"

#define TRACE 0
#include "dptrace.h"

#if (TRACE != 0)
    static FILE *dptrace = NULL;
    static uint32_t _dt = 0;
#endif

extern emcmot_status_t *emcmotStatus;


static int
ON_SearchMonotoneArray(const double* array, int length, double t)
/*****************************************************************************
Find interval in an increasing array of doubles

INPUT:
  array
    A monotone increasing (array[i] <= array[i+1]) array of length doubles.
  length (>=1)
    number of doubles in array
  t
    parameter
OUTPUT:
  ON_GetdblArrayIndex()
    -1:         t < array[0]
     i:         (0 <= i <= length-2) array[i] <= t < array[i+1]
     length-1:  t == array[length-1]
     length:    t  > array[length-1]
COMMENTS:
  If length < 1 or array is not monotone increasing, you will get a meaningless
  answer and may crash your program.
EXAMPLE:
  // Given a "t", find the knots that define the span used to evaluate a
  // nurb at t; i.e., find "i" so that
  // knot[i] <= ... <= knot[i+order-2]
  //   <= t < knot[i+order-1] <= ... <= knot[i+2*(order-1)-1]
  i = ON_GetdblArrayIndex(knot+order-2,cv_count-order+2,t);
  if (i < 0) i = 0; else if (i > cv_count - order) i = cv_count - order;
RELATED FUNCTIONS:
  ON_
  ON_
*****************************************************************************/

{
  int
    i, i0, i1;

  length--;

  /* Since t is frequently near the ends and bisection takes the
   * longest near the ends, trap those cases here.
   */
  if (t < array[0])
    return -1;
  if (t >= array[length])
    return (t > array[length]) ? length+1 : length;
  if (t < array[1])
    return 0;
  if (t >= array[length-1])
    return (length-1);


  i0 = 0;
  i1 = length;
  while (array[i0] == array[i0+1]) i0++;
  while (array[i1] == array[i1-1]) i1--;
  /* From now on we have
   *  1.) array[i0] <= t < array[i1]
   *  2.) i0 <= i < i1.
   * When i0+1 == i1, we have array[i0] <= t < array[i0+1]
   * and i0 is the answer we seek.
   */
  while (i0+1 < i1) {
    i = (i0+i1)>>1;
    if (t < array[i]) {
      i1 = i;
      while (array[i1] == array[i1-1]) i1--;
    }
    else {
      i0 = i;
      while (array[i0] == array[i0+1]) i0++;
    }
  }
  return i0;
}

/**
 * n - number of control points - 1
 * p - order - 1 (degree)
 * u - parametric point
 * U - knot sequence
 **/
int nurbs_findspan (int n, int p, double u, double *U)
{
    int span;
    span = ON_SearchMonotoneArray(U, (n+p), u);
    return (span);
}

// Basis Function.
//
// INPUT:
//
//   i - knot span  ( from FindSpan() )
//   u - parametric point
//   p - spline degree
//   U - knot sequence
//
// OUTPUT:
//
//   N - Basis functions vector[p+1]  sizeof(double)*(p+1)
//
// Algorithm A2.2 from 'The NURBS BOOK' pg70.
void nurbs_basisfun(int i, double u, int p,
              double *U,
              double *N)
{
  int j,r, id;
  double saved, temp, denom;

  double *left = (double*)malloc(sizeof(double)*(p+1));
  double *right = (double*)malloc(sizeof(double)*(p+1));

  N[0] = 1.0;
  for (j = 1; j <= p; j++)
    {
      id = i+1-j;
      if (id < 0) id = 0;
      left[j]  = u - U[id];
      right[j] = U[i+j] - u;
      saved = 0.0;

      for (r = 0; r < j; r++)
        {
          denom = (right[r+1] + left[j-r]);
          if (denom != 0)
          {
              temp = N[r] / denom;
              N[r] = saved + right[r+1] * temp;
              saved = left[j-r] * temp;
          }
        }

      N[j] = saved;

    }
  free(left);
  free(right);

}

#if 0
// was used for blending; need to review for S-curve velocity blending
PmCartesian tcGetStartingUnitVector(TC_STRUCT *tc) {
    PmCartesian v;

    if(tc->motion_type == TC_LINEAR || tc->motion_type == TC_SPINDLE_SYNC_MOTION) {
        pmCartCartSub(tc->coords.line.xyz.end.tran, tc->coords.line.xyz.start.tran, &v);
    } else {
        PmPose startpoint;
        PmCartesian radius;
        PmCartesian tan, perp;

        pmCirclePoint(&tc->coords.circle.xyz, 0.0, &startpoint);
        pmCartCartSub(startpoint.tran, tc->coords.circle.xyz.center, &radius);
        pmCartCartCross(tc->coords.circle.xyz.normal, radius, &tan);
        pmCartUnit(tan, &tan);

        pmCartCartSub(tc->coords.circle.xyz.center, startpoint.tran, &perp);
        pmCartUnit(perp, &perp);

        pmCartScalMult(tan, tc->maxaccel, &tan);
        pmCartScalMult(perp, pmSq(0.5 * tc->reqvel)/tc->coords.circle.xyz.radius, &perp);
        pmCartCartAdd(tan, perp, &v);
    }
    pmCartUnit(v, &v);
    return v;
}

// was used for blending; need to review for S-curve velocity blending
PmCartesian tcGetEndingUnitVector(TC_STRUCT *tc) {
    PmCartesian v;

    if(tc->motion_type == TC_LINEAR || tc->motion_type == TC_SPINDLE_SYNC_MOTION) {
        pmCartCartSub(tc->coords.line.xyz.end.tran, tc->coords.line.xyz.start.tran, &v);
    } else {
        PmPose endpoint;
        PmCartesian radius;

        pmCirclePoint(&tc->coords.circle.xyz, tc->coords.circle.xyz.angle, &endpoint);
        pmCartCartSub(endpoint.tran, tc->coords.circle.xyz.center, &radius);
        pmCartCartCross(tc->coords.circle.xyz.normal, radius, &v);
    }
    pmCartUnit(v, &v);
    return v;
}
#endif

/*! tcGetPos() function
 *
 * \brief This function calculates the machine position along the motion's path.
 *
 * As we move along a TC, from zero to its length, we call this function repeatedly,
 * with an increasing tc->progress.
 * This function calculates the machine position along the motion's path 
 * corresponding to the current progress.
 * It gets called at the end of tpRunCycle()
 * 
 * @param    tc    the current TC that is being planned
 *
 * @return	 EmcPose   returns a position (\ref EmcPose = datatype carrying XYZABC information
 */   

EmcPose tcGetPos(TC_STRUCT * tc) {
    return tcGetPosReal(tc, 0);
}

EmcPose tcGetEndpoint(TC_STRUCT * tc) {
    return tcGetPosReal(tc, 1);
}


EmcPose tcGetPosReal(TC_STRUCT * tc, int of_endpoint)
{
    EmcPose pos;
    PmPose xyz;
    PmPose abc;
    PmPose uvw;
    double s;
    
    double progress = of_endpoint? tc->target: tc->progress;
#if(TRACE != 0)
    static double last_l, last_u,last_x = 0 , last_y = 0, last_z = 0, last_a = 0;
#endif

    // update spindle position
    s = emcmotStatus->carte_pos_cmd.s;
    if (tc->motion_type == TC_SPINDLE_SYNC_MOTION) {
        // for RIGID_TAPPING(G33.1), CSS(G33 w/ G96), and THREADING(G33 w/ G97)
        pmLinePoint(&tc->coords.spindle_sync.xyz, tc->coords.spindle_sync.xyz.tmag * (progress / tc->target) , &xyz);
        // no rotary move allowed while tapping
        abc.tran = tc->coords.spindle_sync.abc;
        uvw.tran = tc->coords.spindle_sync.uvw;
        if (!of_endpoint)
        {
            s = tc->coords.spindle_sync.spindle_start_pos + tc->coords.spindle_sync.spindle_dir * progress;
        }
    } else if (tc->motion_type == TC_LINEAR) {

        if (tc->coords.line.xyz.tmag > 0.) {
            // progress is along xyz, so uvw and abc move proportionally in order
            // to end at the same time.
            pmLinePoint(&tc->coords.line.xyz, progress, &xyz);
            pmLinePoint(&tc->coords.line.uvw,
                        progress * tc->coords.line.uvw.tmag / tc->target,
                        &uvw);
            pmLinePoint(&tc->coords.line.abc,
                        progress * tc->coords.line.abc.tmag / tc->target,
                        &abc);
        } else if (tc->coords.line.uvw.tmag > 0.) {
            // xyz is not moving
            pmLinePoint(&tc->coords.line.xyz, 0.0, &xyz);
            pmLinePoint(&tc->coords.line.uvw, progress, &uvw);
            // abc moves proportionally in order to end at the same time
            pmLinePoint(&tc->coords.line.abc,
                        progress * tc->coords.line.abc.tmag / tc->target,
                        &abc);
        } else {
            // if all else fails, it's along abc only
            pmLinePoint(&tc->coords.line.xyz, 0.0, &xyz);
            pmLinePoint(&tc->coords.line.uvw, 0.0, &uvw);
            pmLinePoint(&tc->coords.line.abc, progress, &abc);
        }
    } else if (tc->motion_type == TC_CIRCULAR) {//we have TC_CIRCULAR
        // progress is always along the xyz circle.  This simplification
        // is possible since zero-radius arcs are not allowed by the interp.

        pmCirclePoint(&tc->coords.circle.xyz,
                      progress * tc->coords.circle.xyz.angle / tc->target,
                      &xyz);
        // abc moves proportionally in order to end at the same time as the
        // circular xyz move.
        pmLinePoint(&tc->coords.circle.abc,
                    progress * tc->coords.circle.abc.tmag / tc->target,
                    &abc);
        // same for uvw
        pmLinePoint(&tc->coords.circle.uvw,
                    progress * tc->coords.circle.uvw.tmag / tc->target,
                    &uvw);

    } else {
        int          s, tmp1, i, id;
        double       u,*N,R, X, Y, Z, A, B, C, U, V, W, D;
        double       curve_accel;
#if(TRACE != 0)
        double delta_l, delta_u, delta_d, delta_x, delta_y, delta_z, delta_a;
#endif
        N = tc->nurbs_block.N;
//        NL = tc->nurbs_block.NL;
        assert(tc->motion_type == TC_NURBS);

        u = progress / tc->target;
        if (u < 1.0) {

            s = nurbs_findspan(tc->nurbs_block.nr_of_ctrl_pts-1,  tc->nurbs_block.order - 1,
                                u, tc->nurbs_block.knots_ptr);  //return span index of u_i
            nurbs_basisfun(s, u, tc->nurbs_block.order - 1 , tc->nurbs_block.knots_ptr , N);    // input: s:knot span index u:u_0 d:B-Spline degree  k:Knots
                           // output: N:basis functions
            // refer to bspeval.cc::line(70) of octave
            // refer to opennurbs_evaluate_nurbs.cpp::line(985) of openNurbs
            // refer to ON_NurbsCurve::Evaluate() for ...
            // refer to opennurbs_knot.cpp::ON_NurbsSpanIndex()
            // http://www.rhino3d.com/nurbs.htm (What is NURBS?)
            //    Some modelers that use older algorithms for NURBS
            //    evaluation require two extra knot values for a total of
            //    degree+N+1 knots. When Rhino is exporting and importing
            //    NURBS geometry, it automatically adds and removes these
            //    two superfluous knots as the situation requires.
            tmp1 = s - tc->nurbs_block.order + 1;

            R = 0.0;
            X = 0.0;
            Y = 0.0;
            Z = 0.0;
            A = 0.0;
            B = 0.0;
            C = 0.0;
            U = 0.0;
            V = 0.0;
            W = 0.0;
            D = 0.0;
            for (i = 0; i < tc->nurbs_block.order; i++) {
                id = tmp1 + i;
                if (id < 0) id = 0;
                else if (id >= tc->nurbs_block.nr_of_ctrl_pts) id = tc->nurbs_block.nr_of_ctrl_pts - 1;
                R += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].R;
                X += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].X;
                Y += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].Y;
                Z += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].Z;
                A += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].A;
                B += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].B;
                C += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].C;
                U += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].U;
                V += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].V;
                W += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].W;
                D += N[i]*tc->nurbs_block.ctrl_pts_ptr[id].D;
            }

            X = X/R;
            xyz.tran.x = X;
            Y = Y/R;
            xyz.tran.y = Y;
            Z = Z/R;
            xyz.tran.z = Z;
            A = A/R;
            abc.tran.x = A;
            B = B/R;
            abc.tran.y = B;
            C = C/R;
            abc.tran.z = C;
            U = U/R;
            uvw.tran.x = U;
            V = V/R;
            uvw.tran.y = V;
            W = W/R;
            uvw.tran.z = W;

            tc->reqvel = tc->nurbs_block.reqvel; // restore reqvel of this curve
            D = D/R;
            // compute allowed feed
            if(!of_endpoint) {
                curve_accel = (tc->cur_vel * tc->cur_vel)/D;

                if(curve_accel > tc->maxaccel) {
                    // modify req_vel
                    // tc->reqvel: unit/s
                    // tc->maxaccel: unit/s * cycle_time * cycle_time
                    DPS ("tc.c: reqvel(%f) cur_vel(%f) curve_accel(%f) maxaccel(%f)\n",
                            tc->reqvel, tc->cur_vel, curve_accel, tc->maxaccel);
                    tc->reqvel = pmSqrt((tc->maxaccel * D)) / tc->cycle_time;
                    DPS ("curvature limited reqvel(%f)\n", tc->reqvel);
                }
            }
        }else {
            xyz.tran.x = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].X;
            xyz.tran.y = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].Y;
            xyz.tran.z = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].Z;
            uvw.tran.x = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].U;
            uvw.tran.y = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].V;
            uvw.tran.z = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].W;
            abc.tran.x = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].A;
            abc.tran.y = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].B;
            abc.tran.z = tc->nurbs_block.ctrl_pts_ptr[tc->nurbs_block.nr_of_ctrl_pts-1].C;
        }
    }
    //DP ("GetEndPoint?(%d) R(%.2f) X(%.2f) Y(%.2f) Z(%.2f) A(%.2f)\n",of_endpoint, R, X, Y, Z, A);
#if (TRACE != 1)
    if( of_endpoint != 1) {

    }
#endif
    pos.tran = xyz.tran;
    pos.a = abc.tran.x;
    pos.b = abc.tran.y;
    pos.c = abc.tran.z;
    pos.u = uvw.tran.x;
    pos.v = uvw.tran.y;
    pos.w = uvw.tran.z;
    pos.s = s;
    DP ("of_endpoint(%d) tc->id(%d) MotionType(%d) X(%.2f) Y(%.2f) Z(%.2f) W(%.2f)\n",
    		of_endpoint, tc->id, tc->motion_type, pos.tran.x,
    		pos.tran.y, pos.tran.z, pos.w);
    return pos;
}


/*!
 * \subsection TC queue functions
 * These following functions implement the motion queue that
 * is fed by tpAddLine/tpAddCircle and consumed by tpRunCycle.
 * They have been fully working for a long time and a wise programmer
 * won't mess with them.
 */


/*! tcqCreate() function
 *
 * \brief Creates a new queue for TC elements.
 *
 * This function creates a new queue for TC elements. 
 * It gets called by tpCreate()
 * 
 * @param    tcq       pointer to the new TC_QUEUE_STRUCT
 * @param	 _size	   size of the new queue
 * @param	 tcSpace   holds the space allocated for the new queue, allocated in motion.c
 *
 * @return	 int	   returns success or failure
 */   
int tcqCreate(TC_QUEUE_STRUCT * tcq, int _size, TC_STRUCT * tcSpace)
{
    if (_size <= 0 || 0 == tcq) {
	return -1;
    } else {
	tcq->queue = tcSpace;
	tcq->size = _size;
	tcq->_len = 0;
	tcq->start = tcq->end = 0;
	tcq->allFull = 0;

	if (0 == tcq->queue) {
	    return -1;
	}
	return 0;
    }
}

/*! tcqDelete() function
 *
 * \brief Deletes a queue holding TC elements.
 *
 * This function creates deletes a queue. It doesn't free the space
 * only throws the pointer away. 
 * It gets called by tpDelete() 
 * \todo FIXME, it seems tpDelete() is gone, and this function isn't used.
 * 
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns success
 */   
int tcqDelete(TC_QUEUE_STRUCT * tcq)
{
    if (0 != tcq && 0 != tcq->queue) {
	/* free(tcq->queue); */
	tcq->queue = 0;
    }

    return 0;
}

/*! tcqInit() function
 *
 * \brief Initializes a queue with TC elements.
 *
 * This function initializes a queue with TC elements. 
 * It gets called by tpClear() and  
 * 	  	   		  by tpRunCycle() when we are aborting
 * 
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns success or failure (if no tcq found)
 */
int tcqInit(TC_QUEUE_STRUCT * tcq)
{
#if (TRACE != 0)
    if(!dptrace){
        dptrace = fopen("tc.log","w");
        fprintf(stderr,"tc.c dptrace not NULL \n");
    }
#endif

    if (0 == tcq) {
	return -1;
    }

    tcq->_len = 0;
    tcq->start = tcq->end = 0;
    tcq->allFull = 0;


    return 0;
}

/*! tcqPut() function
 *
 * \brief puts a TC element at the end of the queue
 *
 * This function adds a tc element at the end of the queue. 
 * It gets called by tpAddLine() and tpAddCircle()
 * 
 * @param    tcq       pointer to the new TC_QUEUE_STRUCT
 * @param	 tc        the new TC element to be added
 *
 * @return	 int	   returns success or failure
 */   
int tcqPut(TC_QUEUE_STRUCT * tcq, TC_STRUCT tc)
{
    /* check for initialized */
    if (0 == tcq || 0 == tcq->queue) {
	    return -1;
    }

    /* check for allFull, so we don't overflow the queue */
    if (tcq->allFull) {
	    return -1;
    }

    /* add it */
    tcq->queue[tcq->end] = tc;
    tcq->_len++;

    /* update end ptr, modulo size of queue */
    tcq->end = (tcq->end + 1) % tcq->size;

    /* set allFull flag if we're really full */
    if (tcq->end == tcq->start) {
	tcq->allFull = 1;
    }

    return 0;
}

/*! tcqRemove() function
 *
 * \brief removes n items from the queue
 *
 * This function removes the first n items from the queue,
 * after checking that they can be removed 
 * (queue initialized, queue not empty, enough elements in it) 
 * Function gets called by tpRunCycle() with n=1
 * \todo FIXME: Optimize the code to remove only 1 element, might speed it up
 * 
 * @param    tcq       pointer to the new TC_QUEUE_STRUCT
 * @param	 n         the number of TC elements to be removed
 *
 * @return	 int	   returns success or failure
 */   
int tcqRemove(TC_QUEUE_STRUCT * tcq, int n)
{
    int i;
    if (n <= 0) {
	    return 0;		/* okay to remove 0 or fewer */
    }

    if ((0 == tcq) || (0 == tcq->queue) ||	/* not initialized */
	((tcq->start == tcq->end) && !tcq->allFull) ||	/* empty queue */
	(n > tcq->_len)) {	/* too many requested */
	    return -1;
    }
    /* if NURBS ?*/
    for(i=tcq->start;i<(tcq->start+n);i++){

        if(tcq->queue[i].motion_type == TC_NURBS) {
            //fprintf(stderr,"Remove TCNURBS PARAM\n");
            free(tcq->queue[i].nurbs_block.knots_ptr);
            free(tcq->queue[i].nurbs_block.ctrl_pts_ptr);
            free(tcq->queue[i].nurbs_block.N);

        }
    }
    /* update start ptr and reset allFull flag and len */
    tcq->start = (tcq->start + n) % tcq->size;
    tcq->allFull = 0;
    tcq->_len -= n;

    return 0;
}

/*! tcqLen() function
 *
 * \brief returns the number of elements in the queue
 *
 * Function gets called by tpSetVScale(), tpAddLine(), tpAddCircle()
 * 
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns number of elements
 */   
int tcqLen(TC_QUEUE_STRUCT * tcq)
{
    if (0 == tcq) {
	    return -1;
    }

    return tcq->_len;
}

/*! tcqItem() function
 *
 * \brief gets the n-th TC element in the queue, without removing it
 *
 * Function gets called by tpSetVScale(), tpRunCycle(), tpIsPaused()
 * 
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 TC_STRUCT returns the TC elements
 */   
TC_STRUCT *tcqItem(TC_QUEUE_STRUCT * tcq, int n, long period)
{
    TC_STRUCT *t;
    if ((0 == tcq) || (0 == tcq->queue) ||	/* not initialized */
	(n < 0) || (n >= tcq->_len)) {	/* n too large */
	return (TC_STRUCT *) 0;
    }
    t = &(tcq->queue[(tcq->start + n) % tcq->size]);
    return t;
}

/*! 
 * \def TC_QUEUE_MARGIN
 * sets up a margin at the end of the queue, to reduce effects of race conditions
 */
#define TC_QUEUE_MARGIN 10

/*! tcqFull() function
 *
 * \brief get the full status of the queue 
 * Function returns full if the count is closer to the end of the queue than TC_QUEUE_MARGIN
 *
 * Function called by update_status() in control.c 
 * 
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int       returns status (0==not full, 1==full)
 */   
int tcqFull(TC_QUEUE_STRUCT * tcq)
{
    if (0 == tcq) {
	   return 1;		/* null queue is full, for safety */
    }

    /* call the queue full if the length is into the margin, so reduce the
       effect of a race condition where the appending process may not see the 
       full status immediately and send another motion */

    if (tcq->size <= TC_QUEUE_MARGIN) {
	/* no margin available, so full means really all full */
	    return tcq->allFull;
    }

    if (tcq->_len >= tcq->size - TC_QUEUE_MARGIN) {
	/* we're into the margin, so call it full */
	    return 1;
    }

    /* we're not into the margin */
    return 0;
}

