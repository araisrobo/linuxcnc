/*
 * nurbs.h
 *
 *  Created on: 2010/3/25
 *      Author: iautsi
 */
#ifndef NURBS_H_
#define NURBS_H_

#include <stdint.h>

typedef struct {          /* type for NURBS control points */
      double X,
             Y,
             Z,
             A,
             B,
             C,
             U,
             V,
             W,
             R;
      } CONTROL_POINT;

typedef struct {
      double X,
             Y;
      } PLANE_POINT;

typedef struct nurbs_block
{
    CONTROL_POINT               *ctrl_pts_ptr;
    uint32_t                    nr_of_ctrl_pts;
    double                      *knots_ptr;
    uint32_t                    nr_of_knots;
    uint32_t                    order;
    double                      curve_len;
    double                      knot;
    double                      weight;
    double                      *N; // basis function buffer
} nurbs_block_t;

extern int nurbs_findspan(int n, int p, double u, double *U);
extern void nurbs_basisfun(int i, double u, int p, double *U, double *N);
#endif /* NURBS_H_ */
