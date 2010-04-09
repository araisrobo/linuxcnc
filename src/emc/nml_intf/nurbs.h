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
    int 						axis_mask;
} nurbs_block_t;

extern int nurbs_findspan(int n, int p, double u, double *U);
extern void nurbs_basisfun(int i, double u, int p, double *U, double *N);

enum {
    AXIS_MASK_X =   1, AXIS_MASK_Y =   2, AXIS_MASK_Z =   4,
    AXIS_MASK_A =   8, AXIS_MASK_B =  16, AXIS_MASK_C =  32,
    AXIS_MASK_U =  64, AXIS_MASK_V = 128, AXIS_MASK_W = 256,
};
#endif /* NURBS_H_ */
