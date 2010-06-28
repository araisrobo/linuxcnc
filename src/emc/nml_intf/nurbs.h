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
             R,
             F;         // add for dynamic feed rate
      } CONTROL_POINT;

typedef struct {
      double X,
             Y;
      } PLANE_POINT;
typedef struct  {
    double              uofl_knot;
    uint32_t            uofl_knot_flag;
    double              uofl_weight;
    uint32_t            uofl_weight_flag;
    double              uofl_ctrl_pt;
    uint32_t            uofl_ctrl_pt_flag;
} uofl_block_t;
typedef struct
{
    // NURBS curve paramters
    CONTROL_POINT               *ctrl_pts_ptr;
    uint32_t                    nr_of_ctrl_pts;
    double                      *knots_ptr;
    uint32_t                    nr_of_knots;
    uint32_t                    order;
    double                      curve_len;
    double                      knot;
    double                      weight;
    // U(L) nurbs parameters
    double                      *uofl_weights;
    double                      *uofl_ctrl_pts_ptr;
    uint32_t                    nr_of_uofl_ctrl_pts;
    double                      *uofl_knots_ptr;
    uint32_t                    nr_of_uofl_knots;
    uint32_t                    uofl_order;
    double                      uofl_cp;
    double                      uofl_knot;
    double                      uofl_weight;

    double                      *N; // basis function buffer
    double                      *NL; // basis function buffer for U(L)
    int 		        axis_mask;
} nurbs_block_t;

extern int nurbs_findspan(int n, int p, double u, double *U);
extern void nurbs_basisfun(int i, double u, int p, double *U, double *N);

enum {
    AXIS_MASK_X =   1, AXIS_MASK_Y =   2, AXIS_MASK_Z =   4,
    AXIS_MASK_A =   8, AXIS_MASK_B =  16, AXIS_MASK_C =  32,
    AXIS_MASK_U =  64, AXIS_MASK_V = 128, AXIS_MASK_W = 256,
};
#endif /* NURBS_H_ */
