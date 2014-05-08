/********************************************************************
 *
 * Author: Manfredi Leto (Xemet)
 * License: GPL Version 2
 * System: Linux
 *    
 * Copyright (c) 2009 All rights reserved.
 *
 ********************************************************************/

/* Those functions are needed to calculate NURBS points */
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include "canon.hh"

static void unit(PLANE_POINT &p) {
    double h = hypot(p.X, p.Y);
    if(h != 0) { p.X/=h; p.Y/=h; }
}

std::vector<unsigned int> knot_vector_creator(unsigned int n, unsigned int k) {
    
    unsigned int i;
    std::vector<unsigned int> knot_vector;
    for (i=0; i<=n+k; i++) {
	if (i < k)  
            knot_vector.push_back(0);
        else if (i >= k && i <= n)
            knot_vector.push_back(i - k + 1);
        else
            knot_vector.push_back(n - k + 2);
    }
    return knot_vector;
 
}

double Nmix(unsigned int i, unsigned int k, double u, 
                    std::vector<unsigned int> knot_vector) {

    if (k == 1){
        if ((u >= knot_vector[i]) && (u <= knot_vector[i+1])) {
            return 1;
        } else {
            return 0;}
    } else if (k > 1) {
        if ((knot_vector[i+k-1]-knot_vector[i] == 0) && 
            (knot_vector[i+k]-knot_vector[i+1] != 0)) {
            return ((knot_vector[i+k] - u)*Nmix(i+1,k-1,u,knot_vector))/
                    (knot_vector[i+k]-knot_vector[i+1]);
        } else if ((knot_vector[i+k]-knot_vector[i+1] == 0) && 
            (knot_vector[i+k-1]-knot_vector[i] != 0)) {
            return ((u - knot_vector[i])*Nmix(i,k-1,u,knot_vector))/
                    (knot_vector[i+k-1]-knot_vector[i]);
        } else if ((knot_vector[i+k-1]-knot_vector[i] == 0) && 
            (knot_vector[i+k]-knot_vector[i+1] == 0)) {
            return 0;
        } else {
            return ((u - knot_vector[i])*Nmix(i,k-1,u,knot_vector))/
                    (knot_vector[i+k-1]-knot_vector[i]) + ((knot_vector[i+k] - u)*
                    Nmix(i+1,k-1,u,knot_vector))/(knot_vector[i+k]-knot_vector[i+1]);
        }
    }
    else return -1;
}



double Rden(double u, unsigned int k,
                  std::vector<CONTROL_POINT> nurbs_control_points,
                  std::vector<unsigned int> knot_vector) {

    unsigned int i;
    double d = 0.0;   
    for (i=0; i<(nurbs_control_points.size()); i++)
        d = d + Nmix(i,k,u,knot_vector)*nurbs_control_points[i].W;
    return d;
}

PLANE_POINT nurbs_point(double u, unsigned int k, 
                  std::vector<CONTROL_POINT> nurbs_control_points,
                  std::vector<unsigned int> knot_vector) {

    unsigned int i;
    PLANE_POINT point;
    point.X = 0;
    point.Y = 0;
    for (i=0; i<(nurbs_control_points.size()); i++) {
        point.X = point.X + nurbs_control_points[i].X*Nmix(i,k,u,knot_vector)
	*nurbs_control_points[i].W/Rden(u,k,nurbs_control_points,knot_vector);
        point.Y = point.Y + nurbs_control_points[i].Y*Nmix(i,k,u,knot_vector)
	*nurbs_control_points[i].W/Rden(u,k,nurbs_control_points,knot_vector);
    }
    return point;
}

#define DU (1e-5)
PLANE_POINT nurbs_tangent(double u, unsigned int k,
                  std::vector<CONTROL_POINT> nurbs_control_points,
                  std::vector<unsigned int> knot_vector) {
    unsigned int n = nurbs_control_points.size() - 1;
    double umax = n - k + 2;
    double ulo = std::max(0.0, u-DU), uhi = std::min(umax, u+DU);
    PLANE_POINT P1 = nurbs_point(ulo, k, nurbs_control_points, knot_vector);
    PLANE_POINT P3 = nurbs_point(uhi, k, nurbs_control_points, knot_vector);
    PLANE_POINT r = {(P3.X - P1.X) / (uhi-ulo), (P3.Y - P1.Y) / (uhi-ulo)};
    unit(r);
    return r;
}

//int nurbs_findspan (int n, int p, double u, const std::vector<double> & U)
// Find the knot span of the parametric point u. 
//
// INPUT:
//
//   n - number of control points - 1
//   p - spline degree       
//   u - parametric point    
//   U - knot sequence
//
// RETURN:
//
//   s - knot span
//
// Note: This is NOT
// Algorithm A2.1 from 'The NURBS BOOK' pg68
// as that algorithm only works for nonperiodic
// knot vectors, nonetheless the results should 
// be EXACTLY the same if U is nonperiodic

/*
Below is the original implementation from the NURBS Book
{
  int low, high, mid;
  // special case
  if (u == U(n+1)) return(n);

  // do binary search
  low = p;
  high = n + 1;
  mid = (low + high) / 2;
  while (u < U(mid) || u >= U(mid+1))
    {

      if (u < U(mid))
	high = mid;
      else
	low = mid;
      mid = (low + high) / 2;
    }  

  return(mid);
}
*/

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

//   n - number of control points - 1
//   p - spline degree
//   u - parametric point
//   U - knot sequence
int nurbs_findspan (int n, int p, double u, double *U)
{
    int span;
    span = ON_SearchMonotoneArray(U, (n+p), u);
    return (span);
}

/*
void nurbs_basisfun(int i, double u, int p, 
              const std::vector<double> & U, 
              std::vector<double> & N)
*/
void nurbs_basisfun(int i, double u, int p, double *U, double *N)
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
{
    int j, r, id;
    double saved, temp, denom;

    // work space
    //std::vector<double> left(p+1);
    //std::vector<double> right(p+1);

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

// vim:sw=4:sts=4:et:
