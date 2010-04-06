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
#include "canon.hh"

std::vector<unsigned int> knot_vector_creator(unsigned int n, unsigned int k) 
{
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

double alpha_finder(double dx, double dy) {
    
    if ((dx > 0 && dy > 0) || (dx > 0 && dy < 0)) {  /* First quadrant, Fourth quadrant */
        return atan(dy/dx) + 2*M_PI;
    } else {                                         /* Second quadrant, Third quadrant */
        return atan(dy/dx) + M_PI; 
    }
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
//   n - number of control points - 1
//   p - spline degree
//   u - parametric point
//   U - knot sequence
int nurbs_findspan (int n, int p, double u, double *U)
{
  // FIXME : this implementation has linear, rather than log complexity
  int ret = 0;
  while ((ret++ < n) && (U[ret] <= u)) {
  };
  return (ret-1);
}

/*
void nurbs_basisfun(int i, double u, int p, 
              const std::vector<double> & U, 
              std::vector<double> & N)
*/
void nurbs_basisfun(int i, double u, int p,
              double *U,
              double *N)
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
  int j,r;
  double saved, temp;

  // work space
  //std::vector<double> left(p+1);
  //std::vector<double> right(p+1);

  double *left = (double*)malloc(sizeof(double)*(p+1));
  double *right = (double*)malloc(sizeof(double)*(p+1));

  

  N[0] = 1.0;
  for (j = 1; j <= p; j++)
    {
      left[j]  = u - U[i+1-j];
      right[j] = U[i+j] - u;
      saved = 0.0;
      
      for (r = 0; r < j; r++)
	{
	  temp = N[r] / (right[r+1] + left[j-r]);
	  N[r] = saved + right[r+1] * temp;
	  saved = left[j-r] * temp;
	} 
      
      N[j] = saved;
    }
  free(left);
  free(right);

}

// vim:sw=4:sts=4:et:
