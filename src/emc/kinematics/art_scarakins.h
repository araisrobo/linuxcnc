/*****************************************************************
* Description: art_scarakins.h
*   Kinematics for a SCARA typed robot
*
*   Derived from a work by Fred Proctor
*
* Author: 
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
*******************************************************************
* This is the header file to accompany art_scarakins.c
*******************************************************************
*/
#ifndef SCARA_H
#define SCARA_H

#define SINGULAR_FUZZ 0.000001

/* flags for inverse and forward kinematics */
#define SCARA_SINGULAR      0x01  /* joints at a singularity */
#define SCARA_LEFTY         0x02  /* RIGHTY(0) or LEFTY(1) */

#endif /* SCARA_H */

