/*
 * Copyright (C) 2016  Hector Garcia de Marina
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file gvf_sin.c
 *
 *  Guidance algorithm based on vector fields
 *  2D sinusoidal trajectory
 */


#include "subsystems/navigation/common_nav.h"
#include "gvf_sin.h"

void gvf_sin_info(float *phi, struct gvf_grad *grad,
        struct gvf_Hess *hess){

    struct EnuCoor_f *p = stateGetPositionEnu_f();
    float px = p->x;
    float py = p->y;
    float a = gvf_param.p1;
    float b = gvf_param.p2;
    float alpha = gvf_param.p3;
    float w = gvf_param.p4;
    float off = gvf_param.p5;
    float A = gvf_param.p6;

    float Xel = -(px-a)*sinf(alpha) - (py-b)*cosf(alpha);
    float Yel = -(px-a)*cosf(alpha) + (py-b)*sinf(alpha);

    // TODO Make it always in (-pi, pi] in an efficient way
    float ang = (w*Xel + off);

    // Phi(x,y)
    *phi = Yel - A*sinf(ang);

    // grad Phi
    grad->nx = -w*A*sinf(alpha)*cosf(ang) - cosf(alpha);
    grad->ny = -w*A*cosf(alpha)*cosf(ang) + sinf(alpha);

    // Hessian Phi
    hess->H11 = (A*w*w*sinf(alpha)*sinf(alpha))*sinf(ang);
    hess->H12 = (A*w*w*sinf(alpha)*cosf(alpha))*sinf(ang);
    hess->H21 = (A*w*w*sinf(alpha)*cosf(alpha))*sinf(ang);
    hess->H22 = (A*w*w*cosf(alpha)*cosf(alpha))*sinf(ang);
}
