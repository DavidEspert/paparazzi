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

/** \file gvf.h
 *
 *  Guidance algorithm based on vector fields
 */

#ifndef GVF_H
#define GVF_H

#define GVF_GRAVITY 9.806

#include "std.h"

// Control
extern float gvf_error;
extern float gvf_ke;
extern float gvf_kn;
extern int8_t gvf_s;

// Trajectories
// 0 - Straight line
// 1 - Ellipse
// 2 - Sin

extern uint8_t gvf_traj_type;
extern struct gvf_p{
    float p1;
    float p2;
    float p3;
    float p4;
    float p5;
    float p6;
    float p7;
} gvf_param;

struct gvf_grad{
    float nx;
    float ny;
    float nz;
};

struct gvf_Hess{
    float H11;
    float H12;
    float H13;
    float H21;
    float H22;
    float H23;
    float H31;
    float H32;
    float H33;
};

extern void gvf_init(void);
void gvf_control_2D(float ke, float kn, float e,
        struct gvf_grad *, struct gvf_Hess *);
extern void gvf_set_gains(float ke, float kd);

// Straigh line
void gvf_line(float x, float y, float alpha);
extern bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_line_wp_heading(uint8_t wp, float alpha);

// Ellipse
extern bool gvf_ellipse(uint8_t wp, float a, float b, float alpha);
extern bool gvf_ellipse_set(uint8_t wp);

// Sinusoidal
void gvf_sin(float x, float y, float alpha, float w, float off, float A);
extern bool gvf_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off,
        float A);
extern bool gvf_sin_wp_heading(uint8_t wp, float alpha, float w, float off,
        float A);


#endif // GVF_H
