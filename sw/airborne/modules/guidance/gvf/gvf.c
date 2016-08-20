/*
 * Copyright (C) 2016 Hector Garcia de Marina
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

#include <math.h>
#include "std.h"

#include "gvf.h"

#include "./trajectories/gvf_ellipse.h"
#include "subsystems/navigation/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/autopilot.h"

// Control
float gvf_error;
float gvf_ke;
float gvf_kd;
float gvf_kn;
int8_t gvf_s;

// Trajectory
uint8_t gvf_traj_type;
struct gvf_p gvf_param;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_GVF(trans, dev, AC_ID, &gvf_error, &gvf_traj_type,
            &gvf_s, &gvf_param.p1, &gvf_param.p2, &gvf_param.p3,
            &gvf_param.p4, &gvf_param.p5, &gvf_param.p6, &gvf_param.p7);
}

#endif

void gvf_init(void)
{
    gvf_ke = 1;
    gvf_kn = 1;
    gvf_kd = 1;
    gvf_s = 1;
    gvf_traj_type = 0;
    gvf_param.p1 = 0;
    gvf_param.p2 = 0;
    gvf_param.p3 = 0;
    gvf_param.p4 = 0;
    gvf_param.p5 = 0;
    gvf_param.p6 = 0;
    gvf_param.p7 = 0;

#if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_control_2D(float ke, float kn, float kd,
        float e, struct gvf_grad *grad, struct gvf_Hess *hess)
{
    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float ground_speed = stateGetHorizontalSpeedNorm_f();
    float course = stateGetHorizontalSpeedDir_f();
    float px_dot = ground_speed*sinf(course);
    float py_dot = ground_speed*cosf(course);
    int s = gvf_s;
    
    // gradient Phi
    float nx = grad->nx;
    float ny = grad->ny;

    // tangent to Phi
    float tx = s*grad->ny;
    float ty = -s*grad->nx;

    // Hessian
    float H11 = hess->H11;
    float H12 = hess->H12;
    float H21 = hess->H21;
    float H22 = hess->H22;

    // Calculation of the desired angular velocity in the vector field
    float pdx_dot = tx - ke*e*nx;
    float pdy_dot = ty - ke*e*ny;

    float norm_pd_dot = sqrtf(pdx_dot*pdx_dot + pdy_dot*pdy_dot);
    float md_x = pdx_dot / norm_pd_dot;
    float md_y = pdy_dot / norm_pd_dot;

    float Apd_dot_dot_x = -ke*e*(nx*px_dot + ny*py_dot)*nx;
    float Apd_dot_dot_y = -ke*e*(nx*px_dot + ny*py_dot)*ny;

    float Bpd_dot_dot_x = ((-ke*e*H11)+s*H21)*px_dot 
        + ((-ke*e*H12)+s*H22)*py_dot;
    float Bpd_dot_dot_y = -(s*H11+(ke*e*H21))*px_dot
        - (s*H12+(ke*e*H22))*py_dot;

    float pd_dot_dot_x = Apd_dot_dot_x + Bpd_dot_dot_x;
    float pd_dot_dot_y = Apd_dot_dot_y + Bpd_dot_dot_y;

    float md_dot_const = -(md_x*pd_dot_dot_y - md_y*pd_dot_dot_x)/norm_pd_dot;
    float md_dot_x = md_y * md_dot_const;
    float md_dot_y = -md_x * md_dot_const;

    float omega_d = -(md_dot_x*md_y - md_dot_y*md_x);

    float mr_x = sinf(course);
    float mr_y = cosf(course);

    float omega = omega_d + kn*(mr_x*md_y - mr_y*md_x);
    
    // Coordinated turn, it is minus since in NED the positive is clockwise
    h_ctl_roll_setpoint =
        -atanf(kd*omega*ground_speed/GVF_GRAVITY/cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

    lateral_mode = LATERAL_MODE_ROLL;
}

// STRAIGHT LINE

void gvf_line_p1_p2(float x1, float y1, float x2, float y2)
{
    float e;
    struct gvf_grad grad_line;
    struct gvf_Hess Hess_line;

    float a = x2-x1;
    float b = y2-y1;
    float c = -((x2-x1)*y1 + (y2-y1)*x1);

    gvf_traj_type = 0;
    gvf_param.p1 = a;
    gvf_param.p2 = b;
    gvf_param.p3 = c;

    gvf_line_info(&e, &grad_line, &Hess_line);
    gvf_control_2D(gvf_ke, gvf_kn, gvf_kd, e, &grad_line, &Hess_line);

    gvf_error = e;
}

bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
    float x1 = waypoints[wp1].x;
    float y1 = waypoints[wp1].y;
    float x2 = waypoints[wp2].x;
    float y2 = waypoints[wp2].y;

    gvf_line_p1_p2(x1, x2, y1, y2);

    return true;
}

bool gvf_line_wp_heading(uint8_t wp, float alpha)
{
    alpha = alpha*M_PI/180;

    float x1 = waypoints[wp].x;
    float y1 = waypoints[wp].y;
    float x2 = waypoints[wp].x -10*sinf(alpha);
    float y2 = waypoints[wp].y +10*cosf(alpha);

    gvf_line_p1_p2(x1, x2, y1, y2);

    return true;
}

bool gvf_segment_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
    return true;
}

bool gvf_segment_wp_heading(uint8_t wp, float alpha, float length)
{
    alpha = alpha*M_PI/180;

    float x1 = waypoints[wp].x;
    float y1 = waypoints[wp].y;
    float x2 = waypoints[wp].x -10*sinf(alpha);
    float y2 = waypoints[wp].y +10*cosf(alpha);

    return true;
}

// ELLIPSE
bool gvf_ellipse(uint8_t wp, float a, float b, float alpha)
{
    float e;
    struct gvf_grad grad_ellipse;
    struct gvf_Hess Hess_ellipse;

    alpha = alpha*M_PI/180;

    gvf_traj_type = 1;
    gvf_param.p1 = waypoints[wp].x;
    gvf_param.p2 = waypoints[wp].y;
    gvf_param.p3 = a;
    gvf_param.p4 = b;
    gvf_param.p5 = alpha;

    // SAFE MODE
    if(a == 0 || b == 0){
        gvf_param.p3 = 60;
        gvf_param.p4 = 60;
    }

    gvf_ellipse_info(&e, &grad_ellipse, &Hess_ellipse);
    gvf_control_2D(gvf_ke, gvf_kn, gvf_kd, e, &grad_ellipse, &Hess_ellipse);

    gvf_error = e;

    return true;
}

bool gvf_ellipse_set(uint8_t wp)
{
    float a = gvf_ellipse_a;
    float b = gvf_ellipse_b;
    float alpha = gvf_ellipse_alpha*M_PI/180;

    gvf_ellipse(wp, a, b, alpha);

    return true;
}

