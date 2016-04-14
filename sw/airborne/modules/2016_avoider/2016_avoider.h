/***********************************************************************
 * 2016 Autonomous flight of M.A.V. course @ TU Delft university       *
 * Group 8 code                                                        *
 * Hielke Krijnen Davide Passoni                                       *
 * Simon Spronk Jacco Zwanepol                                         *
************************************************************************/

#ifndef AVOIDER_H
#define AVOIDER_H
#include <inttypes.h>

#ifndef __MAIN_H__
#define __MAIN_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

// Computer vision
#include "modules/computer_vision/colorfilter.h"
#include "modules/computer_vision/cv.h"

// Autopilot
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"

// System
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <std.h>
#include <inttypes.h>

// World2Cam
#include "lib/world2cam/rt_nonfinite.h"
#include "lib/world2cam/world2cam.h"
#include "lib/world2cam/world2cam_terminate.h"
#include "lib/world2cam/world2cam_initialize.h"

#endif

/*
 * File trailer for main.h
 *
 * [EOF]
 */

/* EXTERN FUNCTIONS ACCESSIBLE FROM OTHER CODE */
extern void init(void);
extern void periodic(void);
extern bool_t compute_calibration_color(struct image_t* img);
extern uint8_t start_ground_color_calibration(void);
extern uint8_t calibrate_ground_color(void);
extern uint8_t position_waypoint(uint8_t waypoint, float distanceMeters);
extern uint8_t set_working_waypoint(uint8_t waypoint);
extern bool_t is_safe(uint8_t waypoint_hover, uint8_t waypoint_goal, uint8_t waypoint_center, float time);

/* EXTERN VARIABLES ACCESSIBLE FROM OTHER CODE */
extern double y_width;
extern double u_width;
extern double v_width;
extern uint8_t r_gain;
extern uint8_t g_gain;
extern uint8_t b_gain;
extern uint8_t t_gain;
extern double ground_y;
extern double ground_u;
extern double ground_v;

/* FUNCTIONS WITH LOCAL SCOPE */
bool_t draw_control_lines(struct image_t* img);
void compute_position_on_camera(double point_X, double point_Y, double *temp_point, double sin_heading, double cos_heading, double sin_pitch, double cos_pitch, double sin_roll, double cos_roll);
bool_t is_obstacle(double y, double u, double v);
uint8_t turn_waypoint(uint8_t waypoint, float distanceMeters, float offset);
uint8_t set_camera_gains(void);

#endif

