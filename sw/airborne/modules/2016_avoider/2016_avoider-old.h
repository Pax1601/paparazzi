/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef AVOIDER_H
#define AVOIDER_H
#include <inttypes.h>

#ifndef __MAIN_H__
#define __MAIN_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "lib/world2cam/rt_defines.h"
#include "lib/world2cam/rt_nonfinite.h"
#include "lib/world2cam/rtwtypes.h"
#include "lib/world2cam/world2cam_types.h"



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

extern int32_t incrementForAvoidance;
extern void init(void);
extern void periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern void update_ground_square(void);
extern bool_t draw_bound_square_func(struct image_t* img);
extern bool_t compute_calibration_color(struct image_t* img);
extern void find_obstacles(struct image_t* img);
extern uint8_t add_corner_point(void);
extern uint8_t reset_corners(void);
extern uint8_t use_saved_corners(void);
extern uint8_t reset_heading(void);
extern uint8_t start_ground_color_calibration(void);
extern uint8_t calibrate_ground_color(void);
void generate_ground_points(void);
extern uint8_t is_safe(uint8_t waypoint);
extern uint8_t obstacle_ahead(void);
extern uint8_t set_heading(double new_heading);
extern uint8_t reset_ground_safety(void);

extern double y_width;
extern double u_width;
extern double v_width;

extern double safety_radius_width;
extern double safety_radius_external;
extern double safety_radius_internal;

extern double ground_y;
extern double ground_u;
extern double ground_v;

void send_telemetry(struct transport_tx *trans, struct link_device *dev);
void generate_safety_radius(void);
bool_t draw_safety_radius(struct image_t* img);
bool_t is_obstacle(double y, double u, double v);
void compute_position_on_camera(double point_X, double point_Y, double *temp_point, double sin_heading, double cos_heading, double sin_pitch, double cos_pitch, double sin_roll, double cos_roll);
void sort_ground_points(double a[], uint8_t b[], int array_size);
uint8_t get_obst_vect_index(double x);

#endif

