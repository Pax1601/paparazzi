/***********************************************************************
 * 2016 Autonomous flight of M.A.V. course @ TU Delft university       *
 * Group 8 code                                                        *
 * Hielke Krijnen Davide Passoni                                       *
 * Simon Spronk Jacco Zwanepol                                         *
************************************************************************/

/***********************************************************************
 *                 GENERAL DESCRIPTION OF THE CODE                     *
 *                                                                     *
 * All the proposed code is identified by the name "2016_avoider".     *
 * The submitted code comprises:                                       *
 *      - this file and the related header;                            *
 *      - a C implementation of the Matlab code available with the     *
 *        OCamLib free toolbox, obtained with the Matlab C Coder,      *
 *        located in ./lib;                                            *
 *      - the flight plan in /conf/flight_plans/TUDELFT/...            *
 *        .../2016_avoider.xml                                         *
 *      - the conf file in /conf/airframes/TUDELFT/2016_avoider.xml;   *
 *      - the conf file in /conf/TUDELFT/tudelft_course2016_conf.xml;  *
 *      - the conf file in /conf/modules/2016_avoider.xml;             *
 *      - the conf file in /conf/telemetry/2016_avoider.xml;           *
 *      - the edited messages file in /conf/messages.xml (entry 251);  *
 *      - a modified file in /sw/airborne/modules/computer_vision/...  *
 *        .../lib/vision/image.c;                                      *
 *                                                                     *
 * A general description of the code is hereby provided.               *
 *                                                                     *
 *                           FLIGHT PLAN                               *
 *                                                                     *
 * In the flight plan, 4 blocks are added:                             *
 *                                                                     *
 * Initialize: is the block that the user must manually call to start  *
 * the autonomous flight. It sets up CENTER (2m in front of the drone) *
 * and GOAL (20m in front of the drone) waypoints.                     *
 *                                                                     *
 * START: is the block where the navigation and obstacle avoidance     *
 * takes place. Two "exception" conditions are used. The first is a    *
 * dummy exception condition, because the called fuction returns       *
 * always FALSE. It is used to call the navigation function at high    *
 * frequency. This is required to allow the periodic call to the       *
 * function and at the same time to use the paparazzi navigation       *
 * capability. The called function is is_safe(). All the steering      *
 * is computed inside is_safe().                                       *
 * The second exception is used to check if the CENTER waypoint is     *
 * outside of the CyberZoo. If it is, the "Go back" block is called.   *
 * In the block itself, the drone is flown from the HOVER waypoint     *
 * to the GOAL waypoint in "route" mode. The GOAL waypoint is always   *
 * placed 20m from the drone, to allow proper speed control. If an     *
 * obstacle is detected in the is_safe() function, the GOAL waypoint   *
 * is moved to the side, according to the required turn direction.     *
 * This allows the steering of the drone. The HOVER waypoint is updated*
 * at a 1Hz frequency inside is_safe(), and is located at the drone's  *
 * position, unless the drone is turning away from an obstacle. This   *
 * decreases the lateral drift.                                        *
 *                                                                     *
 * Go back: when the drone gets close to the CyberZoo border, it's     *
 * turned back towards the HOME waypoint. The system waits 1 second    *
 * before the START block is called again.                             *
 *                                                                     *
 *                                                                     *
 *                   FLOW GRAPH REPRESENTATION                         *
 *                                                                     *
 *                                                                     *
 *                           User input                                *
 *                               |                                     *
 *                               |                                     *
 *                               v                                     *
 *                           Initialize                                *
 *                               |                                     *
 *                               |                                     *
 *                               v                                     *
 *                             START <------                           *
 *                               |         |                           *
 *   WP_CENTER outisde CyberZoo  |         |                           *
 *                               v         |                           *
 *                            Go back-------                           *
 *                                                                     *
 * The drone enters an infinite loop that is stopped either by the user*
 * or by the code when the battery is empty.                           *
 *                                                                     *
 *                                                                     *
 * MODULE CODE DESCRIPTION                                             *
 *                                                                     *
 * When the module is loaded, the init() function is called, which     *
 * sets up the variables and allocates the required memory. The        *
 * periodic() function is called at a fixed frequency to update the    *
 * position of the drone.                                              *
 * Two filters are added to the computer vision periodic task. One is  *
 * used for obstacle detection, the other for ground color calibration.*
 * The obstacle detection filter is called draw_control_lines. It is   *
 * called periodically and it is used to extract the image points along* 
 * the control lines, using the compute_position_on_camera() function  *
 * to project the points on the camera using a corrected pinhole model.* 
 * To do so, the OCamLib world2cam() function is used.                 *
 * Inside draw_control_lines() image points are then searched for      *
 * obstacles. The bitmask variable is set accordingly. The bitmask is  *
 * a global variables accessible from the is_safe() function.          *
 * The autopilot "communicates" with the module via the is_safe()      *
 * function, which is called periodically. The bitmask is inspected,   *
 * and if a turn is required, the turn_waypoint() function is called to*
 * move the GOAL waypoint in the correct position.                     *
 * Inside is_safe(), the outer control line is checked too. If an      * 
 * obstacle is detected, the speed of the drone is decreased.          *
 **********************************************************************/
 

/***********************************************************************
 *                            DEFINITIONS                              *
 **********************************************************************/

/* INCLUDES */
#include "modules/2016_avoider/2016_avoider.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "boards/bebop/video.h"
#include "boards/bebop/mt9f002.h"

#include <time.h>
#include <stdlib.h>

#ifndef VIDEO_STREAM
#define VIDEO_STREAM            FALSE           /* IF FALSE, NO VIDEO STREAM IS PERFORMED AND NOTHING IS DRAWN ON THE IMAGE */
#endif

/* COMPILATION TIME DEFINES */
#define THETA_OFF 		-0.43           // angle of which the front camera is pointing down
#define CALIB_SQUARE_DIM	10              // dimension of the calibrating area square, in pixels
#define OBSTACLE_THR            3               // obstacle threshold. If more than OBSTACLE_THR pixels are detected as NOT GROUND
                                                // an obstacle is detected.

/* OBSTACLE BITMASK VALUES */
#define OBST_RIGHT              4
#define OBST_LEFT               1
#define OBST_CENTER             2
#define OBST_CENTER_RIGHT       6
#define OBST_CENTER_LEFT        3
#define OBST_LEFT_RIGHT         5
#define OBST_ALL                7


/* SPEED AND TURNS DEFINITIONS */
#define ANGLE_HARD              0.2             // radians
#define ANGLE_SOFT              0.1             // radians

#define SPEED_FREE              1.0             // m/s
#define SPEED_OBST_AHEAD        0.4             // m/s
#define SPEED_TURN_SOFT         0.3             // m/s
#define SPEED_TURN_HARD         0.1             // m/s

/* DEFINES FOR SETTINGS VALUES DEFINED IN THE AIRFRAME'S XML FILE*/
/* GROUND FILTHER WIDTHS */
#ifndef Y_FILTER_WIDTH
#define Y_FILTER_WIDTH          10
#endif

#ifndef U_FILTER_WIDTH
#define U_FILTER_WIDTH          10
#endif

#ifndef V_FILTER_WIDTH
#define V_FILTER_WIDTH          10
#endif

/* GROUND COLOR */
#ifndef Y_FILTER
#define Y_FILTER                225
#endif

#ifndef U_FILTER
#define U_FILTER                116
#endif

#ifndef V_FILTER
#define V_FILTER                119
#endif

/* CAMERA GAINS */
#ifndef R_GAIN
#define R_GAIN                  5
#endif

#ifndef G_GAIN
#define G_GAIN                  5
#endif

#ifndef B_GAIN
#define B_GAIN                  5
#endif

#ifndef T_GAIN
#define T_GAIN                  5
#endif

/***********************************************************************
 *                             VARIABLES                               *
 **********************************************************************/

/*  
 *                               ^ Y
 *                               |
 *                               | 
 *                       1 ______|______ 2
 *                        /      |      \
 *                       /       |       \
 *                      /        |        \
 *                     /         |         \
 *                    /          |          \
 *                   /           |           \
 *                  /            |            \
 *                 /             |             \
 *              0 /              |              \ 3
 *                               ---------------------->  X
 * 
 */

/* CONTROL AND SAFETY POINTS DEFINITION */
// Control points are used to turn the drone 
double control_points_positions_X[4] = {-0.8, -0.5, 0.5, 0.8};
double control_points_positions_Y[4] = {0, 1.25, 1.25, 0};

// Safety points are used to choose the velocity of the drone 
double safety_points_positions_X[4] = {-0.6, -0.4, 0.4, 0.6};
double safety_points_positions_Y[4] = {2.0, 3.0, 3.0, 2.0};

// Last time variable used to disable the HOVER waypoint positioning 
float last_time = 0;

// Points for drawing lines
struct point_t point1;
struct point_t point2;

// "Widths" of the yuv filter for picking up the ground color
double y_width;
double u_width;
double v_width;

// Stucture holding the pixels located on the corner lines
struct pixels_array pixels_array_border;

// Pointer to an index element that holds the amount of pixels actually used in pixels_array_border. This
// way no dynamic memory allocation is required.
uint32_t *pixels_index;

// Drone position in space
double drone_X = 0;
double drone_Y = 0;
double drone_Z = 0;

// Heading is provided as 0 pointing north, positive clockwise. This allows to revert to the normal cartesian representation, with 0 pointing
// East, positive anticlockwise
double heading_offset = -M_PI / 2;

// Ground color mean values. When calibrating the ground color, this values hold the mean yuv values of the pixels inside the calibration box
double ground_y_mean = 0;
double ground_u_mean = 0;
double ground_v_mean = 0;

// Ground color values
double ground_y;
double ground_u;
double ground_v;

// Camera gains
uint8_t r_gain;
uint8_t r_gain_old;

uint8_t g_gain;
uint8_t g_gain_old;

uint8_t b_gain;
uint8_t b_gain_old;

uint8_t t_gain;
uint8_t t_gain_old;

// Output bitmask, used to chose the direction of the turn.
uint8_t bitmask = 0;

// TRUE if an obstacle is sensed by the "safety" points. If FALSE, the drone flies at maximum speed.
uint8_t obstacle_ahead = 0;

// Ground calibration flag
bool calibrating_ground = FALSE;


/***********************************************************************
 *                             FUNCTIONS                               *
 **********************************************************************/

/* Positions a given waypoint distanceMeters in front of the drone, along the current navigation heading */
uint8_t position_waypoint(uint8_t waypoint, float distanceMeters){
                    
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
          
	  // Calculate the sine and cosine of the navigation heading
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
          
          return FALSE;
}

/* Positions a given waypoint distanceMeters in front of the drone, along the current attitude heading, plus an offset in radians */
uint8_t turn_waypoint(uint8_t waypoint, float distanceMeters, float offset){
    
        struct EnuCoor_i new_coor;
        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
        struct FloatEulers *att = stateGetNedToBodyEulers_f(); 
        
        // Calculate the sine and cosine of the attitude heading, plus the offset
        float sin_heading = sinf(att->psi + offset);
        float cos_heading = cosf(att->psi + offset);
            
        // Now determine where to place the waypoint you want to go to
        new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
        new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
        new_coor.z = pos->z; // Keep the height the same

        // Set the waypoint to the calculated position
        waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
        nav_set_heading_towards_waypoint(waypoint);

        return FALSE;
}

/* Draw the control and safety lines, and extract the values of the image pixels to check for obstacles */
/* NOTE: - points in X-Y coordinates are referred to the real world. Positions are expressed in meters, relative to the drone. See graph above for axis definition.
 *       - points in x-y coordinates are referred to the camera. Points are expressed in pixels, and express the position of a pixel in the image. */
bool_t draw_control_lines(struct image_t* img){
    
        // Drone attitude
        struct FloatEulers *att = stateGetNedToBodyEulers_f(); 

        // Sines and cosines of attitude angles
        
        float sin_pitch = -sinf(att->theta + THETA_OFF);
        float cos_pitch = cosf(att->theta + THETA_OFF);
        
        float sin_roll = -sinf(att->phi);
        float cos_roll = cosf(att->phi);
        
        // Temporary points to store computation results
        double temp_point[2];
        
        
        // Control points positions on image
        double control_points_x[4];
        double control_points_y[4];
        
        // Safety points positions on image
        double safety_points_x[4];
        double safety_points_y[4];
             
        // Compute position of real world points on the camera.    
	for(uint8_t i = 0; i < 4; i++){
                compute_position_on_camera(-control_points_positions_X[i] * 256, -control_points_positions_Y[i] * 256, temp_point, 1, 0, sin_pitch, cos_pitch, sin_roll, cos_roll);
                control_points_x[i] = temp_point[0]; 
                control_points_y[i] = temp_point[1]; 

                compute_position_on_camera(-safety_points_positions_X[i] * 256, -safety_points_positions_Y[i] * 256, temp_point, 1, 0, sin_pitch, cos_pitch, sin_roll, cos_roll);
                safety_points_x[i] = temp_point[0]; 
                safety_points_y[i] = temp_point[1]; 
        } 
        
        // Vector of NOT GROUND points in the three control segments.
        uint32_t obstacle_count[3] = {0, 0, 0};
        
        
        for(uint8_t i = 0; i < 3; i++){
		point1.x = control_points_x[i];
		point1.y = control_points_y[i];
		point2.x = control_points_x[i + 1];
		point2.y = control_points_y[i + 1];
                
                // Extract the image points that lay on the line connecting 2 control points and store it in the pixels_array_border vector. The number of extracted points is variable
                // and it is stored in the value pointed by pixels_index.
                *pixels_index = 0;
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
		
		// Search for obstacles in the extracted points. NOTE, pixels_array_border uses 4 bytes per pixel couple, as specified in the YUV422 format. Only one of the couple of
		// pixels is used to find the obstacles.
		for (uint32_t j = 0; j < *pixels_index / 4; j++){
                    if (is_obstacle(*(pixels_array_border.pixels + j * 4 + 1), *(pixels_array_border.pixels + j * 4), *(pixels_array_border.pixels + j * 4 + 2))){
                         obstacle_count[i]++;
                    }
                }		
	}
	
	// Reset the obstacles bitmask to 0.
	bitmask = 0;
        
        // Set the bitmask value.
        if (obstacle_count[0] > OBSTACLE_THR){
            bitmask |= OBST_LEFT;
        }
        
        if (obstacle_count[1] > OBSTACLE_THR){
            bitmask |= OBST_CENTER;
        }
        if (obstacle_count[2] > OBSTACLE_THR){
            bitmask |= OBST_RIGHT;
        }

        // Repeat the same procedure for the safety points.
        uint32_t safety_obstacle_count[3] = {0, 0, 0};
        
        for(uint8_t i = 0; i < 3; i++){
		point1.x = safety_points_x[i];
		point1.y = safety_points_y[i];
		point2.x = safety_points_x[i + 1];
		point2.y = safety_points_y[i + 1];
                
                // Extract the image points that lay on the line connecting 2 control points and store it in the pixels_array_border vector. The number of extracted points is variable
                // and it is stored in the value pointed by pixels_index.
                *pixels_index = 0;
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
		
		for (uint32_t j = 0; j < *pixels_index / 4; j++){
                    
                    // Search for obstacles in the extracted points. NOTE, pixels_array_border uses 4 bytes per pixel couple, as specified in the YUV422 format. Only one of the couple of
                    // pixels is used to find the obstacles.
                    if (is_obstacle(*(pixels_array_border.pixels + j * 4 + 1), *(pixels_array_border.pixels + j * 4), *(pixels_array_border.pixels + j * 4 + 2))){
                         safety_obstacle_count[i]++;
                    }
                }
	}
	
	// This time we don't set the bitmask, we simply look if an obstacle is sensed by any of the 3 sectors of the safety line. If it is, we set the obstacle_ahead
	// variable to decrease the speed of the drone.
	if(safety_obstacle_count[0] > OBSTACLE_THR || safety_obstacle_count[1] > OBSTACLE_THR || safety_obstacle_count[2] > OBSTACLE_THR){
            obstacle_ahead = 1;
        } else {
            obstacle_ahead = 0;
        }
        
        // Gaphics are enabled only if the video stream is enabled too.
        #if VIDEO_STREAM
        
        // Check if we are not calibrating the ground color.
        if (!calibrating_ground){
            
            // Apply the color filter to the entire image. Draw the ground green, while keep obstacles of their original color. This is used during testing to check
            // for the correct behaviour of the system.
            image_yuv422_colorfilt(img, img, (ground_y - y_width > 0) ? (uint8_t) (ground_y - y_width) : 0, (ground_y + y_width < 255) ?  (uint8_t)(ground_y + y_width) : 255, (ground_u - u_width > 0) ? (uint8_t) (ground_u - u_width) : 0, (ground_u + u_width < 255) ?  (uint8_t)(ground_u + u_width) : 255, (ground_v - v_width > 0) ? (uint8_t) (ground_v - v_width) : 0, (ground_v + v_width < 255) ?  (uint8_t)(ground_v + v_width) : 255);
            
            // Draw the control and safety lines on the image.
            for(uint8_t i = 0; i < 3; i++){
                    point1.x = control_points_x[i];
                    point1.y = control_points_y[i];
                    point2.x = control_points_x[i + 1];
                    point2.y = control_points_y[i + 1];
                    
                    if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
                            image_draw_line_color(img, &point1, &point2, 255, 0, 148);	
                    }
            }
            
            for(uint8_t i = 0; i < 3; i++){
                    point1.x = safety_points_x[i];
                    point1.y = safety_points_y[i];
                    point2.x = safety_points_x[i + 1];
                    point2.y = safety_points_y[i + 1];
                    
                    if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
                            image_draw_line_color(img, &point1, &point2, 255, 0, 148);	
                    }
            }
	
	}

        #endif
	
	return FALSE;
	
}

// Function called by the autopilot. The return value of the function is always FALSE because all the obstacle avoidance manoeuvres are commanded here, and it is not 
// required to "stop" the autopilot.
bool_t is_safe(uint8_t waypoint_hover, uint8_t waypoint_goal, uint8_t waypoint_center, float time){
    
    // The flight speed is set in a "cascade" manner, for the highest to the lowest value. Lower values are set if the associated checks are TRUE. The first set 
    // value is the maximum flight speed.
    double speed_to_set = SPEED_FREE;
    
    // If there is an obstacle ahead, slow down.
    if (obstacle_ahead){
        speed_to_set = SPEED_OBST_AHEAD;
    }
            
    /* Switch construct used to decide the required avoidance manoeuvre. The bitmask is used as control variable.
     * Each case is made of:
     *      - the GOAL waypoint is turned of the required angle.
     *      - the speed is set to the appropriate value.
     * 
     * Control is applied in this way:
     *      - if only the left or right sectors detect an obstacles, only a small turn is required. The ANGLE_SOFT 
     *        variable is used to turn the drone. The velocity is slightly decreased;
     *      - if both the center and one of the later sectors detect and obstacle, a tighter turn is required. The 
     *        ANGLE_HARD is used. The velocity is decreased more;
     *      - if both the left and right sectors sense an obstacle, the drone behaves as on the point above, but 
     *        the turn direction is hard-coded;
     *      - if ALL the sectors sense an obstacle, a very hard turn is commanded. Again, the turn direction is 
     *        hard-coded;
     */    
    
    switch (bitmask){
            case OBST_LEFT:
                turn_waypoint(waypoint_goal, 20, ANGLE_SOFT);
                speed_to_set = SPEED_TURN_SOFT;
                break;
                
            case OBST_RIGHT:
                turn_waypoint(waypoint_goal, 20, -ANGLE_SOFT);
                speed_to_set = SPEED_TURN_SOFT;
                break;
                
            case OBST_CENTER_LEFT:
                turn_waypoint(waypoint_goal, 20, ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_CENTER_RIGHT:
                turn_waypoint(waypoint_goal, 20, -ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_CENTER:
                turn_waypoint(waypoint_goal, 20, ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_LEFT_RIGHT:
                turn_waypoint(waypoint_goal, 20,  ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_ALL:
                turn_waypoint(waypoint_goal, 20, 8 * ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            default:
                
                break;
        }
        
        // We are at the end of the speed "cascade", we se the maximum speed value for the autopilot.
        guidance_h_SetMaxSpeed(speed_to_set);
        
        // The HOVER waypoint is set at a maxumum frequency of 1Hz, but only if we are not manoeuvring to avoid an obstacle.
        if (bitmask == 0 && time - last_time > 1.0){
            last_time = time;
            NavSetWaypointHere(waypoint_hover);
        }
        
        // The last_time variable is not updated during obstacle avoiding. This causes the fact that, as soon as the obstacle is cleared,
        // the HOVE waypoint is immidiately positioned.
        if (bitmask != 0){
            last_time = time;
        }
        
        // Position the CENTER waypoint in front of the drone to remain inside the CyberZoo.
        position_waypoint(waypoint_center, 2.0);
        
        return FALSE;
}

// Check if a pixel is ground or not by applying the YUV filter.
bool_t is_obstacle(double y, double u, double v){

	if (y < ground_y - y_width || y > ground_y + y_width || u < ground_u - u_width || u > ground_u + u_width || v < ground_v - v_width || v > ground_v + v_width) {
		return TRUE;
	} else {
		return FALSE;
	}

}

// Calibrate the ground color. A small square is drawn in the center of the image. The user points the drone in such a way that the square 
// contains only ground pixels. 
bool_t compute_calibration_color(struct image_t* img){

        // Only run the if calibrating_ground flag is TRUE. Such flag is set by the user in the GCS via the flightplan.
	if (calibrating_ground){

                // Border of the calibration box.
		uint16_t calibration_box_x[5] = {136 + CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2}; 
		uint16_t calibration_box_y[5] = {136 - CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2}; 
		
                
		uint8_t *img_buf = (uint8_t *)img->buf;
		uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;
	
		uint32_t index = 0;

                // Reset to 0 the mean values for the ground color.
		ground_y_mean = 0;
		ground_u_mean = 0;
		ground_v_mean = 0;

                // Scan all the points in the calibration box, and add them to the mean values.
		for (uint8_t row = calibration_box_y[3]; row < calibration_box_y[3] + CALIB_SQUARE_DIM; row += 1) {
			for (uint8_t column = calibration_box_x[3]; column < calibration_box_x[3] + CALIB_SQUARE_DIM; column += 2) {

				index = img->w * row  * pixel_width + column * pixel_width - 2;

				ground_u_mean += img_buf[index];
		
				if (img->type == IMAGE_YUV422) {
					ground_y_mean += img_buf[index + 1];	
				}

				ground_v_mean += img_buf[index + 2];
				ground_y_mean += img_buf[index + 3];
			}
		}

		// Finalize the mean values by dividing by the number of pixels in the square.
		ground_y_mean = ground_y_mean / (CALIB_SQUARE_DIM * CALIB_SQUARE_DIM);
		ground_u_mean = ground_u_mean / (CALIB_SQUARE_DIM * CALIB_SQUARE_DIM / 2);
		ground_v_mean = ground_v_mean / (CALIB_SQUARE_DIM * CALIB_SQUARE_DIM / 2);

                // Print on screen for debug
		printf("%f, %f, %f\n", ground_y_mean, ground_u_mean, ground_v_mean);
				
                
                // If we are videostreaming, draw the calibration box and apply the color filter using the mean values computed. This way 
                // the user can immediately see the effects of the new ground color.
                
                #if VIDEOSTREAM
                
		for (uint8_t i = 0; i < 4; i++){
			point1.x = calibration_box_x[i];
			point1.y = calibration_box_y[i];
			point2.x = calibration_box_x[i + 1];
			point2.y = calibration_box_y[i + 1];	
			image_draw_line_color(img, &point1, &point2, 149, 43, 21);	
		}
		
                image_yuv422_colorfilt(img, img, (ground_y_mean - y_width > 0) ? (uint8_t) (ground_y_mean - y_width) : 0, (ground_y_mean + y_width < 255) ?  (uint8_t)(ground_y_mean + y_width) : 255, (ground_u_mean - u_width > 0) ? (uint8_t) (ground_u_mean - u_width) : 0, (ground_u_mean + u_width < 255) ?  (uint8_t)(ground_u_mean + u_width) : 255, (ground_v_mean - v_width > 0) ? (uint8_t) (ground_v_mean - v_width) : 0, (ground_v_mean + v_width < 255) ?  (uint8_t)(ground_v_mean + v_width) : 255);
                
                #endif
	}
	
	return FALSE;
}

// Apply the inverse camera calibration function to obtain the image position of a real world point. Real world points are specified with X and Y values on the ground.
// The code is specific for points on the ground, but can be extended for points in every X-Y-Z position.
void compute_position_on_camera(double point_X, double point_Y, double *temp_point, double sin_heading, double cos_heading, double sin_pitch, double cos_pitch, double sin_roll, double cos_roll){
    
    double rel_position_x = point_X;
    double rel_position_y = point_Y;

    // First rotation: yaw
    double optic_z_1 = cos_heading * rel_position_x + sin_heading * rel_position_y;
    double optic_x_1 = -sin_heading * rel_position_x + cos_heading * rel_position_y;
    double optic_y_1 = drone_Z;

    // Second rotation: pitch
    double optic_z_2 = cos_pitch * optic_z_1 - sin_pitch * optic_y_1;
    double optic_y_2 = sin_pitch * optic_z_1 + cos_pitch * optic_y_1;
    double optic_x_2 = optic_x_1;

    // Final rotation: roll
    double optic_x = cos_roll * optic_x_2 - sin_roll * optic_y_2;
    double optic_y = sin_roll * optic_x_2 + cos_roll * optic_y_2;
    double optic_z = optic_z_2;

    // Find the real world point on the picture
    world2cam(optic_x, optic_y, optic_z, temp_point);
    
}

// Send telemetry values to the user for debugging and operation control.
static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
    
    uint8_t bitmask_vector[3];
    uint8_t k;
    int c;
        
    // Write the bitmask in a 3 values array.
    for (c = 2; c >= 0; c--){
        k = bitmask >> c;
        if (k & 1)
            bitmask_vector[c] = 1;
        else
            bitmask_vector[c] = 0;
    }
    pprz_msg_send_2016_AVOIDER(trans, dev, AC_ID, 3, bitmask_vector, &obstacle_ahead);
}

// INITIALIZATION
void init() { 

	// Initialize the structure holding the pixels values
	pixels_array_border.pixels = malloc(sizeof(uint8_t) * 4 * 272 * 50);
	pixels_array_border.x = malloc(sizeof(uint8_t) * 272 * 50);
	pixels_array_border.y = malloc(sizeof(uint8_t) * 272 * 50);
	pixels_index = malloc(sizeof(uint32_t));

	// Initialize Matlab functions
        world2cam_initialize();
	
	// Initialize the filters to apply to the image
        cv_add(draw_control_lines);
	cv_add(compute_calibration_color);

        // Apply the values specified by the user in the XML file.
	y_width = Y_FILTER_WIDTH;
	u_width = U_FILTER_WIDTH;
	v_width = V_FILTER_WIDTH;
        
        ground_y = Y_FILTER;
        ground_u = U_FILTER;
        ground_v = V_FILTER;
        
        r_gain = R_GAIN;
        r_gain_old = 0;
        
        g_gain = G_GAIN;
        g_gain_old = 0;
        
        b_gain = B_GAIN;
        b_gain_old = 0;
        
        t_gain = T_GAIN;
        t_gain_old = 0;
        
        // Register the telemetry function.
        #if PERIODIC_TELEMETRY
            register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_2016_AVOIDER, send_telemetry);
        #endif

}

// PERIODIC
void periodic() {
	
        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

        // Set the position of the drone
	drone_X = pos->x;
        drone_Y = pos->y;
        drone_Z = pos->z;

        // Check if the user has changed the camera gains via the GCS.
        set_camera_gains();
        
}


// Called by the user from the flightplan to start the ground calibration
uint8_t start_ground_color_calibration()
{
	printf("Starting ground color calibration...\n");
	calibrating_ground = TRUE;
	return FALSE;
}

// When the user is satisfied with the values, this function is called to save the mean values
uint8_t calibrate_ground_color()
{
	printf("Ground color calibrated.\n");

	calibrating_ground = FALSE;

	ground_y = ground_y_mean;
	ground_u = ground_u_mean;
	ground_v = ground_v_mean;

	return FALSE;
}

// Set the camera gains according to the camera datasheet
uint8_t set_camera_gains(){
     if (r_gain != r_gain_old){
        r_gain_old = r_gain;
        
        switch(r_gain){
            case 1:
                mt9f002_write_reg16(MT9F002_RED_GAIN, 5128);
                break;
                
            case 2:
                mt9f002_write_reg16(MT9F002_RED_GAIN, 6239);
                break;
            
            case 3:
                mt9f002_write_reg16(MT9F002_RED_GAIN, 7295);
                break;
                
            case 4:
                mt9f002_write_reg16(MT9F002_RED_GAIN, 11391);
                break;
                
            case 5:
                mt9f002_write_reg16(MT9F002_RED_GAIN, 19583);
                break;
                
            default:
                
                break;
        }
    }
        
    if (g_gain != g_gain_old){
        g_gain_old = g_gain;
        
        switch(g_gain){
            case 1:
                mt9f002_write_reg16(MT9F002_GREEN1_GAIN, 5128);
                mt9f002_write_reg16(MT9F002_GREEN2_GAIN, 5128);
                break;
                
            case 2:
                mt9f002_write_reg16(MT9F002_GREEN1_GAIN, 6239);
                mt9f002_write_reg16(MT9F002_GREEN2_GAIN, 6239);
                break;
            
            case 3:
                mt9f002_write_reg16(MT9F002_GREEN1_GAIN, 7295);
                mt9f002_write_reg16(MT9F002_GREEN2_GAIN, 7295);
                break;
                
            case 4:
                mt9f002_write_reg16(MT9F002_GREEN1_GAIN, 11391);
                mt9f002_write_reg16(MT9F002_GREEN2_GAIN, 11391);
                break;
                
            case 5:
                mt9f002_write_reg16(MT9F002_GREEN1_GAIN, 19583);
                mt9f002_write_reg16(MT9F002_GREEN2_GAIN, 19583);
                break;
                
            default:
                
                break;
        }
    }
    
    if (b_gain != b_gain_old){
        b_gain_old = b_gain;
        
        switch(b_gain){
            case 1:
                mt9f002_write_reg16(MT9F002_BLUE_GAIN, 5128);
                break;
                
            case 2:
                mt9f002_write_reg16(MT9F002_BLUE_GAIN, 6239);
                break;
            
            case 3:
                mt9f002_write_reg16(MT9F002_BLUE_GAIN, 7295);
                break;
                
            case 4:
                mt9f002_write_reg16(MT9F002_BLUE_GAIN, 11391);
                break;
                
            case 5:
                mt9f002_write_reg16(MT9F002_BLUE_GAIN, 19583);
                break;
                
            default:
                
                break;
        }
    }
    
    if (t_gain != t_gain_old){
        t_gain_old = t_gain;
        
        switch(t_gain){
            case 1:
                mt9f002_write_reg16(MT9F002_GLOBAL_GAIN, 5128);
                break;
                
            case 2:
                mt9f002_write_reg16(MT9F002_GLOBAL_GAIN, 6239);
                break;
            
            case 3:
                mt9f002_write_reg16(MT9F002_GLOBAL_GAIN, 7295);
                break;
                
            case 4:
                mt9f002_write_reg16(MT9F002_GLOBAL_GAIN, 11391);
                break;
                
            case 5:
                mt9f002_write_reg16(MT9F002_GLOBAL_GAIN, 19583);
                break;
                
            default:
                
                break;
        }
    }      
    
    return FALSE;
}






