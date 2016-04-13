/***********************************************************************
 *                            DEFINITIONS                              *
 **********************************************************************/

// Main
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
#define THETA_OFF 		-0.43         // angle of which the front camera is pointing down
#define CALIB_SQUARE_DIM	10             // dimension of the calibrating area square, in pixels
#define OBSTACLE_THR            3


#define OBST_RIGHT              4
#define OBST_LEFT               1
#define OBST_CENTER             2
#define OBST_CENTER_RIGHT       6
#define OBST_CENTER_LEFT        3
#define OBST_LEFT_RIGHT         5
#define OBST_ALL                7

#define ANGLE_HARD              0.2
#define ANGLE_SOFT              0.1

#define SPEED_FREE              1.0
#define SPEED_OBST_AHEAD        0.4
#define SPEED_TURN_SOFT         0.3
#define SPEED_TURN_HARD         0.1

/* DEFINES FOR SETTINGS VALUES DEFINED IN THE AIRFRAME'S XML FILE*/
#ifndef Y_FILTER_WIDTH
#define Y_FILTER_WIDTH          10
#endif

#ifndef U_FILTER_WIDTH
#define U_FILTER_WIDTH          10
#endif

#ifndef V_FILTER_WIDTH
#define V_FILTER_WIDTH          10
#endif


#ifndef Y_FILTER
#define Y_FILTER                225
#endif

#ifndef U_FILTER
#define U_FILTER                116
#endif

#ifndef V_FILTER
#define V_FILTER                119
#endif

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


uint8_t r_gain;
uint8_t r_gain_old;

uint8_t g_gain;
uint8_t g_gain_old;

uint8_t b_gain;
uint8_t b_gain_old;

uint8_t t_gain;
uint8_t t_gain_old;

double control_points_positions_X[4] = {-0.8, -0.5, 0.5, 0.8};
double control_points_positions_Y[4] = {0, 1.25, 1.25, 0};

double safety_points_positions_X[4] = {-0.6, -0.4, 0.4, 0.6};
double safety_points_positions_Y[4] = {2.0, 3.0, 3.0, 2.0};

float last_time = 0;

/***********************************************************************
 *                             VARIABLES                               *
 **********************************************************************/

// Points for drawing lines
struct point_t point1;
struct point_t point2;

// "Widths" of the yuv filter for picking up the ground color
double y_width;
double u_width;
double v_width;

// Stucture holding the pixels located on the corner lines
struct pixels_array pixels_array_border;

// Pointer to an index element that hold the amount of pixels actually used in pixels_array_border. This
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

uint8_t bitmask = 0;
uint8_t obstacle_ahead = 0;

uint8_t working_waypoint; 

// Ground calibration flag
bool calibrating_ground = FALSE;

/***********************************************************************
 *                             FUNCTIONS                               *
 **********************************************************************/

bool_t draw_control_lines(struct image_t* img);
void compute_position_on_camera(double point_X, double point_Y, double *temp_point, double sin_heading, double cos_heading, double sin_pitch, double cos_pitch, double sin_roll, double cos_roll);
bool_t is_obstacle(double y, double u, double v);
uint8_t turn_waypoint(uint8_t waypoint, float distanceMeters, float offset);

uint8_t set_working_waypoint(uint8_t waypoint){
    working_waypoint = waypoint;
    return FALSE;
}

uint8_t position_waypoint(uint8_t waypoint, float distanceMeters){
                    
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
          
	  // Calculate the sine and cosine of the heading the drone is keeping
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

uint8_t turn_waypoint(uint8_t waypoint, float distanceMeters, float offset){
    
        struct EnuCoor_i new_coor;
        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
        struct FloatEulers *att = stateGetNedToBodyEulers_f(); 

        // Calculate the sine and cosine of the heading the drone is keeping
        
        /*
        float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading) + offset);
        float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading) + offset);
        */
        
        //offset = 0;
        
        
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

bool_t draw_control_lines(struct image_t* img){
    
        // Drone attitude
        struct FloatEulers *att = stateGetNedToBodyEulers_f(); 

        // Sines and cosines of attitude angles
        
        float sin_pitch = -sinf(att->theta + THETA_OFF);
        float cos_pitch = cosf(att->theta + THETA_OFF);
        
        
        float sin_roll = -sinf(att->phi);
        float cos_roll = cosf(att->phi);
        
        double temp_point[2];
        
        double active_points_x[4];
        double active_points_y[4];
        
        double safety_points_x[4];
        double safety_points_y[4];
             
            
	for(uint8_t i = 0; i < 4; i++){
                compute_position_on_camera(-control_points_positions_X[i] * 256, -control_points_positions_Y[i] * 256, temp_point, 1, 0, sin_pitch, cos_pitch, sin_roll, cos_roll);
                active_points_x[i] = temp_point[0]; 
                active_points_y[i] = temp_point[1]; 

                compute_position_on_camera(-safety_points_positions_X[i] * 256, -safety_points_positions_Y[i] * 256, temp_point, 1, 0, sin_pitch, cos_pitch, sin_roll, cos_roll);
                safety_points_x[i] = temp_point[0]; 
                safety_points_y[i] = temp_point[1]; 
        }
        
        uint32_t obstacle_count[3] = {0, 0, 0};
        
        for(uint8_t i = 0; i < 3; i++){
		point1.x = active_points_x[i];
		point1.y = active_points_y[i];
		point2.x = active_points_x[i + 1];
		point2.y = active_points_y[i + 1];
                
                *pixels_index = 0;
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
		
		for (uint32_t j = 0; j < *pixels_index / 4; j++){
                    
                    if (is_obstacle(*(pixels_array_border.pixels + j * 4 + 1), *(pixels_array_border.pixels + j * 4), *(pixels_array_border.pixels + j * 4 + 2))){
                         obstacle_count[i]++;
                    }
                }		
	}
	
	bitmask = 0;
        
        if (obstacle_count[0] > OBSTACLE_THR){
            bitmask |= 1;
        }
        
        if (obstacle_count[1] > OBSTACLE_THR){
            bitmask |= 2;
        }
        if (obstacle_count[2] > OBSTACLE_THR){
            bitmask |= 4;
        }

        uint32_t safety_obstacle_count[3] = {0, 0, 0};
        
        for(uint8_t i = 0; i < 3; i++){
		point1.x = safety_points_x[i];
		point1.y = safety_points_y[i];
		point2.x = safety_points_x[i + 1];
		point2.y = safety_points_y[i + 1];
                
                *pixels_index = 0;
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
		
		for (uint32_t j = 0; j < *pixels_index / 4; j++){
                    
                    if (is_obstacle(*(pixels_array_border.pixels + j * 4 + 1), *(pixels_array_border.pixels + j * 4), *(pixels_array_border.pixels + j * 4 + 2))){
                         safety_obstacle_count[i]++;
                    }
                }
	}
	
	if(safety_obstacle_count[0] > OBSTACLE_THR || safety_obstacle_count[1] > OBSTACLE_THR || safety_obstacle_count[2] > OBSTACLE_THR){
            
            obstacle_ahead = 1;

        } else {
         
            obstacle_ahead = 0;
            
        }
        
        #if VIDEO_STREAM
        
        if (!calibrating_ground){
            image_yuv422_colorfilt(img, img, (ground_y - y_width > 0) ? (uint8_t) (ground_y - y_width) : 0, (ground_y + y_width < 255) ?  (uint8_t)(ground_y + y_width) : 255, (ground_u - u_width > 0) ? (uint8_t) (ground_u - u_width) : 0, (ground_u + u_width < 255) ?  (uint8_t)(ground_u + u_width) : 255, (ground_v - v_width > 0) ? (uint8_t) (ground_v - v_width) : 0, (ground_v + v_width < 255) ?  (uint8_t)(ground_v + v_width) : 255);
        
            for(uint8_t i = 0; i < 3; i++){
                    point1.x = active_points_x[i];
                    point1.y = active_points_y[i];
                    point2.x = active_points_x[i + 1];
                    point2.y = active_points_y[i + 1];
                    
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

bool_t is_safe(uint8_t waypoint_hover, uint8_t waypoint_goal, uint8_t waypoint_center, float time){
            
    double speed_to_set = SPEED_FREE;
    
    if (obstacle_ahead){
        speed_to_set = SPEED_OBST_AHEAD;
    }
            
    switch (bitmask){
            case OBST_LEFT:
                printf("Turn right.\n");
                turn_waypoint(waypoint_goal, 20, ANGLE_SOFT);
                speed_to_set = SPEED_TURN_SOFT;
                break;
                
            case OBST_RIGHT:
                printf("Turn left.\n");
                turn_waypoint(waypoint_goal, 20, -ANGLE_SOFT);
                speed_to_set = SPEED_TURN_SOFT;
                break;
                
            case OBST_CENTER_LEFT:
                printf("Turn hard right.\n");
                turn_waypoint(waypoint_goal, 20, ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_CENTER_RIGHT:
                printf("Turn hard left.\n");
                turn_waypoint(waypoint_goal, 20, -ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_CENTER:
                printf("Go back.\n");
                turn_waypoint(waypoint_goal, 20, ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_LEFT_RIGHT:
                printf("Going through.\n");
                turn_waypoint(waypoint_goal, 20,  ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            case OBST_ALL:
                printf("Go back!!!.\n");
                turn_waypoint(waypoint_goal, 20, 8 * ANGLE_HARD);
                speed_to_set = SPEED_TURN_HARD;
                break;
                
            default:
                
                break;
        }
        
        guidance_h_SetMaxSpeed(speed_to_set);
        
        if (bitmask == 0 && time - last_time > 1.0){
            last_time = time;
            NavSetWaypointHere(waypoint_hover);
        }
        
        if (bitmask != 0){
            last_time = time;
        }
        
        position_waypoint(waypoint_center, 2.0);
        
        return FALSE;
}

bool_t is_obstacle(double y, double u, double v){

	if (y < ground_y - y_width || y > ground_y + y_width || u < ground_u - u_width || u > ground_u + u_width || v < ground_v - v_width || v > ground_v + v_width) {
		return TRUE;
	} else {
		return FALSE;
	}

}

bool_t compute_calibration_color(struct image_t* img){

	if (calibrating_ground){

		uint16_t calibration_box_x[5] = {136 + CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2}; 
		uint16_t calibration_box_y[5] = {136 - CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2, 136 + CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2, 136 - CALIB_SQUARE_DIM / 2}; 
		 
		uint8_t *img_buf = (uint8_t *)img->buf;
		uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;
	
		uint32_t index = 0;

		ground_y_mean = 0;
		ground_u_mean = 0;
		ground_v_mean = 0;

		/* draw the line  UYVY */
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

		ground_y_mean = ground_y_mean / (CALIB_SQUARE_DIM * CALIB_SQUARE_DIM);
		ground_u_mean = ground_u_mean / (CALIB_SQUARE_DIM * CALIB_SQUARE_DIM / 2);
		ground_v_mean = ground_v_mean / (CALIB_SQUARE_DIM * CALIB_SQUARE_DIM / 2);

		printf("%f, %f, %f\n", ground_y_mean, ground_u_mean, ground_v_mean);
				
		for (uint8_t i = 0; i < 4; i++){
			point1.x = calibration_box_x[i];
			point1.y = calibration_box_y[i];
			point2.x = calibration_box_x[i + 1];
			point2.y = calibration_box_y[i + 1];	
			image_draw_line_color(img, &point1, &point2, 149, 43, 21);	
		}
		
                image_yuv422_colorfilt(img, img, (ground_y_mean - y_width > 0) ? (uint8_t) (ground_y_mean - y_width) : 0, (ground_y_mean + y_width < 255) ?  (uint8_t)(ground_y_mean + y_width) : 255, (ground_u_mean - u_width > 0) ? (uint8_t) (ground_u_mean - u_width) : 0, (ground_u_mean + u_width < 255) ?  (uint8_t)(ground_u_mean + u_width) : 255, (ground_v_mean - v_width > 0) ? (uint8_t) (ground_v_mean - v_width) : 0, (ground_v_mean + v_width < 255) ?  (uint8_t)(ground_v_mean + v_width) : 255);
	}
	
	return FALSE;
}


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

static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
    
    uint8_t bitmask_vector[3];
    
    uint8_t k;
    
    int c;
        
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
        
        #if PERIODIC_TELEMETRY
            register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_2016_AVOIDER, send_telemetry);
        #endif

}

// PERIODIC
void periodic() {
	
        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	drone_X = pos->x;
        drone_Y = pos->y;
        drone_Z = pos->z;
        //drone_Z = 256;

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
}

uint8_t start_ground_color_calibration()
{
	printf("Starting ground color calibration...\n");
	
	calibrating_ground = TRUE;

	return FALSE;
}

uint8_t calibrate_ground_color()
{
	printf("Ground color calibrated.\n");

	calibrating_ground = FALSE;

	ground_y = ground_y_mean;
	ground_u = ground_u_mean;
	ground_v = ground_v_mean;

	return FALSE;
}



