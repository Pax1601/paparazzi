/***********************************************************************
 *                            DEFINITIONS                              *
 **********************************************************************/

// Main
#include "modules/2016_avoider/2016_avoider.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"

#define VIDEO_STREAM		/* IF NOT DEFINED, NO VIDEO STREAM IS PERFORMED AND NOTHING IS DRAWN ON THE IMAGE */

/* COMPILATION TIME DEFINES */
#define THETA_OFF 		-0.395         // angle of which the front camera is pointing down
#define CALIB_SQUARE_DIM	10             // dimension of the calibrating area square, in pixels
#define MAX_NUM_CORNERS 	10             // maximum number of ground corners
#define POINTS_PER_EDGE		10             // number of inner points per side
#define POINTS_SAFETY_RADIUS	10             // number of points used to draw the safety lines
#define NUM_SECTORS             20             // number of vertical sectors used to compute the position of obstacles
#define MAX_NUM_OBSTCLES        20             // maximum number of obstacles that can be detected          

/* DEFINES FOR SETTINGS VALUES DEFINED IN THE AIRFRAME'S XML FILE*/
#ifndef WIDTH_SAFETY_RADIUS
#define WIDTH_SAFETY_RADIUS	40
#endif

#ifndef SAFETY_RADIUS_INTERNAL
#define SAFETY_RADIUS_INTERNAL	175
#endif

#ifndef SAFETY_RADIUS_EXTERNAL
#define SAFETY_RADIUS_EXTERNAL	225
#endif

#ifndef Y_FILTER_WIDTH
#define Y_FILTER_WIDTH          10
#endif

#ifndef U_FILTER_WIDTH
#define U_FILTER_WIDTH          10
#endif

#ifndef V_FILTER_WIDTH
#define V_FILTER_WIDTH          10
#endif


/***********************************************************************
 *                             VARIABLES                               *
 **********************************************************************/

// Points for drawing lines
struct point_t point1;
struct point_t point2;

// Ground square points "real world" coordinates.
double ground_points_X[POINTS_PER_EDGE * MAX_NUM_CORNERS];
double ground_points_Y[POINTS_PER_EDGE * MAX_NUM_CORNERS];

// Ground square camera coordinates
double ground_points_x[POINTS_PER_EDGE * MAX_NUM_CORNERS];
double ground_points_y[POINTS_PER_EDGE * MAX_NUM_CORNERS];

// Safety radius "real world" coordinates
double safety_radius_external_X[POINTS_SAFETY_RADIUS];
double safety_radius_external_Y[POINTS_SAFETY_RADIUS];
double safety_radius_internal_X[POINTS_SAFETY_RADIUS];
double safety_radius_internal_Y[POINTS_SAFETY_RADIUS];

// Safety radius camera coordinates
double safety_radius_external_x[POINTS_SAFETY_RADIUS];
double safety_radius_external_y[POINTS_SAFETY_RADIUS];
double safety_radius_internal_x[POINTS_SAFETY_RADIUS];
double safety_radius_internal_y[POINTS_SAFETY_RADIUS];

// Angular and radial dimensions of the safety radius
double safety_radius_width;
double safety_radius_external;
double safety_radius_internal;

// Old values of the safety radius, used to understand if the user has modified them via the GCS
double old_safety_radius_width;
double old_safety_radius_external;
double old_safety_radius_internal;

// Vector of obstacles "sectors". The FOV is divided in NUM_SECTORS slices. If a slice is 1, there is an obstacle there.
bool_t obst_vector[NUM_SECTORS];

// "Widths" of the yuv filter for picking up the ground color
double y_width;
double u_width;
double v_width;

// Stucture holding the pixels located on the corner lines
struct pixels_array pixels_array_border;

// Pointer to an index element that hold the amount of pixels actually used in pixels_array_border. This
// way no dynamic memory allocation is required.
uint16_t *pixels_index;

// Obstacle object
struct obstacle {
	uint8_t start_x;
	uint8_t end_x;
	double start_psi;
	double end_psi;
	double X;
	double Y;
};

// Ground corner structure holding the real world position of the corners of the border area.
struct ground_corner {
	double X;
	double Y;
};

// Array of obstacles
struct obstacle obstacles[MAX_NUM_OBSTCLES];

// Array of ground corners
struct ground_corner ground_corners[MAX_NUM_CORNERS];

// Number of corners actually used. Again, to prevent the use of dynamic memory allocation
uint8_t active_corners = 0;

// Index used to specify which ground point (not only corners) is the current "target"
uint8_t active_ground_point_index = 0;

// Array of bools storing if a ground point is safe or not from the current position. The drone will automatically
// go to the most distant safe ground point.
bool_t  ground_points_safety[POINTS_PER_EDGE * MAX_NUM_CORNERS]; 

// Array storing the indexes of the ground points. Ground points are sorted according to distance, from closest to further. The same
// sorting is applied to this array, which allow to backtrace the position of the ground points after the sorting.
uint8_t ground_points_indexes[POINTS_PER_EDGE * MAX_NUM_CORNERS];

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
double ground_y = 226;
double ground_u = 116;
double ground_v = 119;

// Ground calibration flag
bool calibrating_ground = FALSE;

// Number of detected obstacles
uint8_t detected_obstacles = 0;

// Flags that are 1 if an obstacle is detected either by the internal or external radius.
uint8_t obstacle_external = 0;
uint8_t obstacle_internal = 0;


/***********************************************************************
 *                             FUNCTIONS                               *
 **********************************************************************/


// Bubble sort algorithm for sorting ground points according to distance. Array a holds the distances, and is the array according to which elements
// are sorted. Array b is any array to which the same sorting must be applied.
void sort_ground_points(double a[], uint8_t b[], int array_size){
    int i, j, temp;
    for (i = 0; i < (array_size - 1); ++i)
    {
        for (j = 0; j < array_size - 1 - i; ++j )
        {
            if (a[j] > a[j+1])
            {
                temp = a[j+1];
                a[j+1] = a[j];
                a[j] = temp;
                
                temp = b[j+1];
                b[j+1] = b[j];
                b[j] = temp;
                
            }
        }
    }
}   

// Reset ground safety. Called by the navigator each time the drone leaves the hover point and goas towards a new waypoint. Used to reset the safety
// values.
uint8_t reset_ground_safety(){

    for (uint8_t i = 0; i < active_corners * POINTS_PER_EDGE; i++){
	
		ground_points_safety[i] = 1;
                ground_points_indexes[i] = i;
	
	}
	
    return FALSE;
        
}

// Get obstacle vector index. Returns the obstacle vector position given the camera x position.
uint8_t get_obst_vect_index(double x){
    
    double sector_width = 272 / NUM_SECTORS;
    return floor(x / sector_width);
    
}

// Checks if the path to a waypoint of chiche is  safe or not. Returns 1 if it safe, 0 if it unsafe or it still can't tell because is not yet pointing in the right direction.
// If it is not pointing in the right direction, it waits until it does. If the path is unsafe, the ground point is marked as "unsafe" and the next further point becomes active.
uint8_t is_safe(uint8_t waypoint){
    
    // Relative positions of the waypoint in real world
    double rel_position_X;
    double rel_position_Y;
    
    // Number of active ground points
    double distance[active_corners * POINTS_PER_EDGE];
    
    // Drone attitude
    struct FloatEulers *att = stateGetNedToBodyEulers_f(); 
   
    // Sines and cosines of attitude angles
    float sin_heading = sinf(-att->psi - heading_offset);
    float cos_heading = cosf(-att->psi - heading_offset);
    float sin_pitch = -sinf(att->theta + THETA_OFF);
    float cos_pitch = cosf(att->theta + THETA_OFF);
    float sin_roll = -sinf(att->phi);
    float cos_roll = cosf(att->phi);
   
    // Compute the distance to all the ground points.
    for(uint8_t i = 0; i < active_corners * POINTS_PER_EDGE; i++){

		rel_position_X = drone_X - ground_points_X[i];
		rel_position_Y = drone_Y - ground_points_Y[i];
                distance[i] = pow(pow(rel_position_X, 2) + pow(rel_position_Y, 2), 0.5);
                ground_points_indexes[i] = i;
                
    }
    
    // Sort the waypoints according to distance, from nearest fo furthest.
    sort_ground_points(distance, ground_points_indexes, active_corners * POINTS_PER_EDGE);  
    
    // Real world position of the selected ground point.
    double waypoint_X;
    double waypoint_Y;
      
    // Fine the furthest ground point marked as "safe". All "safe" points are checked, but "unsafe" points are skipped.
    for(uint8_t i = active_corners * POINTS_PER_EDGE - 1; i >= 0; i--){
        
        if (ground_points_safety[ground_points_indexes[i]]){
            waypoint_X = ground_points_X[ground_points_indexes[i]];
            waypoint_Y = ground_points_Y[ground_points_indexes[i]];
            active_ground_point_index = i;
            break;
        }
        
        if (i == 0){
            
            // If all ground points are unsafe, it means we are surrounded by obstacles. We keep looking for a safe path. TODO adding some way to get unstuck.
            reset_ground_safety();
            printf("Warning: No safe path could be found, starting again.\n");
            return 0;
            
        }
        
    }
    
    // Set the new position of the target waypoint so that the drone can turn in its direction and check if the path is safe or not. 
    waypoint_set_xy_i(waypoint, waypoint_X, waypoint_Y);
    
    // Compute the position of the ground point on the camera plane. Safety is only assessed if the drone is pointing to the ground point, because of the high
    // distortion of the camera, in that position we have the maximum angular resolution.
    double temp_point[2];
    compute_position_on_camera(drone_X - waypoint_X, drone_Y - waypoint_Y, temp_point, sin_heading, cos_heading, sin_pitch, cos_pitch, sin_roll, cos_roll);
    double active_ground_point_x = temp_point[0]; 
    
    // If we are pointing towards the waypoint, check for safety
    if (active_ground_point_x > 136 - 20 && active_ground_point_x < 136 + 20){
        
        for (int j = -1; j <= 2; j++){
            if (obst_vector[j + NUM_SECTORS / 2]){
                // There is an obstacle in the same sector, point is marked as unsafe.
                printf("Warning: Path is unsafe.\n");
                ground_points_safety[ground_points_indexes[active_ground_point_index]] = 0;
                return 0;
            }
        }
        
        printf("Path is safe.\n");
        return 1;
    } else {
        
        // We are not yet pointing towards the waypoint, let's wait... 
        return 0;
        
    }
            
}

// Check if an obstacle is inside the safety area. Called by the navigator to stop.
uint8_t obstacle_ahead(){
    /*
    if (obstacle_external && obstacle_internal){
        return 1;
    }
    */
    
    for (uint8_t i = NUM_SECTORS / 2 - 1; i <= NUM_SECTORS / 2 + 2; i++){
        if (obst_vector[i]){
            printf("Obstacle ahead!\n");
            return 1;
        }
    }
    
    
    return 0;
        
}


// Generate the ground points for the safety radius according to the specified parameters.
void generate_safety_radius(){
	
	double theta;
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS; i++){
		theta = (-safety_radius_width / 2.0 + i * safety_radius_width / (POINTS_SAFETY_RADIUS - 1)) * 3.14 / 180.0;
		safety_radius_external_X[i] = safety_radius_external * cosf(theta);
		safety_radius_external_Y[i] = safety_radius_external * sinf(theta);
		safety_radius_internal_X[i] = safety_radius_internal * cosf(theta);
		safety_radius_internal_Y[i] = safety_radius_internal * sinf(theta);
	}
}

// Draw the safety radius. The points on the safety radius are checked for obstacles. 
extern bool_t draw_safety_radius(struct image_t* img){


	*pixels_index = 0;
		
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS - 1; i++){
		point1.x = safety_radius_external_x[i];
		point1.y = safety_radius_external_y[i];
		point2.x = safety_radius_external_x[i + 1];
		point2.y = safety_radius_external_y[i + 1];
		
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
	}
	
	obstacle_external = 0;
	
	for(uint16_t i = 0; i < *pixels_index; i++){
		if (is_obstacle(*(pixels_array_border.pixels + i * 4 + 1), *(pixels_array_border.pixels + i * 4), *(pixels_array_border.pixels + i * 4 + 2)) && *(pixels_array_border.x + i) < safety_radius_external_x[0] && *(pixels_array_border.x + i) > safety_radius_external_x[POINTS_SAFETY_RADIUS - 1] ){
			obstacle_external++;
		}
	}
	
	*pixels_index = 0;
		
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS - 1; i++){
		point1.x = safety_radius_internal_x[i];
		point1.y = safety_radius_internal_y[i];
		point2.x = safety_radius_internal_x[i + 1];
		point2.y = safety_radius_internal_y[i + 1];
		
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
	}
	
	obstacle_internal = 0;
	
	for(uint16_t i = 0; i < *pixels_index; i++){
		if (is_obstacle(*(pixels_array_border.pixels + i * 4 + 1), *(pixels_array_border.pixels + i * 4), *(pixels_array_border.pixels + i * 4 + 2)) && *(pixels_array_border.x + i) < safety_radius_internal_x[0] && *(pixels_array_border.x + i) > safety_radius_internal_x[POINTS_SAFETY_RADIUS - 1] ){
			obstacle_internal++;
		}
	}
	
	
	if (obstacle_external > 10 && obstacle_internal > 10){
                obstacle_external = 1;
                obstacle_internal = 1;
                /*
		printf("Warning, obstacle inside safety zone!\n");
		*/
	} else {
             obstacle_external = 0;
                obstacle_internal = 0;
        }
            
	
	
	
	
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS - 1; i++){
		point1.x = safety_radius_external_x[i];
		point1.y = safety_radius_external_y[i];
		point2.x = safety_radius_external_x[i + 1];
		point2.y = safety_radius_external_y[i + 1];
		
		// Only draw points that fit in the image
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_draw_line_color(img, &point1, &point2, 255, 0, 148);
		}
	}
	
	
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS - 1; i++){
		point1.x = safety_radius_internal_x[i];
		point1.y = safety_radius_internal_y[i];
		point2.x = safety_radius_internal_x[i + 1];
		point2.y = safety_radius_internal_y[i + 1];
		
		// Only draw points that fit in the image
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_draw_line_color(img, &point1, &point2, 76, 84, 255);
		}
	}
	
	
	
	
	
	
	return FALSE;
	
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
		
		
		
                image_yuv422_colorfilt(img, img, (ground_y - y_width > 0) ? (uint8_t) (ground_y - y_width) : 0, (ground_y + y_width < 255) ?  (uint8_t)(ground_y + y_width) : 255, (ground_u - u_width > 0) ? (uint8_t) (ground_u - u_width) : 0, (ground_u + u_width < 255) ?  (uint8_t)(ground_u + u_width) : 255, (ground_v - v_width > 0) ? (uint8_t) (ground_v - v_width) : 0, (ground_v + v_width < 255) ?  (uint8_t)(ground_v + v_width) : 255);
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

// UPDATE GROUND SQUARE
void update_ground_square(void){

	double temp_point[2]; // Temporary point to store world2cam results

	struct FloatEulers *att = stateGetNedToBodyEulers_f(); // Drone attitude

	// Sines and cosines of attitude angles
	float sin_heading = sinf(-att->psi - heading_offset);
  	float cos_heading = cosf(-att->psi - heading_offset);
	float sin_pitch = -sinf(att->theta + THETA_OFF);
  	float cos_pitch = cosf(att->theta + THETA_OFF);
	float sin_roll = -sinf(att->phi);
  	float cos_roll = cosf(att->phi);
        
	// Cycle all ground points
  	for(uint8_t i = 0; i < active_corners * POINTS_PER_EDGE; i++){
                compute_position_on_camera(drone_X - ground_points_X[i], drone_Y - ground_points_Y[i], temp_point, sin_heading, cos_heading, sin_pitch, cos_pitch, sin_roll, cos_roll);
                ground_points_x[i] = temp_point[0]; 
                ground_points_y[i] = temp_point[1]; 
	}
	
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS; i++){
		compute_position_on_camera(-safety_radius_external_X[i], -safety_radius_external_Y[i], temp_point, 0, 1, sin_pitch, cos_pitch, sin_roll, cos_roll);
		safety_radius_external_x[i] = temp_point[0];
		safety_radius_external_y[i] = temp_point[1];
	} 
	
	for(uint8_t i = 0; i < POINTS_SAFETY_RADIUS; i++){
                compute_position_on_camera(-safety_radius_internal_X[i], -safety_radius_internal_Y[i], temp_point, 0, 1, sin_pitch, cos_pitch, sin_roll, cos_roll);
		safety_radius_internal_x[i] = temp_point[0];
		safety_radius_internal_y[i] = temp_point[1];
	}	
}


// DRAW GROUND SQUARE
bool_t draw_bound_square_func(struct image_t* img)
{ 
	*pixels_index = 0;
	int i = 0;
	// Cycle all ground points
  	for(i = 0; i < active_corners * POINTS_PER_EDGE - 1; i++){
		point1.x = ground_points_x[i];
		point1.y = ground_points_y[i];
		point2.x = ground_points_x[i + 1];
		point2.y = ground_points_y[i + 1];
		
		// Only draw points that fit in the image
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_extract_points_from_line(img, &point1, &point2, &pixels_array_border, pixels_index);	
		}
	}

	#ifndef VIDEO_STREAM
	#else

	for(i = 0; i < active_corners * POINTS_PER_EDGE - 1; i++){
		point1.x = ground_points_x[i];
		point1.y = ground_points_y[i];
		point2.x = ground_points_x[i + 1];
		point2.y = ground_points_y[i + 1];
		
		// Only draw points that fit in the image
		if (point1.x > 0 && point1.x < 272 && point1.y > 0 && point1.y < 272 && point2.x > 0 && point2.x < 272 && point2.y > 0 && point2.y < 272){
			image_draw_line_color(img, &point1, &point2, 149, 43, 21);
		}
	}

	#endif

	find_obstacles(img);

	// Don't know why it is here, but it was before
  	return FALSE;
}

// FIND OBSTACLES

void find_obstacles(struct image_t* img){

	// Cycle all pixels
	uint32_t i = 0;
        /*
	bool object_started = false;
	int object_ending_index = 0;
	int obstacle_index = 0;
        */
        
        for (i = 0; i < NUM_SECTORS; i++){
            obst_vector[i] = 0;
        }
        
        for(i = 0; i < *pixels_index / 4; i++){
		if (is_obstacle(*(pixels_array_border.pixels + i * 4 + 1), *(pixels_array_border.pixels + i * 4), *(pixels_array_border.pixels + i * 4 + 2)) && *(pixels_array_border.x + i) > 10 && *(pixels_array_border.x + i) < 262 ){
                     obst_vector[get_obst_vect_index(*(pixels_array_border.x + i))] = 1;
                }
        }
        
        
        /*
		
  	for(i = 0; i < *pixels_index; i ++){
		if (is_obstacle(*(pixels_array_border.pixels + i * 4 + 1), *(pixels_array_border.pixels + i * 4), *(pixels_array_border.pixels + i * 4 + 2)) && *(pixels_array_border.x + i) > 10 && *(pixels_array_border.x + i) < 262 ){
			if (!object_started){			
				if (obstacle_index < MAX_NUM_OBSTCLES){
					point1.x = *(pixels_array_border.x + i);
					point1.y = 10;
					point2.x = *(pixels_array_border.x + i);
					point2.y = 250;
					image_draw_line(img, &point1, &point2);	
					object_started = true;
					object_ending_index = i;
					obstacles[obstacle_index].start_x = *(pixels_array_border.x + i);
					
				} else {
                                    
                                            //printf("Warning: too many obstacles detected!\n");
				}
			} else {
				object_ending_index++;
			}
		} else {
			if (object_started && i - object_ending_index > 5){
				object_started = false;
				point1.x = *(pixels_array_border.x + i - 5);
				point1.y = 10;
				point2.x = *(pixels_array_border.x + i - 5);
				point2.y = 250;
				image_draw_line(img, &point1, &point2);	
				obstacles[obstacle_index].end_x = *(pixels_array_border.x + i - 5);
				obstacle_index++;
			} 	
		}
	}

	for (i = 0; i < obstacle_index; i++){
		
                
                uint8_t obstacle_start_index = get_obst_vect_index(obstacles[i].start_x);
                uint8_t obstacle_end_index = get_obst_vect_index(obstacles[i].end_x);
                
                for (uint8_t j = obstacle_start_index; j <= obstacle_end_index; j++){
                    
                    obst_vector[j] = 1;
                    
                }

		//printf("Obstacle %d from %f to %f \n", i + 1, atan2(start_optic[0], -start_optic[2]) * 180.0 / 3.14, atan2(end_optic[0], -end_optic[2]) * 180.0 / 3.14);
	} */
	
	for (i = 0; i < NUM_SECTORS; i++){
            printf("%d", obst_vector[i]);
        }
        printf("\n");

}

bool_t is_obstacle(double y, double u, double v){

	if (y < ground_y - y_width || y > ground_y + y_width || u < ground_u - u_width || u > ground_u + u_width || v < ground_v - v_width || v > ground_v + v_width) {
		return TRUE;
	} else {
		return FALSE;
	}

}

void generate_ground_points(){

	for (uint8_t i = 0; i < active_corners - 1; i++){
	
		double delta_X = (ground_corners[i + 1].X - ground_corners[i].X) / POINTS_PER_EDGE;
		double delta_Y = (ground_corners[i + 1].Y - ground_corners[i].Y) / POINTS_PER_EDGE;
		
		for (uint8_t j = 0; j < POINTS_PER_EDGE; j++){
			ground_points_X[i * POINTS_PER_EDGE + j] = ground_corners[i].X + delta_X * j;
			ground_points_Y[i * POINTS_PER_EDGE + j] = ground_corners[i].Y + delta_Y * j;
		}
			
	}
	
	double delta_X = (ground_corners[0].X - ground_corners[active_corners - 1].X) / POINTS_PER_EDGE;
	double delta_Y = (ground_corners[0].Y - ground_corners[active_corners - 1].Y) / POINTS_PER_EDGE;
	
	for (uint8_t j = 0; j < POINTS_PER_EDGE; j++){
		ground_points_X[(active_corners - 1) * POINTS_PER_EDGE + j] = ground_corners[active_corners - 1].X + delta_X * j;
		ground_points_Y[(active_corners - 1) * POINTS_PER_EDGE + j] = ground_corners[active_corners - 1].Y + delta_Y * j;
	}

}

void send_telemetry(struct transport_tx *trans, struct link_device *dev)
{
        uint8_t x = 0;
	pprz_msg_send_2016_AVOIDER(trans, dev, AC_ID, &x, &x, &obstacle_internal, &obstacle_external, &ground_y, &ground_u, &ground_v);
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
	cv_add(draw_bound_square_func);
	cv_add(compute_calibration_color);
        /*
	cv_add(draw_safety_radius);	
*/
	y_width = Y_FILTER_WIDTH;
	u_width = U_FILTER_WIDTH;
	v_width = V_FILTER_WIDTH;
	
        safety_radius_width = old_safety_radius_width = WIDTH_SAFETY_RADIUS;
        safety_radius_external = old_safety_radius_external = SAFETY_RADIUS_EXTERNAL;
        safety_radius_internal = old_safety_radius_internal = SAFETY_RADIUS_INTERNAL;
        
	generate_safety_radius();
        
        reset_ground_safety();
        
        for (uint8_t i = 0; i < NUM_SECTORS; i++){
            obst_vector[i] = 0;      
        }

        use_saved_corners();
        
        
	#if PERIODIC_TELEMETRY
	  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_2016_AVOIDER, send_telemetry);
	#endif

}

// PERIODIC
void periodic() {
	
	// Recompute the ground square position
	update_ground_square();
        
        if (safety_radius_width != old_safety_radius_width || safety_radius_external != old_safety_radius_external || safety_radius_internal != old_safety_radius_internal){
            generate_safety_radius();
            old_safety_radius_width = safety_radius_width;
            old_safety_radius_external = safety_radius_external;
            old_safety_radius_internal = safety_radius_internal;
        
        }
               

        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	drone_X = pos->x;
        drone_Y = pos->y;
        drone_Z = gps.hmsl / 1000.0 * 256.0;
        
        active_ground_point_index = active_corners * POINTS_PER_EDGE - 1;
        
        //printf("%d, %d \n", obstacle_internal, obstacle_external);

	
}



uint8_t add_corner_point()
{
	printf("Adding corner point...\n");
	return FALSE;
}

uint8_t reset_corners()
{
	printf("Resetting corner points...\n");
	return FALSE;
}

uint8_t use_saved_corners()
{
	printf("Loading saved corner points...\n");
	
	ground_corners[0].X = -800;
	ground_corners[0].Y = 061;
	
	ground_corners[1].X = 071;
	ground_corners[1].Y = 867;
	
	ground_corners[2].X = 900;
	ground_corners[2].Y = -050;
	
	ground_corners[3].X = -050;
	ground_corners[3].Y = -900;
	
	active_corners = 4;
	
	generate_ground_points();
	
	return FALSE;
}

uint8_t reset_heading()
{
	printf("Resetting heading...\n");

	heading_offset = ANGLE_FLOAT_OF_BFP(nav_heading);

	printf("Angle offset is %f: \n", heading_offset);

	return FALSE;
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



