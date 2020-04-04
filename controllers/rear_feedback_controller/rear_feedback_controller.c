/*
 * File:          rear_feedback_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/supervisor.h>
#include <webots/pen.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * You may want to add macros here.
 */
// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 20
#define UNKNOWN 99999.99

#define TIME_STEP 50

#define LABEL_X 0.05
#define LABEL_Y 0.95
#define GREEN 0x008800

#define SAMPLE_STEP ( 0.01 )
#define M_PI        ( 3.1415926535897932384626433832795 )
#define RADIUS      ( 10.0 )

typedef struct _TargetTrack
{
    double x;
    double y;
    double yaw;
    double curvature;
    double velocity;
}TargetTrack;

TargetTrack TargetCurvature[(unsigned int)(M_PI/SAMPLE_STEP)+1];


// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;
bool has_pen = false;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// Pen
WbDeviceTag pen;
int color, red, green, blue;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

void update_display() {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_supervisor_set_label(5, txt, LABEL_X, LABEL_Y, 0.07, GREEN, 0, "Arial");
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
  
  
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double speed_ms = wb_gps_get_speed(gps);
  // store into global variables
  gps_speed = speed_ms * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
	unsigned int i,max_cnt;
  	double index = 0;
	double first_derivative_x,first_derivative_y;
    double second_derivative_x,second_derivative_y;
	/* necessary to initialize webots stuff */
	wbu_driver_init();
	max_cnt = (unsigned int)(M_PI/SAMPLE_STEP)+1;
	index = 0;
	for(i=0;i<max_cnt;i++)
	{
		TargetCurvature[i].x = RADIUS * cos(index);
		TargetCurvature[i].y = RADIUS * sin(index);
		index = index +  SAMPLE_STEP;                                     
	}
	for(i=0;i<max_cnt;i++)
	{
		if(i == 0)
		{
			first_derivative_x = TargetCurvature[2].x - TargetCurvature[1].x;
        	first_derivative_y = TargetCurvature[2].y - TargetCurvature[1].y;
        	second_derivative_x = TargetCurvature[2].x + TargetCurvature[0].x - 2 * TargetCurvature[1].x;
        	second_derivative_y = TargetCurvature[2].y + TargetCurvature[0].y - 2 * TargetCurvature[1].y;
		}
		else if(i == (max_cnt - 1))
		{
			first_derivative_x = TargetCurvature[max_cnt-1].x - TargetCurvature[max_cnt-2].x;
        	first_derivative_y = TargetCurvature[max_cnt-1].y - TargetCurvature[max_cnt-2].y;
        	second_derivative_x = TargetCurvature[max_cnt-1].x + TargetCurvature[max_cnt-3].x - 2 * TargetCurvature[max_cnt-2].x;
        	second_derivative_y = TargetCurvature[max_cnt-1].y + TargetCurvature[max_cnt-3].y - 2 * TargetCurvature[max_cnt-2].y;
		}
		else
		{
			first_derivative_x = TargetCurvature[i+1].x - TargetCurvature[i].x;
        	first_derivative_y = TargetCurvature[i+1].y - TargetCurvature[i].y;
        	second_derivative_x = TargetCurvature[i+1].x + TargetCurvature[i-1].x - 2 * TargetCurvature[i].x;
        	second_derivative_y = TargetCurvature[i+1].y + TargetCurvature[i-1].y - 2 * TargetCurvature[i].y;
		}
		TargetCurvature[i].yaw = atan2(first_derivative_y,first_derivative_x);
		TargetCurvature[i].curvature = (second_derivative_y*first_derivative_x - second_derivative_x*first_derivative_y)
                                           / pow(pow(first_derivative_x,2)+pow(first_derivative_y,2),1.5);
	}
	/*
	 * You should declare here WbDeviceTag variables for storing
	 * robot devices like this:
	 *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
	 *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
	 */
     // check if there is a SICK and a display
	int j = 0;
	for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
		WbDeviceTag device = wb_robot_get_device_by_index(j);
		const char *name = wb_device_get_name(device);
		if (strcmp(name, "Sick LMS 291") == 0)
		{
			enable_collision_avoidance = true;
		}
		else if (strcmp(name, "display") == 0)
		{
			enable_display = true;
			printf("display enable!\r\n");
		}
		else if (strcmp(name, "gps") == 0)
		{
			has_gps = true;
			printf("gps enable!\r\n");
		}
		else if (strcmp(name, "camera") == 0)
		{
			has_camera = true;
			printf("camera enable!\r\n");
		}
		else if(strcmp(name, "pen") == 0)
		{
			has_pen = true;
		}
	}
  
	// camera device
	if (has_camera) {
		camera = wb_robot_get_device("camera");
		wb_camera_enable(camera, TIME_STEP);
		camera_width = wb_camera_get_width(camera);
		camera_height = wb_camera_get_height(camera);
		camera_fov = wb_camera_get_fov(camera);
	}
  
	// initialize gps
	if (has_gps) {
		gps = wb_robot_get_device("gps");
		wb_gps_enable(gps, TIME_STEP);
	}
	
	if(has_pen)
	{
		pen = wb_robot_get_device("pen");
		wb_pen_write(pen,true); 
		red = 0;
		green = 0;
		blue = 0xff;
		color = (((red << 8) | green) << 8) | blue;
		wb_pen_set_ink_color(pen,color,1.0);
	}

	// initialize display (speedometer)
	if (enable_display) {
		display = wb_robot_get_device("display");
		speedometer_image = wb_display_image_load(display, "speedometer.png");
	}
  
	wbu_driver_set_cruising_speed(1);
	wbu_driver_set_hazard_flashers(true);
	wbu_driver_set_dipped_beams(true);
	wbu_driver_set_antifog_lights(true);
	wbu_driver_set_wiper_mode(SLOW);
	wbu_driver_set_brake_intensity(0.0);
	/* main loop
	 * Perform simulation steps of TIME_STEP milliseconds
	 * and leave the loop when the simulation is over
	 */
	while (wbu_driver_step(TIME_STEP) != -1) {
		static int i = 0;
		// updates sensors only every TIME_STEP milliseconds
		if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
		/*
		 * Read the sensors :
		 * Enter here functions to read sensor data, like:
		 *  double val = wb_distance_sensor_get_value(my_sensor);
		 */
		const unsigned char *camera_image = NULL;
      
		if (has_camera){
			camera_image = wb_camera_get_image(camera);
		}
		/* Process sensor data here */

		// update stuff
		if (has_gps) { compute_gps_speed();}
		if (enable_display) { update_display(); }
		//if (has_pen) { wb_pen_write(pen,true); }

		/*
		 * Enter here functions to send actuator commands, like:
		 * wb_motor_set_position(my_actuator, 10.0);
		 */
		}
		++i;
	}

	/* Enter your cleanup code here */

	/* This is necessary to cleanup webots resources */
	wbu_driver_cleanup();

	return 0;
}
