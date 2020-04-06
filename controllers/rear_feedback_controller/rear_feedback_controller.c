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
#include <webots/vehicle/car.h>
#include <webots/supervisor.h>
#include <webots/pen.h>
#include <webots/utils/system.h>

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

#define LABEL_X 0.05
#define LABEL_Y 0.95
#define GREEN 0x008800

#define SAMPLE_STEP ( 0.01 )
//#define M_PI        ( 3.1415926535897932384626433832795 )
#define RADIUS      ( 8.0 )

#define COEFFICIENT_KPSI	( 1.5f  )   // K_psi
#define COEFFICIENT_KE		( 1.0f  )   // K_e

typedef struct _TargetTrack
{
    double x;
    double y;
    double yaw;
    double curvature;
    double velocity;
}TargetTrack;

TargetTrack TargetCurvature[(unsigned int)(M_PI/SAMPLE_STEP)+1];
unsigned int max_cnt;

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

// car
double wheel_base;
double last_steering_angle = 0.0;

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

double set_steering_angle(double wheel_angle,double rate)
{
	// limit the steering anigle rate
	if(wheel_angle - last_steering_angle > rate)
	{
		wheel_angle = last_steering_angle + rate;
	}
	else if(wheel_angle - last_steering_angle < -rate)
	{
		wheel_angle = last_steering_angle - rate;
	}
	else
	{
		wheel_angle = last_steering_angle;
	}
	// limit the range of steering angle 
	return wheel_angle > 0.5 ? 0.5 : wheel_angle < -0.5 ? -0.5 : wheel_angle;
}

double NormalizeAngle(double angle)
{
	double a = fmod(angle + M_PI,2.0 * M_PI);
	if(a < 0.0)
	{
		a += 2.0 * M_PI;
	}
	return a - M_PI;
}

void GenerateCurvatureSets()
{
	unsigned int i;
  	double index = 0;
	double first_derivative_x,first_derivative_y;
    double second_derivative_x,second_derivative_y;

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
}

TargetTrack CalculateNearestPointByPosition(double x, double y)
{
	unsigned int i,index;
	double distance;
	double min_dis;

	index = 0;
	min_dis = distance = sqrt(pow(TargetCurvature[index].x - x,2.0) + pow(TargetCurvature[index].y - y,2.0));
	for(i=0;i<max_cnt;i++)
	{
		distance = sqrt(pow(TargetCurvature[i].x - x,2.0) + pow(TargetCurvature[i].y - y,2.0)); 
		if(distance < min_dis)
		{
			min_dis = distance;
			index = i;
		}                                     
	}
	return TargetCurvature[index];
}

void RearWheelControl(TargetTrack target,double x,double y,double yaw)
{
	double vecter_d_x,vecter_d_y;
	double vecter_t_x,vecter_t_y;
	double yaw_err,cro_err;
	double v_x,k;
	double psi_omega,delta_ctl;

	vecter_d_x = x - target.x;
	vecter_d_y = y - target.y;
	int gear = wbu_driver_get_gear_number();
	// printf("Gear:%d\r\n",gear);

	vecter_t_x = cos(target.yaw);
	vecter_t_y = sin(target.yaw);

	yaw_err = NormalizeAngle(yaw - target.yaw);

	v_x = wbu_driver_get_current_speed();

	k = target.curvature;

	cro_err = vecter_t_x * vecter_d_y - vecter_t_y *  vecter_d_x;

	psi_omega = v_x * k * cos(yaw_err)/(1.0 - k * cro_err)
              - COEFFICIENT_KE   * v_x  * sin(yaw_err) * cro_err / yaw_err
              - COEFFICIENT_KPSI * fabs(v_x) * yaw_err;

	if(fabs(psi_omega) < 1.0e-6f || fabs(yaw_err) < 1.0e-6f)
	{
		wbu_driver_set_steering_angle(set_steering_angle(0.0,0.5));
	}
	else
	{
        delta_ctl = atan(psi_omega * wheel_base / v_x);
        delta_ctl = delta_ctl > 0.54 ? 0.54 : delta_ctl < -0.54 ? -0.54:delta_ctl;
		wbu_driver_set_steering_angle(-set_steering_angle(delta_ctl,0.5));
	}
}
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {

	TargetTrack target_curvature;
	/* necessary to initialize webots stuff */
	wbu_driver_init();
	
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
  
    // 获取vehicle.translation 属性
	WbNodeRef vehicle = wb_supervisor_node_get_from_def("BMW");
	WbFieldRef translationField = wb_supervisor_node_get_field(vehicle, "translation");
	WbFieldRef rotationField    = wb_supervisor_node_get_field(vehicle, "rotation");

	wheel_base = wbu_car_get_wheelbase();
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
  
	GenerateCurvatureSets();

	wbu_driver_set_cruising_speed(36);
	wbu_driver_set_hazard_flashers(false);
	wbu_driver_set_dipped_beams(false);
	wbu_driver_set_antifog_lights(false);
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

		const double *translation = wb_supervisor_field_get_sf_vec3f(translationField);
		const double *rotation = wb_supervisor_field_get_sf_rotation(rotationField);
		
		printf("x:%f,y:%f,yaw:%f\r\n",translation[2],translation[0],rotation[3]);
		/* Process sensor data here */
		target_curvature = CalculateNearestPointByPosition(translation[2],translation[0]);
		printf("target x:%f,y:%f,yaw:%f\r\n",target_curvature.x,target_curvature.y,target_curvature.yaw);
		// wbu_driver_set_steering_angle(-0.5);
                      RearWheelControl(target_curvature,translation[2],translation[0],rotation[3]);
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
