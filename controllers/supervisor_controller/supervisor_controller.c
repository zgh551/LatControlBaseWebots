/*
 * File:          supervisor_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/display.h>
#include <webots/supervisor.h>
#include <webots/utils/system.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define GROUND_X 30.0
#define GROUND_Z 30.0

#define LIGHT_GRAY 0x505050
#define RED 0xBB2222
#define GREEN 0x22BB11
#define BLUE 0x2222BB

#define SAMPLE_STEP ( 0.01 )
#define M_PI        ( 3.1415926535897932384626433832795 )
#define RADIUS      ( 8.0 )

typedef struct _TargetTrack
{
    double x;
    double y;
    double yaw;
    double curvature;
    double velocity;
}TargetTrack;

TargetTrack TargetCurvature[(unsigned int)(M_PI/SAMPLE_STEP)+1];

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  unsigned int i,max_cnt;
  double index = 0;
  /* necessary to initialize webots stuff */
  wb_robot_init();


  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag ground_display = wb_robot_get_device("ground_display");

  // get the properties of the Display
  int width  = wb_display_get_width(ground_display);
  int height = wb_display_get_height(ground_display);
  
  printf("width:%d,height:%d\r\n",width,height);
  // 获取vehicle.translation 属性
  WbNodeRef vehicle = wb_supervisor_node_get_from_def("BMW");
  WbFieldRef translationField = wb_supervisor_node_get_field(vehicle, "translation");

  // paint the display's background
  wb_display_set_color(ground_display, 0x010101);
  wb_display_fill_rectangle(ground_display, 0, 0, width, height);
  wb_display_set_color(ground_display, BLUE);
  wb_display_draw_line(ground_display, 0, height / 2, width - 1, height / 2);
  //wb_display_draw_text(ground_display, "x", width - 10, height / 2 - 10);
  wb_display_set_color(ground_display, BLUE);
  wb_display_draw_line(ground_display, width / 2, 0, width / 2, height - 1);
  //wb_display_draw_text(ground_display, "z", width / 2 - 10, height - 10);
  max_cnt = (unsigned int)(M_PI/SAMPLE_STEP)+1;
  index = 0;
  wb_display_set_color(ground_display, GREEN);
  for(i=0;i<max_cnt;i++)
  {
    TargetCurvature[i].x = RADIUS * cos(index);
    TargetCurvature[i].y = RADIUS * sin(index);
    wb_display_draw_pixel(ground_display, width  * (TargetCurvature[i].y + GROUND_X / 2) / GROUND_X,
                                          height * (TargetCurvature[i].x + GROUND_Z / 2) / GROUND_Z);
    index = index +  SAMPLE_STEP;                                     
  }

  // init image ref used to save into the image file
  WbImageRef to_store = NULL;

  // init a variable which counts the time steps
  int counter = 0;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    const double *translation = wb_supervisor_field_get_sf_vec3f(translationField);
    //printf("x:%f,y:%f,z:%f\r\n",translation[0],translation[1],translation[2]);
    // Update the counter
    counter++;

    // display the robot position
    wb_display_set_opacity(ground_display, 0.9);
    wb_display_set_color(ground_display, RED);
    wb_display_draw_pixel(ground_display, width * (translation[0] + GROUND_X / 2) / GROUND_X,
                         height * (translation[2] + GROUND_Z / 2) / GROUND_Z);

    // Clear previous to_store
    if (to_store) {
      wb_display_image_delete(ground_display, to_store);
      to_store = NULL;
    }
    /* Process sensor data here */
    // Every 50 steps, store the resulted image into a file
    if (counter % 50 == 0) {
      to_store = wb_display_image_copy(ground_display, 0, 0, width, height);
      // compute the path to store the image in user directory
      char *filepath;
#ifdef _WIN32
      const char *user_directory = wbu_system_short_path(wbu_system_getenv("USERPROFILE"));
      filepath = (char *)malloc(strlen(user_directory) + 16);
      strcpy(filepath, user_directory);
      strcat(filepath, "\\screenshot.png");
#else
      const char *user_directory = wbu_system_getenv("HOME");
      filepath = (char *)malloc(strlen(user_directory) + 16);
      strcpy(filepath, user_directory);
      strcat(filepath, "/screenshot.png");
#endif
      wb_display_image_save(ground_display, to_store, filepath);
      free(filepath);
    }
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
