/*
 * File:          pen_controller.c
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
#include <webots/supervisor.h>
#include <webots/pen.h>

#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 1

#define SAMPLE_STEP ( 0.01 )
#define RADIUS      ( 8.0 )

// Pen
WbDeviceTag pen;
int color, red, green, blue;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  double translation[3];
  unsigned int max_cnt,i;
  double index;
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   WbDeviceTag pen_robot = wb_robot_get_device("pen_robot");
   
   WbNodeRef target_pen = wb_supervisor_node_get_from_def("TARGET_PEN");
   WbFieldRef translationField = wb_supervisor_node_get_field(target_pen, "translation");
   
   translation[0] = 0.0;
   translation[1] = 0.09;
   translation[2] = 0.0;
   
    max_cnt = (unsigned int)(M_PI/SAMPLE_STEP)+1;
    index = 0.0;
    // for(i=0;i<max_cnt;i++)
    // {
      // translation[2] = RADIUS * cos(index);
      // translation[0] = RADIUS * sin(index);
      // index = index +  SAMPLE_STEP;  
      // wb_supervisor_field_set_sf_vec3f(translationField,translation);                                   
    // }
    
    pen = wb_robot_get_device("pen");
    wb_pen_write(pen,true); 
    red = 0;
    green = 0xff;
    blue = 0;
    color = (((red << 8) | green) << 8) | blue;
    wb_pen_set_ink_color(pen,color,1.0);
  
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
     // for(i=0;i<max_cnt;i++)
     // {
    if(index < M_PI)
    {
      translation[2] = RADIUS * cos(index);
      translation[0] = RADIUS * sin(index);
      index = index +  SAMPLE_STEP;  
      wb_supervisor_field_set_sf_vec3f(translationField,translation);  
    }
      // }
      
    /* Process sensor data here */

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
