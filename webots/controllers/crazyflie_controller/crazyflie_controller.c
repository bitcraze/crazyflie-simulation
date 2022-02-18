#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64


int main(int argc, char **argv) {
  wb_robot_init();




          WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
    wb_motor_set_position(m1_motor, INFINITY);
    
          WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
    wb_motor_set_position(m2_motor, INFINITY);
    
    
     WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
    wb_motor_set_position(m3_motor, INFINITY);
    
          WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
    wb_motor_set_position(m4_motor, INFINITY);
    
    float speed=0;

  while (wb_robot_step(TIME_STEP) != -1) {

    
    wb_motor_set_velocity(m1_motor,-speed);
    wb_motor_set_velocity(m2_motor,speed);
    wb_motor_set_velocity(m3_motor, -speed);
    wb_motor_set_velocity(m4_motor,speed);
    
    speed = speed+0.1;


  };


  wb_robot_cleanup();

  return 0;
}

