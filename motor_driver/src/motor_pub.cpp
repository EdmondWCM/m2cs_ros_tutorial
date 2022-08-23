#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <serial/serial.h>
#include <sstream>
#include "motor_driver/pos_time.h"
#include "motor_driver/pos_vel.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "motor_pub");

  ros::NodeHandle n;

  ros::Publisher p_pub = n.advertise<motor_driver::pos_vel>("/motor_driver/pv_setpoint", 1);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {

    motor_driver::pos_vel msg;
    if (count %2 == 0){
      msg.pos = 0;
      msg.vel = 2000;
    }
    else {
      msg.pos = 500000;
      msg.vel = 2000;
    }




    p_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    if (count == 3){
      ros::shutdown();
    }
  }

  return 0;
}