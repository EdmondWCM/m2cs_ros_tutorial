#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <serial/serial.h>
#include <sstream>
#include "motor_driver/pos_time.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "motor_pub");

  ros::NodeHandle n;

  ros::Publisher p_pub = n.advertise<motor_driver::pos_time>("/motor_driver/p_setpoint", 1);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {

    motor_driver::pos_time msg;
    if (count %2 == 0){
      msg.pos = 0;
      msg.time = 2000;
    }
    else {
      msg.pos = 500000;
      msg.time = 2000;
    }




    p_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    if (count == 5){
      ros::shutdown();
    }
  }

  return 0;
}