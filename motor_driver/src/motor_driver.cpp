#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <serial/serial.h>
#include <sstream>
#include "motor_driver/pos_time.h"
#include <fstream>
using namespace std;
std::string old_command = "";
std::string command = "f \r\n";

// if the command is p, run this callback.
void pCallback(const motor_driver::pos_time::ConstPtr &value)
{
    command = "p ";
    int position = value->pos;
    int time = value->time;
    command += std::to_string(position);
    command += " ";
    command += std::to_string(time);
    command += "\r\n";
    ROS_INFO("command changed");

    return;
}

//  if the command is v, run this callback.
void vCallback(const std_msgs::Int32::ConstPtr &value)
{
    command = "v ";
    int temp = value->data;
    command += std::to_string(temp);
    command += "\r\n";
    ROS_INFO("command changed");

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::Time::init();
    ros::Rate loop_rate(100); // if Arduino is too slow, may have to lower this rate
    ros::NodeHandle nh("~");

    // create subscribers and publishers here
    ros::Publisher p_pub = nh.advertise<std_msgs::Int32>("p_feedback", 1);
    ros::Publisher v_pub = nh.advertise<std_msgs::Int32>("v_feedback", 1);
    ros::Publisher pexp_pub = nh.advertise<std_msgs::Int32>("p_expect", 1);
    ros::Publisher vexp_pub = nh.advertise<std_msgs::Int32>("v_expect", 1);
    ros::Publisher aexp_pub = nh.advertise<std_msgs::Int32>("a_expect", 1);
    ros::Publisher jerkexp_pub = nh.advertise<std_msgs::Int32>("jerk_expect", 1);
    ros::Publisher i_pub = nh.advertise<std_msgs::Int32>("i_feedback", 1);
    ros::Subscriber p_sub = nh.subscribe("p_setpoint", 1, pCallback);
    ros::Subscriber v_sub = nh.subscribe("v_setpoint", 1, vCallback);

    serial::Serial motor("/dev/ttyUSB0", 1000000);

    ofstream fout;

    if (motor.isOpen())
    {
        ROS_INFO("Serial opened.");
    }
    else
    {
        ROS_INFO("Serial failed.");
    }

    while (ros::ok())
    {

        // subscriber callback should update the command, do the send here
        motor.write(command);
        if (command != "f \r\n")
        {
            command = "f \r\n";
        }

        string feedback = motor.readline(); // read until ‘\n’
        if (feedback.size())
        {
            ROS_INFO("feedback received");
            std::cout << "read: " << feedback;
            // --------------- debug ---------------
            fout.open("/home/edmond/Desktop/data.txt", ios::app);
            fout << feedback; // add feedback into txt file
            fout.close();
            // --------------- debug ---------------
            std_msgs::Int32 p;
            std_msgs::Int32 v;
            std_msgs::Int32 pexp;
            std_msgs::Int32 vexp;
            std_msgs::Int32 aexp;
            std_msgs::Int32 jerkexp;
            std_msgs::Int32 t;
            std_msgs::Int32 i;
            stringstream ss;

            ss.clear();
            ss << feedback;
            ss >> p.data >> v.data >> i.data >> pexp.data >> vexp.data >> aexp.data >> jerkexp.data;

            p_pub.publish(p);
            v_pub.publish(v);
            i_pub.publish(i);
            pexp_pub.publish(pexp);
            vexp_pub.publish(vexp);
            aexp_pub.publish(aexp);
            jerkexp_pub.publish(jerkexp);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
