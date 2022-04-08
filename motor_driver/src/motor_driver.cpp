#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <serial/serial.h>
#include <sstream>


using namespace std;
std::string old_command = "";
std::string command = "f \n";
// I have no idea how to add the (Int32) data to the (string) command.
void pCallback(const std_msgs::Int32::ConstPtr& value){
    command = "p ";
    int temp= value->data;
    command += std::to_string(temp);
    command+='\n';
    ROS_INFO("command changed");

    return;
}

void vCallback(const std_msgs::Int32::ConstPtr& value){
    command = "v ";
    int temp= value->data;
    command += std::to_string(temp);
    command+='\n';
    ROS_INFO("command changed");

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::Time::init();
    ros::Rate loop_rate(1); // if Arduino is too slow, may have to lower this rate
    ros::NodeHandle nh("~");

    // create subscribers and publishers here
    ros::Publisher p_pub = nh.advertise<std_msgs::Int32>("p_feedback",1);
    ros::Publisher v_pub = nh.advertise<std_msgs::Int32>("v_feedback",1);
    ros::Publisher i_pub = nh.advertise<std_msgs::Int32>("i_feedback",1);
    ros::Subscriber p_sub = nh.subscribe("p_setpoint",1,pCallback);
    ros::Subscriber v_sub = nh.subscribe("v_setpoint",1,vCallback);

    serial::Serial motor("/dev/ttyUSB0", 1000000);

    if (motor.isOpen()) {
        ROS_INFO("Serial opened.");
    } else {
        ROS_INFO("Serial failed.");
    }

    while (ros::ok()) {
        
        // subscriber callback should update the command, do the send here
        motor.write(command);
        old_command = command;
        if (command != "f \n"){
            command = "f \n";
            ROS_INFO("command sent and changed back to \'f\'");
        }
        
        string feedback = motor.readline(); // read until ‘\n’
        if (feedback.size() > 2 && feedback!=old_command) {
            ROS_INFO("feedback received");
            std::cout << "read: " << feedback;
            // parse the feedback (3 int separated by space) and publish here
            std_msgs::Int32 p;
            std_msgs::Int32 v;
            std_msgs::Int32 i;
            stringstream ss;


            
            ss << feedback;
            ss >> p.data >> v.data >> i.data;
            
            p_pub.publish(p);
            v_pub.publish(v);
            i_pub.publish(i);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
