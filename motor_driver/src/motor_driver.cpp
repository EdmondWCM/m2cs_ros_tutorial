#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <serial/serial.h>
#include <sstream>


using namespace std;

std::string command = "f";
// I have no idea how to add the (Int32) data to the (string) command.
void pCallback(const std_msgs::Int32::ConstPtr& value){
    command = "p ";
    int temp= value->data;
    command += std::to_string(temp);
    return;
}

void vCallback(const std_msgs::Int32::ConstPtr& value){
    command = "v";
    int temp= value->data;
    command += std::to_string(temp);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::Time::init();
    ros::Rate loop_rate(100); // if Arduino is too slow, may have to lower this rate
    ros::NodeHandle nh("~");

    // create subscribers and publishers here
    ros::Publisher p_pub = nh.advertise<std_msgs::Int32>("p_feedback",1);
    ros::Publisher v_pub = nh.advertise<std_msgs::Int32>("v_feedback",1);
    ros::Publisher i_pub = nh.advertise<std_msgs::Int32>("i_feedback",1);
    ros::Subscriber p_sub = nh.subscribe("p_setpoint",1,pCallback);
    ros::Subscriber v_sub = nh.subscribe("v_setpoint",1,vCallback);

    serial::Serial motor("/dev/ttyUSB0", 115200);

    if (motor.isOpen()) {
        ROS_INFO("Serial opened.");
    } else {
        ROS_INFO("Serial failed.");
    }

    while (ros::ok()) {
        // subscriber callback should update the command, do the send here

        motor.write(command);

        string feedback = motor.readline(); // read until ‘\n’
        if (feedback.size() > 2) {
            std::cout << "read: " << feedback;
            // parse the feedback (3 int separated by space) and publish here
            string p="";
            string v="";
            string i="";
            stringstream ss;

            std_msgs::Int32 convertedp;
            std_msgs::Int32 convertedv;
            std_msgs::Int32 convertedi;
            int count=0;
            for(int i = 0; i<feedback.size();i++){
                if (count == 0){
                    if(feedback[i] ==' ' ){
                        count++;
                    }else{
                        p+=feedback[i];
                    }
                }
                if (count == 1){
                    if(feedback[i] ==' ' ){
                        count++;
                    }else{
                        v+=feedback[i];
                    }
                }
                if (count == 2){
                    i+=feedback[i];
                }
            }
            ss << p;
            ss >> convertedp.data;
            p_pub.publish(convertedp);
            ss << v;
            ss >> convertedv.data;
            v_pub.publish(convertedv);
            ss << i;
            ss >> convertedi.data;
            i_pub.publish(convertedi);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
