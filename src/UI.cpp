#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

void sendCommand(const std::string& turtle_name, double linear, double angular) {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/" + turtle_name + "/cmd_vel", 10);
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    // Publish command for 1 second
    for (int i = 0; i < 10; ++i) {
        pub.publish(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the turtle
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub.publish(cmd);
}

int main(int argc, char** argv) {