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
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Spawn turtle2
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 2;
    spawn_srv.request.y = 2;
    spawn_srv.request.name = "turtle2";

    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Successfully spawned turtle2.");
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
        return 1;
    }

    // Main loop for user commands
    while (ros::ok()) {
        std::string turtle_name;
        double linear, angular;

        std::cout << "Enter the turtle to control (turtle1/turtle2): ";
        std::cin >> turtle_name;
        if (turtle_name != "turtle1" && turtle_name != "turtle2") {
            std::cout << "Invalid turtle name. Please try again.\n";
            continue;
        }

        std::cout << "Enter linear velocity: ";
        std::cin >> linear;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        sendCommand(turtle_name, linear, angular);
    }

    return 0;
}
