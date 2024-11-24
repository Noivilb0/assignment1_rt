#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

turtlesim::Pose turtle1_pose, turtle2_pose;
ros::Publisher stop_pub;

void poseCallback1(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
}

void poseCallback2(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
}

float calculateDistance() {
    return std::sqrt(std::pow(turtle2_pose.x - turtle1_pose.x, 2) +
                     std::pow(turtle2_pose.y - turtle1_pose.y, 2));
}

void stopTurtle(const std::string& turtle_name) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    stop_pub = ros::NodeHandle().advertise<geometry_msgs::Twist>("/" + turtle_name + "/cmd_vel", 10);
    stop_pub.publish(stop_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Subscribers for turtle poses
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, poseCallback1);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, poseCallback2);

    // Publisher for distance
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("turtle_distance", 10);

    ros::Rate rate(10); // 10 Hz loop
    const float threshold_distance = 1.0; // Threshold for stopping
    const float boundary_limit = 10.0;

    while (ros::ok()) {
        ros::spinOnce();

        float distance = calculateDistance();
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        dist_pub.publish(distance_msg);

        // Stop turtles if conditions are met
        if (distance < threshold_distance) {
            ROS_WARN("Turtles are too close! Stopping moving turtle.");
            stopTurtle("turtle2");
        }
        if (turtle1_pose.x > boundary_limit || turtle1_pose.x < 0.0 ||
            turtle1_pose.y > boundary_limit || turtle1_pose.y < 0.0 ||
            turtle2_pose.x > boundary_limit || turtle2_pose.x < 0.0 ||
            turtle2_pose.y > boundary_limit || turtle2_pose.y < 0.0) {
            ROS_WARN("Turtle near boundary! Stopping moving turtle.");
            stopTurtle("turtle2");
        }

        rate.sleep();
    }

    return 0;
}

