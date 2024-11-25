#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include <cmath>

ros::Publisher cmd_pub;
ros::Publisher distance_pub;
ros::Subscriber pose_sub1, pose_sub2;

turtlesim::Pose pose_turtle1;
turtlesim::Pose pose_turtle2;

double stop_distance = 0.5; // Distance threshold to stop turtles
double boundary_limit = 10.0; // Boundary limit for x and y positions

// Forward declaration of stopTurtle() function
void stopTurtle();

// Callback function for turtle1 pose
void poseCallback1(const turtlesim::Pose::ConstPtr& msg) {
    pose_turtle1 = *msg;
}

// Callback function for turtle2 pose
void poseCallback2(const turtlesim::Pose::ConstPtr& msg) {
    pose_turtle2 = *msg;
}

// Function to stop turtle2
void stopTurtle() {
    geometry_msgs::Twist stop_msg;
    cmd_pub.publish(stop_msg);  // Publish a zero velocity to stop the turtle
}

// Function to check distance between turtles and boundaries
void checkDistanceAndBoundaries() {
    // Calculate the Euclidean distance between the two turtles
    double distance = std::sqrt(std::pow(pose_turtle1.x - pose_turtle2.x, 2) +
                                std::pow(pose_turtle1.y - pose_turtle2.y, 2));

    // Publish the calculated distance on /turtle_distance topic
    std_msgs::Float32 dist_msg;
    dist_msg.data = distance;
    distance_pub.publish(dist_msg);

    // Check if the turtles are too close
    if (distance < stop_distance) {
        ROS_WARN("Turtles are too close! Stopping turtle2.");
        stopTurtle();
    }

    // Check if the turtles are out of bounds
    if (pose_turtle2.x > boundary_limit || pose_turtle2.x < 1.0 ||
        pose_turtle2.y > boundary_limit || pose_turtle2.y < 1.0) {
        ROS_WARN("Turtle2 is out of bounds! Stopping turtle2.");
        stopTurtle();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Create a publisher for turtle2's velocity
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Create a publisher for the distance (std_msgs/Float32)
    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);

    // Subscribe to the poses of both turtles
    pose_sub1 = nh.subscribe("/turtle1/pose", 10, poseCallback1);
    pose_sub2 = nh.subscribe("/turtle2/pose", 10, poseCallback2);

    ros::Rate loop_rate(10); // Set loop rate to 10 Hz

    while (ros::ok()) {
        ros::spinOnce();  // Process any incoming messages

        // Check the distance and boundaries
        checkDistanceAndBoundaries();

        loop_rate.sleep();
    }

    return 0;
}

