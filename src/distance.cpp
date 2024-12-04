#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

class DistanceNode {
public:
    DistanceNode() {
        ros::NodeHandle nh;

        // Subscribers for turtle1 and turtle2 poses
        turtle1_pose_sub_ = nh.subscribe("/turtle1/pose", 10, &DistanceNode::turtle1PoseCallback, this);
        turtle2_pose_sub_ = nh.subscribe("/turtle2/pose", 10, &DistanceNode::turtle2PoseCallback, this);

        // Publisher for distance
        distance_pub_ = nh.advertise<std_msgs::Float32>("/turtles/distance", 10);

        // Publisher to stop the turtle
        turtle1_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        turtle2_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

        // Initialize threshold values
        distance_threshold_ = 1.0;
        boundary_min_ = 1.0;
        boundary_max_ = 10.0;
    }

    void spin() {
        ros::Rate rate(100);
        while (ros::ok()) {
            publishDistance();
            checkProximity();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber turtle1_pose_sub_;
    ros::Subscriber turtle2_pose_sub_;
    ros::Publisher distance_pub_;
    ros::Publisher turtle1_cmd_pub_;
    ros::Publisher turtle2_cmd_pub_;

    turtlesim::Pose turtle1_pose_;
    turtlesim::Pose turtle2_pose_;

    double distance_threshold_;
    double boundary_min_;
    double boundary_max_;

    void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
        turtle1_pose_ = *msg;
    }

    void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
        turtle2_pose_ = *msg;
    }

    void publishDistance() {
        std_msgs::Float32 distance_msg;
        distance_msg.data = calculateDistance(turtle1_pose_, turtle2_pose_);
        distance_pub_.publish(distance_msg);
    }

    double calculateDistance(const turtlesim::Pose& pose1, const turtlesim::Pose& pose2) {
        double dx = pose1.x - pose2.x;
        double dy = pose1.y - pose2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void checkProximity() {
        double distance = calculateDistance(turtle1_pose_, turtle2_pose_);

        // Stop turtles if they are too close
        if (distance < distance_threshold_) {
            stopTurtles();
            ROS_WARN("Turtles are too close. Movement stopped.");
        }

        // Stop turtles if they are close to boundaries
        if (isOutOfBounds(turtle1_pose_) || isOutOfBounds(turtle2_pose_)) {
            stopTurtles();
            ROS_WARN("Turtle near boundary. Movement stopped.");
        }
    }

    bool isOutOfBounds(const turtlesim::Pose& pose) {
        return pose.x < boundary_min_ || pose.x > boundary_max_ ||
               pose.y < boundary_min_ || pose.y > boundary_max_;
    }

    void stopTurtles() {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        turtle1_cmd_pub_.publish(stop_msg);
        turtle2_cmd_pub_.publish(stop_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    DistanceNode distance_node;
    distance_node.spin();
    return 0;
}
