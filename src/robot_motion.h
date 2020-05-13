#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

class motion{
    public:
        motion(ros::NodeHandle &n);
        void publishVelocity(geometry_msgs::Twist vel_msg);
        void move(double speed, double distance, bool isForward);
        void rotate (double angular_speed, double angle, bool clockwise);
        double degrees2radians(double angle_in_degrees);
        void setDesiredOrientation(double desired_angle_radians);
        void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
        void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
        double getDistance(double x1, double y1, double x,double y);
        void spiralmotion(double &constant_speed, double &linear_speed);
    private:
        ros::Publisher velocity_publisher;
        ros::Subscriber pose_subscriber;
};

#endif