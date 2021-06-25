#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool Motor_command_handle(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    motor_command_publisher.publish(motor_command);

    res.msg_feedback = "The linear value is: " + std::to_string(req.linear_x) + " and the angular value is: " + std::to_string(req.angular_z);

    ROS_INFO_STREAM(res);

    return true;
}


int main(int argc, char** argv)
{
    // init ros node
    ros::init(argc,argv, "drive_bot");

    // create ros node handle
    ros::NodeHandle n;

    //init publisher
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer motor_command_service = n.advertiseService("/ball_chaser/command_robot", Motor_command_handle);

    //  Handle ROS communication events
    ros::spin();

    return 0;
}