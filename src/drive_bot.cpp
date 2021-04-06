#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

int main(int argc, char** argv)
{
    // init ros node
    ros::init(argc,argv, "drive_bot");

    // create ros node handle
    ros::NodeHandle n;

    //init publisher
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    
    while(ros::ok())
    {
        // create motor command object
        geometry_msgs::Twist motor_command;

        motor_command.linear.x = 0.5;
        motor_command.angular.z = 0.0;

        motor_command_publisher.publish(motor_command);
    }

    // TODO: Handle ROS communication events
    //ros::spin();

    return 0;
}