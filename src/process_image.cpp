#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <string>

// Settings
#define FORWARD_VELOCITY    (0.5f)
#define ANGULAR_VELOCITY    (0.5f)
#define STOP_VELOCITY       (0.0f)

// White Pixel tolerance
#define WHITE_PIXEL         (255u - 5u)

// Defines for numberOfPixelsInAreas
#define NUMBER_OF_AREAS     (3u)
#define INDEX_AREA_LEFT     (0u)
#define INDEX_AREA_CENTER   (1u)
#define INDEX_AREA_RIGHT    (2u)

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    /* create new msg */
    ball_chaser::DriveToTarget msg;
    
    // add the function parameters to the msg
    msg.request.linear_x = lin_x;
    msg.request.angular_z = ang_z;

    /* Send new velocity values to the robot */
    if(!client.call(msg))
    {
        /* Sending failed. */
        ROS_ERROR("Failed to call service /ball_chaser/command_robot!");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    /* local variables */
    uint32_t pixelCounter = 0u;
    uint32_t stepCounter = 0u;
    uint32_t rowCounter = 0u;
    uint32_t columnCounter = 0u;
    uint32_t pixelRgb = 0u;

    /* local variables which defines the areas in the image */
    uint32_t leftDirection = uint32_t(float(img.width) * (1.0f/3.0f));
    uint32_t forwardDirection = uint32_t(float(img.width) * (2.0f/3.0f));

    /* local variable to count how many white pixels are detected in each area */
    uint32_t numberOfPixelsInAreas[NUMBER_OF_AREAS] = {0u, 0u, 0u};

    /* Ball detected flag */
    bool ballDetected = false;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    /* go through the rows in the center area of the image */
    for(rowCounter = ((img.height/2u) - 10u); rowCounter < ((img.height/2u) + 10u); rowCounter++)
    {
        /* go through the columns, per pixel (r,g,b)*/
        for(stepCounter = 0u; stepCounter < img.step; stepCounter+=3u)
        {
            /* pixel position in linear array */
            pixelCounter = stepCounter + (rowCounter * img.step);

            /* current column */
            columnCounter = stepCounter / 3u;

            /* calc pixel means value */
            pixelRgb = (img.data[pixelCounter] + img.data[pixelCounter + 1u] + img.data[pixelCounter + 2u]) / 3u;

            /* compare pixel value with a white pixel */
            if(pixelRgb > WHITE_PIXEL)
            {
                /* ball detected */
                ballDetected = true;

                /* in which area is the white pixel? */
                if(columnCounter < leftDirection)
                {
                    /* a part of the ball is on the left side */
                    numberOfPixelsInAreas[INDEX_AREA_LEFT]++;
                }
                else if(columnCounter < forwardDirection)
                {
                    /* a part of the ball is in front of the robot */
                    numberOfPixelsInAreas[INDEX_AREA_CENTER]++;
                }
                else
                {
                    /* a part of the ball is on the right side */
                    numberOfPixelsInAreas[INDEX_AREA_RIGHT]++;
                }
            }
        }/* end for-loop stepCounter */
    }/* end for-loop rowCounter */

    /* check if a ball was detected */
    if(ballDetected)
    {
        /* check in which image area are the most pixels of the ball */
        if(numberOfPixelsInAreas[INDEX_AREA_LEFT] > numberOfPixelsInAreas[INDEX_AREA_CENTER] && numberOfPixelsInAreas[INDEX_AREA_LEFT] > numberOfPixelsInAreas[INDEX_AREA_RIGHT])
        {
            /* the ball is on the left side */
            drive_robot(STOP_VELOCITY, ANGULAR_VELOCITY);
        }
        else if(numberOfPixelsInAreas[INDEX_AREA_CENTER] > numberOfPixelsInAreas[INDEX_AREA_RIGHT])
        {
            /* the ball is in front of the robot */
            drive_robot(FORWARD_VELOCITY, STOP_VELOCITY);
        }
        else
        {
            /* the ball is on the right side */
            drive_robot(STOP_VELOCITY, ANGULAR_VELOCITY * -1.0f);
        }
    }
    else
    {
        /* Ball wasn't detected so stop the robot. */
        drive_robot(STOP_VELOCITY, STOP_VELOCITY);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}