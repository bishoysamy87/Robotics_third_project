#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities


// This function should publish the requested linear x and angular velocities to the robot wheel joints
std::vector<float> set_motor_vel_values(float requested_linear_x, float requested_angular_vel)
{
    // Define clamped joint angles and assign them to the requested ones
    float linear_x = requested_linear_x;
    float ang_vel = requested_angular_vel;


    // Store clamped joint angles in a clamped_data vector
    std::vector<float> clamped_data = { linear_x, ang_vel };

    return clamped_data;
}

//Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_vel:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Check if requested joint angles are in the safe zone, otherwise clamp them
    std::vector<float> joints_angles = set_motor_vel_values(req.linear_x, req.angular_z);

	// Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
	// Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = joints_angles[0];
    motor_command.angular.z = joints_angles[1];
	// Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);


    // Return a response message
    res.msg_feedback = "Set wheel velocities - linear x: " + std::to_string(joints_angles[0]) + " , angular z: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send joint commands");

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}

