#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
     ROS_INFO("Moving the robot with linear x = %1.2f and angular z = %1.2f",lin_x, ang_z);
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
// Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive to target");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    float ang_z = 0.5;
    float speed_factor = 1.0;
    float Lin_speed_factor = 1.0;
    float lin_x = 0.5;
    static int old_index = img.step/2;
    int h_index = 0; 
    int c_index = 0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    bool ball_inside_image = false;
    int index = 0;
    

    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; (i < img.height * img.step) && (i+2 < img.height * img.step); i+=1) {
        if (img.data[i] == white_pixel && (img.data[i+1] == white_pixel) &&img.data[i+2] == white_pixel) {
            ball_inside_image = true; 
            index = i;
            break;
        }
    }

    if(ball_inside_image)
    {
	    h_index = index / img.step ;
        c_index = h_index * img.step;
        index = index - c_index ;
        ROS_INFO("ball found in image index = %d old= %d",index, old_index);
        int move_left_right = 1;
        if(index < img.step/3)
        {
            if(index < img.step/4)
            {
                speed_factor = 5;
            }
            else
            {
                speed_factor = 1;
            }
           // if(old_index>index)
         //   	move_left_right =-1;
          //  else
                move_left_right = 1;
            ang_z = move_left_right * ang_z* speed_factor;
        }
	else if(index > img.step*2/3)
	{
            if(index > img.step*3/4)
            {
                speed_factor = 5;
            }
            else
            {
                speed_factor = 1;
            }
	   // if(old_index>index)
            	move_left_right = -1;
           // else
              //  move_left_right = 1;
            ang_z = move_left_right * ang_z* speed_factor;
	}
        else 
        {
            move_left_right = 0;
            ang_z = move_left_right * ang_z;
            Lin_speed_factor = 5;
        }
        old_index = index;
        drive_robot(lin_x*Lin_speed_factor,ang_z);
    }
    else
    {
        //old_index = img.width/2;
        ROS_INFO_STREAM("ball not found in image stop Robot!");
        drive_robot(0,0);
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
