#include "ros/ros.h"
#include <rsd_vision/bricks_to_robot.h>   	// Added to include my custum msg file,bricks_to_robo.msg

#define projectName rsd_vision

#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

// The r_and_theta callback function
void brickPoseCallBack(const projectName::bricks_to_robot::ConstPtr& msg)
//void brickPoseCallBack(const rsd_project::bricks_to_robot& header)
{
    // Receive the pose
    //geometry_msgs::Pose pose;
    //pose = msg-> pose;
    //cout << "\n------------------ Brick received ------------------\n" << pose << "\n";

    // Receive the x,y,z,roll,pitch and yaw
    double x,y,z,roll,pitch,yaw;
    x = msg->       x;
    y = msg->       y;
    z = msg->       z;
    roll = msg->    roll;
    pitch = msg->   pitch;
    yaw = msg->     yaw;

    cout << "x: "       << x        << endl;
    cout << "y: "       << y        << endl;
    cout << "z: "       << z        << endl;
    cout << "roll: "    << roll     << endl;
    cout << "pitch: "   << pitch    << endl;
    cout << "yaw: "     << yaw      << endl;


    // Receive the speed
    double speed;
    speed = msg->speed;
    cout << "Speed: " << speed << endl;

    // Receive the ID
    string color;
    color = msg->header.frame_id;
    cout << "Color: " << color << endl;

    // Receive the timestamp
    double timeStamp;
    timeStamp = msg->header.stamp.toSec();
    cout << setprecision(15);
    cout << "Time stamp: " << timeStamp << endl;

    // Get the current time
    double currentTime = ros::Time::now().toSec();
    cout << setprecision(15);
    cout << "Current time: " << currentTime << endl;

    // Calculate the differnce in time from send to received
    double offset;
    offset = currentTime-timeStamp;
    cout << setprecision(15);
    cout <<"Time offset: " << offset << endl;
	
    //ros::NodeHandle n;
	// Publisher cmd_velo
    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener_nodeRSD");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
    //rsd_project::bricks_to_robot:: num msg;

    //Subscriber lego_pose
    ros::Subscriber lego_pose_sub = n.subscribe("/lego_pose", 1000, brickPoseCallBack);
	
    // Publisher cmd_vel
    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	while (ros::ok())
	{	
        //loop_rate.sleep();
		ros::spinOnce();
	}	
	return 0;
}
