/***
Authors: J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
**/

// INCLUIR LA LIBRERIA DE ROS
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>

// INCLUIR RESTO DE LIBERIAS A UTILIZAR
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h> 
#include <unistd.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <termios.h>

using std::string;
using std::cout;
using std::cin;
using std::endl;
using std::exception;
using std::vector;
using namespace std;

/*** ROS PUBLISHERS ***/
ros::Publisher pose_pub;

/*** ROS SUBSCRIBERS ***/
ros::Subscriber pose_rviz_sub;

// VARIABLES 
nav_msgs::Odometry pose_data;
std::string map_frame, baseLink_frame;
std::vector<double> initial_pose;

void timer_callback(const ros::TimerEvent& event){
	pose_data.header.stamp = ros::Time::now();	
	pose_pub.publish(pose_data);
}

void pose_rviz_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	geometry_msgs::PoseWithCovarianceStamped pose_rviz = *msg; 

	tf::Quaternion q(pose_rviz.pose.pose.orientation.x, pose_rviz.pose.pose.orientation.y, pose_rviz.pose.pose.orientation.z, pose_rviz.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw); 
	yaw = yaw * 180 / M_PI;

	cout << "Data received:  x = " << pose_rviz.pose.pose.position.x << " ; y = " << pose_rviz.pose.pose.position.y <<" ; yaw = " << yaw <<endl;
	
	pose_data.pose = pose_rviz.pose;
	pose_data.pose.pose.position.z=initial_pose[2];
}

// PROGRAMA PRINCIPAL
int main(int argc, char **argv){
	// INICIALIZAR ROS
	ros::init(argc, argv, "t4ac_pose_rviz_publisher_node");
	ros::NodeHandle nh;
	
	/*** PUBLISHERS  ****/
	pose_pub = nh.advertise<nav_msgs::Odometry>("/t4ac/localization/pose", 10, true);
	
	/*** SUBSCRIBERS  ****/
	pose_rviz_sub = nh.subscribe("/t4ac/localization/pose_rviz", 1, pose_rviz_Callback);

	// VARIABLES
	double frequency;

	// PARAMETERS
	nh.param("/t4ac_pose_rviz_publisher_node/frequency", frequency, 10.0);
	nh.param<std::string>("/t4ac/frames/map", map_frame, "map");
	nh.param<std::string>("/t4ac/frames/base_link", baseLink_frame, "base_link");

	std::vector<double> default_pose{0,0,0,0,0,0};
	nh.param<std::vector<double>>("t4ac_pose_rviz_publisher_node/initial_pose", initial_pose, default_pose);

	tf::Quaternion q;
	double yaw;
	yaw = initial_pose[5]*M_PI/180;
	q.setRPY(0, 0, yaw);

	pose_data.header.frame_id = map_frame;
	pose_data.child_frame_id = baseLink_frame;
	pose_data.pose.pose.position.x = initial_pose[0];
	pose_data.pose.pose.position.y = initial_pose[1];	
	pose_data.pose.pose.position.z = initial_pose[2];
	pose_data.pose.pose.orientation.x=q[0];
	pose_data.pose.pose.orientation.y=q[1];
	pose_data.pose.pose.orientation.z=q[2];
	pose_data.pose.pose.orientation.w=q[3];
	
	ros::Timer timer = nh.createTimer(ros::Duration(1/frequency), timer_callback); 
	ros::spin();
	return 0;
}
