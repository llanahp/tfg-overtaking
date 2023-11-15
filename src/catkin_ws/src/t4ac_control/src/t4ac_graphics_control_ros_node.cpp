// ROS libraries
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "t4ac_msgs/CarControl.h"
#include "t4ac_msgs/Path.h"
#include <visualization_msgs/Marker.h>
#include "tf/tf.h"

// Other libraries
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>


using std::string;
using std::cout;
using std::cin;
using std::endl;
using std::exception;
using std::vector;

// Variables
nav_msgs::Odometry odometry_state;
nav_msgs::Path trajectory_state;
nav_msgs::Path waypoints;
t4ac_msgs::CarControl vel_state;
std_msgs::Float64 oe;
std_msgs::Float64 de;

double start_time=0, time_running=0; 
bool start = false;

std::ofstream logFile1, logFile2, logFile3;

void odometryCallback(const boost::shared_ptr<nav_msgs::Odometry const>& msg)
{

if (start){
	odometry_state = *msg;

	logFile1.open ("/home/docker_robesafe/shared_home/Logs/odometry.log", std::ios_base::app);
	logFile1 << odometry_state.pose.pose.position.x << "," << odometry_state.pose.pose.position.y << "," << tf::getYaw(odometry_state.pose.pose.orientation) <<  endl;
	logFile1.close();
	}	
}

void trajectoryCallback(const boost::shared_ptr<nav_msgs::Path const>& msg)
{

	trajectory_state = *msg;
	int i=0;
	for(i=0;i<msg->poses.size();i++){
	logFile1.open ("/home/docker_robesafe/shared_home/Logs/trajectory.log", std::ios_base::app);
	logFile1 << trajectory_state.poses[i].pose.position.x << "," << trajectory_state.poses[i].pose.position.y <<  endl;
	logFile1.close();}

}

void waypointsCallback(const boost::shared_ptr<t4ac_msgs::Path const>& msg)
{
	int i=0;
	for(i=0;i<msg->waypoints.size();i++){
	logFile1.open ("/home/docker_robesafe/shared_home/Logs/waypoints.log", std::ios_base::app);
	logFile1 << msg->waypoints[i].transform.location.x << "," << msg->waypoints[i].transform.location.y <<  endl;
	logFile1.close();}

}

void velCallback(const boost::shared_ptr<t4ac_msgs::CarControl const>& msg)
{
	if (!start){
		start_time = ros::Time::now().toSec();
		start = true;		
	}

if (start){

	vel_state = *msg;

	logFile1.open ("/home/docker_robesafe/shared_home/Logs/linear.log", std::ios_base::app);
	logFile1 << vel_state.velocity <<","<< time_running <<  endl;
	logFile1.close();
	logFile1.open ("/home/docker_robesafe/shared_home/Logs/angular.log", std::ios_base::app);
	logFile1 << vel_state.steer <<","<< time_running <<  endl;
	logFile1.close();
	}
}

void oeCallback(const boost::shared_ptr<std_msgs::Float64 const>& msg)
{

if (start){
	time_running = ros::Time::now().toSec() - start_time;
	oe = *msg; 
	logFile1.open ("/home/docker_robesafe/shared_home/Logs/oe.log", std::ios_base::app);
	logFile1 << oe.data <<","<< time_running <<  endl;
	logFile1.close();
	}
}

void deCallback(const boost::shared_ptr<std_msgs::Float64 const>& msg)
{

if (start){
	time_running = ros::Time::now().toSec() - start_time;
	de = *msg; 
	logFile1.open ("/home/docker_robesafe/shared_home/Logs/de.log", std::ios_base::app);
	logFile1 << de.data <<","<< time_running <<  endl;
	logFile1.close();
	}
}

int main (int argc, char **argv)
{
	remove("/home/docker_robesafe/shared_home/Logs/odometry.log");
	remove("/home/docker_robesafe/shared_home/Logs/trajectory.log");
	remove("/home/docker_robesafe/shared_home/Logs/waypoints.log");
	remove("/home/docker_robesafe/shared_home/Logs/linear.log");
	remove("/home/docker_robesafe/shared_home/Logs/angular.log");	
	remove("/home/docker_robesafe/shared_home/Logs/oe.log");
	remove("/home/docker_robesafe/shared_home/Logs/de.log");
	remove("/home/docker_robesafe/shared_home/Logs/actual_speed.log");

	// ROS init
	ros::init(argc, argv, "graphics_node");
	ros::NodeHandle n;

	// Subscribers

	ros::Subscriber odom_sub = n.subscribe("/t4ac/localization/pose", 1000, odometryCallback);
	ros::Subscriber trajectory_spline_sub = n.subscribe("/t4ac/control/spline", 1000, trajectoryCallback);
	ros::Subscriber waypoints_sub = n.subscribe("/t4ac/planning/route", 1000, waypointsCallback);
	ros::Subscriber vel_sub = n.subscribe("/t4ac/control/cmd_vel", 1000, velCallback); 
	ros::Subscriber oe_sub = n.subscribe("/t4ac/control/oe", 1000, oeCallback);
	ros::Subscriber de_sub = n.subscribe("/t4ac/control/de", 1000, deCallback);

	ros::spin();	
	return 0;
}
