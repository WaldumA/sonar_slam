#include <ros/ros.h>
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>
#include <sonar_msgs/sonar_processed_data.h>
#include "uncertainty_calculating.hpp"

// Quality of life
using namespace Eigen;
using namespace std;

class sonarSlam {
private:
	// ROS handler
	ros::NodeHandle nh;

	// ROS parameters
	ros::Subscriber sonar_processed_data;

	// ROS params
	string sub_topic;

	// Line extractor
	lineExtractor lE;

public:
sonarSlam() {

	// Regarding ROS parameter
	nh.getParam("sonar_slam_sub_topic",sub_topic);
	// Initiate subscribers
	sonar_processed_data = nh.subscribe(sub_topic,1000,&sonarSlam::sonarCallback,this);
	}


~sonarSlam() {}	


// Callback functions
void sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg);

};


