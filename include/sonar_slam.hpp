#include <ros/ros.h>
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>
#include <sonar_msgs/sonar_processed_data.h>
#include "uncertainty_calculating.hpp"
#include <nav_msgs/Odometry.h>
#include "ekf_slam.hpp"

// Quality of life
using namespace Eigen;
using namespace std;






class sonarSlam {
private:
	// ROS handler
	ros::NodeHandle nh;

	// ROS parameters
	ros::Subscriber sonar_sub;
	ros::Subscriber dvl_sub;

	// ROS params
	string sonar_sub_topic;
	string dvl_sub_topic;
	string imu_sub_topic;

	// Line extractor
	lineExtractor lE;

	// EKF_SLAM
	ekfSLAM eS;

	// Variables
	float prevT, T;	

public:
sonarSlam() {
	initializeSlam();
	
	}


~sonarSlam() {}	


// Callback functions
void initializeSlam();
void sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg);
void dvlCallback(const nav_msgs::Odometry::ConstPtr& msg);

};


