#include <ros/ros.h>
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>
#include <sonar_msgs/sonar_processed_data.h>
#include <nav_msgs/Odometry.h>
#include "ekf_slam.hpp"
#include <sensor_msgs/Imu.h>

// Quality of life
using namespace Eigen;
using namespace std;






class sonarSlam {
private:
	// ROS handler
	ros::NodeHandle nh;

	// ROS parameters
	ros::Publisher odometry_pub;
	ros::Subscriber sonar_sub;
	ros::Subscriber dvl_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber ekf_sub;

	// ROS params
	string sonar_sub_topic;
	string dvl_sub_topic;
	string imu_sub_topic;
	

	// Line extractor
	lineExtractor lE;

	// EKF_SLAM
	ekfSLAM eS;

	// Variables
	bool first_time;
	bool debug;
	float prevT, T;	
	float yaw,dt;
	float ekf_yaw,ekf_x,ekf_y;
	ros::Time prev_stamp, time_stamp;
	vector<normalDistribution> uncertainty; 
	votingBins thresholded_data;
	Vector3f statesToPublish;
	nav_msgs::Odometry msgToPublish;

public:
sonarSlam() {
	initializeSlam();
	
	}


~sonarSlam() {}	


// Callback functions
void initializeSlam();
void sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg);
void dvlCallback(const nav_msgs::Odometry::ConstPtr& msg);
void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
void EKFCallback(const nav_msgs::Odometry::ConstPtr& msg);
void sonarEKFcallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg);





};


