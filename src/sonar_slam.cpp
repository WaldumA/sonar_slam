#include "sonar_slam.hpp"



void sonarSlam::EKFCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	float x,y,qw,qx,qy,qz, yaw;
	ekf_x = msg->pose.pose.position.x;
	ekf_y = msg->pose.pose.position.y;
	qw = msg->pose.pose.orientation.w;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	ekf_yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
	//cout << "EKF yaw: " << yaw << endl;



}

void sonarSlam::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	
	float tmp_yaw, time_step, dt, test_time;
	theQuaternion q;



	if (first_time) {
	tmp_yaw = msg->angular_velocity.z;
	time_stamp = msg->header.stamp;
	//cout << "The time: " << time_stamp.nsec << endl;


	if (tmp_yaw > 0.5) {
		//cout << "TMP_YAW: " << tmp_yaw << endl;
	}




	dt = (time_stamp.nsec - prev_stamp.nsec)*pow(10,-9);
	//cout << (time_stamp.nsec - prev_stamp.nsec)*pow(10,-9) << endl;


	//dt = (ros::Time::now().nsec - prev_time.nsec);
	time_step = float(dt)*2;
	test_time = 1.0/126.0;
	
	
	
	//cout << "Changing: " << time_step << ", Constant: " << test_time << endl;
		//prev_time = ros::Time::now();
		//cout << "dt: " << dt*0.000000001 << endl;

	
	prev_stamp = time_stamp;
	}
	else {
		tmp_yaw = 0;
		prev_stamp = time_stamp;
		time_step = 0;
		first_time = true;
	}
	
	if (time_step > 0.01) {
		time_step = (1/126.0);
	}
	yaw += tmp_yaw * time_step;
	if (yaw > 3.1415) {
		yaw = yaw - 2*3,1415;
	}
	if (yaw < -3.1415) {
		yaw = yaw + 2*3.1415;
	}
	q = ToQuaternion(yaw,0,0);
	//cout << "YAW: " << yaw << endl;
	
	
	//cout << "Orientation: " << endl;
	//cout << "x: " << q.x << endl << "y: " << q.y << endl << "z: " << q.z << endl << "w: " << q.w << endl;
	
}

void sonarSlam::sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg) {
	
	// Predict
	eS.getMeasurements(ekf_x,ekf_y,ekf_yaw);
	eS.Fx();
	eS.prediction();
	eS.covariancePrediction();

	// Variables
	vector<float> ranges, angles, thresholdRanges, thresholdAngles;
	vector<bestLine> bestLines;
	votingBins bins;
	MatrixXi votingSpace;
	bestLine simplyTheBest;

	// Extracting data from msg	
	//cout << "Sonar1" << endl;
	
	ranges = msg->range;
	angles = msg->angles;
	thresholdAngles = msg->thresholdAngles;
	thresholdRanges = msg->thresholdRange;

	// Running line extraction
	
	bins = lE.processSegmentedScan(ranges, angles);
	thresholded_data = bins;
	votingSpace = lE.votingProcess(bins);
	bestLines = lE.find4BestLine(votingSpace);
	

	// Only to be used when a new feature is explored
	uncertainty = calculateNormalDistributions(bestLines, thresholdAngles, thresholdRanges, lE.returnAngleResolution(), lE.returnRangeResolution());


	// Update
	
	for (int i = 0; i < uncertainty.size(); i++) {
	eS.getLines(uncertainty[i].mean_rho, uncertainty[i].mean_theta, uncertainty[i].variance_rho, uncertainty[i].variance_theta);
	}
	
	eS.predictLandmarks();
	eS.H();
	eS.sonarS();
	eS.sonarInnovation();
	eS.sonarKalmanGain();
	eS.sonarUpdate();
	

	//if (eS.enoughLandmarks()) {
	//		eS.addLandmark();
	//}
	
	
	
	//eS.visualiseMap(thresholded_data);
	//eS.printStates(true	, false, false);
	eS.visualiseTheMap(thresholded_data);
	eS.resetVariables();
	statesToPublish = eS.getStates();
	
	msgToPublish.pose.pose.position.x = statesToPublish(0); 
	msgToPublish.pose.pose.position.y = statesToPublish(1);
	msgToPublish.pose.pose.position.z = statesToPublish(2);
	odometry_pub.publish(msgToPublish);
	//cout << "One loop" << endl;
	
	
}

void sonarSlam::dvlCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	// Prediction
	
	eS.getDVLMeasurements(msg->twist.twist.linear.x,msg->twist.twist.linear.y, msg->header.stamp.nsec * pow(10,-9), yaw);
	eS.Fx();
	eS.odomPrediction();
	eS.covariancePrediction();
	
	// Update
	
	for (int i = 0; i < uncertainty.size(); i++) {
	eS.getLines(uncertainty[i].mean_rho, uncertainty[i].mean_theta, uncertainty[i].variance_rho, uncertainty[i].variance_theta);
	}
	
	eS.predictLandmarks();
	eS.H();
	eS.sonarS();
	eS.sonarInnovation();
	eS.sonarKalmanGain();
	eS.sonarUpdate();
	//(eS.enoughLandmarks()) {

	//	if (eS.enoughLandmarks()) {
	
	//	eS.addLandmark();
	//}
	
	
	
	eS.visualiseMap(thresholded_data);
	//eS.printStates(true	, false, false);
	eS.resetVariables();
	statesToPublish = eS.getStates();
	
	msgToPublish.pose.pose.position.x = statesToPublish(0); 
	msgToPublish.pose.pose.position.y = statesToPublish(1);
	msgToPublish.pose.pose.position.z = statesToPublish(2);
	odometry_pub.publish(msgToPublish);
	//cout << "One loop" << endl;
	
	
}

void sonarSlam::initializeSlam() {

	
	// tmpVariables
	MatrixXf tmp_Q(3,3);
	tmp_Q.setZero();
	XmlRpc::XmlRpcValue Q;

	MatrixXf tmp_P(3,3);
	tmp_P.setZero();
	XmlRpc::XmlRpcValue P;

	VectorXf tmp_X(3);
	tmp_X.setZero();
	XmlRpc::XmlRpcValue X;
	first_time = false;
	
	
	




	// Collecting ros_parameters
	// Subscribers
	nh.getParam("sonar_slam_sub_topic",sonar_sub_topic);
	nh.getParam("dvl_slam_sub_topic",dvl_sub_topic);
	nh.getParam("imu_slam_sub_topic",imu_sub_topic);
	// Parameters for EKF-SLAM
	nh.getParam("Q",Q);
	nh.getParam("P0",P);
	nh.getParam("X0",X);



	// Filling the matrices

	for (int i = 0; i < tmp_X.rows();i++) {
			ostringstream ostr;
            ostr << X[ tmp_X.rows()* i];
            std::istringstream istr(ostr.str());
            istr >> tmp_X(i);
	}

	for (int i = 0; i < tmp_Q.rows();i++) {
		for (int j = 0; j < tmp_Q.rows(); j++) {
			ostringstream ostr;
            ostr << Q[ tmp_Q.rows()* i + j];
            std::istringstream istr(ostr.str());
            istr >> tmp_Q(i, j);
		}
	}
	
	for (int i = 0; i < tmp_P.rows();i++) {
		for (int j = 0; j < tmp_P.rows(); j++) {
			ostringstream ostr;
            ostr << P[ tmp_P.rows()* i + j];
            std::istringstream istr(ostr.str());
            istr >> tmp_P(i, j);
		}
	}

	
	// Iniitializing the ekfSLAM class
	eS.initialization(tmp_X, tmp_P, tmp_Q);

	// Initialize the lineExtractor class




	
	// Initiating variables
	prevT = 0;
	yaw = 0;
		
	// Initiate subscribers
	sonar_sub = nh.subscribe(sonar_sub_topic,1000,&sonarSlam::sonarCallback,this);
	ekf_sub = nh.subscribe("odometry/filtered",100,&sonarSlam::EKFCallback,this);
	//dvl_sub = nh.subscribe(dvl_sub_topic,1000,&sonarSlam::dvlCallback,this);
	//imu_sub = nh.subscribe(imu_sub_topic,1000,&sonarSlam::IMUCallback, this);
	

	// Initiate publisher
	
	odometry_pub = nh.advertise<nav_msgs::Odometry>("/MANTA/SLAM",100);
	

}
	



int main(int argc, char** argv) {
	ros::init(argc,argv,"sonarSlam");
	sonarSlam run_slam;
	ros::spin();
	return 0;
}


/* TO-DO
1. Skru på spotify og Song of the people, IKKE musikk på Youtube :)
2. Fiks compability, cout verdier på stuff og sørg for at noe kommer seg igjennom gating test. Mulig
landmark prediction også må fikses. Anyway fiks it :) 
3. Få til og kjøre 6m baggen frem og tilbake med kun og finne noen få landmarks. Tune på verdier etc 
ettersom hva som trengs.
4. Få inn landmarksa som blir funnet som update i state estimation.
*/