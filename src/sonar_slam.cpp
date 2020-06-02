#include "sonar_slam.hpp"



void sonarSlam::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	float tmp_yaw, time_step;
	theQuaternion q;
	tmp_yaw = msg->angular_velocity.z;
	//dt = (ros::Time::now().nsec - prev_time.nsec);
	time_step = 1.0/126.0;
		//prev_time = ros::Time::now();
		//cout << "dt: " << dt*0.000000001 << endl;
	
	yaw += tmp_yaw * time_step;
	q = ToQuaternion(yaw,0,0);
	
	//cout << "Orientation: " << endl;
	//cout << "x: " << q.x << endl << "y: " << q.y << endl << "z: " << q.z << endl << "w: " << q.w << endl;
	
}

void sonarSlam::sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg) {
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
	eS.addLandmark();
	eS.resetVariables();
	
	

	//eS.visualiseMap(thresholded_data);
	eS.printStates(true, true, false);
	
	
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
	dvl_sub = nh.subscribe(dvl_sub_topic,1000,&sonarSlam::dvlCallback,this);
	imu_sub = nh.subscribe(imu_sub_topic,1000,&sonarSlam::IMUCallback, this);

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