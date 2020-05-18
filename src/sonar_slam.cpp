#include "sonar_slam.hpp"


void sonarSlam::sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg) {
	// Variables
	vector<normalDistribution> uncertainty; 
	vector<float> ranges, angles, thresholdRanges, thresholdAngles;
	vector<bestLine> bestLines;
	votingBins bins;
	MatrixXi votingSpace;
	bestLine simplyTheBest;

	// Extracting data from msg	
	ranges = msg->range;
	angles = msg->angles;
	thresholdAngles = msg->thresholdAngles;
	thresholdRanges = msg->thresholdRange;

	// Running line extraction
	bins = lE.processSegmentedScan(ranges, angles);
	votingSpace = lE.votingProcess(bins);
	//simplyTheBest = lE.findBestLine(votingSpace);
	bestLines = lE.find4BestLine(votingSpace);
	lE.visualise4Line(bestLines, thresholdRanges, thresholdAngles);
	lE.visualiseMatrix(votingSpace);
	//lE.visualiseLine(simplyTheBest, ranges, angles);
	
	// Only to be used when a new feature is explored
	
	uncertainty = calculateNormalDistributions(bestLines, thresholdAngles, thresholdRanges, lE.returnAngleResolution(), lE.returnRangeResolution());
	//helloWorld();
}

void sonarSlam::dvlCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	// Prediction
	eS.Fx();
	eS.odomPrediction();
	eS.covariancePrediction();

	// Update
	eS.getDVLMeasurements(msg->twist.twist.linear.x,msg->twist.twist.linear.y, msg->header.stamp.nsec * pow(10,-9));
	eS.DVLH();
	eS.DVLinnovation();
	eS.DVLcovariance();
	eS.KalmanGainDVL();
	eS.DVLupdate();
	eS.printStates(false, false, false);
	
}

void sonarSlam::initializeSlam() {

	
	// tmpVariables
	MatrixXf tmp_Q(8,8);
	tmp_Q.setZero();
	XmlRpc::XmlRpcValue Q;

	MatrixXf tmp_P(8,8);
	tmp_P.setZero();
	XmlRpc::XmlRpcValue P;

	VectorXf tmp_X(8);
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
            ostr << Q[ tmp_Q.rows()* i + 1];
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
		
	// Initiate subscribers
	sonar_sub = nh.subscribe(sonar_sub_topic,1000,&sonarSlam::sonarCallback,this);
	dvl_sub = nh.subscribe(dvl_sub_topic,1000,&sonarSlam::dvlCallback,this);

}
	



int main(int argc, char** argv) {
	ros::init(argc,argv,"sonarSlam");
	sonarSlam run_slam;
	ros::spin();
	return 0;
}
