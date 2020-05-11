#include "sonar_slam.hpp"





void sonarSlam::sonarCallback(const sonar_msgs::sonar_processed_data::ConstPtr& msg) {
	// Variables
	vector<float> ranges, angles;
	vector<bestLine> bestLines;
	votingBins bins;
	MatrixXi votingSpace;
	bestLine simplyTheBest;

	// Extracting data from msg	
	ranges = msg->range;
	angles = msg->angles;

	// Running line extraction
	
	bins = lE.processSegmentedScan(ranges, angles);
	votingSpace = lE.votingProcess(bins);
	//simplyTheBest = lE.findBestLine(votingSpace);
	bestLines = lE.find4BestLine(votingSpace);

	lE.visualise4Line(bestLines, ranges, angles);
	//lE.visualiseMatrix(votingSpace);
	//lE.visualiseLine(simplyTheBest, ranges, angles);
	
	

	
	

}

int main(int argc, char** argv) {
	ros::init(argc,argv,"sonarSlam");
	sonarSlam run_slam;
	ros::spin();
	return 0;
}
