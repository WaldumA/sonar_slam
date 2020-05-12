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
	//lE.visualise4Line(bestLines, thresholdRanges, thresholdAngles);
	//lE.visualiseMatrix(votingSpace);
	//lE.visualiseLine(simplyTheBest, ranges, angles);
	
	// Only to be used when a new feature is explored
	
	uncertainty = calculateNormalDistributions(bestLines, thresholdAngles, thresholdRanges, lE.returnAngleResolution(), lE.returnRangeResolution());
	//helloWorld();
	
	

}

int main(int argc, char** argv) {
	ros::init(argc,argv,"sonarSlam");
	sonarSlam run_slam;
	ros::spin();
	return 0;
}
