// My own libaries
#include "search_algorithms.hpp"

// ROS
#include <ros/ros.h>

// Vector because its awsome
#include<vector>
#include<algorithm>
// IOSTREAM for debugging
#include<iostream>

// EIGEN
#include <Eigen/Dense>

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// Quality of life
using namespace std;
using namespace Eigen;
using namespace cv;

// Declaring custom structures
// For visualisatin
struct localMaxima {
    vector<int> beam_number;
    vector<int> bin_number;
};
// For voting
struct votingBins {
    vector<float> angles;
    vector<float> ranges;
};

struct bestLine {
    float rho;
    float theta;
};

class lineExtractor{
    private:
    MatrixXi processedScan, votingSpace;
    int width, height;
    float rangeResolution, angleResolution, maxRange, minRange, maxAngle, minAngle;
    vector<float> votingRanges, votingAngles;
    bool visualizationFlag;

    // ROS handler
    ros::NodeHandle nh;

    public: 
    // Constructor and destructor
    lineExtractor(){
        // Variables -- Angles in Radians
        nh.getParam("minRange",minRange);
        nh.getParam("maxRange",maxRange);
        nh.getParam("maxAngle",maxAngle);
        nh.getParam("minAngle",minAngle);
        nh.getParam("rangeResolution",rangeResolution);
        nh.getParam("angleResolution",angleResolution);
        nh.getParam("angleResolution",visualizationFlag);
        
        // Cheking if stuff should be visualised
        if (visualizationFlag) {
            height = ceil((float)maxRange/(float)rangeResolution);
            width = 2*ceil((float)maxRange/(float)rangeResolution);
        }
        // Initializing voting space based on variables
        initializeVotingSpace();


    }
    ~lineExtractor() {

    }

    // Functions
    void visualiseMatrix(MatrixXi matrix);
    votingBins processSegmentedScan(vector<float> ranges, vector<float> angles);
    MatrixXi votingProcess(votingBins bins);
    bestLine findBestLine(MatrixXi votingSpace);
    vector<bestLine> find4BestLine(MatrixXi votingSpace);
    void initializeVotingSpace();
    void visualiseLine(bestLine line, vector<float> ranges, vector<float> angles);
    void visualise4Line(vector<bestLine> line, vector<float> ranges, vector<float> angles);

};