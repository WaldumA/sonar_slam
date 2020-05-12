#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>
#include "line_extractor.hpp"


// Structs
struct normalDistribution {
    float mean_rho;
    float mean_theta;
    float variance_rho;
    float variance_theta;
};

// Functions
vector<normalDistribution> calculateNormalDistributions(vector<bestLine> bestLines, vector<float> thresholdAngles, vector<float> thresholdRanges, float angleResolution, float rangeResolution);
int overlapRatio(MatrixXi scan, bestLine line);







/* TODO
1. Calculate the uncertainty
2. Find line segments
*/