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
bool isSame(vector<bestLine> accaptedLine,vector<bestLine> candidate_lines, bestLine line);
void debug(MatrixXi scan, vector<bestLine> lines);
bool closeEnough(bestLine tmp_line, float best_rho, float best_theta, float rangeResolution, float angleResolution);
normalDistribution findDistribution(vector<bestLine> accepted_lines);




/* TODO
1. Calculate the uncertainty
2. Find line segments
*/