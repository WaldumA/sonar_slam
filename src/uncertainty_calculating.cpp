#include "uncertainty_calculating.hpp"


// Functions


// Finds distrubtion based on lines with sufficient overlap 
normalDistribution findDistribution(vector<bestLine> accepted_lines) {
    normalDistribution distribution;
    float mean_theta, mean_rho, variance_theta, variance_rho;
    mean_theta = mean_rho = variance_theta = variance_rho = 0;
    for (int i = 0; i < accepted_lines.size(); i++) {
        mean_theta += accepted_lines[i].theta;
        mean_rho += accepted_lines[i].rho;
    }
    mean_theta = mean_theta/float(accepted_lines.size());
    mean_rho = mean_rho/float(accepted_lines.size());
    for (int i = 0; i < accepted_lines.size(); i++) {
        variance_theta += pow(accepted_lines[0].theta-mean_theta,2.0);
        variance_rho += pow(accepted_lines[0].rho-mean_rho,2.0);
    }
    variance_theta = variance_theta/float(accepted_lines.size());
    variance_rho = variance_rho/float(accepted_lines.size());
    distribution.mean_rho = mean_rho;
    distribution.mean_theta = mean_theta;
    distribution.variance_rho = variance_rho;
    distribution.variance_theta = variance_theta;
    return distribution;
}




// Calculating overlap_ratio
int overlapRatio(MatrixXi scan, bestLine line) {
    // Necesarry variables
    int scale = 50;
    float n, origo_height, origo_width, lineAngle;
    int overlap_score = 0;
    int y;
    Mat gray_display,tmp_image;

    // Calculating line parameters
    origo_width = (line.rho+0.1)*sin(line.theta);
    origo_height = sqrt(pow((line.rho+0.1),2.0) - pow(origo_width,2.0));
    origo_width = (500/2) - (origo_width*scale);
    origo_height = (500-5) - (origo_height*scale);
    lineAngle = -line.theta;
    n = origo_height - (tan(lineAngle) * origo_width);

    // Going over the scan 
    for (int x = 0; x < 500; x++) {
        y = int(x * tan(lineAngle) + n);
        if (y >= 0 && y < 500) {
            for (int j = -2; j < 2; j++) {
                for (int k = -2; k < 2; k++) {
                    if (y+j >= 0 && y+j < 500 && x+k >= 0 && x+k < 500) {     
                        if (scan(y+j,x+k) != 0) {
                            overlap_score += 1;
                        }
                    }
                }
            }
        }
    }
    return overlap_score;
}




// Calculates the normal distributions of the best extracted lines based on the sonar imprint 
vector<normalDistribution> calculateNormalDistributions(vector<bestLine> bestLines, vector<float> thresholdAngles, vector<float> thresholdRanges, float angleResolution, float rangeResolution ) {
    // Creating matriXi of full scan
    float tmp_width, tmp_height;
    MatrixXi scan;
    int scale = 50;
    int overlap_score;
    int threshold = 40;
    scan = MatrixXi(500,500);
    scan.setConstant(0);
    
    for (int i = 0; i < thresholdAngles.size(); i++) {
        if (thresholdRanges[i] != -1) {
            
            tmp_width = sin(thresholdAngles[i])*thresholdRanges[i];
            tmp_height = sqrt(pow(thresholdRanges[i],2.0) - pow(tmp_width,2.0));
            tmp_width = (500/2) - (tmp_width*scale);
            tmp_height = (500-5) - (tmp_height*scale);
            scan((int)tmp_height,(int)tmp_width) = 255;
           
        }
    }

    

    // Necesarry variables
    normalDistribution tmp_distribution;
    vector<normalDistribution> finalDistributions;
    vector<bestLine> candidate_lines, accepted_lines;
    bestLine winning_candidate, tmp_line;
    for (int i = 0; i < bestLines.size(); i++) {
        overlap_score = 0;
        winning_candidate = bestLines[i];
        candidate_lines.push_back(winning_candidate);
        overlap_score = overlapRatio(scan, winning_candidate);
        
        while (candidate_lines.size() > 0) {
            // If line has enough overlap with sonar scan, check for lines in the neigbhour hood
            if (overlap_score > threshold) {
                accepted_lines.push_back(candidate_lines[0]); 
                tmp_line.theta = candidate_lines[0].theta;
                tmp_line.rho = candidate_lines[0].rho + rangeResolution;
                candidate_lines.push_back(tmp_line);
                tmp_line.rho = candidate_lines[0].rho - rangeResolution;
                candidate_lines.push_back(tmp_line);
                tmp_line.rho = candidate_lines[0].rho;
                tmp_line.theta = candidate_lines[0].theta + angleResolution;
                candidate_lines.push_back(tmp_line);
                tmp_line.theta = candidate_lines[0].theta - angleResolution;
                candidate_lines.push_back(tmp_line);
            }
            // Erase check line from candidates
            candidate_lines.erase(candidate_lines.begin());
            // If there still are candidates in vector pick the first one and calcualte overlap score
            if (candidate_lines.size() > 0) {
                winning_candidate = candidate_lines[0];
                overlap_score = overlapRatio(scan, winning_candidate);
            }
        }
        tmp_distribution = findDistribution(accepted_lines);
        finalDistributions.push_back(tmp_distribution);

    }
  
    
   
    

    return finalDistributions;
}

// Could also just set uncertainty based on the sonar error from the datasheet?

/*
TODO:
1. Sjekk om usikkerhets beregningene fungerer(HAHA) og gjør fikses til det faktisk fungerer som det skal
2. Gjør line segmentation som foreslått i paperet for og lage kartet i en Occupancy Grid
3. Begynn og implementer EKF likningene, tror det er viktig for og verifisere denne implementasjonen av feature extraction greiene
4. Test, kjør, tune!
5. Fiks matlab scriptene slik at alt kan sammenlignes
















*/