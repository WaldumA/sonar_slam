#include "line_extractor.hpp"


float lineExtractor::returnAngleResolution() {
    return angleResolution;
 }

float lineExtractor::returnRangeResolution() {
    return rangeResolution;
}



MatrixXi lineExtractor::votingProcess(votingBins bins) {
    // Declaring variables
    float alpha, beta, delta_alpha, delta_beta, range, tmp_range, tmp_angle;
    int vote_range, vote_angle;
    list<float> checkAngle;
    list<float> checkRange;
    alpha = 0.020943951;
    beta = 0.523598776;
    delta_alpha = float(alpha)/float(5.0);
    delta_beta = float(beta)/float(10.0);
    votingSpace.setConstant(0);

    // Parametrising each bin to possible lines and votes in the vote space :D 
    for (int i = 0; i < bins.ranges.size();i++) {
        if (bins.ranges[i] > 1.0) {
            checkAngle.clear();
            checkRange.clear();
            for (float j = bins.angles[i]-alpha; j < bins.angles[i]+alpha; j += delta_alpha) {
            for (float k = (j-beta); k < (j+beta); k += delta_beta) {
                range = bins.ranges[i]*cos(bins.angles[i]-k);
                tmp_range = binarySearch(votingRanges,range,votingRanges.size());
                tmp_angle = binarySearch(votingAngles,k,votingAngles.size());
                
                

                if (checkRange.empty() || find(checkRange.begin(),checkRange.end(),tmp_range)!=checkRange.end() || 
                find(checkAngle.begin(),checkAngle.end(),tmp_range)!=checkAngle.end()) { 
                    
                    votingSpace(int(tmp_range/rangeResolution),int((tmp_angle+1.57)/angleResolution)) += 1;
                    //votingSpace(0,int(tmp_range/rangeResolution)) += 1;
                    //votingSpace(vote_angle,vote_range) += 1;
            
                }        
            }
            
        }
        }

    }
    return votingSpace;

}

bestLine lineExtractor::findBestLine(MatrixXi votingSpace) {
    // Variables
    
    MatrixXi::Index maxRow, maxCol;
    bestLine tmpLine;
    // Finding max coefficients
    votingSpace.maxCoeff(&maxRow,&maxCol);
    // Calculating maxCoeff into line
    tmpLine.rho = votingRanges[maxRow];
    tmpLine.theta = votingAngles[maxCol];    
    return tmpLine;
}

vector<bestLine> lineExtractor::find4BestLine(MatrixXi votingSpace) {
    // Variables
    
    MatrixXi::Index maxRow, maxCol;
    vector<float> bestAngles;
    int threshold = 150;
    bool run_check;
    bool duplicate;
    run_check = true;
    duplicate = false;
    vector<bestLine> bestLines;
    bestLine tmpLine;
    // Finding max coefficients for the first line
    votingSpace.maxCoeff(&maxRow,&maxCol);
    tmpLine.rho = votingRanges[maxRow];
    tmpLine.theta = votingAngles[maxCol]; 
    bestAngles.push_back(tmpLine.theta); 
    bestLines.push_back(tmpLine);
    
    for (int i = maxRow - 5; i < maxRow +5; i++) {
        for (int j = maxCol - 10; j < maxCol + 10; j++ )
            if (i >= 0 && j>= 0) {
            votingSpace(i,j) = 0;
            }
    }
    

    // Finding the most voted(up to a maximum of 4) lines
    while (bestLines.size() < 1 && run_check) {
        votingSpace.maxCoeff(&maxRow,&maxCol);
        
        if (votingSpace(maxRow,maxCol) > threshold) {
            //cout << votingSpace(maxRow,maxCol) << endl;
            for (int j = 0; j < bestAngles.size(); j++) {
                if (bestAngles[j] + 0.5 > votingAngles[maxCol] && bestAngles[j]-0.5 < votingAngles[maxCol]) {
                    duplicate = true;
                }
            }
            if (!duplicate) {
                
                tmpLine.rho = votingRanges[maxRow];
                tmpLine.theta = votingAngles[maxCol]; 
                bestAngles.push_back(tmpLine.theta);
                bestLines.push_back(tmpLine);
            }
            for (int i = maxRow - 5; i < maxRow +5; i++) {
                for (int j = maxCol - 10; j < maxCol + 10; j++ )
                    if (i >= 0 && j>= 0) {
                    votingSpace(i,j) = 0;
                    }
            }
            duplicate = false;
        }
        else {
           run_check = false; 
        }
  
    // Her kan threshold, gapeter i if på linje 90 og den dobble for løkken tunes for mer consistent plott
    }
    




    return bestLines;
}

votingBins lineExtractor::processSegmentedScan(vector<float> ranges, vector<float> angles) {
    // Declaring variables
    float tmp_width, tmp_height, angle, range;
    votingBins voteBins;

    // Scaling bins
    
    for (int i = 0; i < ranges.size(); i++) {
        if (ranges[i] >= 1.0) {
            angle = angles[i];
            range = ranges[i];
            voteBins.angles.push_back(angle);
            voteBins.ranges.push_back(range);
        }
    }
    return voteBins;
    
}

void lineExtractor::initializeVotingSpace() {
    votingSpace = MatrixXi((int)((maxRange - minRange)/rangeResolution),(int)((float(maxAngle-minAngle))/float(angleResolution)));
    votingSpace.setConstant(0);
    int count = 0;
 
    for (float i = minRange; i < maxRange; i += rangeResolution) {
        votingRanges.push_back(i);
        

    }
    for (float i = minAngle; i < maxAngle; i += angleResolution) {
        votingAngles.push_back(float(i));
        
    }
    
}

void lineExtractor::visualiseMatrix(MatrixXi matrix ) {
    // Declaring variables
    Mat visualisation, gray_visualisation;

    // Converting eigentocv
    eigen2cv(matrix,visualisation);
    visualisation.convertTo(gray_visualisation, CV_8U);

    // Displaying the converted matrix on screen
    namedWindow("VisualisationOfMatrix",0);
    resizeWindow("VisualisationOfMatrix", 1000,1000);
    imshow("VisualisationOfMatrix",gray_visualisation);
    waitKey(1);
    
}

void lineExtractor::visualiseLine(bestLine line, vector<float> ranges, vector<float> angles) {
    // Variables
    MatrixXi tmp_scan;
    int scale = 50;
    float origo_height, origo_width, lineAngle;
    float tmp_height, tmp_width, n;
    int y;
    tmp_scan = MatrixXi(500,500);
    tmp_scan.setConstant(0);
    
    Mat gray_display,tmp_image;
    
    // Draw Scan
    
    for (int i = 0; i < ranges.size(); i++) {
        tmp_width = ranges[i]*sin(angles[i]);
        tmp_height = sqrt(pow(ranges[i],2.0) - pow(tmp_width,2.0));
        tmp_width = (500/2) - (tmp_width*scale);
        tmp_height = (500-5) - (tmp_height*scale);
        tmp_scan((int)tmp_height,(int)tmp_width) = 255;      
    }
    
    // Drawing the line 
    origo_width = line.rho*sin(line.theta);
    origo_height = sqrt(pow(line.rho,2.0) - pow(origo_width,2.0));
    origo_width = (500/2) - (origo_width*scale);
    origo_height = (500-5) - (origo_height*scale);
    lineAngle = -line.theta;
    n = origo_height - (tan(lineAngle) * origo_width);
    
    
    for (int x = 0; x < 500; x++) {
        y = int(x * tan(lineAngle) + n);
        if (y >= 0 && y < 500) {
            tmp_scan(y,x) = 255;
        }
    }
    //tmp_scan(origo_height,origo_width) = 100;
    

    // Convert to openCV and display
    eigen2cv(tmp_scan,tmp_image);
    tmp_image.convertTo(gray_display, CV_8U);
    namedWindow("Line display", 0);
    resizeWindow("Line display", 1200, 1200);
    imshow("Line display", gray_display);
    waitKey(1);
    
}   

void lineExtractor::visualise4Line(vector<bestLine> line, vector<float> ranges, vector<float> angles) {
    // Variables
    MatrixXi tmp_scan;
    int scale = 50;
    float origo_height, origo_width, lineAngle;
    float tmp_height, tmp_width, n;
    int y;
    tmp_scan = MatrixXi(500,500);
    tmp_scan.setConstant(0);
    
    Mat gray_display,tmp_image;
    
    // Draw Scan
    
    for (int i = 0; i < ranges.size(); i++) {
        tmp_width = ranges[i]*sin(angles[i]);
        tmp_height = sqrt(pow(ranges[i],2.0) - pow(tmp_width,2.0));
        tmp_width = (500/2) - (tmp_width*scale);
        tmp_height = (500-5) - (tmp_height*scale);
        tmp_scan((int)tmp_height,(int)tmp_width) = 255;      
    }
    
    // Drawing the line 
    for (int j = 0; j < line.size(); j++) {
        origo_width = (line[j].rho+0.1)*sin(line[j].theta);
        origo_height = sqrt(pow((line[j].rho+0.1),2.0) - pow(origo_width,2.0));
        origo_width = (500/2) - (origo_width*scale);
        origo_height = (500-5) - (origo_height*scale);
        lineAngle = -line[j].theta;
        n = origo_height - (tan(lineAngle) * origo_width);
        
        
        for (int x = 0; x < 500; x++) {
            y = int(x * tan(lineAngle) + n);
            if (y >= 0 && y < 500) {
                tmp_scan(y,x) = 255;
            }
        }
    }
    //tmp_scan(origo_height,origo_width) = 100;
    

    // Convert to openCV and display
    eigen2cv(tmp_scan,tmp_image);
    tmp_image.convertTo(gray_display, CV_8U);
    namedWindow("Line display", 0);
    resizeWindow("Line display", 1200, 1200);
    imshow("Line display", gray_display);
    waitKey(1);
    
}  



int main(int argc, char** argv) {
    lineExtractor iE;
    return 0;
}



/*
////////////////////////////////// TO DO TOMORROW ////////////////////////////////////////
1. Sørge for at hvert bin max kan stemme på en linje en gang Done
2. Visualiser linjene med sonar dataen for debugging Done
3. Finne en god løsning på og hente ut mer enn 1 linje omgangen Done
4. Få inn samme prosessering som gjøres i paperet
5. Deretter begynn og se på usikkerhet 
*/