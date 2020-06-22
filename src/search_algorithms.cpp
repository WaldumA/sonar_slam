#include <search_algorithms.hpp>




// Method to compare which one is the more close. 
// We find the closest by taking the difference 
// between the target and both values. It assumes 
// that val2 is greater than val1 and target lies 
// between these two. 
float getClosest(float val1, float val2, 
               float target) 
{ 
    if (target - val1 >= val2 - target) 
        return val2; 
    else
        return val1; 
} 


float binarySearch(vector<float> theList, float target, int n) {
     // Corner cases )
    if (target <= theList[0])  {
        return theList[0]; 
    }
    if (target >= theList[n - 1]) 
        return theList[n - 1]; 
  
    // Doing binary search 
    int i = 0, j = n, mid = 0; 
    while (i < j) { 
        mid = (i + j) / 2; 
  
        if (theList[mid] == target) { 
            return theList[mid]; 
            }
  
        /* If target is less than array element, 
            then search in left */
        if (target < theList[mid]) { 
  
            // If target is greater than previous 
            // to mid, return closest of two 
            if (mid > 0 && target > theList[mid - 1]) 
                return getClosest(theList[mid - 1], 
                                  theList[mid], target); 
  
            /* Repeat for left half */
            j = mid; 
        } 
  
        // If target is greater than mid 
        else { 

            if (mid < n - 1 && target < theList[mid + 1]) 
                return getClosest(theList[mid], 
                                  theList[mid + 1], target); 
            // update i 
            i = mid + 1;  
        } 
    } 
  
    // Only single element left after search 
    return theList[mid]; 
} 
  

MatrixXi measurementLineCompability(lines2Check foundLines, lines2Check predictedLines) {
    MatrixXi compability(foundLines.meanRho.size(),predictedLines.meanRho.size());
    //cout << "Predicted lines: " <<predictedLines.meanRho.size() << endl;
    compability.setZero();
    int debug;
    float theta_gate = 0.5;
    float rho_gate = 0.7;
    float tmp_term;
 
    // Looping through all landmarks and sees who is close enough
    
    for (int i = 0; i < predictedLines.meanRho.size(); i++) {
        for (int j = 0; j < foundLines.meanRho.size(); j++) {
            //cout << "Found Rho: " << foundLines.meanRho[j] << ", Predicted Rho: " << predictedLines.meanRho[i] << endl;
            //cout << "Found Theta: " << foundLines.meanTheta[j] << ", Predicted Theta: " << predictedLines.meanTheta[i] << endl;
            if (foundLines.meanRho[j] < predictedLines.meanRho[i] + rho_gate && foundLines.meanRho[j] > predictedLines.meanRho[i] - rho_gate) {
                if (foundLines.meanTheta[j] < predictedLines.meanTheta[i] + theta_gate && foundLines.meanTheta[j] > predictedLines.meanTheta[i] - theta_gate) {
                    compability(j,i) = 1;
                    
                }
            }
            
        }
    }
    
    return compability;
}

JCBB find_best_streak(int col, int row, MatrixXi mat, int streak, list<int> usedCols, int start_row,JCBB tmp_struct) {
        int debug,tmp_streak, end_row;
        JCBB tmpList, bestList;
        bool update = false;
        bestList = tmp_struct;
        
        end_row = start_row-1;
        
        if (start_row == 0) {
            end_row = mat.rows() - 1;
            
        }


          

        
        if (mat(row,col) == 1 && (row<mat.rows())) {      
            
            streak += 1;
            usedCols.push_back(col);
            tmp_struct.matchedPrediction.push_back(col);
            tmp_struct.matchedMeasurement.push_back(row);
            
            if (end_row != row) {
                   
            for (int i = 0; i < mat.cols(); i++) {
                if ((find(usedCols.begin(), usedCols.end(), i) == usedCols.end())) {     
                    if (row+1 != mat.rows()) {
                    tmpList = find_best_streak(i, row+1, mat, streak, usedCols,start_row, tmp_struct);
                    }
                    else {
                        tmpList = find_best_streak(i, 0, mat, streak, usedCols,start_row, tmp_struct);
                    }
                }   
                if (tmpList.matchedPrediction.size() > bestList.matchedPrediction.size()) {
                    bestList = tmpList;
                    update = true;
                }
            }
            if (update) {
                
                tmp_struct = bestList;

            }
            }

        }
        
        /*
        if (row == end_row) {
            cout << "YOLO" << endl;
            return tmp_struct;
        }     
        

        /*
        if (update == false && bestList.matchedPrediction.size() > 0) {
            
            for (int i = 0; i < mat.cols(); i++) {
                
                if ((find(usedCols.begin(), usedCols.end(), i) == usedCols.end())) {   
                    
                    if (row+2 < mat.rows()) {  
                        cout << "row+2" << endl;
                        tmpList = find_best_streak(i, row+2, mat, streak, usedCols,start_row,tmp_struct);
                    }
                    else {
                        cout << "row0" << endl;
                        tmpList = find_best_streak(i, 0, mat, streak, usedCols,start_row,tmp_struct);
                    }

                    if (tmpList.matchedPrediction.size() > bestList.matchedPrediction.size()) {
                    bestList = tmpList;
                    update = true;
                    }
            }
            if (update) {
                tmp_struct = bestList;
            }

            }
        
        }*/
        

        return tmp_struct;
}

float gatingTest(JCBB potentialStreak, lines2Check foundLines, lines2Check predictedLines) {
    float score = 0;
    int test;
    for (int i = 0; i < potentialStreak.matchedMeasurement.size(); i++) {
        score += sqrt(pow(foundLines.meanRho[potentialStreak.matchedMeasurement[i]]-predictedLines.meanRho[potentialStreak.matchedPrediction[i]],2));
        score += sqrt(pow(foundLines.meanTheta[potentialStreak.matchedMeasurement[i]]-predictedLines.meanTheta[potentialStreak.matchedPrediction[i]],2));
    }

    
    return score; 
}


JCBB jointCompability(lines2Check foundLines, lines2Check predictedLines) {
    // Variables
    MatrixXi compabilityMatrix;
    JCBB bestMatch, bestList, tmpList;
    int debug;
    float gateScore, gateScoreBest;
    
    // Finding possible compability between found lines and landmarks
    compabilityMatrix = measurementLineCompability(foundLines, predictedLines);
    
    // Looping through the compability matrix finding the longest streak
    for (int j = 0; j < compabilityMatrix.rows(); j++) {
        for (int i = 0; i < compabilityMatrix.cols(); i++) {
            tmpList = find_best_streak(i, j, compabilityMatrix, 0, {},j, bestMatch);
            if (tmpList.matchedPrediction.size() > bestList.matchedPrediction.size()) {
                gateScore = gatingTest(tmpList, foundLines, predictedLines);
                if (gateScore < 1) {
                    bestList = tmpList;
                    gateScoreBest = gateScore;
                }
            }
            if (tmpList.matchedPrediction.size() == bestList.matchedPrediction.size() && bestList.matchedPrediction.size() != 0) {
                gateScore = gatingTest(tmpList, foundLines, predictedLines);
                if (gateScore < gateScoreBest) {
                   bestList = tmpList;
                   gateScoreBest = gateScore; 
                }
                
            }
        }
    }
    //cout << "Returned JCBB " << bestList.matchedPrediction.size() << endl;
    //for (int i = 0; i < bestList.matchedPrediction.size(); i++) {
    //    cout << "Matched line: " << bestList.matchedMeasurement[i] << ", Matched prediction: " << bestList.matchedPrediction[i] << endl;
    //}
    //cout << "Actual JCBB" << endl;
    return bestList;
}