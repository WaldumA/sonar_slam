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
 
    // Looping through all landmarks and sees who is close enough
    
    for (int i = 0; i < predictedLines.meanRho.size(); i++) {
        for (int j = 0; j < foundLines.meanRho.size(); j++) {
            //cout << "Found Rho: " << foundLines.meanRho[j] << ", Predicted Rho: " << predictedLines.meanRho[i] << endl;
            //cout << "Found Theta: " << foundLines.meanTheta[j] << ", Predicted Theta: " << predictedLines.meanTheta[i] << endl;
            if (foundLines.meanRho[j] < predictedLines.meanRho[i] + 1 && foundLines.meanRho[j] > predictedLines.meanRho[i] - 1) {
                if (foundLines.meanTheta[j] < predictedLines.meanTheta[i] + 0.57 && foundLines.meanTheta[j] > predictedLines.meanTheta[i] - 0.57) {
                    compability(j,i) = 1;
                    //cout << "Compability Achived" << endl;
                }
            }
            
        }
    }
    
    return compability;
}

JCBB find_best_streak(int col, int row, MatrixXi mat, int streak, list<int> usedCols, int start_row,JCBB tmp_struct) {
        int debug,tmp_streak;
        JCBB tmpList, bestList;
        bool update = false;
        bestList = tmp_struct;

        if (mat(row,col) == 1 and (row<mat.rows())) {      
            streak += 1;
            usedCols.push_back(col);
            tmp_struct.matchedPrediction.push_back(col);
            tmp_struct.matchedMeasurement.push_back(row);
            for (int i = 0; i < mat.cols(); i++) {
                if ((find(usedCols.begin(), usedCols.end(), i) == usedCols.end())) {      
                    tmpList = find_best_streak(i, row+1, mat, streak, usedCols,start_row, tmp_struct);
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
        /*
        else if (update == false and (row != start_row or row!= start_row-1)) {
            for (int i = 0; i < mat.cols(); i++) {
                if ((find(usedCols.begin(), usedCols.end(), i) == usedCols.end())) {      
                    tmpList = find_best_streak(i, row+2, mat, streak, usedCols,start_row);
                    }
                    if (tmpList.size() > bestList.size()) {
                    bestList = tmpList;
                    update = true;
                }
   
            
                }
             if (update) {
                usedCols = bestList;
            }  

        }
        */

        return tmp_struct;
}


JCBB jointCompability(lines2Check foundLines, lines2Check predictedLines) {
    // Variables
    MatrixXi compabilityMatrix;
    JCBB bestMatch, bestList, tmpList;
    int debug;
    
    // Finding possible compability between found lines and landmarks
    compabilityMatrix = measurementLineCompability(foundLines, predictedLines);
    
    // Looping through the compability matrix finding the longest streak
    for (int j = 0; j < compabilityMatrix.rows(); j++) {
        for (int i = 0; i < compabilityMatrix.cols(); i++) {
            tmpList = find_best_streak(i, j, compabilityMatrix, 0, {},j, bestMatch);
        
            if (tmpList.matchedPrediction.size() > bestList.matchedPrediction.size()) {
                bestList = tmpList;
            }
        }
    }
    
    /*
    cout << "Compability Matrix: " << endl << compabilityMatrix << endl;
    cout << "Matched predictions and measurements: " << endl;
    for (int i = 0; i < bestList.matchedPrediction.size(); i++) {
        cout << i+1 << ". Measurement: " << bestList.matchedMeasurement[i] << " , Prediction: " << bestList.matchedPrediction[i] << endl;
    }
    cin >> debug;
    */

    // Returning matches
    //list<int>::iterator it = bestList.begin();
    //for (int i = 0; i < bestList.size(); i++) {
    //    
    //    bestMatch.matchedLine.push_back(i);
    //    bestMatch.matchedMeasurement.push_back(*it); 
    //    advance(it, 1);
    //}
    
    return bestList;
}