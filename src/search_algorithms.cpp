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
  
