// Vector because its awsome
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <list>

// Quality of life
using namespace std;
using namespace Eigen;

// Lines
struct lines2Check {
    vector<float> meanRho;
    vector<float> meanTheta;
    vector<float> varRho;
    vector<float> varTheta;
};

// JCBB struct
struct JCBB {
    vector<int> matchedPrediction;
    vector<int> matchedMeasurement;
};

struct STREAK {
    int streak;
    list<int> path;
};


float binarySearch(vector<float> theList, float number, int sizeOfList);
MatrixXi measurementLineCompability(lines2Check foundLines, lines2Check predictedLines);
JCBB jointCompability(lines2Check foundLines, lines2Check predictedLines);
JCBB find_best_streak(int row, int col, MatrixXi mat, int streak, list<int> usedCols,int start_row,JCBB tmp_struct );