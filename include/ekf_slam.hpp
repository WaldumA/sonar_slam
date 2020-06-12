
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <list>
#include "uncertainty_calculating.hpp"

// Quality of life
using namespace Eigen;
using namespace std;

// Measurements 
struct sensorMeasurements{
    float x; // Position x
    float y; // Position y
    float yaw; // Orientation yaw
    float u; // Velocity X
    float v; // Velocity Y
    float r; // Angular velocity yaw
    float ax; // Acceleration in x
    float ay; // Acceleration in y
};

// States
struct odomStates {
    float x; // Position x
    float y; // Position y
    float yaw; // Orientation yaw
    float u; // Velocity X
    float v; // Velocity Y
    float r; // Angular velocity yaw
    float ax; // Acceleration in x
    float ay; // Acceleration in y
    
};



struct theQuaternion
{
    double w, x, y, z;
};

theQuaternion ToQuaternion(double yaw, double pitch, double roll);




// EKF_SLAM class
class ekfSLAM {
    private:
        // ROS handler
        MatrixXf scan;
        JCBB FORDEBUGGING;
        int debug;
        vector<int> newLandmarks;
        lines2Check possibleLines, predictedLines, landmarkLines;
        bool newLandmark, DEBUG, show_landmark;
        float R, doAsso, alpha, sensOffset, T, prev_yaw, test_yaw, estiamted_YAW; 
        float prev_ekf_yaw, prev_ekf_x, prev_ekf_y;
        odomStates states, measurements;
        float change_in_x, change_in_y, change_in_angle;
        // EKF Matrices
        MatrixXf predictedP, estimatedP, matrixF, matrixH, matrixS, matrixW, assosiationS, assosiationH, estimatedPtest;
        MatrixXf matrixQ, matrixI, statesEta, statesP, matrixMap, matrixG;
        VectorXf estimatedStatesX, predictedStatesX, measurementsZ, innovation, predictedLandmarks, matrixR;
        VectorXf possibleInnovation;

    public:
    ekfSLAM() {
    }

    ~ekfSLAM() {}

    // Functions
    void initialization(VectorXf X0, MatrixXf P0, MatrixXf Q);
    void getDVLMeasurements(float u, float v, float time, float yaw);

    // Odom Prediction
    void predict();
    void odomPrediction();
    void Fx();
    void covariancePrediction();
    void G();
    



    // Sonar update
    void H();
    void getLines(float meanRho, float meanTheta, float varRho, float varTheta);
    void sonarInnovation();
    void sonarH();
    void addLandmark();
    void gatingTest();
    void sonarS();
    void sonarKalmanGain();
    void sonarUpdate();
    void predictLandmarks();
    void resetVariables();
    void getMeasurements(float ekf_x,float ekf_y,float ekf_yaw);
    void prediction();
    
    // Debugging
    void printStates(bool xEstimate, bool xPrediction, bool pCoovariance);
    void visualiseMap(votingBins data);
    bool enoughLandmarks();

    // DVL update
    void DVLH();
    void DVLupdate();
    void DVLinnovation();
    void KalmanGainDVL();
    void DVLcovariance();

    // Get states for publisher
    Vector3f getStates();
};