
#include <iostream>
#include <list>
#include <vector>
#include <Eigen/Dense>


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



// Quality of life
using namespace Eigen;
using namespace std;

// EKF_SLAM class
class ekfSLAM {
    private:
        // ROS handler
        

        
        
        float R, doAsso, alpha, sensOffset, T;
        odomStates states, measurements;
        // EKF Matrices
        MatrixXf matrixP, matrixF, matrixSDVL, matrixHDVL, matrixHsonar, matrixSsonar, matrixW;
        MatrixXf matrixQ, matrixI;
        VectorXf estimatedStatesX, predictedStatesX, measurementsZ, innovation;
     

    public:
    ekfSLAM() {
    }

    ~ekfSLAM() {}

    // Functions
    void initialization(VectorXf X0, MatrixXf P0, MatrixXf Q);
    void getDVLMeasurements(float u, float v, float time);

    // Odom Prediction
    void predict();
    void odomPrediction();
    void covariancePrediction();
    void Fx();

    // DVL update
    void DVLH();
    void DVLupdate();
    void DVLinnovation();
    void KalmanGainDVL();
    void DVLcovariance();

    // Debugging
    void printStates(bool xEstimate, bool xPrediction, bool pCoovariance);

};