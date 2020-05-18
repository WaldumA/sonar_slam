#include "ekf_slam.hpp"









// INITIALIZATION ///////////////////////////////////////////////////////////////////////////
void ekfSLAM::initialization(VectorXf X0, MatrixXf P0, MatrixXf Q) {
    
    // Initiating the states 
    states.x = X0(0); 
    states.y = X0(1);
    states.yaw = X0(2);
    states.u = X0(3);
    states.v = X0(4);
    states.r = X0(5);
    states.ax = X0(6);
    states.ay = X0(7);

    estimatedStatesX = X0;

    // Initiating the covariance matrix
    matrixP = P0;

    // Initiating the process noise matrix
    matrixQ = Q;

    // Initiating a indentity matrix
    matrixI.Identity(8,8);

    // Initiating size of vectors;
    predictedStatesX = VectorXf(8);
    predictedStatesX.setZero();
    measurementsZ = VectorXf(8);
    measurementsZ.setZero();
    innovation = VectorXf(8);
    innovation.setZero();

    // Frequency
    T = 1.0/6.0;



    

}

void ekfSLAM::getDVLMeasurements(float u, float v, float time) {
    measurements.u = u;
	measurements.v = v;
	measurementsZ(3) = u;
	measurementsZ(4) = v;
}






// PREDICTION FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////


void ekfSLAM::odomPrediction() {
    //states.x = states.x + (states.u*T+(states.ax/2)*pow(T,2))*cos(states.yaw) - (states.v*T+(states.ay/2)*pow(T,2))*sin(states.yaw);
    //states.y = states.x + (states.u*T+(states.ax/2)*pow(T,2))*sin(states.yaw) + (states.v*T+(states.ay/2)*pow(T,2))*cos(states.yaw);
    //states.yaw = states.yaw + states.r*T;
    //states.u = states.u+(states.ax)*T;
    //states.v = states.v+states.ay*T;

    predictedStatesX(0) = estimatedStatesX(0)+(estimatedStatesX(3)*T+(estimatedStatesX(6)/2.0)*pow(T,2))*cos(estimatedStatesX(2)) - (estimatedStatesX(4)*T+(estimatedStatesX(7)/2.0)*pow(T,2))*sin(estimatedStatesX(2)); 
    predictedStatesX(1) = estimatedStatesX(1)+(estimatedStatesX(3)*T+(estimatedStatesX(6)/2.0)*pow(T,2))*sin(estimatedStatesX(2)) + (estimatedStatesX(4)*T+(estimatedStatesX(7)/2.0)*pow(T,2))*cos(estimatedStatesX(2));
    predictedStatesX(2) = estimatedStatesX(3) + estimatedStatesX(5)*T;
    predictedStatesX(3) = estimatedStatesX(3) + estimatedStatesX(6)*T;
    predictedStatesX(4) = estimatedStatesX(4) + estimatedStatesX(7)*T;
    
}

void ekfSLAM::Fx() {
    MatrixXf m(8,8); 
        m << 1, 0, 0, -states.u*T*sin(states.yaw)-states.v*T*cos(states.yaw), T*cos(states.yaw),-T*sin(states.yaw) ,0, 0, 
        0, 1, 0, states.u*T*cos(states.yaw)-states.v*T*sin(states.yaw), T*sin(states.yaw), T*cos(states.yaw), 0, 0, 
        0, 0, 1, 0, 0, 0, T, 0, 
        0, 0, 0, 1, 0, 0, 0, T,
        0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 1;
    matrixF = m;
}

void ekfSLAM::covariancePrediction() {
    matrixP = matrixF*matrixP*matrixF.transpose();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// DVL UPDATE FUNCTIONS //////////////////////////////////////////////////////////////////////////////////
void ekfSLAM::DVLH() {
    MatrixXf h(8,8);
    h << 0.00001,0,0,0,0,0,0,0,
    0,0.00001,0,0,0,0,0,0,
    0,0,0.00001,0,0,0,0,0,
    0,0,0,1,0,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,1;
    matrixHDVL = h;
}

void ekfSLAM::KalmanGainDVL() {
    matrixW = matrixP*matrixHDVL.transpose()*matrixSDVL.inverse();
}
void ekfSLAM::DVLupdate() {
    //cout << "Innovation: " << innovation << endl;
    cout << "PredictedX: " << predictedStatesX << endl;
    cout << "Z: " << measurementsZ << endl;
    cout << "Hx: " << matrixHDVL*predictedStatesX << endl;
    estimatedStatesX = predictedStatesX + matrixW*innovation;
    matrixP = (-matrixW*matrixHDVL)*matrixP;
}

void ekfSLAM::DVLinnovation() {
    innovation = measurementsZ - matrixHDVL*predictedStatesX;
}

void ekfSLAM::DVLcovariance() {
    matrixSDVL = matrixHDVL*matrixP*matrixHDVL.transpose();  // + R, Spør Øyvind om measurement matrix
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sonar UPDATE FUNCTIONS ////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////



// DEBUGGING /////////////////////////////////////////////////////////////////////////////////////////////

void ekfSLAM::printStates(bool xEstimate, bool xPrediction, bool pCoovariance) {
    
    if (xEstimate) {
        cout << "Position X: " << estimatedStatesX(0) << endl << "Velocity X: " << estimatedStatesX(4) << endl;
    }
    if (xPrediction) {
        cout << "Predict states: " << predictedStatesX << endl;
    }
    if (pCoovariance) {
        cout << "Estimated Coovariance: " << matrixP << endl;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////













/*
int main(){
    ekfSLAM slam;
    return 1;
}
*/