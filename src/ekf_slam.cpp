#include "ekf_slam.hpp"


// Turn euler to Quaternions

theQuaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    theQuaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}




// INITIALIZATION /////////////////////////////////////////////////////////////////////////////////////////
void ekfSLAM::initialization(VectorXf X0, MatrixXf P0, MatrixXf Q) {
    
    // Initiating the states 
    states.x = X0(0); 
    states.y = X0(1);
    states.yaw = X0(2);

    estimatedStatesX = X0;
    statesEta = X0;

    // Initiating the covariance matrix
    estimatedP = P0;
    statesP = P0;

    // Initiating the process noise matrix
    matrixQ = Q;

    // Initiating a indentity matrix
    matrixI.Identity(3,3);

    // Initiating size of vectors;
    predictedStatesX = VectorXf(3);
    predictedStatesX.setZero();
    measurementsZ = VectorXf(8);
    measurementsZ.setZero();
    innovation = VectorXf(8);
    innovation.setZero();

    // Frequency
    T = 1.0/6.3;
    
    matrixMap.setZero();

    scan = MatrixXf(1000,1000);
    scan.setZero();
    

    

}

void ekfSLAM::getDVLMeasurements(float u, float v, float time, float yaw) {
    measurements.u = u;
	measurements.v = v;
	measurementsZ(3) = u;
	measurementsZ(4) = v;
    measurementsZ(2) = yaw;
}

// PREDICTION FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////
void ekfSLAM::odomPrediction() {
    predictedStatesX(0) = predictedStatesX(0) + (measurementsZ(3)*T)*cos(measurementsZ(2)) + (measurementsZ(4)*T)*sin(measurementsZ(2)) ;
    predictedStatesX(1) = predictedStatesX(1) + (measurementsZ(3)*T)*sin(measurementsZ(2)) - (measurementsZ(4)*T)*cos(measurementsZ(2));
    predictedStatesX(2) = measurementsZ(2);
    
}

void ekfSLAM::Fx() {
     // Skal jeg sette inn dT her? :S
    MatrixXf m(3,3); 
        m << 1, 0, -measurementsZ(3)*T*sin(predictedStatesX(2))+measurementsZ(4)*T*cos(predictedStatesX(2)), 
        0, 1, measurementsZ(3)*T*cos(predictedStatesX(2))+measurementsZ(4)*T*sin(predictedStatesX(2)), 
        0, 0, 1;   
    matrixF = MatrixXf::Identity(3+matrixMap.rows(),3+matrixMap.rows());
    matrixF.topLeftCorner(3,3) = m;


}

 void ekfSLAM::G() {
    matrixG = MatrixXf(matrixMap.rows()+3,3);
    matrixG.setZero();
    matrixG.topLeftCorner(3,3).Identity(3,3); 
 }


void ekfSLAM::covariancePrediction() {
    
    predictedP = matrixF*estimatedP*matrixF.transpose(); //+ matrixI*matrixQ*matrixI.transpose();
    
    //cout << "Early P: " << endl << predictedP << endl;
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////


 
// Sonar UPDATE FUNCTIONS ////////////////////////////////////////////////////////////////////////////////

void ekfSLAM::H() {

    matrixH = MatrixXf(matrixMap.rows(),matrixMap.rows()+3);
    matrixH.setZero();
    for (int i = 0; i < matrixMap.rows();i+=2) {
        matrixH(i,0) = -cos(matrixMap(i+1,i+1));
        matrixH(i,1) = -sin(matrixMap(i+1,i+1));
        matrixH(i+1,2) = -1;
        matrixH(i,i+3) = 1;
        matrixH(i,i+4) = predictedStatesX(0)*sin(matrixMap(i+1,i+1))-predictedStatesX(1)*cos(matrixMap(i+1,i+1));
        matrixH(i+1,i+4) = 1;
    
    }

    // Hm


    // Combined H 
}

void ekfSLAM::getLines(float meanRho, float meanTheta, float varRho, float varTheta) {
    if (isnan(meanRho) != true and meanRho > 1) {
        possibleLines.meanRho.push_back(meanRho);
        possibleLines.meanTheta.push_back(meanTheta);
        possibleLines.varRho.push_back(varRho);
        possibleLines.varTheta.push_back(varTheta);
        //cout << "Rho: " << meanRho << endl;
        //cout << "Theta: " << meanTheta << endl;
    }
}

void ekfSLAM::sonarInnovation() {
    vector<float> tmp_innovation;
    vector<float> associated_predictions, associated_measurements;  
    // Variables
    newLandmarks.clear();
    JCBB bestMatch;
    if (predictedLines.meanRho.size() != 0) {

    //cout << "[INFO] Starting jointCompability..." << endl;
    bestMatch = jointCompability(possibleLines,predictedLines);
    }
    
    if (possibleLines.meanRho.size() > 0) {
        
        
        for (int i = 0; i < possibleLines.meanTheta.size(); i++) {
            if (find(bestMatch.matchedMeasurement.begin(),bestMatch.matchedMeasurement.end(),i) == bestMatch.matchedMeasurement.end()) {
                //newLandmarks.push_back(i); 
                newLandmark = true;
                float Xl, Yl, rhoL, thetaL;
                Xl = cos(possibleLines.meanTheta[i])*possibleLines.meanRho[i];
                Yl = sin(possibleLines.meanTheta[i])*possibleLines.meanRho[i];
                rhoL = sqrt(pow(Xl+predictedStatesX(0),2)+pow(Yl+predictedStatesX(1),2));
                landmarkLines.meanRho.push_back(rhoL);
                if (Yl > 0) {
                    thetaL = acos((Xl+predictedStatesX(0))/rhoL) + predictedStatesX(2);
                }
                else {
                    thetaL = -acos((Xl+predictedStatesX(0))/rhoL) + predictedStatesX(2);
                }
                landmarkLines.meanTheta.push_back(thetaL); 
                landmarkLines.varRho.push_back(possibleLines.varRho[i]);
                landmarkLines.varTheta.push_back(possibleLines.varTheta[i]);
        
            }

            else {
                associated_predictions.push_back(bestMatch.matchedPrediction[i]);
                associated_measurements.push_back(bestMatch.matchedMeasurement[i]);
                tmp_innovation.push_back(possibleLines.meanRho[i]-predictedLines.meanRho[bestMatch.matchedPrediction[i]]);
                tmp_innovation.push_back(possibleLines.meanTheta[i]-predictedLines.meanTheta[bestMatch.matchedPrediction[i]]);

            }      
            
        }
        
        if (associated_predictions.size()) {
            
            innovation = VectorXf(tmp_innovation.size());
            for (int i = 0; i < innovation.size(); i++) {
                innovation(i) = tmp_innovation[i];
            }
            
            assosiationS = MatrixXf(associated_predictions.size()*2,associated_predictions.size()*2);
            assosiationH = MatrixXf(associated_predictions.size()*2,matrixH.rows()+3);
            assosiationS.setZero();
            assosiationH.setZero();
            //cout << "H size assosiation, rows: " << assosiationH.rows() << " cols: " << assosiationH.cols() << endl;
            //cout << "H size, rows: " << matrixH.rows() << " cols: " << matrixH.cols() << endl;
            for (int i = 0; i < associated_predictions.size()*2; i+=2) {
                assosiationH(i,0) = matrixH(i,0);
                assosiationH(i,1) = matrixH(i,1);
                assosiationH(i+1,2) = matrixH(i+1,2);
                
                for (int j = 0; j < associated_predictions.size(); j++) {
                    assosiationH(i,3+j*2) = matrixH(associated_measurements[i],3+associated_predictions[j]*2);
                    assosiationH(i,4+j*2) = matrixH(associated_measurements[i],4+associated_predictions[j]*2);
                    assosiationH(i+1,4+j*2) = matrixH(associated_measurements[i]+1,4+associated_predictions[j]*2);
                    
                   
                    //cout << "AssosiationS: " << endl << assosiationS << endl;
                    //cout << "Matrix S: " << endl << matrixS << endl;

                }
            }
            
            for (int i = 0; i < associated_predictions.size(); i++) {
                for (int j=0; j < associated_predictions.size(); j++ ) {
                    assosiationS(i*2,j*2) = matrixS(associated_predictions[i]*2,associated_predictions[j]*2);
                    assosiationS(i*2,j*2+1) = matrixS(associated_predictions[i]*2,associated_predictions[j]*2+1);
                    assosiationS(i*2+1,j*2) = matrixS(associated_predictions[i]*2+1,associated_predictions[j]*2);
                    assosiationS(i*2+1,j*2+1) = matrixS(associated_predictions[i]*2+1,associated_predictions[j]*2+1);
            }
                }
                
               
            

        }
        
        
      
    }


}


void ekfSLAM::addLandmark () {
    int debug;
    



    if (newLandmark) {
    //cout << "addLandmark1" << endl;
    MatrixXf tmp_predictedP;
    tmp_predictedP = predictedP;
    

    
    for (int i = 0; i < landmarkLines.meanRho.size(); i++) { 
        
        //cout << "addLandmark2" << endl;
        matrixMap.conservativeResize(matrixMap.rows()+2,matrixMap.cols()+2);
        matrixMap(matrixMap.rows()-2,matrixMap.rows()-2) = landmarkLines.meanRho[i];
        matrixMap(matrixMap.rows()-1,matrixMap.rows()-1) = landmarkLines.meanTheta[i];
        predictedP.conservativeResize(predictedP.rows()+2,predictedP.cols()+2);
        predictedP.row(predictedP.rows()-2).setZero();
        predictedP.row(predictedP.rows()-1).setZero();
        predictedP.col(predictedP.cols()-2).setZero();
        predictedP.col(predictedP.cols()-1).setZero();

        predictedP(predictedP.rows()-2,predictedP.rows()-2) = landmarkLines.varRho[i];
        predictedP(predictedP.rows()-1,predictedP.rows()-1) = landmarkLines.varTheta[i];

    }

    newLandmark = false;
    }

    
    
    
    //cout << "Count: " <<  count << endl;
    cout << "Amount of landmarks found: "<< matrixMap.rows()/2 << endl;
    
    
    //cout << "MapMatrix" << endl << matrixMap << endl;
    
    //cout << "Amount of landmarks found: "<< matrixMap.rows()/2 << endl;
    //cout << "LANDMARKS" << endl;
    //for (int i = 0; i < matrixMap.rows(); i+=2) {
    //    cout << "Rho: " << matrixMap(i,i) << endl << "Theta: " << matrixMap(i+1,i+1) << endl;
    //}
}

void ekfSLAM::predictLandmarks() {
    // Variables
    float thetaL, Xl, Yl, rhoL;
    predictedLandmarks = VectorXf(matrixMap.rows());

    // Predict landmarks
    //cout << "PREDICTED: " << endl;
    for (int i = 0; i < matrixMap.rows(); i += 2 ) {
        
        Xl = matrixMap(i,i)*cos(matrixMap(i+1,i+1));
        Yl = matrixMap(i,i)*sin(matrixMap(i+1,i+1));
        rhoL = sqrt(pow(Xl - predictedStatesX[0],2)+pow(Yl-predictedStatesX(1),2));
        if (Yl > 0) {
            thetaL = acos((Xl-predictedStatesX(0))/rhoL)-predictedStatesX(3);
                }
        else {
            thetaL = -(acos((Xl-predictedStatesX(0))/rhoL)-predictedStatesX(3));
        }
        //cout << "Rho: " << rhoL << endl;
        //cout << "Theta: " << thetaL << endl;
        predictedLines.meanRho.push_back(rhoL);
        predictedLines.meanTheta.push_back(thetaL);

        } 
}


void ekfSLAM::resetVariables() {
    theQuaternion q;
    possibleLines.meanRho.clear();
    possibleLines.meanTheta.clear();
    possibleLines.varRho.clear();
    possibleLines.varTheta.clear();
    predictedLines.meanRho.clear();
    predictedLines.meanTheta.clear();
    predictedLines.varRho.clear();
    predictedLines.varTheta.clear();
    landmarkLines.meanRho.clear();
    landmarkLines.meanTheta.clear();
    landmarkLines.varRho.clear();
    landmarkLines.varTheta.clear();
    //cout << "Position x: " << predictedStatesX(0) << endl;
    //cout << "Position y: " << predictedStatesX(1) << endl;
    //q = ToQuaternion(predictedStatesX(2),0,0);
    //cout << "Yaw: " << q.z << endl;

}

void ekfSLAM::gatingTest() {
    newLandmark = false;
    if (innovation.transpose()*matrixS.inverse()*innovation < 0.5) {
        newLandmark = true;
    }

}

void ekfSLAM::sonarS() {
    //cout << "H - rows: " << matrixH.rows() << ", cols: " << matrixH.cols() << endl;
    //cout << "P - rows: " << predictedP.rows() << ", cols: " << predictedP.cols() << endl;
     matrixS = matrixH*predictedP*matrixH.transpose();
}

void ekfSLAM::sonarKalmanGain() {
    MatrixXf m;
    MatrixXf A;
    
    A = assosiationS;
    m = predictedP*assosiationH.transpose();
    matrixW =  A.inverse()*m;

}

void ekfSLAM::sonarUpdate() {
    //estimatedStatesX = predictedStatesX + matrixW * innovation;
    //estimatedP = (matrixI - matrixW*matrixH)*predictedP;
    //cout << "P: " << endl << predictedP << endl;
    estimatedP = predictedP;
    
}   






//////////////////////////////////////////////////////////////////////////////////////////////////////////



// DEBUGGING /////////////////////////////////////////////////////////////////////////////////////////////

void ekfSLAM::printStates(bool xEstimate, bool xPrediction, bool pCoovariance) {
    
    if (xEstimate) {
        cout << "Position X: " << estimatedStatesX(0) << endl << "Velocity X: " << estimatedStatesX(4) << endl;
    }
    if (xPrediction) {
        cout << "Position X: " << predictedStatesX(0) << ", Velocity X: " << predictedStatesX(3) << endl;
    }
    if (pCoovariance) {
        cout << "Estimated Coovariance: " << estimatedP << endl;
    }
}


void ekfSLAM::visualiseMap(votingBins data) {
    // Visualisering av landmarks found
    int scale = 50;
    float tmp_width, tmp_height;
    float origo_height, origo_width, lineAngle;
    int y;
    float n;
    int count = 0;
    Mat gray_display,tmp_image;

    // Scan:
    for (int i = 0; i < data.ranges.size(); i++) {
        tmp_width = predictedStatesX(1) + data.ranges[i]*sin(data.angles[i]+predictedStatesX(2));
        tmp_height = predictedStatesX(0) + data.ranges[i]*cos(data.angles[i]+predictedStatesX(2));
        tmp_width = (500) - (tmp_width*scale);
        tmp_height = (1000-5) - (tmp_height*scale);
        scan((int)tmp_height,(int)tmp_width) = 255;      
    }
    
    // Lines :)
    /*
    for (int i = 0; i < matrixMap.rows(); i+=2) {
            origo_width = (matrixMap(i,i)+0.1)*sin(matrixMap(i+1,i+1));
            origo_height = sqrt(pow((matrixMap(i,i)+0.1),2.0) - pow(origo_width,2.0));
            origo_width = (500) - (origo_width*scale);
            origo_height = (1000-5) - (origo_height*scale);
            lineAngle = -matrixMap(i+1,i+1);
            n = origo_height - (tan(lineAngle) * origo_width);
            count += 1;
            
     
    
     for (int x = 0; x < 1000; x++) {
            y = int(x * tan(lineAngle) + n);
            if (y >= 0 && y < 1000) {
                scan(y,x) = 255;
            }
     }
    }
    //tmp_scan(origo_height,origo_width) = 100;
    */
    
    // Convert to openCV and display
    eigen2cv(scan,tmp_image);
    tmp_image.convertTo(gray_display, CV_8U);
    namedWindow("Line display2", 0);
    resizeWindow("Line display2", 2000, 2000);
    imshow("Line display2", gray_display);
    waitKey(1);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////













/*
int main(){
    ekfSLAM slam;
    return 1;
}
*/



/*
// DVL UPDATE FUNCTIONS //////////////////////////////////////////////////////////////////////////////////
void ekfSLAM::DVLH() {
    MatrixXf h(8,8);
    h << 1,0,0,0,0,0,0,0,
    0,1,0,0,0,0,0,0,
    0,0,1,0,0,0,0,0,
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
*/