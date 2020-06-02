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
    
    DEBUG = false;
    

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
    predictedStatesX(0) = estimatedStatesX(0) + (measurementsZ(3)*T)*cos(measurementsZ(2)) + (measurementsZ(4)*T)*sin(measurementsZ(2)) ;
    predictedStatesX(1) = estimatedStatesX(1) + (measurementsZ(3)*T)*sin(measurementsZ(2)) - (measurementsZ(4)*T)*cos(measurementsZ(2));
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
    MatrixXf G(estimatedP.rows(),3);
    G.setZero();
    G.topLeftCorner(3,3) = MatrixXf::Identity(3,3);
    /*
    cout << "EstimatedP" << endl << estimatedP << endl;
    cout << "FHF: " << endl << matrixF*estimatedP*matrixF.transpose() << endl;
    cout << "Q: " << endl << G*matrixQ*G.transpose() << endl;
    cout << "Q: " << endl << matrixQ << endl;
    cout << "G: " << endl << matrixQ*G.transpose() << endl;
    cin >> debug;
    */
    predictedP = matrixF*estimatedP*matrixF.transpose() + G*matrixQ*G.transpose(); 
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
    bool InnovationDebug = false;

    if (DEBUG) {
        cout << "SONAR INNOVATION" << endl;
    }


    vector<float> tmp_innovation;
    vector<float> associated_predictions, associated_measurements;  
    // Variables
    newLandmarks.clear();
    JCBB bestMatch;
    if (predictedLines.meanRho.size() != 0) {

    if (InnovationDebug) {
        cout << "[INFO] Starting jointCompability..." << endl;
    }
    bestMatch = jointCompability(possibleLines,predictedLines);
    FORDEBUGGING = bestMatch;
    }
    if (InnovationDebug) {
        cout << "[INFO] Ending jointCompability..." << endl;
    }
    
    if (possibleLines.meanRho.size() > 0) {
        
        if (InnovationDebug) {
            cout << "[INFO] Calculating global positions..." << endl;
        }
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
                tmp_innovation.push_back(possibleLines.meanRho[bestMatch.matchedMeasurement[i]]-predictedLines.meanRho[bestMatch.matchedPrediction[i]]);
                tmp_innovation.push_back(possibleLines.meanTheta[bestMatch.matchedMeasurement[i]]-predictedLines.meanTheta[bestMatch.matchedPrediction[i]]);

            }      
            
        }
        
        if (associated_predictions.size()) {
            if (InnovationDebug) {
                cout << "[INFO] Calculating innovation..." << endl;
            }
            innovation = VectorXf(tmp_innovation.size());
            for (int i = 0; i < innovation.size(); i++) {
                innovation(i) = tmp_innovation[i];
            }
            if (InnovationDebug) {
                cout << "[INFO] Calculating assosiations for H..." << endl;
                cout << "H size assosiation, rows: " << assosiationH.rows() << " cols: " << assosiationH.cols() << endl;
                cout << "H size, rows: " << matrixH.rows() << " cols: " << matrixH.cols() << endl;
            }
            assosiationS = MatrixXf(associated_predictions.size()*2,associated_predictions.size()*2);
            assosiationH = MatrixXf(associated_predictions.size()*2,matrixH.rows()+3);
            assosiationS.setZero();
            assosiationH.setZero();
            
            for (int i = 0; i < associated_predictions.size()*2; i+=2) {
                assosiationH(i,0) = matrixH(i,0);
                assosiationH(i,1) = matrixH(i,1);
                assosiationH(i+1,2) = matrixH(i+1,2);
                
                for (int j = 0; j < associated_predictions.size(); j++) {
                    assosiationH(i,3+j*2) = matrixH(associated_measurements[i],3+associated_predictions[j]*2);
                    assosiationH(i,4+j*2) = matrixH(associated_measurements[i],4+associated_predictions[j]*2);
                    assosiationH(i+1,4+j*2) = matrixH(associated_measurements[i]+1,4+associated_predictions[j]*2);
                }
            }
            if (InnovationDebug) {
                cout << "[INFO] Calculating assosiations for S..." << endl;
                cout << "S size assosiation, rows: " << assosiationS.rows() << " cols: " << assosiationS.cols() << endl;
                cout << "S size, rows: " << matrixS.rows() << " cols: " << matrixS.cols() << endl;
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
    if (DEBUG) {
        cout << "ADD LANDMARK" << endl;
    }



    if (newLandmark) {
    
    MatrixXf tmp_predictedP;
    tmp_predictedP = predictedP;
    

    
    for (int i = 0; i < landmarkLines.meanRho.size(); i++) { 
        
        //cout << "addLandmark2" << endl;
        matrixMap.conservativeResize(matrixMap.rows()+2,matrixMap.cols()+2);
        matrixMap.row(matrixMap.rows()-2).setZero();
        matrixMap.row(matrixMap.rows()-1).setZero();
        matrixMap.col(matrixMap.cols()-2).setZero();
        matrixMap.col(matrixMap.cols()-1).setZero();
        matrixMap(matrixMap.rows()-2,matrixMap.rows()-2) = landmarkLines.meanRho[i];
        matrixMap(matrixMap.rows()-1,matrixMap.rows()-1) = landmarkLines.meanTheta[i];
        estimatedP.conservativeResize(estimatedP.rows()+2,estimatedP.cols()+2);
        estimatedP.row(estimatedP.rows()-2).setZero();
        estimatedP.row(estimatedP.rows()-1).setZero();
        estimatedP.col(estimatedP.cols()-2).setZero();
        estimatedP.col(estimatedP.cols()-1).setZero();
        estimatedP(estimatedP.rows()-2,estimatedP.rows()-2) = 1;
        estimatedP(estimatedP.rows()-1,estimatedP.rows()-1) = 0.57;
        //predictedStatesX.conservativeResize(predictedStatesX.rows()+2,1);
        //predictedStatesX(predictedStatesX.rows()-2) = landmarkLines.meanRho[i];
        //predictedStatesX(predictedStatesX.rows()-1) = landmarkLines.meanTheta[i];
        //cout << "PredictedX: "<< endl << predictedStatesX << endl;


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
    if (DEBUG) {
        cout << "PREDICT LANDMARKS" << endl;
    }

    // Variables
    float thetaL, Xl, Yl, rhoL;
    predictedLandmarks = VectorXf(matrixMap.rows());

    // Predict landmarks
    //cout << "PREDICTED: " << endl;
    for (int i = 0; i < matrixMap.rows(); i += 2 ) {
        
        Xl = matrixMap(i,i)*cos(matrixMap(i+1,i+1));
        Yl = matrixMap(i,i)*sin(matrixMap(i+1,i+1));
        rhoL = sqrt(pow(Xl - predictedStatesX(0),2)+pow(Yl-predictedStatesX(1),2));
        if (Yl > 0) {
            thetaL = acos((Xl-predictedStatesX(0))/rhoL)-predictedStatesX(2);
                }
        else {
            thetaL = -(acos((Xl-predictedStatesX(0))/rhoL)-predictedStatesX(2));
        }
        //cout << "Rho: " << rhoL << endl;
        //cout << "Theta: " << thetaL << endl;
        predictedLines.meanRho.push_back(rhoL);
        predictedLines.meanTheta.push_back(thetaL);

        } 
}


void ekfSLAM::resetVariables() {

    bool DEBUGvar = false;
   
    if (DEBUGvar) {
        cout << "LINES AND PREDICTIONS:" << endl;
        for (int i = 0; i < possibleLines.meanRho.size(); i++) {
            cout << "Line number " << i << ", RHO: " << possibleLines.meanRho[i] << ", Theta: " << possibleLines.meanTheta[i] << endl;
        }
        for (int i = 0; i < predictedLines.meanRho.size(); i++) {
            cout << "Prediction number " << i << ", RHO: " << predictedLines.meanRho[i] << ", Theta: " << predictedLines.meanTheta[i] << endl;
        }
        cout << "CORRESPONDING H AND S MATRIX: " << endl;
        cout << "Matrix H: " << endl << matrixH << endl;
        cout << "Matrix S: " << endl << matrixS << endl;
        cout << "ASSOSICATED PREDICTIONS AND LINES: " << endl;
        for (int i = 0; i < FORDEBUGGING.matchedMeasurement.size(); i++) {
            cout << "Measurement: " << FORDEBUGGING.matchedMeasurement[i] << ", Prediction: " <<  FORDEBUGGING.matchedPrediction[i] << endl;
        }
        cout << "ASSOCIATED H AND S MATRIXES" << endl;
        cout << "ASSOSICATED H" << endl << assosiationH << endl;
        cout << "ASSOSICATED S" << endl << assosiationS << endl;
        cout << "KALMAN GAIN" << endl << matrixW << endl;
        cout << "COVARIANCE AND STATE ESTIMATION" << endl;
        cout << "STATES: " << endl << estimatedStatesX << endl;
        cout << "COVARIANCE estimated: " << endl << estimatedP << endl;
        cout << "COVARIANCE predicted: " << endl << predictedP << endl;

        cin >> debug;    
    }




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


    
    

}

void ekfSLAM::gatingTest() {
    newLandmark = false;
    if (innovation.transpose()*matrixS.inverse()*innovation < 0.5) {
        newLandmark = true;
    }

}

void ekfSLAM::sonarS() {
    bool matrixSDebug = false;
    if (DEBUG) {
        cout << "S-MATRIX" << endl;
    }
    MatrixXf R(predictedP.rows()-3,predictedP.cols()-3);
    R.setZero();
    for (int i = 0; i < R.rows(); i++) {
        if ( i % 2 == 0) {
            R(i,i) = 1000;
        }
        else {
            R(i,i) = 1000;
        }
    }
    matrixS = 100*matrixH*predictedP*matrixH.transpose() + R;
    if (matrixSDebug) {
    cout << "MatrixH: " << endl << matrixH << endl;
    cout << "MatrixP " << endl << predictedP << endl;    
    cout << "MatrixS: " << endl << matrixS << endl;
    }
}

void ekfSLAM::sonarKalmanGain() {

    if (DEBUG) {
        cout << "KALMANGAIN" << endl;
    }


    MatrixXf m;
    MatrixXf A;
    
    A = assosiationS;
    m = predictedP*assosiationH.transpose();
    matrixW =  m*A.inverse();
    
    //cout << "MatrixS: " << endl << A.inverse() << endl;
    //cout << "MatrixH:" << endl << assosiationH << endl;
    //cout << "MatrixP: " << endl << predictedP << endl;
    //cout << "MatrixW: " << endl << matrixW << endl;
    //cout << "Innovation: " << endl << innovation << endl;
}

void ekfSLAM::sonarUpdate() {

    if (DEBUG) {
        cout << "SONAR UPDATE" << endl;
    }

    MatrixXf I;
    VectorXf tmp;
    I = MatrixXf::Identity(predictedP.rows(),predictedP.rows()); 
    if (matrixW.rows() > 0 && matrixW.cols() > 0) {
        tmp = predictedStatesX;
        predictedStatesX = VectorXf(3+matrixMap.rows());
        predictedStatesX(0) = tmp(0);
        predictedStatesX(1) = tmp(1);
        predictedStatesX(2) = tmp(2);
        
        
        for(int i = 3; i < predictedStatesX.size(); i++) {
            predictedStatesX(i) = matrixMap(i-3,i-3);
        }
        
        estimatedStatesX = predictedStatesX + (matrixW * innovation);

        for(int i = 3; i < predictedStatesX.size(); i++) {
            matrixMap(i-3,i-3) = estimatedStatesX(i);
        }

        estimatedP = (I - matrixW*matrixH)*predictedP;
        /*
        cout << "EstimatedP: " << endl << estimatedP << endl;
        cout << "MatrixW: " << endl << matrixW << endl;
        cout << "MatrixH: " << endl << matrixH << endl;


        cout << "Matrix I rows: " << I.rows() << ", cols: " << I.cols() << endl;
        cout << "W*H rows: " << (matrixW*matrixH).rows() << ", cols: " << (matrixW*matrixH).cols() << endl;
        
        */
        
    }
    else {
        estimatedStatesX = predictedStatesX;
        estimatedP = predictedP;
    }
    //estimatedStatesX = predictedStatesX;
    //cout << "The P-matrix: " << estimatedP << endl;
    //estimatedP = predictedP;
    
}   






//////////////////////////////////////////////////////////////////////////////////////////////////////////



// DEBUGGING /////////////////////////////////////////////////////////////////////////////////////////////

void ekfSLAM::printStates(bool xEstimate, bool xPrediction, bool pCoovariance) {
    
    if (xEstimate) {
        cout << "Position EstimatedX: " << estimatedStatesX(0) << ", EstimatedY: " << estimatedStatesX(1) << ", yaw: " << estimatedStatesX(2) << endl;
    }
    if (xPrediction) {
        cout << "Position PredictedX: " << predictedStatesX(0) << ", PredictedY: " << predictedStatesX(1) << ", yaw: " << predictedStatesX(2) << endl;
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











