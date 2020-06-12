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
    test_yaw = 0;
    estiamted_YAW = 0;

    // Frequency
    T = 1.0/6.3;
    

    matrixMap.setZero();
    // Predefined 40min MAP
    if (false) {
        matrixMap = MatrixXf(6,6);
        estimatedP = MatrixXf(9,9);
        estimatedP.setZero();
        matrixMap.setZero();
        matrixMap(0,0) = 3.2;
        matrixMap(1,1) = 1.72;
        matrixMap(2,2) = 3.49;
        matrixMap(3,3) = -1.4963;
        matrixMap(4,4) = 4.17247;
        matrixMap(5,5) = -2.7;
        estimatedP(3,3) = 0.3;
        estimatedP(4,4) = 0.3;
        estimatedP(3,0) = cos(1.67);
        estimatedP(3,1) = sin(1.67);
        estimatedP(4,2) = -1;
        estimatedP(0,3) = cos(1.4962);
        estimatedP(1,3) = sin(1.4962);
        estimatedP(2,4) = -1;
        estimatedP(5,5) = 0.3;
        estimatedP(6,6) = 0.3;
        estimatedP(5,0) = cos(-1.4963);
        estimatedP(5,1) = sin(-1.4963);
        estimatedP(6,2) = -1;
        estimatedP(0,5) = cos(-1.4963);
        estimatedP(1,5) = sin(-1.4963);
        estimatedP(2,6) = -1;
        estimatedP(7,7) = 0.3;
        estimatedP(8,8) = 0.3;
        estimatedP(7,0) = cos(-2.7);
        estimatedP(7,1) = sin(-2.7);
        estimatedP(8,2) = -1;
        estimatedP(0,7) = cos(-2.7);
        estimatedP(1,7) = sin(-2.7);
        estimatedP(2,8) = -1;
    }




    scan = MatrixXf(1000,1000);
    scan.setZero();
    
    DEBUG = false;
    prev_ekf_yaw = 0;
    prev_ekf_x = 0;
    prev_ekf_y = 0;
    

}

void ekfSLAM::getDVLMeasurements(float u, float v, float time, float yaw) {
    //cout << "YAY: " << yaw << endl;
	float change_in_yaw = yaw - prev_yaw;
    //cout << "change in yaw: " << change_in_yaw << endl;
    if (change_in_yaw > 1 || change_in_yaw < -0.1) {
        change_in_yaw = 0;
    }
    measurementsZ(3) = u;
	measurementsZ(4) = v;
    if (change_in_yaw > 0.2 || change_in_yaw < -0.2  ) {
        
    measurementsZ(2) = prev_yaw + 0.8*change_in_yaw;
    //test_yaw = change_in_yaw;
    //}
    //if (change_in_yaw < -0.4) {
    //    measurementsZ(2) = prev_yaw + 0.4;
    }
    else {
    measurementsZ(2) = yaw;  
    prev_yaw = yaw;  
    }
    test_yaw = change_in_yaw;

    cout << "Measurement u: " << u << endl;
    cout << "Meausrement v: " << v << endl;    
}

// PREDICTION FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////
void ekfSLAM::odomPrediction() {
    theQuaternion q;
    predictedStatesX(0) = estimatedStatesX(0) + (measurementsZ(3)*T)*cos(measurementsZ(2)) + (measurementsZ(4)*T)*sin(measurementsZ(2)) ;
    predictedStatesX(1) = estimatedStatesX(1) + (measurementsZ(3)*T)*sin(measurementsZ(2)) - (measurementsZ(4)*T)*cos(measurementsZ(2));
    //predictedStatesX(2) = measurementsZ(2);//estimatedStatesX(2) + measurementsZ(2);//estimatedStatesX(2)  + measurementsZ(2);
    predictedStatesX(2) = estimatedStatesX(2) + test_yaw;



        if (predictedStatesX(2) < -3.1415 ) {
            predictedStatesX(2) = predictedStatesX(2) + 2*3.1415;
        }

        if (predictedStatesX(2) > 3.1415 ) {
            predictedStatesX(2) = predictedStatesX(2) - 2*3.1415;
        }

    /*
    q = ToQuaternion(predictedStatesX(2), 0, 0);
    cout << "QUATS:" << endl;
    cout << "x = " << q.x << endl;
    cout << "y = " << q.y << endl;
    cout << "z = " << q.z << endl;
    cout << "w = " << q.w << endl;
    cout << "X: " << predictedStatesX(0) << endl;
    cout << "Y: " << predictedStatesX(1) << endl;
    cout << "YAW: " << predictedStatesX(2) << endl;

    //cout << "YAW: " << predictedStatesX(2) << endl;;
    */
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
        //cout << "HELLO!?!?!!=!=" << endl;
    }
}

void ekfSLAM::sonarInnovation() {
    bool InnovationDebug = false;

    if (DEBUG) {
        cout << "SONAR INNOVATION" << endl;
    }

    possibleInnovation = VectorXf(matrixMap.rows());
    possibleInnovation.setZero();
    vector<float> tmp_innovation;
    vector<float> associated_predictions, associated_measurements;  
    vector<int>::iterator it;
    // Variables
    newLandmarks.clear();
    JCBB bestMatch;
    int index; 
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
        int count = 0;
        for (int i = 0; i < possibleLines.meanTheta.size(); i++) {

            
            it = find(bestMatch.matchedMeasurement.begin(),bestMatch.matchedMeasurement.end(),i);
            index = distance(bestMatch.matchedMeasurement.begin(), it);
            
            
            if (it == bestMatch.matchedMeasurement.end()) {
                
                //newLandmarks.push_back(i); 
                newLandmark = true;
                float Xlocal, Ylocal, Xl, Yl, rhoL, thetaL, a, b, c, x, y, n, X, Y, test_rho;
                //cout << "Line RHO: " << possibleLines.meanRho[i] << ", Line Theta: " << possibleLines.meanTheta[i] << endl;
                X = predictedStatesX(0) + cos(possibleLines.meanTheta[i]+predictedStatesX(2))*possibleLines.meanRho[i];
                Y = predictedStatesX(1) + sin(possibleLines.meanTheta[i]+predictedStatesX(2))*possibleLines.meanRho[i];

                //cout << "Estimated X: " << predictedStatesX(0) << endl;
                //cout << "Esimtaed Y: " << predictedStatesX(1) << endl;
                //cout << "Estiamted Yaw: " << predictedStatesX(2) << endl;

                //cout << "X: " << X <<  ", Y: " << Y << endl;

                //cout << "X: " << X << ", Y: " << Y << endl;

                test_rho = sqrt(pow(X,2)+pow(Y,2));

                
                /*
                if (Y > 0) {
                    thetaL = acos(X/test_rho);
                }
                else {
                    thetaL = -acos(X/test_rho);
                }
                */
                thetaL = possibleLines.meanTheta[i] + predictedStatesX(2);
                

                thetaL = thetaL + (3.1415/2);

                if (thetaL > 3.1415) {
                    thetaL = thetaL - 2*3.1415;
                }
                if (thetaL < - 3.1415) {
                    thetaL = thetaL + 2*3.1415;
                }

                //cout << "Line Angle: " << thetaL << endl;


                n = (Y - (tan(thetaL) * (X)));
                a = 1;
                b = -tan(thetaL);
                c = -n;
                y = (b*(0*b-0*a)-a*c)/(pow(a,2)+pow(b,2));
                x = (a*(-0*b+0*a)-b*c)/(pow(a,2)+pow(b,2));
                
                //cout << "X: " << x << ", Y: " << y << endl;
                float true_rho, true_theta;

                true_rho = sqrt(pow(x,2)+pow(y,2));
                if (y > 0 ) {
                    true_theta = acos(x/true_rho);
                }
                else {
                    true_theta = -acos(x/true_rho);
                }


                if (true_theta > 3.1415) {
                    true_theta = true_theta - 2*3.1415;
                }
                if (true_theta < - 3.1415) {
                    true_theta = true_theta + 2*3.1415;
                }

                

                
                landmarkLines.meanRho.push_back(true_rho);
                landmarkLines.meanTheta.push_back(true_theta); 
                landmarkLines.varRho.push_back(possibleLines.varRho[i]);
                landmarkLines.varTheta.push_back(possibleLines.varTheta[i]);


            }
           
            else {
                //cout << "IT: " << *it << endl;
                associated_predictions.push_back(bestMatch.matchedPrediction[i]);
                
                associated_measurements.push_back(bestMatch.matchedMeasurement[i]);
                

                //if (predictedLines.meanRho[i] > 1.5) {
                tmp_innovation.push_back(possibleLines.meanRho[i]-predictedLines.meanRho[bestMatch.matchedPrediction[index]]);
                
                tmp_innovation.push_back(possibleLines.meanTheta[i]-predictedLines.meanTheta[bestMatch.matchedPrediction[index]]);
                
                possibleInnovation(bestMatch.matchedPrediction[index]*2) = possibleLines.meanRho[i]-predictedLines.meanRho[bestMatch.matchedPrediction[index]];
                
                possibleInnovation(bestMatch.matchedPrediction[index]*2+1) = possibleLines.meanTheta[i]-predictedLines.meanTheta[bestMatch.matchedPrediction[index]];



                //}
                //else {
                //    possibleInnovation(bestMatch.matchedPrediction[index]*2) = 0;
                
                //    possibleInnovation(bestMatch.matchedPrediction[index]*2+1) = 0;

                //}
                //cout << "Place: " << bestMatch.matchedPrediction[i] << endl;
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
        estimatedP(estimatedP.rows()-2,estimatedP.rows()-2) = 0.3;
        estimatedP(estimatedP.rows()-1,estimatedP.rows()-1) = 0.3;
        estimatedP(estimatedP.rows()-2,0) = cos(landmarkLines.meanTheta[i]);
        estimatedP(estimatedP.rows()-2,1) = sin(landmarkLines.meanTheta[i]);
        estimatedP(estimatedP.rows()-1,2) = -1;
        estimatedP(0,estimatedP.rows()-2) = cos(landmarkLines.meanTheta[i]);
        estimatedP(1,estimatedP.rows()-2) = sin(landmarkLines.meanTheta[i]);
        estimatedP(2,estimatedP.rows()-1) = -1;
        //predictedStatesX.conservativeResize(predictedStatesX.rows()+2,1);
        //predictedStatesX(predictedStatesX.rows()-2) = landmarkLines.meanRho[i];
        //predictedStatesX(predictedStatesX.rows()-1) = landmarkLines.meanTheta[i];
        //cout << "PredictedX: "<< endl << predictedStatesX << endl;
        cout << "LANDMARK, rho: " << landmarkLines.meanRho[i] << ", theta: " << landmarkLines.meanTheta[i] << endl;

    }

    newLandmark = false;
    }

    
    
    
    //cout << "Count: " <<  count << endl;
    //cout << "Amount of landmarks found: "<< matrixMap.rows()/2 << endl;
    
    
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
    float thetaL, Xl, Yl, rhoL, a, b, c, x, y, n, localY;
    float debug_theta, theta_2;
    bool test;
    test = false;


    // Predict landmarks
    //cout << "PREDICTED: " << endl;

    for (int i = 0; i < matrixMap.rows(); i += 2 ) {
        
        Xl = matrixMap(i,i)*cos(matrixMap(i+1,i+1));
        Yl = matrixMap(i,i)*sin(matrixMap(i+1,i+1));

        //cout << "PREDICTION!!!" << endl;
        
       

        thetaL = matrixMap(i+1,i+1) + (3.1415/2);

        if (thetaL > 3.1415) {
            thetaL = thetaL - 2*3.1415;
        }
        if (thetaL < - 3.1415) {
            thetaL = thetaL + 2*3.1415;
        }

        debug_theta = thetaL;

        //cout << "Line theta: " << thetaL << endl;

        



        //n = Yl - (tan(thetaL)*Xl);
        n = Yl - (tan(thetaL)*Xl);
        a = 1;
        b = -tan(thetaL);
        c = -n;

        

        float B1, C1, B2, C2, sumX, sumC, localX;
        float xTerm, yTerm;







        B1 = -b;
        C1 = -c;
        B2 = -(1/B1);
        C2 = predictedStatesX(1) - B2*predictedStatesX(0);
        sumX = B1 - B2;
        sumC = C1 - C2;
        x = -float(sumC)/float(sumX);
        y = B1*x + C1;

        rhoL = sqrt(pow(x-predictedStatesX(0),2)+pow(y-predictedStatesX(1),2));
        if (Xl > 0) {
            localY = sin(predictedStatesX(2))*(x-predictedStatesX(0)) + cos(predictedStatesX(2))*(y-predictedStatesX(1)); 
        }
        if (Xl < 0) {
           localY =  -sin(predictedStatesX(2))*(x-predictedStatesX(0)) + cos(predictedStatesX(2))*(y-predictedStatesX(1));
        }

        if (predictedStatesX(2) < -3.1415/2 && Yl < 0) {
            //cout << "TRUE" << ", LocalY " << localY << endl;
            localY = abs(localY);
        }

        if (predictedStatesX(2) > 3.1415/2 && Yl > 0) {
            localY = -localY;
        }


        //if (predictedStatesX(2) < -3.1415/2 || predictedStatesX(2) > 3.1415/2) {
        //    localY = -localY;
        //}
        //if (predictedStatesX(2) < -3.1415/2 && Yl < 0) {
        //    localY = -localY;
        //    cout << "IS THIS USED?" << endl;
        //}





        
        




        localX = cos(predictedStatesX(2))*(x-predictedStatesX(0)) - sin(predictedStatesX(2))*(y-predictedStatesX(1)); 
        if (predictedStatesX(2) < -3.1415/2 || predictedStatesX(2) > 3.1415/2) {
        if (localY > 0) {
            thetaL = -acos((x-predictedStatesX(0))/rhoL) - predictedStatesX(2);
        }
        else {
            thetaL = acos((x-predictedStatesX(0))/rhoL) - predictedStatesX(2);
            }

        }
        else {
        if (localY > 0) {
            thetaL = acos((x-predictedStatesX(0))/rhoL) - predictedStatesX(2);
        }
        else {
            thetaL = -acos((x-predictedStatesX(0))/rhoL) - predictedStatesX(2);
        }
        }
  





        
        



        if (thetaL > 3.1415) {
            thetaL = thetaL - 2*3.1415;
        }
        if (thetaL < - 3.1415) {
            thetaL = thetaL + 2*3.1415;
        }

        

        //cout << "Predicted Theta: " << thetaL << endl;

        //cout << "Rho: " << rhoL << endl;
        //cout << "Theta: " << thetaL << endl;
        //cout << "YAW: " << predictedStatesX(2) << endl;
        predictedLines.meanRho.push_back(rhoL);
        predictedLines.meanTheta.push_back(thetaL);
        //cin >> debug;
        theta_2 = thetaL;
        
         
        /*
        if (matrixMap.rows() > 20 ) {
           
           
            //cout << "LANDMARK AND STATES: " << i << endl;
            cout << "Landmark "<< i/2 << ", Rho: " << matrixMap(i,i) << ", Theta: " << matrixMap(i+1,i+1) << endl;
            cout << "X: " << predictedStatesX(0) << ", Y: " << predictedStatesX(1) << ", YAW: " << predictedStatesX(2) <<  endl;
            
            //cout << "X,Y, first theta" << endl;
            //cout << "X: " << Xl << ", Y: " << Yl << ", Theta: " << debug_theta<< endl;
            
            //cout << "N, A, B, C, X, Y" << endl;
            //cout << "n: " << n << ", a: " << a << ", b: " << b << ", c: " << c << endl;


            //cout << "B1 " << B1 << ", C1 " << C1 << ", B2 " << B2 << ", C2 " << C2 << endl;
            //cout << "SumC " << sumC << ", SumX " << sumX << endl;
            cout << "Predicted: "; 
            cout << "X: " << x << "Y: " << y << endl;
            
            cout << "LOCAL Y, THETA and RHO" << endl;
            cout << "Rho: " << rhoL << ", Theta: " << thetaL << ", LocalY: " << localY << endl;
            cout << "THETA OF INTEREET: " << theta_2 << endl;
            cout << "Test " << test << endl;
            cin >> debug; 
            

        }
         
           
        //cout << "YAW: " << predictedStatesX(2) << endl;
        test = false;
         for (int i = 0; i < possibleLines.meanRho.size(); i++ ) {
            cout << "Possible line " << i << ", RHO: " << possibleLines.meanRho[i] << ", Theta: " << possibleLines.meanTheta[i] << endl;
            cout << "Desired X " << cos(predictedStatesX(2)+possibleLines.meanTheta[i])*possibleLines.meanRho[i] + predictedStatesX(0)<< ", Desired Y: " << sin(predictedStatesX(2)+possibleLines.meanTheta[i])*possibleLines.meanRho[i] + predictedStatesX(1)<< endl;
            }  
     */           
    }
    

     //cout << "XL: " << Xl << ", YL: " << Yl << endl;

    /*
    if (possibleLines.meanRho.size() > 0) {

        for (int i = 0; i < possibleLines.meanRho.size(); i++) {
            cout << "Possible line " << i << ", Rho: " << possibleLines.meanRho[i] << ", Theta: " << possibleLines.meanTheta[i] << endl;
        }

        for (int j = 0; j < predictedLines.meanRho.size(); j++) {
            cout << "Predicted line " << j << ", Rho: " << predictedLines.meanRho[j] << ", Theta: " << predictedLines.meanTheta[j] << endl;
        }

        cout << "ESTIMATED ANGLE: " << predictedStatesX(2) << ", X: " << predictedStatesX(0) << ", Y: " << predictedStatesX(1) << endl;
        cout << "Amount of landmarks found: "<< matrixMap.rows()/2 << endl;
        for (int i = 0; i < matrixMap.rows(); i+=2) {
        cout << "Rho: " << matrixMap(i,i) << endl << "Theta: " << matrixMap(i+1,i+1) << endl;
    }
    }
    */
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

        //cin >> debug;    
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
            R(i,i) = 0.3;
        }
        else {
            R(i,i) = 0.3;
        }
    }
    matrixS = (matrixH*predictedP*matrixH.transpose() + R)*3; // 5 er for faktiske landmarks
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
    
    //A = assosiationS;
    //m = predictedP*assosiationH.transpose();
    
    A = matrixS;
    m = predictedP*matrixH.transpose();
    matrixW =  m*A.inverse();
    
    //cout << "S: " << A.size() << ", M: " << m.size() << endl;
    
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
        
        

        


        //cout << "KALMAN matrix" << endl;
        //cout << matrixW << endl;

        //cout << "GAIN" << endl;

        //cout << "YAW: " << estimatedStatesX(2) << endl;
        //cout << "UPDATE:" << endl;
        //cout << "X: " << (matrixW * possibleInnovation)(0) <<  endl;
        //cout << "Y: " << (matrixW * possibleInnovation)(1) << endl;
        //cout << "YAW: " << (matrixW * possibleInnovation)(2) << endl;

        //cout << "INNOVATION " << possibleInnovation << endl;
        
        /*
        if ((matrixW * possibleInnovation)(2) < 0) {
            cout << "HØYRE SVING" << endl;
        }

        if ((matrixW * possibleInnovation)(2) > 0) {
            cout << "VENSTIRE SVING" << endl;
        }
        */
        //cout << matrixW * possibleInnovation << endl;
            estimatedStatesX(0) = predictedStatesX(0);
            estimatedStatesX(1) = predictedStatesX(1);

            for (int i = 0; i < predictedStatesX.rows(); i++) {
                if ((matrixW*possibleInnovation)(i) > 0.05 ) {
                    estimatedStatesX(i) = predictedStatesX(i) + 0.05;
                }
                if ((matrixW*possibleInnovation)(i) < -0.05) {
                    estimatedStatesX(i) = predictedStatesX(i) - 0.05;
                }
                else {
                    estimatedStatesX(i) = predictedStatesX(i) + (matrixW * possibleInnovation)(i);
                }
            }


        
            
            estimatedP = (I - matrixW*matrixH)*predictedP;
            for(int i = 3; i < predictedStatesX.size(); i++) {
                matrixMap(i-3,i-3) = estimatedStatesX(i);
            }
        
            if (possibleInnovation.rows() > 0) {
                //cout << "UPDATE" << endl;
                //cout << matrixW*possibleInnovation << endl;
            }

        
        
    }
    else {
        cout << "PREDICTION" << endl;
        estimatedStatesX = predictedStatesX;
        estimatedP = predictedP;
    }
    //estimatedStatesX = predictedStatesX;
    //cout << "The P-matrix: " << estimatedP << endl;
    //estimatedP = predictedP;
    
}   


void ekfSLAM::getMeasurements(float ekf_x,float ekf_y,float ekf_yaw) {
    change_in_x = ekf_x - prev_ekf_x;
    change_in_y = ekf_y - prev_ekf_y;
    change_in_angle = ekf_yaw - prev_ekf_yaw;

    prev_ekf_x = ekf_x;
    prev_ekf_y = ekf_y;
    prev_ekf_yaw = ekf_yaw;

}

void ekfSLAM::prediction() {
    predictedStatesX(0) = estimatedStatesX(0) + change_in_x;
    predictedStatesX(1) = estimatedStatesX(1) + change_in_y;
    predictedStatesX(2) = estimatedStatesX(2) + change_in_angle;



    if (predictedStatesX(2) < -3.1415 ) {
        predictedStatesX(2) = predictedStatesX(2) + 2*3.1415;
    }

    if (predictedStatesX(2) > 3.1415 ) {
        predictedStatesX(2) = predictedStatesX(2) - 2*3.1415;
    }

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////

// NiceFunction //////////////////////////////////////////////////////////////////////////////////////////
Vector3f ekfSLAM::getStates() {
    Vector3f states;
    states(0) = estimatedStatesX(0);
    states(1) = estimatedStatesX(1);
    states(2) = estimatedStatesX(2);
    return states;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////



// DEBUGGING /////////////////////////////////////////////////////////////////////////////////////////////

void ekfSLAM::printStates(bool xEstimate, bool xPrediction, bool pCoovariance) {
    
    if (xEstimate) {
        cout << "EstimatedX: " << estimatedStatesX(0) << endl << "EstimatedY: " << estimatedStatesX(1) << endl << "Yaw: " << estimatedStatesX(2) << endl;
    }
    if (xPrediction) {
        cout << "Position PredictedX: " << predictedStatesX(0) << ", PredictedY: " << predictedStatesX(1) << ", yaw: " << predictedStatesX(2) << endl;
    }
    if (pCoovariance) {
        cout << "Estimated Coovariance: " << estimatedP << endl;
    }

    //cout << "Predited Yaw: " << estimatedStatesX(2) << " , True Yaw: " << estiamted_YAW << endl;

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
    scan.setZero();

    // SCAN
    for (int i = 0; i < data.ranges.size(); i++) {
        tmp_width = predictedStatesX(1) + data.ranges[i]*sin(data.angles[i]+predictedStatesX(2));
        tmp_height = predictedStatesX(0) + data.ranges[i]*cos(data.angles[i]+predictedStatesX(2));
        tmp_width = (500) - (tmp_width*scale);
        tmp_height = (500) - (tmp_height*scale);
        scan((int)tmp_height,(int)tmp_width) = 255;      
    }
    
    // LANDMARKS
    
    for (int i = 0; i < matrixMap.rows(); i+=2) {

            //cout << "Landmark: " << i/2 << ", RHO: " << matrixMap(i,i) << ", Theta: " << matrixMap(i+1,i+1) << endl;
            origo_width = (matrixMap(i,i)+0.1)*sin(matrixMap(i+1,i+1));
            origo_height = (matrixMap(i,i)+0.1)*cos(matrixMap(i+1,i+1));
            origo_width = (500) - (origo_width*scale);
            origo_height = (500) - (origo_height*scale);
            lineAngle = matrixMap(i+1,i+1)+(3.1415/2);
            n = origo_width - (tan(lineAngle) * origo_height);
            for (int i = -3; i < 3; i++) {
                for (int j = -3; j < 3; j++) {
                scan(origo_height+i,origo_width+j) = 255;
            }
        }
            
            
    
        
     for (int x = 0; x < 1000; x++) {
            y = int(x * tan(lineAngle) + n);
            if (y >= 0 && y < 1000) {
                scan(x,y) = 255;
            }
     }
     }
    
 
    

    // CENTRUM
    for (int i = -5; i < 5; i++) {
            for (int j = -5; j < 5; j++) {
                scan(500+i,500+j) = 255;
            }
        }

  

    // MANTA
    for (int i = -5; i < 5; i++) {
            for (int j = -5; j < 5; j++) {
                scan(500-estimatedStatesX(0)*scale+i,500-estimatedStatesX(1)*scale+j) = 255;
            }
        }

    // FIELD OF VIEW
    float test_rho = 2;
    float test_angle = estimatedStatesX(2);
    float test_x, test_y;
    test_x = (estimatedStatesX(0) + cos(test_angle)*test_rho)*scale;
    test_y = (estimatedStatesX(1) + sin(test_angle)*test_rho)*scale;
        
    for (int i = -3; i < 3; i++) {
            for (int j = -3; j < 3; j++) {
                scan(500-test_x+i,500-test_y+j) = 255;
            }
        }

    
    // POSSIBLE LINES
    
    for (int i = 0; i < possibleLines.meanRho.size(); i++) {
        //cout << "RHO: " << possibleLines.meanRho[i] << ", THETA: " << possibleLines.meanTheta[i] << endl;;
        float line_rho = possibleLines.meanRho[i];
        float line_theta = possibleLines.meanTheta[i];
        float line_x, line_y;
        line_x = (estimatedStatesX(0) + line_rho*cos(line_theta+estimatedStatesX(2)))*scale;
        line_y = (estimatedStatesX(1) + line_rho*sin(line_theta+estimatedStatesX(2)))*scale;
        
        float a, b, c, x, y;
        float line_agnle = line_theta+estimatedStatesX(2)+(3.1415/2);
        n = (500-line_y) - (tan(line_agnle) * (500-line_x));
        a = 1;
        b = -tan(line_agnle);
        c = -n;
        y = (b*(500*b-500*a)-a*c)/(pow(a,2)+pow(b,2));
        x = (a*(-b*500+a*500)-b*c)/(pow(a,2)+pow(b,2));



        
        //cout << "Possible Line " << i << ", RHO: " << possibleLines.meanRho[i] << endl;
        //cout << "Estunated X; " << estimatedStatesX(0) << ", Estiamted Y: " << estimatedStatesX(1) << endl;
        
        
        //for (int i = -3; i < 3; i++) {
        //    for (int j = -3; j < 3; j++) {
        //        scan(x+i,y+j) = 255;
        //    }
        //}



        for (int x = 0; x < 1000; x++) {
            y = int(x * tan(line_agnle) + n);
            if (y >= 0 && y < 1000) {
                scan(x,y) = 255;
            }
        }
        //for (int i = -3; i < 3; i++) {
        //    for (int j = -3; j < 3; j++) {
        //        scan(500-line_x+i,500-line_y+j) = 255;
        //    }
        //} 
        
        
       
        
        //cout << "Possibe line: " << i << ", RHO: " << possibleLines.meanRho[i] << ", Theta: " << possibleLines.meanTheta[i] << endl;
    }
    
   

    //cout << "AMOUNT OF LINES: " << possibleLines.meanRho.size() << endl;

    






    //tmp_scan(origo_height,origo_width) = 100;
    //cin >> debug;
    
    // Convert to openCV and display
    
    eigen2cv(scan,tmp_image);
    tmp_image.convertTo(gray_display, CV_8U);
    namedWindow("Line display2", 0);
    resizeWindow("Line display2", 2000, 2000);
    imshow("Line display2", gray_display);
    waitKey(1);
    show_landmark = false;
    
}

bool ekfSLAM::enoughLandmarks() {
    if (matrixMap.rows() > 10) {
        return false;
    }
    return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////











