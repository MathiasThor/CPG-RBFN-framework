//
// Created by mat on 11/20/17.
//

#ifndef NEUTRON_CONTROLLER_DUALINTEGRALLEARNER_H
#define NEUTRON_CONTROLLER_DUALINTEGRALLEARNER_H

#include <vector>
#include <cmath>
#include <iostream>

class dualIntegralLearner {
public:
    dualIntegralLearner(bool _twoSideError, int delayedStart);
    void step(double error, double base_state);

    // accessor functions
    float getControlOutput();
    float getError();
    float getIntegralError();
    float getControllerCorrection();

    // mutator functions
    void setWeights(float,float,float,float);
    void setLowerThreshold(float);

    float trueError = 0;

private:
    float fastLearner = 0;
    float slowLearner = 0;

    float scaler = 2.1;
    float Af = 0.2*scaler;      //0.59;
    float As = 0.4*scaler;      //0.992;
    float Bf = 0.1*scaler;      //0.1665;
    float Bs = 0.01*scaler;     //0.0162;
    float Cf = 0.001*scaler;    //0.0081;
    float Cs = 0.0001*scaler;   //0.00018*multi;

    float error = 0, oldError = 0;
    int runs = 0;
    float integralError = 0;
    float lowerThreshold = 0.03;
    float controlOutput = 0;
    float dualLearner = 0;
    bool twoSideError = true;

    double baseState = 0;

    int delayedStart = 150;

};

#endif //NEUTRON_CONTROLLER_DUALINTEGRALLEARNER_H
