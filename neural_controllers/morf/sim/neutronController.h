//
// Created by mat on 12/30/17.
//

#ifndef NEUTRON_CONTROLLER_NEUTRONCONTROLLER_H
#define NEUTRON_CONTROLLER_NEUTRONCONTROLLER_H

#include <ctime>       /* time */
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"
#include "simRosClass.h"
#include "modularController.h"
#include "neutronMotorDefinition.h"
#include "delayline.h"
#include "postProcessing.h"
#include "cpg_rbfn.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "environment.h"

class simRosClass;
class postProcessing;
class cpg_rbfn;

struct lpf{
    float a=0.95;
    float b=1-a;
    float output=0;
    float prevOutput=0;
    float getA(){ return a; }
    void setA(float newA){
        a = newA;
        b=1-a;
    }
    float step(float input) {
        prevOutput = output;
        output += b*(input-prevOutput);
        return output;
    }
};

class neutronController {
public:
    neutronController(int argc,char* argv[]);
    ~neutronController();
    int runController();

private:
    vector<float> readParameterSet(string parametertype);
    double rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter);
    void tripodGaitRangeOfMotion(vector<vector<float>> policyWeights, string encoding);
    void tripodGaitRBFN();
    void fitnessLogger();
    void logData( double sensor, const string& option);
    void logBehaviorData(double sim_time, vector<vector<float>> postProcessedSensoryFeedback, const string& option);
    vector<float> jointLimiterMORF(vector<float> jointValues);
    float jointLimiter(float jointValue, float jointMin, float jointMax);

    vector<lpf *> LPF_1;
    vector<lpf *> LPF_2;
    vector<lpf *> LPF_3;
    vector<lpf *> LPF_4;
    vector<lpf *> LPF_5;
    vector<lpf *> LPF_6;

    bool primitive      = false;
    bool useLogData     = false;
    bool simulation     = true;
    bool blackOut       = false;
    int policySelector= 1;
    string behaviour    = "none";
    int rollout         = -1;
    int simulationID    = -1;
    int CPGPeriod       = 0;
    int taudil          = 5;
    int tau             = 300;
    int simulationTime  = 6;
    int iteration       = 0;
    float startPhi      = 0.015 * M_PI;
    string   encoding;
    ofstream myFile;

    int currentSimTime  = 0;

    // If TRUE then disable rosinterfacehelper
    bool useAPItrigger  = true;

    int skipTrigger = 3;

    vector<Delayline>   delayline;
    vector<float>       positions;
    vector<float>       data;
    vector<float>       sensorInputIntegrated;
    vector<vector<float>> policyWeights;
    vector<vector<float>> policyWeightsSensor;

    cpg_rbfn * CPG_RBFN;
    environment * env;
    simRosClass * rosHandle;
    modularController * CPG;
    postProcessing * CPGPeriodPostprocessor;
};


#endif //NEUTRON_CONTROLLER_NEUTRONCONTROLLER_H
