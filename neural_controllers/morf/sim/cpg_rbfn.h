//
// Created by mat on 8/17/17.
//

#ifndef rbfcpg_H
#define rbfcpg_H

#include "ann-framework/ann.h"
#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <string.h>
#include "neutronMotorDefinition.h"
#include "rbfn.h"
#include "ann-library/so2cpg.h"
#include "ann-framework/neuron.h"
#include "ann-library/adaptiveso2cpgsynplas.h"

using namespace std;

// forward declarations
class SO2CPG;
class PCPG; // postCPG / PCPG
class AdaptiveSO2CPGSynPlas;
class rbfn;

class cpg_rbfn: public ANN {
public:
    cpg_rbfn(vector<vector<float>> _weights, string _encoding, int _numberOfKernels, string behaviour, vector<vector<float>> weightSensors = {});

    void    setCPGPeriod(int _period);
    void    step(vector<vector<float>> sensorOutput = {});
    double  getCpgOutput(int output);
    double  getCpgActivity(int output);
    double  MI;
    double  cpg_bias;
    double  getCpgWeight(int neuron1, int neuron2);
    double  getCpgBias(int neuron);
    void    setPerturbation(double value);
    double  getPhi();
    void    setPhii(double value);
    void    calculateRBFCenters(int period, std::vector<float> sig1, std::vector<float> sig2);
    vector<double>  getNetworkOutput();
    vector<vector<float>> getContribution();
private:
    vector<double> signal1;
    vector<double> signal2;

    vector<double>  networkOutput = {0,0,0};
    int     period;
    int     cpg_option = 0;

    AdaptiveSO2CPGSynPlas * cpg;
    rbfn * rbf;
};

#endif //rbfcpg_H
