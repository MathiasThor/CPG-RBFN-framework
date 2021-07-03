//
// Created by mat on 8/17/17.
//

#ifndef NEUTRON_CONTROLLER_MODULARCONTROLLER_H
#define NEUTRON_CONTROLLER_MODULARCONTROLLER_H

#include "ann-framework/ann.h"
#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <string.h>
#include "neutronMotorDefinition.h"

using namespace std;

// forward declarations
class SO2CPG;
class PCPG; // postCPG / PCPG
class AdaptiveSO2CPGSynPlas;
class PSN;
class PMN;
class VRN;

class modularController: public ANN {
public:
    modularController(int cpg_choice, bool legged_or_robot);

    void step() override;
    double getCpgOutput(int output);
    double getCpgActivity(int output);
    double getpcpgOutput(int output);
    double getPsnOutput(int output);
    double getVrnLeftOutput(int output);
    double getVrnRightOutput(int output);
    void setInputVrnLeft(int input, double value);
    void setInputVrnRight(int input, double value);
    void setInputPsn(int input, double value);
    void setInputNeuronInput(int input, double value);
    double MI;
    double cpg_bias;
    double getCpgWeight(int neuron1, int neuron2);
    double getCpgBias(int neuron);
    double getFinalNeuronOutput(int output);
    void setPerturbation(double value);
    void setMI(double value);
    double getPhi();
    void setPhii(double value);

private:
    int cpg_option = 0;
    SO2CPG * cpg;
    AdaptiveSO2CPGSynPlas * cpg_s;
    PCPG * pcpg; // postCPG / PCPG
    PSN * psn;
    VRN * vrnLeft;
    VRN * vrnRight;
    PMN* pmn;
    std::vector<Neuron*> inputNeurons;
    Neuron* perturbationNeuron;
};

#endif //NEUTRON_CONTROLLER_MODULARCONTROLLER_H
