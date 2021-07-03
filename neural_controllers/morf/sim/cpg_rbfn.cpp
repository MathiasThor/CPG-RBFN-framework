#include "cpg_rbfn.h"

cpg_rbfn::cpg_rbfn(vector<vector<float>> _weights, string _encoding, int _numberOfKernels, string _behaviour, vector<vector<float>> _weightsSensor){
    /*******************************************************************************
    *  CPG
    *******************************************************************************/
    cpg = new AdaptiveSO2CPGSynPlas();
    cpg->setPhi     ( 0.02*M_PI );   // Frequency term - Influences w00 w01 w10 w11 of the SO(2) oscillator (long term)
    cpg->setEpsilon ( 0.1 );        // Value should depend on the initial and external freq - from P to h2 (short term)
    cpg->setAlpha   ( 1.01);        // Amplitude and linearity between phi and the frequency
    cpg->setGamma   ( 1.0 );        // Synaptic weight from h2 to h0 - Governed by a Hebbian-type learning (short term)
    cpg->setBeta    ( 0.0 );        // Synaptic weight from h0 to h2 - Governed by a Hebbian-type learning (short term)
    cpg->setMu      ( 1.0 );        // Learning rate - Value should depend on the given initial and external freq
    cpg->setBetaDynamics   ( -1.0, 0.010, 0.00); // Heppian Rate, Decay Rate, Beta_0
    cpg->setGammaDynamics  ( -1.0, 0.010, 1.00); // --- || ---
    cpg->setEpsilonDynamics(  1.0, 0.010, 0.01); // --- || ---

    //destabilize cpg to oscillate
    cpg->setOutput(0,0.2012);
    cpg->setOutput(1,0);

    addSubnet(cpg);

    /*******************************************************************************
    *  RBF
    *******************************************************************************/
    // Calculate Centers
    rbf = new rbfn(_numberOfKernels, _weights, _encoding, _behaviour, _weightsSensor);
    //rbf->setWeights(vector<double> (numberOfKernels, 0));
}

void cpg_rbfn::calculateRBFCenters(int period, std::vector<float> sig1, std::vector<float> sig2) {
    rbf->calculateCenters(period, sig1, sig2);
}

void cpg_rbfn::setPerturbation(double value){
    cpg->setPerturbation(value);
}

void cpg_rbfn::setPhii(double value){
    cpg->setPhi(value);
}

double cpg_rbfn::getCpgOutput(int output){
    return cpg->getOutput(output);
}

double cpg_rbfn::getCpgActivity(int output){
    return cpg->getActivity(output);
}

double cpg_rbfn::getCpgWeight(int neuron1, int neuron2){
    return cpg->getWeight(neuron1, neuron2);
}

double cpg_rbfn::getCpgBias(int neuron){
    return cpg->getBias(neuron);
}

vector<vector<float>> cpg_rbfn::getContribution(){
    return rbf->contributions;
}

double cpg_rbfn::getPhi(){
    return cpg->getPhi();
}

vector<double> cpg_rbfn::getNetworkOutput(){
    return networkOutput;
}

void cpg_rbfn::step(vector<vector<float>> sensorOutput){
    updateActivities();
    updateWeights();
    updateOutputs();
    postProcessing();
    networkOutput = rbf->step(getCpgOutput(0), getCpgOutput(1), sensorOutput);
}

void cpg_rbfn::setCPGPeriod(int _period) {
    rbf->setCPGPeriod(_period);
}

