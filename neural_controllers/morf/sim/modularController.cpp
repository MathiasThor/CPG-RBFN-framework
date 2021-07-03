/*
 *  Created on: Apr 10, 2012
 *      Author: eduard grinke
 *
 *  edited by degoldschmidt
 *  edited and refactored for v-rep by mathias thor (Aug. 17 - 2017)
 */

#include "modularController.h"
#include "ann-library/so2cpg.h"
#include "ann-library/psn.h"
#include "ann-library/pcpg.h"
#include "ann-library/vrn.h"
#include "ann-library/pmn.h"
#include "ann-framework/neuron.h"
#include "ann-library/adaptiveso2cpgsynplas.h"

modularController::modularController(int _cpg_option, bool legged_or_arm){
    cpg_option=_cpg_option;
    /*******************************************************************************
    *  MODULE 0 IO'S for modularController
    *******************************************************************************/

    // Create 5 input neurons for modularController
    for(int i=0;i<5;i++){
        inputNeurons.push_back(addNeuron());
        inputNeurons.at(i)->setTransferFunction(identityFunction());
    }

    /*******************************************************************************
    *  MODULE 1 CPG
    *******************************************************************************/

    switch(cpg_option) {
        case 1: // Standard SO2 CPG

            cpg = new SO2CPG();
            cpg_bias = 0.0;

            //From 0.02-1.5
            // MI = 0.02;   // slow Wave
            // MI = 0.03;   // Wave
            // MI = 0.04;   // fast Wave
            // MI = 0.13;   // Terapod
            MI = 0.18;      // Tripod fast
            // MI = 0.34;   // Faster than tripod

            //destabilize cpg to oscillate
            cpg->setOutput(0, 0.1);
            cpg->setOutput(1, 0.1);
            cpg->setActivity(0, 0.1);
            cpg->setActivity(1, 0.1);

            //set cpg weights
            cpg->setWeight(0, 0, 1.4);
            cpg->setWeight(0, 1, 0.18 + MI);
            cpg->setWeight(1, 0,-0.18 - MI);
            cpg->setWeight(1, 1, 1.4);

            //set bias
            cpg->setBias(0, cpg_bias);
            cpg->setBias(1, cpg_bias);

            //for updating the sub nets (to do the time step)
            addSubnet(cpg);
            break;

        case 2: // Adaptive SO2 CPG

            cpg_s = new AdaptiveSO2CPGSynPlas();
            cpg_s->setPhi(0.04);         // Frequency term - Influences w00 w01 w10 w11 of the SO(2) oscillator (long term)
            cpg_s->setEpsilon ( 0.1 );   // Value should depend on the initial and external freq - from P to h2 (short term)
            cpg_s->setAlpha(1.01);
            cpg_s->setGamma   ( 1.0 );    // Synaptic weight from h2 to h0 - Governed by a hebbian-type learning (short term)
            cpg_s->setBeta    ( 0.0 );    // Synaptic weight from h0 to h2 - Governed by a hebbian-type learning (short term)
            cpg_s->setMu      ( 1.0 );    // Learning rate - Value should depend on the given initial and external freq
            cpg_s->setBetaDynamics   ( -1.0, 0.010, 0.00); // Heppian Rate, Decay Rate, Beta_0
            cpg_s->setGammaDynamics  ( -1.0, 0.010, 1.00); // --- || ---
            cpg_s->setEpsilonDynamics(  1.0, 0.010, 0.01); // --- || ---

            //destabilize cpg to oscillate
            cpg_s->setOutput(0,0.2);

            //for updating the sub nets (to do the time step)
            addSubnet(cpg_s);
            break;

        case 3: // Standard SO2 CPG with Phi instead of MI
            cpg = new SO2CPG();
            cpg_bias = 0.0;

            //destabilize cpg to oscillate
            cpg->setOutput(0, 0.1);// cpg->setOutput(0, 0.1);
            cpg->setOutput(1, 0.1);
            cpg->setActivity(0, 0.1);
            cpg->setActivity(1, 0.1);

            //set bias
            cpg->setBias(0, cpg_bias);
            cpg->setBias(1, cpg_bias);

            //for updating the sub nets (to do the time step)
            addSubnet(cpg);

            break;

        default:break;
    };

    /*******************************************************************************
    *  MODULE 2 PCPG // Makes it sawtooth
    *******************************************************************************/
    pcpg = new PCPG(); // postCPG / PCPG

    // CPG to PCPG
    if(cpg_option == 3)
    {
        w(pcpg->getNeuron(0),cpg->getNeuron(0),5);
        w(pcpg->getNeuron(1),cpg->getNeuron(1),5);
    }
    else if(cpg_option == 2){
        w(pcpg->getNeuron(0),cpg_s->getNeuron(0),10);
        w(pcpg->getNeuron(1),cpg_s->getNeuron(1),10);
    } else{
        w(pcpg->getNeuron(0),cpg->getNeuron(0),1);
        w(pcpg->getNeuron(1),cpg->getNeuron(1),1);
    }

    addSubnet(pcpg);

    /*******************************************************************************
    *  MODULE 3 PSN
    *******************************************************************************/
    psn = new PSN();

    inputNeurons[2]->setInput(0);
    w(psn->getNeuron(0), inputNeurons[2], -1);
    w(psn->getNeuron(1), inputNeurons[2], 1);

    // PCPG to PSN
    w(psn->getNeuron(2), pcpg->getNeuron(0), 0.5);
    w(psn->getNeuron(3), pcpg->getNeuron(1), 0.5);
    w(psn->getNeuron(4), pcpg->getNeuron(1), 0.5);
    w(psn->getNeuron(5), pcpg->getNeuron(0), 0.5);

    addSubnet(psn);

    /*******************************************************************************
    *  MODULE 4 VRN
    *******************************************************************************/
    vrnLeft  = new VRN();
    vrnRight = new VRN();

    // PSN to VRN
    w(vrnLeft->getNeuron(0), psn->getNeuron(11), 1.75);
    w(vrnRight->getNeuron(0), psn->getNeuron(11), 1.75);

    inputNeurons[3]->setInput(1);
    inputNeurons[4]->setInput(-1);
    w(vrnLeft ->getNeuron(1), inputNeurons[3], 5);
    w(vrnRight->getNeuron(1), inputNeurons[4], 5);

    addSubnet(vrnLeft);
    addSubnet(vrnRight);

    /*******************************************************************************
     *  MODULE 5 Pre Motor Neurons PMN
    *******************************************************************************/
    pmn     = new PMN();

    if(cpg_option == 1 || legged_or_arm){
        w(pmn->getNeuron(0), psn->getNeuron(10), 5); // Edit this and remove rescale (was 5)
        b(pmn->getNeuron(0), -0.5);
        w(pmn->getNeuron(1), psn->getNeuron(11), -5); // Edit this and remove rescale (was -5)
        b(pmn->getNeuron(1), -0.5);
        w(pmn->getNeuron(2), psn->getNeuron(11), 5); // Edit this and remove rescale (was 5)
        b(pmn->getNeuron(2), -0.5);
        w(pmn->getNeuron(3), vrnLeft->getNeuron(6), -2.5); // Edit this and remove rescale (was -2.5)
        w(pmn->getNeuron(4), vrnRight->getNeuron(6), -2.5); // Edit this and remove rescale (was -2.5)
    } else if(cpg_option == 2){
        w(pmn->getNeuron(0), cpg_s->getNeuron(0), 10); // Edit this and remove rescale (was 5)
//        b(pmn->getNeuron(0), -0.5);
        w(pmn->getNeuron(1), cpg_s->getNeuron(1), -10); // Edit this and remove rescale (was -5)
//        b(pmn->getNeuron(1), -0.5);
        w(pmn->getNeuron(2), cpg_s->getNeuron(1), 10); // Edit this and remove rescale (was 5)
//        b(pmn->getNeuron(2), -0.5);
        w(pmn->getNeuron(3), cpg_s->getNeuron(1), -10); // Edit this and remove rescale (was -2.5)
        w(pmn->getNeuron(4), cpg_s->getNeuron(1), -10); // Edit this and remove rescale (was -2.5);
    }
    addSubnet(pmn);

};

void modularController::setPerturbation(double value)
{
    cpg_s->setPerturbation(value);
}

void modularController::setPhii(double value)
{
    if(cpg_option == 2)
        cpg_s->setPhi(value);
    else
        cpg->setPhi(value);
}

void modularController::setMI(double value)
{
    MI = value;
    cpg->setWeight(0, 1, 0.18 + MI);
    cpg->setWeight(1, 0,-0.18 - MI);
}

void modularController::setInputNeuronInput(int input, double value)
{
    setInput(inputNeurons[input],value);
}

double modularController::getFinalNeuronOutput(int output)
{
    return pmn->getOutput(output);
}

double modularController::getCpgOutput(int output)
{
    if(cpg_option == 2)
        return cpg_s->getOutput(output);
    else
        return cpg->getOutput(output);
}

double modularController::getCpgActivity(int output)
{
    if(cpg_option == 2)
        return cpg_s->getActivity(output);
    else
        return cpg->getActivity(output);
}

double modularController::getCpgWeight(int neuron1, int neuron2)
{
    if(cpg_option == 2)
        return cpg_s->getWeight(neuron1, neuron2);
    else
        return cpg->getWeight(neuron1, neuron2);

}

double modularController::getCpgBias(int neuron)
{
    if(cpg_option == 2)
        return cpg_s->getBias(neuron);
    else
        return cpg->getBias(neuron);
}

double modularController::getpcpgOutput(int output)
{
    return pcpg->getOutput(output);
}

void modularController::setInputPsn(int input, double value)
{
    psn->setInput(input,value);
}

void modularController::setInputVrnLeft(int input, double value)
{
    vrnLeft->setInput(input,value);
}

void modularController::setInputVrnRight(int input, double  value)
{
    vrnRight->setInput(input,value);
}

double modularController::getPsnOutput(int output)
{
    return psn->getOutput(output);
}

double modularController::getVrnLeftOutput(int output)
{
    return vrnLeft->getOutput(output);
}

double modularController::getVrnRightOutput(int output)
{
    return vrnRight->getOutput(output);
}

double modularController::getPhi()
{
    if(cpg_option == 2)
        return cpg_s->getPhi();
    else
        return cpg->getPhi();
}

void modularController::step()
{
    updateActivities();
    updateWeights();
    updateOutputs();
    postProcessing();
}
