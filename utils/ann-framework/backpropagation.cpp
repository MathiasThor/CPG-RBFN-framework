/*
 * backpropagation.cpp
 *
 *  Created on: May 14, 2012
 *      Author: timo
 */

#include "backpropagation.h"

#include <fstream>

#include "ann.h"
#include "neuron.h"
#include "synapse.h"

Backpropagation::Backpropagation()
{
  rate = 0;
  net  = 0;
}

Backpropagation::~Backpropagation()
{
  while (patterns.size()>0)
  {
    delete (*patterns.begin());
    patterns.erase(patterns.begin());
  }
}

void Backpropagation::addTrainingPattern(TrainingPattern* p)
{
  patterns.push_back(p);
}

void Backpropagation::defineInputNeuron(const int& index, Neuron* neuron)
{
  inputNeurons[index] = neuron;
}

void Backpropagation::defineOutputNeuron(const int& index, Neuron* neuron)
{
  outputNeurons[index] = neuron;
}

void Backpropagation::excludeNeuronBias(Neuron * neuron)
{
  neurons.erase(neuron);
}

void Backpropagation::includeAllNeuronBiases()
{
  std::vector<Neuron*> allNeurons = net->getAllNeurons();
  neurons.insert(allNeurons.begin(), allNeurons.end());
}

void Backpropagation::includeAllSynapses()
{
  std::vector<Synapse*> allSynapses = net->getAllSynapses();
  synapses.insert(allSynapses.begin(), allSynapses.end());
}

void Backpropagation::includeNeuronBias(Neuron * neuron)
{
  neurons.insert(neuron);
}

void Backpropagation::includeSynapse(Synapse * synapse)
{
  synapses.insert(synapse);
}

void Backpropagation::learn(const unsigned int& epochs)
{
  typedef std::map<Synapse*, double> WeightMap;
  typedef std::map<Neuron*, double> BiasMap;

  // open file to log error function
  std::ofstream errorFunctionFile("errorFunction.dat");

  // iterate over epochs
  for (unsigned int e=0; e<epochs; e++)
  {
    WeightMap weightChange;
    BiasMap biasChange;

    // variable to calculate the error function in this epoch
    double epochErrorFunction = 0;

    // iterate over all patterns
    for (PatternVector::iterator patternIt = patterns.begin();
        patternIt < patterns.end(); patternIt++)
    {

      // set the neuron inputs
      for (NeuronMap::iterator it = inputNeurons.begin(); it!=inputNeurons.end();
          it++)
      {
        it->second->setInput((*patternIt)->inputs[it->first]);
      }

      // perform the feed forward step
      net->feedForwardStep();

      // calculate error signal for output neurons
      double patternError = 0;
      for (NeuronMap::iterator it = outputNeurons.begin();
          it!=outputNeurons.end(); it++)
      {
        const double output = net->getOutput(it->second);
        const double target = (*patternIt)->outputs[it->first];
        const double error  = output-target;
        it->second->setErrorInput(error);
        patternError += error * error;
      }
      epochErrorFunction += 0.5*patternError;

      // perform backpropagation step
      net->backpropagationStep();

      // calculate synaptic weight changes for all synapses
      for (SynapseSet::iterator it = synapses.begin(); it!=synapses.end(); it++)
      {
        const double dw = -(*it)->getPre()->getOutput() * (*it)->getPost()->getError();
        weightChange[*it] += dw;
      }

      // update neuron biases
      for (NeuronSet::iterator it = neurons.begin(); it!=neurons.end(); it++)
      {
        const double db = -(*it)->getError();
        biasChange[*it] += db;
      }

    }

    // apply weight changes
    for (SynapseSet::iterator it = synapses.begin(); it!=synapses.end(); it++)
    {
      (*it)->setWeight( (*it)->getWeight() + rate * weightChange[*it]);
    }

    // apply bias changes
    for (NeuronSet::iterator it = neurons.begin(); it!=neurons.end(); it++)
    {
      (*it)->setBias( (*it)->getBias() + rate * biasChange[*it]);
    }

    errorFunctionFile << e << "\t" << epochErrorFunction << std::endl;
  }
}

void Backpropagation::setNeuralNetwork(ANN* network)
{
  net = network;
}

void Backpropagation::setLearningRate(const double& arate)
{
  rate = arate;
}
