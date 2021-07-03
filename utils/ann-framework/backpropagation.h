/*
 * backpropagation.h
 *
 *  Created on: May 14, 2012
 *      Author: timo
 */

#ifndef BACKPROPAGATION_H_
#define BACKPROPAGATION_H_

#include <vector>
#include <map>
#include <set>

// forward declarations
class ANN;
class Neuron;
class Synapse;

struct TrainingPattern{
  std::map<int, double> inputs;
  std::map<int, double> outputs;
};

class Backpropagation {
  public:
    Backpropagation();
    ~Backpropagation();
    void addTrainingPattern(TrainingPattern* p);
    void defineInputNeuron(const int& index, Neuron* neuron);
    void defineOutputNeuron(const int& index, Neuron* neuron);
    void excludeNeuronBias(Neuron* neuron);
    void includeAllNeuronBiases();
    void includeAllSynapses();
    void includeNeuronBias(Neuron* neuron);
    void includeSynapse(Synapse* synapse);
    void learn(const unsigned int& epochs=1);
    void setNeuralNetwork(ANN* network);
    void setLearningRate(const double& arate);
  private:
    typedef std::map<int, Neuron*> NeuronMap;
    typedef std::set<Neuron*> NeuronSet;
    typedef std::set<Synapse*> SynapseSet;
    typedef std::vector<TrainingPattern*> PatternVector;
    ANN* net;
    PatternVector patterns;
    NeuronMap inputNeurons;
    NeuronMap outputNeurons;
    NeuronSet neurons;
    SynapseSet synapses;
    double rate;
};


#endif /* BACKPROPAGATION_H_ */
