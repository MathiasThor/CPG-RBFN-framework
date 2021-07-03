/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/


#include "ann.h"

#include "neuron.h"
#include "synapse.h"
#include <algorithm>
#include <set>
#include <iostream>
#include <sstream>

/** initialization of static const member variables */
TanhFunction const * const ANN::tanhFunctionPointer =
        new TanhFunction();
LogisticFunction const * const ANN::logisticFunctionPointer =
        new LogisticFunction();
LinearFunction const * const ANN::identityFunctionPointer =
    new LinearFunction(1,0);
LinearThresholdFunction const * const ANN::linthresFunctionPointer =
    new LinearThresholdFunction(1.,0.);
SignFunction const * const ANN::signFunctionPointer =
    new SignFunction(0.9);
ThresholdFunction const * const ANN::thresholdFunctionPointer =
    new ThresholdFunction(0.9);


ANN::ANN()
{
    setDefaultTransferFunction(tanhFunctionPointer);
}

ANN::ANN(int numneurons)
{
    setNeuronNumber(numneurons);
    setDefaultTransferFunction(tanhFunctionPointer);
}

ANN::~ANN()
{
    setNeuronNumber(0);
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        delete (*it);
    }
}

Neuron* ANN::addNeuron()
{
    Neuron * const neuron = new Neuron();
    neuron->setTransferFunction(defaultTransferFunction);
    neurons.push_back(neuron);
    return neuron;
}

void ANN::addSubnet(ANN * subnet)
{
    subnets.push_back(subnet);
}

Synapse* ANN::addSynapse(Neuron * post, Neuron * pre)
{
    Synapse * const synapse = new Synapse(post, pre);
    return synapse;
}

void ANN::b(const int neuron, const double& abias)
{
    setBias(neuron, abias);
}

void ANN::b(Neuron* neuron, const double& abias)
{
    setBias(neuron, abias);
}

const double& ANN::b (const int neuron)
{
    return getBias(neuron);
}

void ANN::backpropagationStep()
{
  for (NeuronList::reverse_iterator rit= topologicalSort.rbegin();
      rit<topologicalSort.rend(); rit++)
  {
    (*rit)->updateError();
  }
}

std::string ANN::dumpBiases()
{
  std::stringstream str;
  for (unsigned int i=0; i<neurons.size(); i++)
  {
    str << "b(" << i << ", " << neurons[i]->getBias() << ");\n";
  }
  return str.str();
}

std::string ANN::dumpWeights()
{
  std::stringstream str;
  for (unsigned int i=0; i<neurons.size(); i++)
  {
    for (unsigned int j=0; j<neurons.size(); j++)
    {
      Synapse * s = getSynapse(i,j);
      if (s) str << "w(" << i << ", " << j << ", " << s->getWeight() << ");\n";
    }
  }
  return str.str();
}

void ANN::dw(const int& post, const int& pre, const double& aweight)
{
    setDeltaWeight(post, pre, aweight);
}

void ANN::dw(Neuron* post, Neuron* pre, const double& aweight)
{
    setDeltaWeight(post, pre, aweight);
}


const double ANN::dw(const int& post, const int& pre)
{
    return getDeltaWeight(post, pre);
}

void ANN::feedForwardStep()
{
  for (NeuronList::iterator it = topologicalSort.begin();
      it!=topologicalSort.end(); it++)
  {
    (*it)->updateActivity();
    (*it)->updateOutput();
  }
}

const double& ANN::getActivity(const int neuron) const
{
    return getActivity(neurons[neuron]);
}
const double& ANN::getActivity(Neuron const * neuron)
{
    return neuron->getActivity();
}

std::vector<Neuron*> ANN::getAllNeurons() const
{
  std::vector<Neuron*> v;
  v.insert(v.begin(), neurons.begin(), neurons.end());
  for (AnnList::const_iterator it=subnets.begin(); it!=subnets.end(); it++)
  {
    std::vector<Neuron*> subV = (*it)->getAllNeurons();
    v.insert(v.begin(),subV.begin(), subV.end());
  }
  return v;
}

std::vector<Synapse*> ANN::getAllSynapses() const
{
  std::vector<Synapse*> v;
  for (NeuronList::const_iterator it=neurons.begin(); it<neurons.end(); it++)
  {
    std::vector<Synapse*> s = (*it)->getSynapsesOut();
    v.insert(v.begin(), s.begin(), s.end());
  }

  for (AnnList::const_iterator it=subnets.begin(); it!=subnets.end(); it++)
  {
    std::vector<Synapse*> subV = (*it)->getAllSynapses();
    v.insert(v.begin(),subV.begin(), subV.end());
  }
  return v;
}

const double& ANN::getBias(const int neuron) const
{
    return getBias(neurons[neuron]);
}
const double& ANN::getBias(Neuron const * neuron)
{
    return neuron->getBias();
}

TransferFunction const* ANN::getDefaultTransferFunction() const
{
    return defaultTransferFunction;
}

const double ANN::getDeltaWeight(const int& post, const int& pre) const
{
    return getDeltaWeight(neurons[post], neurons[pre]);
}

const double ANN::getDeltaWeight(Neuron const * post, Neuron const * pre) const
{
    Synapse * synapse = post->getSynapseFrom(pre);
    return synapse->getDeltaWeight();
}

const double& ANN::getInput(const int neuron) const
{
    return getInput(neurons[neuron]);
}
const double& ANN::getInput(Neuron const * neuron)
{
    return neuron->getInput();
}

const double& ANN::getInputScaling(const int neuron) const
{
    return getInputScaling(neurons[neuron]);
}
const double& ANN::getInputScaling(Neuron const * neuron)
{
    return neuron->getInputScaling();
}

Neuron* ANN::getNeuron(unsigned int const index)
{
    return neurons[index];
}

unsigned int ANN::getNeuronNumber() const
{
    return neurons.size();
}

const double& ANN::getOutput(const int neuron) const
{
	if(neuron >= N()){
		cout << "Warning: neuron index " << neuron <<  " out of bounds (" << N() << "). Return 0.\n";
		return 0;
	}
    return neurons[neuron]->getOutput();
}

const double& ANN::getOutput(Neuron const * neuron)
{
    return neuron->getOutput();
}

ANN* ANN::getSubnet(unsigned int const index)
{
	if(index >= subnets.size()){
		cout << "Warning: subnet index out of bounds. Return 0.\n";
		return new ANN();
	}
    return subnets[index];
}

Synapse* ANN::getSynapse(const unsigned int& post, const unsigned int& pre)
        const
{
    return getSynapse(neurons[post], neurons[pre]);
}

Synapse* ANN::getSynapse(Neuron const * const post, Neuron const * const pre)
{
    return post->getSynapseFrom(pre);
}

std::vector<Neuron*> ANN::getTopologicalSort()
{
  return topologicalSort;
}

unsigned int ANN::getTotalNeuronNumber() const
{
    int number = neurons.size();
    for (AnnList::const_iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        number+=(*it)->getTotalNeuronNumber();
    }
    return number;
}

const double ANN::getWeight(const int& post, const int& pre) const
{
    return getWeight(neurons[post], neurons[pre]);
}

const double ANN::getWeight(Neuron const * post, Neuron const * pre) const
{
    Synapse * synapse = post->getSynapseFrom(pre);
    if (synapse == NULL) return 0.0;
    return synapse->getWeight();

}

unsigned int ANN::N() const
{
    return getTotalNeuronNumber();
}

Neuron* ANN::n(unsigned int const index)
{
    return getNeuron(index);
}

LinearFunction const * const ANN::identityFunction() {
    return identityFunctionPointer;
}

LinearThresholdFunction const * const ANN::linthresholdFunction() {
    return linthresFunctionPointer;
}

LogisticFunction const * const ANN::logisticFunction() {
    return logisticFunctionPointer;
}

void ANN::postProcessing()
{
    // nothing to do here
}

void ANN::removeNeuron(Neuron const * neuron)
{
    NeuronList::iterator it = find(neurons.begin(), neurons.end(), neuron);
    if (it != neurons.end()) {
        delete neuron;
        neurons.erase( find(neurons.begin(), neurons.end(), neuron));
    }
    /** @todo give out error message if neuron does not belong to this net */
}

void ANN::setActivity(const int& neuron, const double& aactivity)
{
    setActivity(neurons[neuron], aactivity);
}

void ANN::setActivity(Neuron* neuron, const double & aactivity)
{
    neuron->setActivity(aactivity);
}

void ANN::setAllTransferFunctions(TransferFunction const * const func,
        const bool& includeSubnets)
{
    for (NeuronList::iterator it=neurons.begin(); it != neurons.end(); it++)
    {
        (*it)->setTransferFunction(func);
    }
    if (includeSubnets)
    {
        for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
        {
            (*it)->setAllTransferFunctions(func, true);
        }
    }
}

void ANN::setBias(Neuron * neuron, const double& abias)
{
    neuron->setBias(abias);
}

void ANN::setBias(const int& neuron, const double& abias)
{
    setBias(neurons[neuron], abias);
}

void ANN::setDefaultTransferFunction(TransferFunction const * const func)
{
    defaultTransferFunction = func;
}

void ANN::setDeltaWeight(Neuron* post, Neuron* pre, const double dweight)
{
    Synapse * synapse = post->getSynapseFrom(pre);
    if (synapse == NULL) synapse = addSynapse(post, pre);
    synapse->setDeltaWeight(dweight);
}

void ANN::setDeltaWeight(const int post, const int pre, const double dweight)
{
    setDeltaWeight(neurons[post], neurons[pre], dweight);
}

void ANN::setInput(const int& neuron, const double& ainput)
{
    setInput(neurons[neuron], ainput);
}

void ANN::setInput(Neuron* neuron, const double ainput)
{
    neuron->setInput(ainput);
}

void ANN::setInputScaling(const int& neuron, const double& ascale)
{
    setInputScaling(neurons[neuron], ascale);
}

void ANN::setInputScaling(Neuron* neuron, const double ascale)
{
    neuron->setInputScaling(ascale);
}

void ANN::setOutput(const int& neuron, const double& aoutput)
{
    setOutput(neurons[neuron], aoutput);
}

void ANN::setOutput(Neuron* neuron, const double aoutput)
{
    neuron->setOutput(aoutput);
}

void ANN::setNeuronNumber(const unsigned int& anumber)
{
    while (neurons.size() > anumber)
    {
        removeNeuron(neurons.back());
    }
    while (neurons.size() < anumber)
    {
        addNeuron();
    }
}

void ANN::setTransferFunction(const int neuron,
        TransferFunction const * const func)
{
    setTransferFunction(neurons[neuron], func);
}

void ANN::setTransferFunction(Neuron * neuron,
        TransferFunction const * const func)
{
    neuron->setTransferFunction(func);
}


void ANN::setWeight(Neuron* post, Neuron* pre, const double weight)
{
    Synapse * synapse = post->getSynapseFrom(pre);
    if (synapse == NULL) synapse = addSynapse(post, pre);
    synapse->setWeight(weight);
}

void ANN::setWeight(const int post, const int pre, const double weight)
{
    setWeight(neurons[post], neurons[pre], weight);
}

SignFunction const * const ANN::signFunction() {
    return signFunctionPointer;
}

void ANN::step()
{
    updateActivities();
    updateOutputs();
    updateWeights();
    postProcessing();
}

TanhFunction const * const ANN::tanhFunction() {
    return tanhFunctionPointer;
}

ThresholdFunction const * const ANN::thresholdFunction() {
    return thresholdFunctionPointer;
}

void ANN::updateActivities()
{
    for (NeuronList::iterator it=neurons.begin(); it != neurons.end(); it++)
    {
        (*it)->updateActivity();
    }
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        (*it)->updateActivities();
    }
}

void ANN::updateOutputs()
{
    for (NeuronList::iterator it=neurons.begin(); it != neurons.end(); it++)
    {
        (*it)->updateOutput();
    }
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        (*it)->updateOutputs();
    }
}

bool ANN::updateTopologicalSort()
{
  typedef std::set<Neuron*>            NeuronSet;
  typedef std::vector<Synapse*>        SynapseVector;
  typedef std::map<Neuron*, NeuronSet> SynapticMap;

  topologicalSort.clear();
  NeuronList  neurons = getAllNeurons();
  SynapticMap synapsesIn;
  SynapticMap synapsesOut;
  NeuronSet   noIncomes;

  // run over all neurons and pouplate the containers
  for (NeuronList::iterator neuronIt = neurons.begin(); neuronIt!=neurons.end();
      neuronIt++)
  {
    Neuron* const n = *neuronIt;
    SynapseVector ins = n->getSynapsesIn();
    SynapseVector outs = n->getSynapsesOut();
    for (SynapseVector::iterator sIt = ins.begin(); sIt!=ins.end(); sIt++)
      synapsesIn[n].insert((*sIt)->getPre());
    for (SynapseVector::iterator sIt = outs.begin(); sIt!=outs.end(); sIt++)
      synapsesOut[n].insert((*sIt)->getPost());
    if (ins.size()==0) noIncomes.insert(n);
  }

  // start the algorithm
  while (noIncomes.size()>0)
  {
    Neuron* n = (*noIncomes.begin());
    noIncomes.erase(n);
    topologicalSort.push_back(n);
    while (synapsesOut.find(n) != synapsesOut.end())
    {
      Neuron* post = *(synapsesOut[n].begin());
      synapsesOut[n].erase(post);
      synapsesIn[post].erase(n);
      if (synapsesIn[post].size()==0) noIncomes.insert(post);
      if (synapsesOut[n].size()==0) synapsesOut.erase(n);
      if (synapsesIn[post].size()==0) synapsesIn[post].erase(post);
    }
  }

  return (synapsesOut.size()==0);
}


void ANN::updateWeights()
{
    for (unsigned int syn_ind = 0; syn_ind < getAllSynapses().size(); syn_ind++)
        getAllSynapses().at(syn_ind)->updateWeight();
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        (*it)->updateWeights();
    }
}

void ANN::w(const int& post, const int& pre, const double& aweight)
{
    setWeight(post, pre, aweight);
}

void ANN::w(Neuron* post, Neuron* pre, const double& aweight)
{
    setWeight(post, pre, aweight);
}


const double ANN::w(const int& post, const int& pre)
{
    return getWeight(post, pre);
}

