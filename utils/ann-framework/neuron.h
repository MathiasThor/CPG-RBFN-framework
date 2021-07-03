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


#ifndef NEURON_H_
#define NEURON_H_

#include <map>
#include <vector>
#include "transferfunction.h"

// forward declarations
class ANN;
class Synapse;
//class TransferFunction;

/**
 * Time Discrete Artificial Neuron Class
 *
 * This class represents a single neuron in a time-discrete artificial neural
 * net. It is described by its current output, activity, bias and input values
 * as well as by the list of incoming and outgoing synapses and the transfer
 * function to use for this neuron.
 *
 * ATTENTION: All methods assume that there are no two synapses with the same
 * postsynaptic and presynaptic neurons.
 */
class Neuron {
public:
    /**
     * the constructor
     */
    Neuron();

    /**
     * the destructor
     */
    ~Neuron();

    /**
     * Adds an entry to the list of incoming synapses
     *
     * This method adds the given synapse to the list of incoming synapses. This
     * does also mean that the synapse will be deleted when the neuron is.
     * This method is called from the synapse constructor, so you do not have
     * to (and also should not) call it manually.
     *
     * @param synapse pointer to the new incoming synapse
     */
    void addSynapseIn(Synapse * synapse);

    /**
     * Adds an entry to the list of outgoing synapses
     *
     * This method adds the given synapse to the list of outgoing synapses. This
     * does also mean that the synapse will be deleted when the neuron is.
     * This method is called from the synapse constructor, so you do not have
     * to (and also should not) call it manually.
     *
     * @param synapse pointer to the new outgoing synapse
     */
    void addSynapseOut(Synapse * synapse);

    /**
     * Returns activity
     *
     * This method returns the activity value of the neuron. The returned
     * reference stays valid for the whole lifetime of the neuron.
     *
     * @return activity value
     */
    const double& getActivity() const;

    /**
     * Returns bias
     *
     * This method returns the bias value of the neuron. The returned
     * reference stays valid for the whole lifetime of the neuron.
     *
     * @return bias value
     */
    const double& getBias() const;

    /**
     * Returns back propagated error
     *
     * @return error
     */
    const double& getError() const;

    /**
     * Returns input
     *
     * This method returns the input value of the neuron. The returned
     * reference stays valid for the whole lifetime of the neuron.
     *
     * @return input value
     */
    const double& getInput() const;

    /**
     * Returns input scaling
     *
     * This method returns the input value of the neuron. The returned
     * reference stays valid for the whole lifetime of the neuron.
     *
     * @return input value
     */
    const double& getInputScaling() const;

    /**
     * Returns output
     *
     * This method returns the output value of the neuron. The returned
     * reference stays valid for the whole lifetime of the neuron.
     *
     * @return output value
     */
    const double& getOutput() const;

    /**
     * Returns incoming synapse from a given neuron
     *
     * This method can be used to get the synapse connecting the presynaptic
     * neuron pre with this neuron. It is assumed that there are not two
     * synapses connecting the same presynaptic and postsynaptic neurons. If
     * there is no such synapse present NULL is returned.
     *
     * @param pre pointer to presynaptic neuron
     * @return pointer to synapse or NULL if no such synapse exists
     */
    Synapse* getSynapseFrom(Neuron const * pre) const;

    std::vector<Synapse*> getSynapsesIn() const;

    std::vector<Synapse*> getSynapsesOut() const;

    /**
     * Returns outgoing synapse to a given neuron
     *
     * This method can be used to get the synapse connecting this neuron with
     * the given postsynaptic neuron. It is assumed that there are not two
     * synapses connecting the same presynaptic and postsynaptic neurons. If
     * there is no such synapse present NULL is returned.
     *
     * @param post pointer to postynaptic neuron
     * @return pointer to synapse or NULL if no such synapse exists
     */
    Synapse* getSynapseTo(Neuron const * post) const;

    /**
     * Removes an entry from the list of incoming synapses
     *
     * This method removes the given synapse from the list of incoming synapses.
     * This method is called from the synapse destructor, so you do not have
     * to (and also should not) call it manually.
     *
     * @param synapse synapse to remove
     */
    void removeSynapseIn(Synapse const * synapse);

    /**
     * Removes an entry from the list of outgoing synapses
     *
     * This method removes the given synapse from the list of outgoing synapses.
     * This method is called from the synapse destructor, so you do not have
     * to (and also should not) call it manually.
     *
     * @param synapse synapse to remove
     */
    void removeSynapseOut(Synapse const * synapse);

    /**
     * Sets the activity
     *
     * This method sets the activity value of the neuron.
     *
     * @param aactivity new activity value
     */
    void setActivity(const double& aactivity);

    /**
     * Sets the bias
     *
     * This method sets the bias value of the neuron.
     *
     * @param abias new bias value
     */
    void setBias(const double& abias);

    /**
     * Sets the error value
     *
     * This method sets the error value used in back propagation algorithm
     *
     * @param aerror new error value
     */
    void setErrorInput(const double& aerror);

    /**
     * Sets the external input
     *
     * This method sets the external input value of the neuron.
     *
     * @param ainput new input value
     */
    void setInput(const double& ainput);

    /**
     * Sets the input scaling factor
     *
     * This method sets the input scaling factor of the neuron.
     *
     * @param ascale new scaling factor
     */
    void setInputScaling(const double& ascale);

    /**
     * Sets the output
     *
     * This method sets the output value of the neuron.
     *
     * @param aoutput new output value
     */
    void setOutput(const double& aoutput);

    /**
     * Sets the transfer function
     *
     * This method defines the transfer function used for this neuron. You have
     * the possibility to define individual transfer function for every neuron
     * of a neural net. Derive a new class from the TransferFunction class to
     * create customn transfer functions.
     *
     * @param afunction pointer to the new transferFunction object
     */
    void setTransferFunction(TransferFunction const * const afunction);

    /**
     * Updates neural activity
     *
     * This method calculates and sets the new activity value of the neuron
     * based on the external input, the bias and the weights and the presynaptic
     * outputs of all incoming synapses.
     */
    void updateActivity();

    /**
     * Updates back propagated error
     *
     * This method calculates the back propagated error for the neuron based
     * on the error values of all postsynaptic neurons.
     */
    void updateError();

    /**
     * Updates neural output
     *
     * This method calculates and sets the new output value of the neuron
     * based on the activity value and the transfer function.
     */
    void updateOutput();
private:
    /** type for holding a list of synapses associated to a neuron*/
    typedef std::map<Neuron const *, Synapse*> SynapseList;
    /** item type of the synapse list */
    typedef std::pair<Neuron const *, Synapse*> SynapseListPair;
    /** output value of the neuron */
    double output;
    /** activity value of the neuron */
    double activity;
    /** bias value of the neuron */
    double bias;
    /** input value of the neuron */
    double input;
    /** input scaling of the neuron */
    double input_scaling;
    /** back propagated error of the neuron */
    double error;
    /** external set error */
    double errorInput;
    /** list of outgoing synapses */
    SynapseList synapsesOut;
    /** list of incoming synapses */
    SynapseList synapsesIn;
    /** pointer to the used TransferFunction object */
    TransferFunction const * func;
    /**
     * pointer to static and constant TanhFunction object representing the
     * default used transfer function
     */
    static TanhFunction const * const tanhFunction;
};

#endif /* NEURON_H_ */
