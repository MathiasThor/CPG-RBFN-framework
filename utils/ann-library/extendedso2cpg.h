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


#ifndef EXTENDEDSO2CPG_H_
#define EXTENDEDSO2CPG_H_

#include "so2cpg.h"

/**
 * Represents an extended SO(2) network
 *
 * This class represents an SO(2) network with an additional neuron that
 * filters the incoming feedback/perturbation signal. The SO(2) network
 * contains the neurons 0 and 1. Neuron 2 is the filter neuron. Compared
 * to the pure SO(2) network two new synapses are introduced with the
 * following abbreviations:
 *   beta  = w(2,0);
 *   gamma = w(0,2);
 * A third synapse is introduced that connects the perturbing signal with the
 * neuron 2. It is called epsilon.
 *
 * This class does not introduce any dynamics on the synapses or the control
 * parameter of the SO(2) network. Instead, it serfs as a base class for
 * different approaches to control this network
 */
class ExtendedSO2CPG : public SO2CPG{
public:
    /**
     * The constructor
     */
    ExtendedSO2CPG(Neuron* perturbingNeuron=0);

    /**
     * The destructor
     */
    virtual ~ExtendedSO2CPG() {};

    /**
     * Defines if resets are allowed
     *
     * This methods defines if resets are allowed for this network. If yes,
     * the network will be set to a defined state once certain conditions are
     * met as defined by shouldReset() and reset().
     *
     * @param allow true if resets should be allowed, false if not
     */
    void allowResets(const bool allow);

    /**
     * Returns synaptic weight beta
     *
     * This method returns the synaptic weight beta = w(2,0). It is an
     * abbreviation for getWeight(2,0).
     *
     * @return the value of beta
     */
    const double& getBeta();

    /**
     * Returns synaptic weight epsilon
     *
     * This method returns the synaptic weight epsilon of the synapse that
     * connects the external perturbation with neuron 2.
     *
     * @return the value of epsilon
     */
    const double& getEpsilon();

    /**
     * Returns synaptic weight gamma
     *
     * This method returns the synaptic weight gamma = w(0,2). It is an
     * abbreviation for getWeight(0,2).
     *
     * @return the value of gamma
     */
    const double& getGamma();

    /**
     * Returns learning rate
     *
     * This method returns the learning rate mu of the system. Note that this
     * is only of relevance for classes that derive from ExtendedSO2CPG as
     * there is no learning mechanism present in ExtendedSO2CPG itself.
     *
     * @return value of the learning constant
     */
    const double& getMu();

    /**
     * Returns perturbation value
     *
     * This method returns the value of the perturbation fed to neuron 2 through
     * synapse epsilon.
     *
     * @return the value of the perturbation
     */
    const double& getPerturbation();

    Neuron* getPerturbingNeuron();

    /**
     * Sets the synaptic weight beta
     *
     * This method sets the synaptic weight beta=w(2,0). It is an abbreviation
     * for setWeight(2,0,double).
     *
     * @param agamma the new value for beta
     */
    void setBeta(const double& abeta);

    /**
     * Sets the synaptic weight gamma
     *
     * This method sets the synaptic weight gamma=w(0,2). It is an abbreviation
     * for setWeight(0,2,double).
     *
     * @param agamma the new value for gamma
     */
    void setGamma(const double& agamma);

    /**
     * Sets the synaptic weight epsilon
     *
     * This method sets the synaptic epsilon of the synapse that connects the
     * external perturbation with neuron 2.
     *
     * @param agamma the new value for epsilon.
     */
    void setEpsilon(const double& aepsilon);

    /**
     * Sets the learning rate
     *
     * This method sets the learning rate mu of the system. Note that this
     * is only of relevance for classes that derive from ExtendedSO2CPG as
     * there is no learning mechanism present in ExtendedSO2CPG itself.
     *
     * @param amu new learning rate
     */
    void setMu(const double& amu);

    /**
     * Sets the perturbation
     *
     * This method sets the value of the perturbation that is fed into the
     * system through synapse epsilon.
     *
     * @param aP new value for the perturbation
     */
    void setPerturbation(const double& aP);

    /**
     * Does post processing stuff
     *
     * This method is called in every simulation step after the update of the
     * neuron outputs. It checks whether resets are allowed and if the
     * conditions for a reset are met and calls reset() if necessary.
     */
    virtual void  postProcessing();
protected:
    /**
     * Decides if systems should be reseted
     *
     * This method is called in every time step by postProcessing(). Its return
     * value defines whether a reset should be carried out. Overload this
     * function to introduce your own criteria. The default implementation
     * always returns false.
     *
     * @return true if system should be reseted
     */
    virtual bool shouldReset() {return false;};

    /**
     * Resets the system
     *
     * This method is called when a reset is supposed to be done. If you use
     * resets you have to overload this function to define which variables to
     * reset. The default implementation does nothing.
     */
    virtual void reset() {};
private:
    /** flag to decide if resets are allowed*/
    bool resetsAllowed;
    /** neuron that delivers external perturbation by synapse epsilon */
    Neuron* P;
    /** learning rate */
    double mu;
    /** synaptic weight from perturbation to neuron 2 */
    double epsilon;
};

#endif /* EXTENDEDSO2CPG_H_ */
