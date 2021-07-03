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


#ifndef SYNAPSE_H_
#define SYNAPSE_H_

//forward declarations
class ANN;
class Neuron;

/**
 * Synapse Class
 *
 * This class represents a single directed synapse connection to neurons
 * of an artificial neural net. The synapse is described by the pre- and
 * postsynaptic neuron and its synaptic weight.
 */
class Synapse {
public:
    /**
     * The constructor
     *
     * You have to give the pre- and postsynaptic neuron here. Those cannot
     * be changed later. Create a new synapse if you want to connect other
     * neurons instead of the present ones.
     *
     * The constructor also "registers" the new synapse at the two neurons
     * so that the synapse is inserted into their corresponding lists.
     *
     * @param apost   pointer to the postsynaptic neuron
     * @param apre    pointer to the presynaptic neuron
     * @param connect if true, the synapse will register itself at the two
     *                neurons it connects
     */
    Synapse(Neuron * const apost, Neuron * const apre,
        const bool& connect=true);

    /**
     * The destructor
     *
     * The destructor also removes the synapse from the corresponding synapse
     * lists of the post- and presynaptic neuron.
     */
    ~Synapse();

    /**
     * Returns synaptic weight change
     *
     * This method returns the weight change of this synapse. The reference stays
     * valid for the whole lifetime of the synapse.
     *
     * @return synaptic weight change
     */
    const double& getDeltaWeight() const;

    /**
     * Returns postsynaptic neuron
     *
     * @return pointer to the postsynaptic neuron
     */
    Neuron* getPost() const;

    /**
     * Returns presynaptic neuron
     *
     * @return pointer to the presynaptic neuron
     */
    Neuron* getPre() const;

    /**
     * Returns synaptic weight
     *
     * This method returns the weight of this synapse. The reference stays
     * valid for the whole lifetime of the synapse.
     *
     * @return synaptic weight
     */
    const double& getWeight() const;

    /**
     * Sets synaptic weight change
     *
     * This method can be used to alter the change in weight of this synapse.
     *
     * @param aweight new synaptic weight change
     */
    void setDeltaWeight(const double& aweight);

    /**
     * Sets synaptic weight
     *
     * This method can be used to alter the weight of this synapse.
     *
     * @param aweight new synaptic weight
     */
    void setWeight(const double& aweight);

    /**
     * Updates synaptic weight
     *
     * This method can be used to update the weight of this synapse due to plasticity.
     *
     */
    void updateWeight();

private:
    /** pointer to presynaptic neuron */
    Neuron * const pre;
    /** pointer to postsynaptic neuron */
    Neuron * const post;
    /** synapse weight */
    double weight;
    /** synaptic weight change */
    double delta_weight;
};

#endif /* SYNAPSE_H_ */
