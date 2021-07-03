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


#ifndef SO2CPG_H_
#define SO2CPG_H_

#include "utils/ann-framework/ann.h"
#include "utils/interpolator2d.h"

/**
 * Class representing a two neuron SO(2) network
 *
 * This class represents the two neuron SO(2) network as introduced by
 * Pasemann et al. in "SO(2)-Networks as Neural Oscillators"
 * ( http://dx.doi.org/10.1007/3-540-44868-3_19 )
 *
 * The weights of the four synapses are governed by the SO(2) weight matrix:
 * @verbatim
 *    /        \                /                     \
 *   | w00  w01 |              |  cos(phi)   sin(phi)  |
 *   |          |   =  alpha * |                       |
 *   | w10  w11 |              | -sin(phi)   cos(phi)  |
 *    \        /                \                     /
 * @endverbatim
 *
 * You should use the tanh transfer function for this network (default)
 */
class SO2CPG : public ANN
{
public:
    /**
     * The constructor
     */
    SO2CPG();

    /**
     * Enables frequency table support
     *
     * Enables or disables the frequency table support. You need to have a file
     * called so2cpg_fVsPhi_aXX.dat in the run folder (XX stands for the current
     * alpha value) that contains in the first row the phi value (in pi) and in
     * the second row the frequency value. Comments can be added by using # at
     * the beginning of the line.
     *
     * @param enabled Switch frequency table usage on (true) or off (false)
     */
    void enableFrequencyTable(const bool enabled=true);

    /**
     * Returns current alpha value
     *
     * @return alpha value
     */
    const double& getAlpha() const;

    /**
     * Returns theoretical frequency
     *
     * This method returns the frequency at which the cpg will oscillatewith the
     * given alpha and phi values if there are not any further inputs or biases.
     * the frequency table is used for this if enabled. Otherwise a linear
     * approximation valid for small alpha values is used.
     *
     * @return frequency
     */
    const double getFrequency() const;

    /**
     * Returns current phi value
     *
     * @return phi value
     */
    const double& getPhi() const;

    /**
     * Returns phi value for a given frequency
     *
     * This method returns the phi value necessary to achieve the given
     * frequency. If enabled the frequency table is used. Otherwise a linear
     * approximation valid for small alpha values is used.
     *
     * @param afrequency desired frequency
     * @return phi value necessary to achieve this frequency
     */
    const double getPhi(const double& afrequency) const;

    /**
     * Sets the alpha value
     *
     * This method sets the alpha value for the SO(2) network. The four synaptic
     * weigths are updated accordingly and if enabled the frequency table for
     * this alpha value is read from file.
     *
     * @param aalpha new alpha value
     */
    void setAlpha(const double& aalpha);

    /**
     * Sets the phi value to achieve the given frequency
     *
     * This method can be used to directly set the phi value necessary to
     * achieve the given frequency. If enabled the frequency table is used for
     * this task. Otherwise a linear approximation valid for small alpha values
     * is used
     *
     * @param afrequency desired frequency
     */
    void setFrequency(const double& afrequency);

    /**
     * Sets the phi value
     *
     * This method sets the phi value for the SO(2) network. The four synaptic
     * weights are updated accordingly.
     *
     * @param aphi desired phi value
     */
    void setPhi(const double& aphi);
protected:

    /**
     * Read frequency table from file
     *
     * If enabled this method rereads the frequency table from the file with
     * name so2cpg_fVsPhi_aXX.dat.
     */
    void updateFrequencyTable();

    /**
     * Applies SO(2) weight matrix
     *
     * This method updates the weights of the four synapses according to the
     * SO(2) weight matrix and the current values of phi and alpha.
     */
    void updateSO2Weights();
private:
    /** parameter phi of the SO(2) weight matrix */
    double phi;
    /** parameter alpha of the SO(2) weight matrix */
    double alpha;
    /** flag to determine if frequency table feature is enabled */
    bool frequencyTableEnabled;
    /** interpolator to store the frequency table */
    Interpolator2d frequencyTable;
};


#endif /* SO2CPG_H_ */
