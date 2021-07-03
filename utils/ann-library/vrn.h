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

#ifndef VRN_H_
#define VRN_H_

#include "utils/ann-framework/ann.h"

/**
 * Velocity Regulating Network Class
 *
 * This class represents a VRN network as e. g. described in
 *
 * Manoonpong, P., Pasemann, F., Woergoetter, F.: Sensor-driven neural control
 * for omnidirectional locomotion and versatile reactive behaviors of walking
 * machines. Robotics and Autonomous Systems 56 (2009). 265-288
 *
 */
class VRN : public ANN {
public:
    /**
     * The constructor
     *
     * Sets synaptic weights and neural biases
     */
    VRN();

    /**
     * Returns pointer to input neuron x
     */
    Neuron* getNeuronX();

    /**
     * Returns pointer to input neuron y
     */
    Neuron* getNeuronY();

    /**
     * Returns pointer to output neuron
     */
    Neuron* getNeuronOutput();
private:

};

#endif /* VRN_H_ */
