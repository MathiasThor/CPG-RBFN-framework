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


#include "vrn.h"

VRN::VRN()
{
    setNeuronNumber(7);

    w(2, 0, 1.7246);
    w(2, 1, 1.7246);
    w(3, 0, -1.7246);
    w(3, 1, -1.7246);
    w(4, 0,  1.7246);
    w(4, 1, -1.7246);
    w(5, 0, -1.7246);
    w(5, 1,  1.7246);
    w(6, 2,  0.5);
    w(6, 3,  0.5);
    w(6, 4, -0.5);
    w(6, 5, -0.5);

    b(2, -2.48285);
    b(3, -2.48285);
    b(4, -2.48285);
    b(5, -2.48285);
}

Neuron* VRN::getNeuronX()
{
    return getNeuron(0);
}

Neuron* VRN::getNeuronY()
{
    return getNeuron(1);
}

Neuron* VRN::getNeuronOutput()
{
    return getNeuron(6);
}
