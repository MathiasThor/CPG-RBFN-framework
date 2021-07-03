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


#include "psn.h"

PSN::PSN()
{
    setNeuronNumber(12);

    // synaptic weights
    w( 2,  0, -5.0);
    w( 3,  1, -5.0);
    w( 4,  0, -5.0);
    w( 5,  1, -5.0);
    w( 6,  2,  0.5);
    w( 7,  3,  0.5);
    w( 8,  4,  0.5);
    w( 9,  5,  0.5);
    w(10,  6,  3.0);
    w(10,  7,  3.0);
    w(11,  8,  3.0);
    w(11,  9,  3.0);

    // neuron biases
    b( 0,  1.0 );
    b( 6,  0.5 );
    b( 7,  0.5 );
    b( 8,  0.5 );
    b( 9,  0.5 );
    b(10, -1.35);
    b(11, -1.35);
}
