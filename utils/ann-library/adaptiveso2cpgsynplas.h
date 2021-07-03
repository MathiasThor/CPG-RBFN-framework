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


#ifndef _ADAPTIVESO2CPGHEBB_H
#define _ADAPTIVESO2CPGHEBB_H


#include "extendedso2cpg.h"

class AdaptiveSO2CPGSynPlas : public ExtendedSO2CPG{
public:
    AdaptiveSO2CPGSynPlas(Neuron* perturbingNeuron=0);
    void updateWeights();
    void setBetaDynamics   (const double& hebbRate, const double& decayRate, const double& zero);
    void setGammaDynamics  (const double& hebbRate, const double& decayRate, const double& zero);
    void setEpsilonDynamics(const double& hebbRate, const double& decayRate, const double& zero);
private:
    double betaZero;
    double betaDecayRate;
    double betaHebbRate;
    double gammaZero;
    double gammaDecayRate;
    double gammaHebbRate;
    double epsilonZero;
    double epsilonDecayRate;
    double epsilonHebbRate;
};

#endif
