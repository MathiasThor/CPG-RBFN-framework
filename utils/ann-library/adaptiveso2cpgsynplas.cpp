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

#include "adaptiveso2cpgsynplas.h"

#include <cmath>

AdaptiveSO2CPGSynPlas::AdaptiveSO2CPGSynPlas(Neuron* perturbingNeuron)
: ExtendedSO2CPG(perturbingNeuron){
    setAlpha(1.01);
    setPhi(0.3);
    setMu(1.00);

    setGamma(0.02);
    setEpsilon(0.03);
    setBeta(0.00);

    // for a range of P from -1 to 1
    setBetaDynamics   (-1.0, 0.003, 0.0000);
    setGammaDynamics  (-1.0, 0.003, 1.0000);
    setEpsilonDynamics(0.04, 0.003, 0.0001);
}

void AdaptiveSO2CPGSynPlas::updateWeights() {
    const double& phi     = getPhi();
    const double& beta    = getBeta();
    const double& gamma   = getGamma();
    const double& epsilon = getEpsilon();
    const double& mu      = getMu();
    const double& F       = getOutput(2);
    const double& w01     = getWeight(0,1);
    const double& x       = getOutput(0);
    const double& y       = getOutput(1);
    const double& P       = getPerturbation();

    // general approach
    setPhi    ( phi     + mu * gamma * F * w01 * y );
    setBeta   ( beta    + betaHebbRate    * x * F - betaDecayRate    * (beta    - betaZero) );
    setGamma  ( gamma   + gammaHebbRate   * x * F - gammaDecayRate   * (gamma   - gammaZero) );
    setEpsilon( epsilon + epsilonHebbRate * F * P - epsilonDecayRate * (epsilon - epsilonZero) );

}

void AdaptiveSO2CPGSynPlas::setBetaDynamics(const double& hebbRate, const double& decayRate, const double& zero)
{
    betaHebbRate  = hebbRate;
    betaDecayRate = decayRate;
    betaZero      = zero;
}

void AdaptiveSO2CPGSynPlas::setGammaDynamics(const double& hebbRate, const double& decayRate, const double& zero)
{
    gammaHebbRate  = hebbRate;
    gammaDecayRate = decayRate;
    gammaZero      = zero;
}

void AdaptiveSO2CPGSynPlas::setEpsilonDynamics(const double& hebbRate, const double& decayRate, const double& zero)
{
    epsilonHebbRate  = hebbRate;
    epsilonDecayRate = decayRate;
    epsilonZero      = zero;
}



