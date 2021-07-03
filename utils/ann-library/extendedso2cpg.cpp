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


#include "extendedso2cpg.h"
#include "utils/ann-framework/synapse.h"
#include "utils/ann-framework/neuron.h"

#include <cmath>

ExtendedSO2CPG::ExtendedSO2CPG(Neuron* perturbingNeuron):SO2CPG()
{
  // add the additional neuron with index 2
  addNeuron();

  // if a neuron was given, this one is used the perturbing neuron. If not, a
  // new neuron with a linear transfer function is created. It will have the
  // index 3
  if (perturbingNeuron)
  {
    P = perturbingNeuron;
  }
  else
  {
    P = addNeuron();
    P->setTransferFunction(identityFunction());
  }


    setAlpha(1.01);
    setPhi(0.3);
    setMu(1.00);
    setGamma(1.0);
    setEpsilon(0.3);
    setBeta(-0.05);
    resetsAllowed = false;
}

void ExtendedSO2CPG::allowResets(const bool allow)
{
    resetsAllowed = allow;
}

const double& ExtendedSO2CPG::getBeta()
{
    return getSynapse(2,0)->getWeight();
}

const double& ExtendedSO2CPG::getEpsilon()
{
    return getSynapse(n(2), P)->getWeight();
}

const double& ExtendedSO2CPG::getGamma()
{
    return getSynapse(0,2)->getWeight();
}

const double& ExtendedSO2CPG::getMu()
{
    return mu;
}

const double& ExtendedSO2CPG::getPerturbation()
{
    return P->getOutput();
}

Neuron* ExtendedSO2CPG::getPerturbingNeuron()
{
  return P;

}


void ExtendedSO2CPG::postProcessing()
{
    if (resetsAllowed && shouldReset()) reset();
}

void ExtendedSO2CPG::setBeta(const double& aBeta) {
    setWeight(2,0,aBeta);
}

void ExtendedSO2CPG::setEpsilon(const double& aepsilon) {
    setWeight(n(2), P, aepsilon);
}

void ExtendedSO2CPG::setGamma(const double& agamma) {
    setWeight(0,2,agamma);
}

void ExtendedSO2CPG::setMu(const double& amu)
{
    mu = amu;
}

void ExtendedSO2CPG::setPerturbation(const double& aP)
{
  P->setInput(aP);
  P->setOutput(aP);
}
