//
// Created by mat on 3/4/18.
//

#include "postProcessing.h"

postProcessing::postProcessing() = default;

void postProcessing::lowPassFiltering(double signal) {
    // Calculate the amplitudes
    LPFSignal = LPFSignal - (LPFbeta * (LPFSignal - signal));
}

void postProcessing::calculateAmplitude(double signal, double signal2=0) {
    // Calculate the derivative
    sigDot = (signal - signalOld)/0.00167; // 25Hz = 0.04s period

    // See if the derivative changes sign
    if ((((sigDot >= 0) ^ (sigPrimeOld < 0)) == 0) && timeSinceZeroDerivative>12) {
        // If it changes sign then it is a local max- or minimum
        if (sigDot > 0 && !lastZeroDerivative) {
            zeroDerivative[0] = signalOld;
            lastZeroDerivative = true;
        } else if (sigDot < 0 && lastZeroDerivative){
            zeroDerivative[1] = signalOld;
            lastZeroDerivative = false;
        }

        // Subtract local minimum from maximum to get amplitude
        amplitude = std::fabs(zeroDerivative[0]-zeroDerivative[1])/2;

        //if(timeSinceZeroDerivative != timeBetweenZeroDerivative)
        timeBetweenZeroDerivative = timeSinceZeroDerivative;

        tmpPeriod += timeBetweenZeroDerivative;

        if(numZeroDev == 1) {
            period = tmpPeriod;
            tmpPeriod = 0;
            numZeroDev = 0;
            periodViz *= -1;
            // Make sure that a proper period is calculated
            if(periodViz < 0 && !periodTrust)
                periodTrust = true;

            // Save recorded signal
            signalRecord = signalRecordTmp;
            signalRecord2 = signalRecordTmp2;
            signalRecordTmp.clear();
            signalRecordTmp2.clear();
        }
        else{
            numZeroDev++;
        }

        timeSinceZeroDerivative = 0;
    }
    else {
        timeSinceZeroDerivative++;
        signalRecordTmp.push_back(signal);
        signalRecordTmp2.push_back(signal2);
    }

    // Advance
    sigPrimeOld = sigDot;
    signalOld = signal;
}

std::vector<float> postProcessing::getSignalPeriod(int signalNum){
    if(signalNum == 0)
        return signalRecord;
    else
        return signalRecord2;
}

double postProcessing::calculateLPFAmplitude(double signal){
    lowPassFiltering(signal);
    calculateAmplitude(LPFSignal);
    return amplitude;
}

double postProcessing::calculateLPFSignal(double signal){
    lowPassFiltering(signal);
    return LPFSignal;
}
double postProcessing::calculateAmplitudeSignal(double signal){
    calculateAmplitude(LPFSignal);
    return amplitude;
}

void postProcessing::setBeta(double beta) {
    LPFbeta=beta;
}

double postProcessing::getLPFSignal() {
    return LPFSignal;
}

double postProcessing::getAmplitudeSignal() {
    return amplitude;
}

double postProcessing::getTimeBetweenZeroDerivative() {
    return timeBetweenZeroDerivative;
}

double postProcessing::getPeriod() {
    return period;
}
