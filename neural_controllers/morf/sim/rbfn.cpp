#include <utility>

//
// Created by mat on 12/21/18.
//

#include "rbfn.h"

rbfn::rbfn(int _numKernels, vector<vector<float>> _weights, string _encoding, string _behaviour, vector<vector<float>> _weightsSensor) {

    numKernels = _numKernels;
    beta = 0.04;
    weights = std::move(_weights);
    weightsSensor = std::move(_weightsSensor);

    if (numKernels == 20){
        centers1 = pythoncenter1;
        centers2 = pythoncenter2;
    } else if (numKernels == 10){
        centers1 = pythoncenter1_10;
        centers2 = pythoncenter2_10;
    }

    encoding = _encoding;
    behaviour = _behaviour;

    /* DELAY LINE */
    Delayline tmp(tau);
    for (int i = 0; i < 2; ++i)
        delayline.push_back(tmp);
    /* DELAY LINE */
}

vector<double> rbfn::step(double input1, double input2, vector<vector<float>> sensorOutput) {
    // We are using a Gaussian radial basis function
    // see https://www.mccormickml.com/2013/08/15/radial-basis-function-network-rbfn-tutorial/
    // and https://www.youtube.com/watch?v=1Cw45yNm6VA

    // Delay Input:
    delayline[0].Write(input1);
    delayline[1].Write(input2);

    double kernelOutput = 0;
    double kernelOutput_delayed = 0;

    vector<double> rbfnOutput = vector<double>(18 + 6, 0);

    if(encoding == "direct" && (behaviour == "flip" || behaviour == "walknomed" || behaviour == "obstacle_direct")){
        for (int i = 0; i < numKernels; ++i) {
            kernelOutput         = exp(-(pow(delayline[0].Read(0) - centers1[i], 2) + pow(delayline[1].Read(0) - centers2[i], 2)) / beta);
            kernelOutput_delayed = exp(-(pow(delayline[0].Read(CPGPeriod*0.5) - centers1[i], 2) + pow(delayline[1].Read(CPGPeriod*0.5) - centers2[i], 2)) / beta);

            int jj = 0;
            // i kernels
            // j groups of joints (BC, CF, FT) = Legs
            // jj individual joints
            if(sensorOutput.empty()) { // If we are not providing any sensory input
                for (int j = 0; j < 3; ++j) {
                    rbfnOutput[jj]     += (weights[0][i + (j * numKernels)] * kernelOutput);
                    rbfnOutput[jj + 1] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    rbfnOutput[jj + 2] += (weights[0][i + (j * numKernels)] * kernelOutput);

                    rbfnOutput[jj + 3] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    rbfnOutput[jj + 4] += (weights[0][i + (j * numKernels)] * kernelOutput);
                    rbfnOutput[jj + 5] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    jj = jj + 6;
                }
            } else if(behaviour != "walk" && behaviour != "multiple") {
                if(behaviour == "flip"){
                    for (int j = 0; j < 3; ++j) {
                        int behaviour_index = 0;
                        rbfnOutput[jj]     += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][0]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 1] += ((weightsSensor[behaviour_index][i+((j+1)*numKernels)] * sensorOutput[behaviour_index][1]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 2] += ((weightsSensor[behaviour_index][i+((j+2)*numKernels)] * sensorOutput[behaviour_index][2]) + weights[0][i + (j * numKernels)]) * kernelOutput;

                        rbfnOutput[jj + 3] += ((weightsSensor[behaviour_index][i+((j+3)*numKernels)] * sensorOutput[behaviour_index][3]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 4] += ((weightsSensor[behaviour_index][i+((j+4)*numKernels)] * sensorOutput[behaviour_index][4]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 5] += ((weightsSensor[behaviour_index][i+((j+5)*numKernels)] * sensorOutput[behaviour_index][5]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        jj = jj + 6;
                    }
                } else {
                    for (int j = 0; j < 3; ++j) {
                        int behaviour_index = 0;
                        rbfnOutput[jj]     += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][0]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 1] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][1]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 2] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][2]) + weights[0][i + (j * numKernels)]) * kernelOutput;

                        rbfnOutput[jj + 3] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][3]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 4] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][4]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 5] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][5]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        jj = jj + 6;
                    }
                }
            } else if(behaviour == "multiple") {
                vector<float> sensorContribution (6,0);
                vector<float> baseWalkingContribution (6,0);

                for (int j = 0; j < 3; ++j) {
                    for (int k = 0; k < 3; ++k) {
                        sensorContribution[0] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][0]);
                        sensorContribution[1] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][1]);
                        sensorContribution[2] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][2]);
                        sensorContribution[3] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][3]);
                        sensorContribution[4] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][4]);
                        sensorContribution[5] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][5]);
                    }

                    baseWalkingContribution[0] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[1] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[2] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[3] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[4] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[5] = weights[0][i + (j * numKernels)];

                    rbfnOutput[jj]     += (sensorContribution[0]+baseWalkingContribution[0]) * kernelOutput;
                    rbfnOutput[jj + 1] += (sensorContribution[1]+baseWalkingContribution[1]) * kernelOutput_delayed;
                    rbfnOutput[jj + 2] += (sensorContribution[2]+baseWalkingContribution[2]) * kernelOutput;

                    rbfnOutput[jj + 3] += (sensorContribution[3]+baseWalkingContribution[3]) * kernelOutput_delayed;
                    rbfnOutput[jj + 4] += (sensorContribution[4]+baseWalkingContribution[4]) * kernelOutput;
                    rbfnOutput[jj + 5] += (sensorContribution[5]+baseWalkingContribution[5]) * kernelOutput_delayed;
                    jj = jj + 6;
                }
            } else {
                cout << "[ ERROR] UNKNOWN SENSOR WEIGHT SIZE!" << endl;
            }
        }

        // SET PHASES TO TRIPOD
        for (int k = 0; k < 6; ++k)
            rbfnOutput[18+k] = 0.5;
    } else if (encoding == "sindirect") {
        for (int i = 0; i < numKernels; ++i) {
            kernelOutput         = exp(-(pow(delayline[0].Read(0) - centers1[i], 2) + pow(delayline[1].Read(0) - centers2[i], 2)) / beta);
            kernelOutput_delayed = exp(-(pow(delayline[0].Read(CPGPeriod*0.5) - centers1[i], 2) + pow(delayline[1].Read(CPGPeriod*0.5) - centers2[i], 2)) / beta);

            int jj = 0;

            if(sensorOutput.empty()) {
                for (int j = 0; j < 18 / 2; j = j + 3) {
                    rbfnOutput[jj]     += (weights[0][i + (j * numKernels)] * kernelOutput);
                    rbfnOutput[jj + 1] += (weights[0][i + ((j + 1) * numKernels)] * kernelOutput_delayed);
                    rbfnOutput[jj + 2] += (weights[0][i + ((j + 2) * numKernels)] * kernelOutput);

                    rbfnOutput[jj + 3] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    rbfnOutput[jj + 4] += (weights[0][i + ((j + 1) * numKernels)] * kernelOutput);
                    rbfnOutput[jj + 5] += (weights[0][i + ((j + 2) * numKernels)] * kernelOutput_delayed);
                    jj = jj + 6;
                }
            } else if(behaviour != "walk" && behaviour != "multiple") {
                for (int j = 0; j < 18 / 2; j = j + 3) {
                    int behaviour_index = 0;
                    rbfnOutput[jj]     += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][0]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                    rbfnOutput[jj + 1] += ((weightsSensor[behaviour_index][i+((j+1)*numKernels)] * sensorOutput[behaviour_index][1]) + weights[0][i + ((j+1) * numKernels)]) * kernelOutput_delayed;
                    rbfnOutput[jj + 2] += ((weightsSensor[behaviour_index][i+((j+2)*numKernels)] * sensorOutput[behaviour_index][2]) + weights[0][i + ((j+2) * numKernels)]) * kernelOutput;

                    rbfnOutput[jj + 3] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][3]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                    rbfnOutput[jj + 4] += ((weightsSensor[behaviour_index][i+((j+1)*numKernels)] * sensorOutput[behaviour_index][4]) + weights[0][i + ((j+1) * numKernels)]) * kernelOutput;
                    rbfnOutput[jj + 5] += ((weightsSensor[behaviour_index][i+((j+2)*numKernels)] * sensorOutput[behaviour_index][5]) + weights[0][i + ((j+2) * numKernels)]) * kernelOutput_delayed;
                    jj = jj + 6;
                }
            }
        }

        // SET PHASES TO TRIPOD
        for (int k = 0; k < 6; ++k)
            rbfnOutput[18+k] = 0.5;

    } else if (encoding == "indirect") {
        for (int i = 0; i < numKernels; ++i) {
            kernelOutput         = exp(-(pow(delayline[0].Read(0) - centers1[i], 2) + pow(delayline[1].Read(0) - centers2[i], 2)) / beta);
            kernelOutput_delayed = exp(-(pow(delayline[0].Read(CPGPeriod*0.5) - centers1[i], 2) + pow(delayline[1].Read(CPGPeriod*0.5) - centers2[i], 2)) / beta);

            int jj = 0;
            // i kernels
            // j groups of joints (BC, CF, FT) = Legs
            // jj individual joints
            if(sensorOutput.empty()) { // If we are not providing any sensory input
                for (int j = 0; j < 3; ++j) {
                    rbfnOutput[jj]     += (weights[0][i + (j * numKernels)] * kernelOutput);
                    rbfnOutput[jj + 1] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    rbfnOutput[jj + 2] += (weights[0][i + (j * numKernels)] * kernelOutput);

                    rbfnOutput[jj + 3] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    rbfnOutput[jj + 4] += (weights[0][i + (j * numKernels)] * kernelOutput);
                    rbfnOutput[jj + 5] += (weights[0][i + (j * numKernels)] * kernelOutput_delayed);
                    jj = jj + 6;
                }
            } else if(behaviour != "walk" && behaviour != "multiple") {
                if(behaviour == "flip"){
                    for (int j = 0; j < 3; ++j) {
                        int behaviour_index = 0;
                        rbfnOutput[jj]     += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][0]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 1] += ((weightsSensor[behaviour_index][i+((j+1)*numKernels)] * sensorOutput[behaviour_index][1]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 2] += ((weightsSensor[behaviour_index][i+((j+2)*numKernels)] * sensorOutput[behaviour_index][2]) + weights[0][i + (j * numKernels)]) * kernelOutput;

                        rbfnOutput[jj + 3] += ((weightsSensor[behaviour_index][i+((j+3)*numKernels)] * sensorOutput[behaviour_index][3]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 4] += ((weightsSensor[behaviour_index][i+((j+4)*numKernels)] * sensorOutput[behaviour_index][4]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 5] += ((weightsSensor[behaviour_index][i+((j+5)*numKernels)] * sensorOutput[behaviour_index][5]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        jj = jj + 6;
                    }
                } else {
                    for (int j = 0; j < 3; ++j) {
                        int behaviour_index = 0;
                        rbfnOutput[jj]     += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][0]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 1] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][1]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 2] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][2]) + weights[0][i + (j * numKernels)]) * kernelOutput;

                        rbfnOutput[jj + 3] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][3]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        rbfnOutput[jj + 4] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][4]) + weights[0][i + (j * numKernels)]) * kernelOutput;
                        rbfnOutput[jj + 5] += ((weightsSensor[behaviour_index][i+(j*numKernels)] * sensorOutput[behaviour_index][5]) + weights[0][i + (j * numKernels)]) * kernelOutput_delayed;
                        jj = jj + 6;
                    }
                }
            } else if(behaviour == "multiple") {
                for (int j = 0; j < 3; ++j) {
                    vector<float> sensorContribution (6,0);
                    vector<float> baseWalkingContribution (6,0);
                    vector<vector<float>> contributions_temp (9, vector<float>(6, 0));

                    for (int k = 0; k < weightsSensor.size(); ++k) {
                        for (int l = 0; l < 6; ++l) {
                            sensorContribution[l] += (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][l]);
                            contributions_temp[k][l] = (weightsSensor[k][i+(j*numKernels)] * sensorOutput[k][l]);
                        }
                    }

                    contributions = contributions_temp;

                    baseWalkingContribution[0] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[1] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[2] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[3] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[4] = weights[0][i + (j * numKernels)];
                    baseWalkingContribution[5] = weights[0][i + (j * numKernels)];

                    rbfnOutput[jj]     += (sensorContribution[0]+baseWalkingContribution[0]) * kernelOutput;
                    rbfnOutput[jj + 1] += (sensorContribution[1]+baseWalkingContribution[1]) * kernelOutput_delayed;
                    rbfnOutput[jj + 2] += (sensorContribution[2]+baseWalkingContribution[2]) * kernelOutput;

                    rbfnOutput[jj + 3] += (sensorContribution[3]+baseWalkingContribution[3]) * kernelOutput_delayed;
                    rbfnOutput[jj + 4] += (sensorContribution[4]+baseWalkingContribution[4]) * kernelOutput;
                    rbfnOutput[jj + 5] += (sensorContribution[5]+baseWalkingContribution[5]) * kernelOutput_delayed;
                    jj = jj + 6;
                }
            } else {
                cout << "[ ERROR] UNKNOWN SENSOR WEIGHT SIZE!" << endl;
            }
        }

        // SET PHASES TO TRIPOD
        for (int k = 0; k < 6; ++k)
            rbfnOutput[18+k] = 0.5;

    } else
        cout << "[ ERROR] unknown encoding (rbfn.cpp: " << encoding << ")" << endl;


    // Delay step:
    for (auto &i : delayline)
        i.Step();

    return rbfnOutput;
}

void rbfn::setBeta(double _beta) {
    beta = _beta;
}

void rbfn::setWeights(vector<vector<float>> _weights) {
    weights = _weights;
}

int rbfn::getNumKernels() {
    return numKernels;
}

void rbfn::setCenters(vector<float> _centers1, vector<float> _centers2) {
    centers1 = _centers1;
    centers2 = _centers2;
}

double rbfn::getBeta() {
    return beta;
}

vector<vector<float>> rbfn::getWeights() {
    return weights;
}

vector<float> rbfn::setCenters(int center) {
    if (center == 1)
        return centers1;
    else
        return centers2;
}

void rbfn::calculateCenters(int period, vector<float> signal1, vector<float> signal2) {
    vector<float> _centers = linspace(1, period, numKernels);
    centers1 = _centers;
    centers2 = _centers;

    for (int i = 0; i < numKernels; ++i) {
        centers1[i] = signal1[_centers[i]];
        centers2[i] = signal2[_centers[i]];
    }
}

template<typename T>
std::vector<float> rbfn::linspace(T start_in, T end_in, int num_in)
{

    std::vector<float> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) { return linspaced; }
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

void rbfn::setCPGPeriod(int _period) {
    CPGPeriod = _period;
}
