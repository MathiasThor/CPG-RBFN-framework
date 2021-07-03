//
// Created by mat on 12/21/18.
//

#ifndef MORF_CONTROLLER_RBFN_H
#define MORF_CONTROLLER_RBFN_H

#include "ann-framework/ann.h"
#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <string.h>
#include "delayline.h"

class rbfn {
public:
    rbfn(int _numKernels, vector<vector<float>> weight, string encoding, string behaviour, vector<vector<float>> weightSensors = {});
    int getNumKernels();

    void setCPGPeriod(int _period);
    void setBeta(double _beta);
    void setWeights(vector<vector<float>> _weights);
    void setCenters(vector<float> _centers1, vector<float> _centers2);

    double getBeta();
    vector<vector<float>> getWeights();
    vector<float> setCenters(int center);

    vector<vector<float>> contributions;

    vector<double> step(double input1, double input2, vector<vector<float>> sensorOutput = {});
    void calculateCenters(int period, vector<float> signal1, vector<float> signal2);

private:
    int            numKernels = 0;
    double         beta = 0;           // controls spread of the kernel todo make unique for each and optimize
    vector<vector<float>>  weights;
    vector<vector<float>>  weightsSensor;
    vector<float>  centers1;
    vector<float>  centers2;
    string         encoding;
    string         behaviour;

    // Fixed
    vector<float> pythoncenter1{-0.19629085003193042, -0.1803803684879207, -0.15030423182556196, -0.1082190635329925, -0.05613592095571705, 0.003603295724092634, 0.06687723768703556, 0.12579827631841214, 0.17021873540086016, 0.1937555738192009, 0.1958824414584636, 0.1798116888694056, 0.14960025950775502, 0.10739906577384264, 0.05522169395134108, -0.004574948781670868, -0.06782267707017003, -0.12657531224953125, -0.17069688838001223, -0.19390211387205736};
    vector<float> pythoncenter2{0.006249717362699904, 0.06958848153189617, 0.1281375358718078, 0.1717668091697633, 0.19437367595071275, 0.1964003046602903, 0.18120355848904637, 0.15172887207945684, 0.11015129843719734, 0.0584812790943683, -0.00721669178311189, -0.0704724839678656, -0.12881358922563063, -0.17213263267456702, -0.19442094097003218, -0.19618250847744553, -0.18077260364239828, -0.15111838485887752, -0.10938829351684973, -0.057593993857780745};
    // Original

    vector<float> pythoncenter1_10{-0.19629085003193042, -0.15030423182556196, -0.05613592095571705, 0.06687723768703556, 0.17021873540086016, 0.1958824414584636, 0.14960025950775502, 0.05522169395134108, -0.06782267707017003, -0.17069688838001223, -0.19646930210892774};

    vector<float> pythoncenter2_10{0.006249717362699904, 0.1281375358718078, 0.19437367595071275, 0.18120355848904637, 0.11015129843719734, -0.00721669178311189, -0.12881358922563063, -0.19442094097003218, -0.18077260364239828, -0.10938829351684973, 0.0019409214184738603};

    template<typename T> std::vector<float> linspace(T start_in, T end_in, int num_in);

    /* DELAY LINE */
    int CPGPeriod   = 0;
    int tau         = 300;
    vector<Delayline>   delayline;
};


#endif //MORF_CONTROLLER_RBFN_H
