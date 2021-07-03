/*
 * delayline.h
 *  Created on: Apr 10, 2012
 *      Author: degoldschmidt
 */

#ifndef DELAYLINE_H_
#define DELAYLINE_H_

#include <vector>
#include <iostream>
#include <cstdlib>
using namespace std;

class Delayline {
public:
    Delayline(int size);
    double Read(int delay);
    void Write(double input);
    void Step();
    void Reset();
    static int mod(int x, int m);
    vector<double> buffer;
    int step;

private:



};


#endif /* DELAYLINE_H_ */
