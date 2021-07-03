/*
 * delayline.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: degoldschmidt
 */

#include "delayline.h"

using namespace std;

Delayline::Delayline(int size) {
  buffer.resize(size);
  step = 0;
}

void Delayline::Write(double input) {
  // write delayed values
  buffer.at(step) = input; //write
}

double Delayline::Read(int delay) {
  // read delayed values
  double y;
  y = buffer.at(mod(step-delay,buffer.size())); //read
  return y;
}

void Delayline::Step() {
  // step
  step++;
  if (step % buffer.size() == 0) {
    step = 0;
  }
}

void Delayline::Reset() {
  // reset buffer
  int newsize = buffer.size();
  buffer.clear();
  buffer.resize(newsize);
}

int Delayline::mod(int x, int m) {
       int r = x%m;
       return r<0 ? r+m : r;
}


