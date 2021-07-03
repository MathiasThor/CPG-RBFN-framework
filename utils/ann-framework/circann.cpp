/*
 * circann.cpp
 *
 *  Created on: 21.10.2015
 *      Author: Dennis Goldschmidt
 */

#include "circann.h"
using namespace std;
#include <osg/Vec3f>

CircANN::CircANN(int numneurons) : ANN(numneurons) {
	setAllTransferFunctions(linthresholdFunction());
	angle = 0.0;
}

CircANN::~CircANN() {

}

double CircANN::getMaxRate() {
	double max_rate = 0;
	for(unsigned int index = 0; index < N(); index++){
		double currOutput = getOutput(index);
		if(currOutput > max_rate)
			max_rate = currOutput;
	}
	return max_rate;
}

double CircANN::getPrefAngle(int index){
	return (2*M_PI*index)/N();
}

double CircANN::getSumRate(){
	double sum = 0;
	for(unsigned int index = 0; index < N(); index++)
		sum += getOutput(index);
	return sum;
}

double CircANN::getVecAvgAngle(){
	return angle;
}

osg::Vec3f CircANN::getVector(){
	return vector;
}

unsigned int CircANN::N() const
{
    return getNeuronNumber();
}

void CircANN::step(){
	ANN::step();
	double sumx = 0;
	double sumy = 0;
	double sum = 0;
	for(unsigned int index = 0; index < N(); index++){
		sumx += getOutput(index) * cos(getPrefAngle(index));
		sumy += getOutput(index) * sin(getPrefAngle(index));
		sum += getOutput(index);
	}
	vector.set(sumx, sumy, 0.);
	angle = ( (atan2(vector.y(),vector.x())>0) ? (atan2(vector.y(),vector.x())) : (atan2(vector.y(),vector.x()) + 2 * M_PI) );
	vector.normalize();
	vector *= scale_factor * sum/double(N()*N());
}
