/*
 * circann.h
 *
 *  Created on: 21.10.2015
 *      Author: Dennis Goldschmidt
 */

#ifndef CIRCANN_H_
#define CIRCANN_H_

#include <cmath>
#include <osg/Vec3f>
#include "ann.h"
using namespace std;

enum NoiseType{correlated, uncorrelated};

class CircANN : public ANN {
public:

    /**
     * The constructor.
     *
     * @param numneurons Number of neurons
     */
	CircANN(int numneurons);

    /**
     * The destructor.
     */
	~CircANN();

    /**
     * Returns the maximum firing rate in the circular array.
     *
     * @return (double)
     */
	double getMaxRate();

    /**
     * Returns the preferred orientation of a given neuron.
     *
     * @param (int) index: neuron index
     * @return (double)
     */
	double getPrefAngle(int index);

    /**
     * Returns the sum of firing rate in the circular array.
     *
     * @return (double)
     */
	double getSumRate();

    /**
     * Returns the angle of the population vector average in the circular array.
     *
     * @return (double)
     */
	double getVecAvgAngle();

    /**
     * Returns the vector representation of the activities in the circular array.
     *
     * @return (double)
     */
	osg::Vec3f getVector();

    /**
     * Returns the number of neurons of this network per layer
     *
     * @return number of neurons
     */
    unsigned int N() const;


	/**
	 * Overloading ANN step function with additional update of vector representation
	 *
	 *
	 */
	void step();

private:
	osg::Vec3f vector;
	double angle;
	const double scale_factor = 2.41456;
};

#endif /* CIRCANN_H_ */
