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


#ifndef TRANSFERFUNCTION_H_
#define TRANSFERFUNCTION_H_

// _USE_MATH_DEFINES is neccessary to make e.g. M_PI available in Microsoft
// Visual Studio
#define _USE_MATH_DEFINES
#include <cmath>

/**
 * Transfer Function Class
 *
 * This class represents the general transfer function interface that can be
 * used by a neuron to update its output value. Every type of transfer function
 * you want to use function has to subclass TransferFunction and to implement
 * the operator() method.
 */
class TransferFunction {
public:
    /**
     * The constructor
     */
    TransferFunction() {};

    /**
     * The destructor
     */
    virtual ~TransferFunction() {};

    /**
     * Method for function call
     *
     * This method overloads the () operator and represents the usage of
     * the transfer function. Every subclass has to implement it.
     *
     * @param a the activity value of the neuron
     * @return the new output value of the neuron
     */
    virtual double operator()(const double& a) const = 0;

    /**
     * Method for derivative call
     *
     * This method calculates the derivative of the transfer function at
     * the activity level a. This is needed for backpropagation.
     *
     * @param a activity value of the nruon
     * @return derivative of the transfer function at activity a
     */
    virtual double derivative(const double& a) const = 0;
};


/**
 * tanh transfer function (sigmoid)
 *
 * o(a) = tanh(a)
 */
class TanhFunction : public TransferFunction {
public:
    inline double operator()(const double& x) const {
        return std::tanh(x);
    }
    inline double derivative(const double& x) const {
      return 1-std::pow(std::tanh(x),2);
    }
};

/*
 * logistic transfer function (sigmoid)
 *
 * o(a) = 1./(1+exp(-a))
 */
class LogisticFunction : public TransferFunction {
public:
    inline double operator()(const double& x) const {
        return 1./(1+std::exp(-x));
    }
    inline double derivative(const double& x) const {
      const double f = (*this)(x);
      return f*(1.0-f);
    }
};

/**
 * Linear transfer function
 *
 * o(a) = m*a+b
 */
class LinearFunction : public TransferFunction {
  public:
    LinearFunction(const double& m=1, const double& b=0):m(m),b(b) {}
    inline double operator()(const double& x) const {
      return m*x+b;
    }
    inline double derivative(const double& x) const {
      return m;
    }
    inline void setM(const double &am) {m=am;}
    inline void setB(const double &ab) {b=ab;}
    inline const double& getM() const{return m;}
    inline const double& getB() const{return b;}
  private:
    double m;
    double b;
};

/**
 * Linear rectifier transfer function
 *
 * o(a) = m*a+b (if m*a+b > 0) 0 (else)
 */
class LinearThresholdFunction : public TransferFunction {
public:
    LinearThresholdFunction(const double& m=1, const double& b=0):m(m),b(b) {}
    inline double operator()(const double& x) const {
        if(m*x+b > 0)
            return m*x+b;
        else
            return 0.;
    }
    inline double derivative(const double& x) const {
      return m;
    }
    inline void setM(const double &am) {m=am;}
    inline void setB(const double &ab) {b=ab;}
    inline const double& getM() const{return m;}
    inline const double& getB() const{return b;}
  private:
    double m;
    double b;
};

/**
 * Sign transfer function
 *
 */
class SignFunction : public TransferFunction {
  public:
    SignFunction(const double& theta=1):theta(theta) {}
    inline double operator()(const double& x) const {
        if(x > theta)
            return 1.0;
        else if (x == theta)
            return 0.0;
        else
            return -1.0;
    }
    inline double derivative(const double& x) const {
      if (x==theta)
        return INFINITY;
      else
        return 0.0;
    }
    inline void setTheta(const double &atheta) {theta=atheta;}
    inline const double& getTheta() const{return theta;}
  private:
    double theta;

};

/**
 * Heaviside transfer function
 *
 */
class ThresholdFunction : public TransferFunction {
  public:
    ThresholdFunction(const double& theta=1):theta(theta) {}
    inline double operator()(const double& x) const {
      if(x > theta)
        return 1.0;
      else
        return 0.0;
    }
    inline double derivative(const double& x) const {
      if (x==theta)
        return INFINITY;
      else
        return 0.0;
    }
    inline void setTheta(const double &atheta) {theta=atheta;}
    inline const double& getTheta() const{return theta;}
  private:
    double theta;

};

#endif /* TRANSFERFUNCTION_H_ */
