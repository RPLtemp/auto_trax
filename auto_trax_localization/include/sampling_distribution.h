/*
 * sampling_distribution.h
 *
 *  Created on: December 4, 2015
 *      Author: Pavel Vechersky
 *
 *  Class for implementing two methods of drawing random samples from
 *  distributions and uniting the two methods under one abstract type.
 */

#ifndef SAMPLING_DISTRIBUTION_H_
#define SAMPLING_DISTRIBUTION_H_

#include "math_utils.h"

#include <math.h>
#include <cstdlib>

class SamplingDistribution {
 public:
  // Draw a random sample from distribution
  virtual double sample(double b_sqrd) = 0;
};

class NormalDistribution : public SamplingDistribution {
 public:
  // Constructor
  NormalDistribution();

  // Destructor
  virtual ~NormalDistribution();

  // Draw a random sample from normal distribution
  double sample(double b_sqrd);

 private:
  // Number of samples to add up for normal estimation
  static constexpr int SAMPLES = 12;
};

class TriangularDistribution : public SamplingDistribution {
 public:
  // Constructor
  TriangularDistribution();

  // Destructor
  virtual ~TriangularDistribution();

  // Draw a random sample from triangular distribution
  double sample(double b_sqrd);

 private:
  // Constant multiplier to take square root off for scaling
  static constexpr double MULTIPLIER = 6.0;
};

#endif /* SAMPLING_DISTRIBUTION_H__ */
