/*
 * sampling_distribution.cpp
 *
 *  Created on: December 4, 2015
 *      Author: Pavel Vechersky
 */

#include "sampling_distribution.h"

NormalDistribution::NormalDistribution() {
}

NormalDistribution::~NormalDistribution() {
}

double NormalDistribution::sample(double b_sqrd) {
  // Take the squareroot of input argument
  double b = fabs(sqrt(b_sqrd));

  // Initialize the sum of randomly drawn samples
  double sum = 0.0;

  // Approximate normal distribution
  for (int i = 0; i < SAMPLES; i++)
    sum += MathUtils::randNumInRange(-b, b);

  return 0.5 * sum;
}

TriangularDistribution::TriangularDistribution() {
}

TriangularDistribution::~TriangularDistribution() {
}

double TriangularDistribution::sample(double b_sqrd) {
  // Take the squareroot of input argument
  double b = fabs(sqrt(b_sqrd));

  // Draw two random values in the range (-b, b)
  double s_1 = MathUtils::randNumInRange(-b, b);
  double s_2 = MathUtils::randNumInRange(-b, b);

  return 0.5 * (fabs(sqrt(MULTIPLIER)) * (s_1 + s_2));
}
