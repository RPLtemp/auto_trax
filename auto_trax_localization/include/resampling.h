/*
 * resampling.h
 *
 *  Created on: December 5, 2015
 *      Author: Pavel Vechersky
 *
 *  Class for implementing two particle resampling methods: stochastic
 *  universal and roulette wheel and uniting the two methods under one abstract
 *  type.
 */

#ifndef RESAMPLING_H_
#define RESAMPLING_H_

#include "wheel_bot.hpp"
#include "sampling_distribution.h"

typedef WheelBot Particle;

class Resampling {
 public:
  struct RougheningParams {
    double xy_roughening_;      // Roughening range in the x and y directions
    double th_roughening_;      // Roughening range for the heading
  };

  // Add a particle to the list to be drawn from later
  void addParticle(Particle p);

  // Draw a new set of particles from the weighted list
  virtual std::vector<Particle> resample() = 0;

 protected:
  // Vector of particles from which to draw samples
  std::vector<Particle> samples_;

  // Roughening parameters
  RougheningParams roughening_params_;
};

class StochasticUniversalResampling : public Resampling {
 public:
  // Constructor
  StochasticUniversalResampling(RougheningParams params);

  // Destructor
  virtual ~StochasticUniversalResampling();

  // Draw a new set of particles from the weighted list
  std::vector<Particle> resample();
};

class RouletteWheelResampling : public Resampling {
 public:
  // Constructor
  RouletteWheelResampling(RougheningParams params);

  // Destructor
  virtual ~RouletteWheelResampling();

  // Draw a new set of particles from the weighted list
  std::vector<Particle> resample();
};


#endif /* RESAMPLING_H__ */
