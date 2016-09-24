/*
 * resampling.cpp
 *
 *  Created on: December 5, 2015
 *      Author: Pavel Vechersky
 */

#include "resampling.h"

void Resampling::addParticle(Particle p) {
  // Accumulate total weight as addition of weights of consequent particles
  if (samples_.size() > 0) {
    double p_weight = p->getWeight();
    double total_weight = samples_.back()->getWeight();
    p->setWeight(total_weight + p_weight);
  }

  // Add it to the vector of samples
  samples_.push_back(p);
}

StochasticUniversalResampling::StochasticUniversalResampling(RougheningParams params) {
  // Set the roughening parameters
  roughening_params_ = params;
}

StochasticUniversalResampling::~StochasticUniversalResampling() {
}

std::vector<Particle> StochasticUniversalResampling::resample() {
  std::vector<Particle> new_particles;

  // Compute the distance between samples
  double sample_d = (samples_.back()->getWeight() / samples_.size());

  // Keep track of last particle we drew so that we can start at that index
  // the next drawing
  int prev_sample_ind = 0;

  // Current weight
  double curr_weight;

  // Roughening samples
  /*double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
  double xy_range = roughening_params_.xy_roughening_;
  double th_range = roughening_params_.th_roughening_;*/

  // Draw the same number of particles as we had before
  for (int i = 0; i < samples_.size(); i++) {
    // Update the weight we are currently checking
    curr_weight = sample_d * 0.5 + sample_d * i;

    // Find the particle that corresponds to the weight we have
    for (int s = prev_sample_ind; s < samples_.size(); s++) {
      if ((s + 1) == samples_.size()) {
        prev_sample_ind = s;
        new_particles.push_back(samples_.at(s + 1));
        break;
      }
      else {
        if (samples_.at(s + 1)->getWeight() > curr_weight) {
          prev_sample_ind = s;

          Particle p = samples_.at(s + 1);

          // Add roughening
          /*dx = MathUtils::randNumInRange(-xy_range, xy_range);
          dy = MathUtils::randNumInRange(-xy_range, xy_range);
          dtheta = MathUtils::randNumInRange(-th_range, th_range);
          p.addRoughening(dx, dy, dtheta);*/

          new_particles.push_back(p);
          break;
        }
      }
    }
  }

  // Clear the particles
  samples_.clear();

  return new_particles;
}

RouletteWheelResampling::RouletteWheelResampling(RougheningParams params) {
  // Set the roughening parameters
  roughening_params_ = params;
}

RouletteWheelResampling::~RouletteWheelResampling() {
}

std::vector<Particle> RouletteWheelResampling::resample() {
  std::vector<Particle> new_particles;

  // Get the total weight for the range within which we'll draw random number
  double total_weight = samples_.back()->getWeight();

  // Current weight
  double curr_weight;

  // Roughening samples
  /*double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
  double xy_range = roughening_params_.xy_roughening_;
  double th_range = roughening_params_.th_roughening_;*/

  // Draw the same number of particles as we had before
  for (int i = 0; i < samples_.size(); i++) {
    // Pick a random number between 0 and the total weight
    curr_weight = MathUtils::randNumInRange(0.0, total_weight);

    // Find the particle that corresponds to the weight we have. The
    // particle has to have smaller weight than the weight we are checking,
    // and the particle immediately after it has to have a larger weight.
    int ind = 0;
    while (samples_.at(ind)->getWeight() < curr_weight) {
      // If we have reached the last particle we stop, otherwise check
      // the next one
      if (ind == (samples_.size() - 1)) {
        ind++;
        break;
      }
      else
        ind++;
    }

    Particle p = samples_.at(ind--);

    // Add roughening
    /*dx = MathUtils::randNumInRange(-xy_range, xy_range);
    dy = MathUtils::randNumInRange(-xy_range, xy_range);
    dtheta = MathUtils::randNumInRange(-th_range, th_range);
    p.addRoughening(dx, dy, dtheta);*/

    new_particles.push_back(p);
  }

  // Clear the particles
  samples_.clear();

  return new_particles;
}
