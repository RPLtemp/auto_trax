#pragma once
#include <vector>
#include <include/wheel_bot.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>


class ParticleFilter{

public:
  ParticleFilter();
  ParticleFilter(int nParticles);
  void setNParticles(int nParticles);
  void spawnParticles();
  void show(int n);
  boost::shared_ptr<WheelBot> getParticle(int i);

private:

  void propagate();
  void perturb();
  void resample();

private:
  int nParticles_;
  std::vector<boost::shared_ptr<WheelBot>> particles_;

};

struct ParticleVisualProperties{
  float length,width,height;
};