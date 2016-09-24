#pragma once
#include <vector>
#include <include/wheel_bot.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include <geometry_msgs/PoseArray.h>

class ParticleFilter{

public:
  ParticleFilter();
  ParticleFilter(int nParticles);
  void setNParticles(int nParticles);
  void spawnParticles();
  void show(int n);
  boost::shared_ptr<WheelBot> getParticle(int i);

  void propagate(float delta_x, float delta_y);

  geometry_msgs::PoseArray particlesToMarkers();

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
