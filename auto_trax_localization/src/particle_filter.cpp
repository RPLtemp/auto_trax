#include "particle_filter.hpp"

ParticleFilter::ParticleFilter()
{


}

ParticleFilter::ParticleFilter(int nParticles) : nParticles_(nParticles)
{


}

void ParticleFilter::setNParticles(int nParticles)
{
  nParticles_ = nParticles;
}

void ParticleFilter::spawnParticles()
{
  particles_.reserve(nParticles_);
  for (int i = 0; i < nParticles_; i++)
  {
    particles_.push_back(boost::shared_ptr<WheelBot>(new WheelBot()));
  }
}

void ParticleFilter::show(int n)
{
  if (n >= nParticles_)
  {
    for (auto particle : particles_)
    {
      std::cout << particle->getX() << '\t';
      std::cout << particle->getY() << '\t';
      std::cout << particle->getTheta() << '\t';
      std::cout << '\n';
    }
  } else {
    for (int i = 0; i < n; i++)
    {
      std::cout << particles_.at(i)->getX() << '\t';
      std::cout << particles_.at(i)->getY() << '\t';
      std::cout << particles_.at(i)->getTheta() << '\t';
      std::cout << '\n';
    }
  }
}

boost::shared_ptr<WheelBot> ParticleFilter::getParticle(int i)
{
  if(i < nParticles_) return particles_.at(i);
  return nullptr;
}

void ParticleFilter::propagate(float delta_x, float delta_y)
{
  for (int i = 0; i < nParticles_; i++)
  {
    particles_.at(i)->addX(delta_x);
    particles_.at(i)->addY(-delta_y);
  }
}

geometry_msgs::PoseArray ParticleFilter::particlesToMarkers() {
  // Create an array of poses message from particle values
  geometry_msgs::PoseArray poses;

  // Particle heading
  double theta;

  for (int i = 0; i < nParticles_; i++) {
    // Create a pose
    geometry_msgs::Pose pose;
    pose.position.x = particles_.at(i)->getX();
    pose.position.y = particles_.at(i)->getY();
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = sin(0.5 * -0.5 * M_PI);
    pose.orientation.w = cos(0.5 * -0.5 * M_PI);

    // Add the pose to the array
    poses.poses.push_back(pose);
  }

  poses.header.frame_id = "map";
  poses.header.seq = 0;

  return poses;
}
