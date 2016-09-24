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

void ParticleFilter::extract_particle_local_scan(boost::shared_ptr<WheelBot>& particle, std::vector<float>& scanRanges)
{

  scanRanges.clear();
  int n = ( (std::abs(laserScanParams_.angle_max - laserScanParams_.angle_min) )/laserScanParams_.angle_increment ) ;
  //set up box to search
  int left_wall  = (particle->getX() - laserScanParams_.range_max - mapParams_.origin_x) / mapParams_.resolution;
  int right_wall = (particle->getX() + laserScanParams_.range_max - mapParams_.origin_x) / mapParams_.resolution;
  int up_wall    = (particle->getY() + laserScanParams_.range_max - mapParams_.origin_y) / mapParams_.resolution ;
  int down_wall  = (particle->getY() - laserScanParams_.range_max - mapParams_.origin_y) / mapParams_.resolution;

  //clip left
  left_wall = left_wall < 0 ? 0 : left_wall; right_wall = right_wall < 0 ? 0 : right_wall;
  //clip right
  right_wall = right_wall > mapParams_.width ? mapParams_.width : right_wall;
  left_wall = left_wall > mapParams_.width ? mapParams_.width : left_wall;
  //clip top
  up_wall = up_wall > mapParams_.height ? mapParams_.height : up_wall;
  down_wall = down_wall > mapParams_.height ? mapParams_.height : down_wall;
  //clip bottom
  up_wall = up_wall < 0 ? 0 : up_wall; down_wall = down_wall < 0 ? 0 : down_wall;

  for (int i = down_wall; i < up_wall; i++)
  {
    for (int j = left_wall; j < right_wall; j++)
    {
      if (map_data_[i * mapParams_.width + j] > 0) {
        std::cout << "Occupied" << std::endl;
      } else {
        std::cout << "Not Occupied" << std::endl;
      }
    }
  }



  for (int i = 0; i < n; i++) scanRanges.push_back( 1.0f );

}

void ParticleFilter::initializeLaserScanParameters(float angle_min, float angle_max,
                                                   float angle_increment, float range_min,
                                                   float range_max)
{
  ParticleLaserScanParams params;
  params.angle_min       = angle_min;
  params.angle_max       = angle_max;
  params.angle_increment = angle_increment;
  params.range_min       = range_min;
  params.range_max       = range_max;

  laserScanParams_ = params;
}

void ParticleFilter::initializeMapParameters(float resolution,
                             int width, int height,
                             float origin_x, float origin_y, float origin_theta)
{
  mapParams_.resolution = resolution;
  mapParams_.width = width;
  mapParams_.height = height;
  mapParams_.origin_x = origin_x;
  mapParams_.origin_y = origin_y;
  mapParams_.origin_theta = origin_theta;
}

void ParticleFilter::setMap(std::vector<int>& map)
{
  map_data_ = map;
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

