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

void ParticleFilter::spawnParticles(WheelBot pose)
{
  particles_.reserve(nParticles_);
  for (int i = 0; i < nParticles_; i++)
  {
    particles_.push_back(boost::shared_ptr<WheelBot>(new WheelBot(pose.getX(),pose.getY(),pose.getTheta(),
                                                         .001, .001, 0.5 )));
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
  int n = ( (std::abs(laserScanParams_.angle_max - laserScanParams_.angle_min) )/laserScanParams_.angle_increment) ;
  //set up box to search
//  int left_wall  = (particle->getX() - laserScanParams_.range_max - mapParams_.origin_x) / mapParams_.resolution;
//  int right_wall = (particle->getX() + laserScanParams_.range_max - mapParams_.origin_x) / mapParams_.resolution;
//  int up_wall    = (particle->getY() + laserScanParams_.range_max - mapParams_.origin_y) / mapParams_.resolution ;
//  int down_wall  = (particle->getY() - laserScanParams_.range_max - mapParams_.origin_y) / mapParams_.resolution;
//
//  //clip left
//  left_wall = left_wall < 0 ? 0 : left_wall; right_wall = right_wall < 0 ? 0 : right_wall;
//  //clip right
//  right_wall = right_wall > mapParams_.width ? mapParams_.width : right_wall;
//  left_wall = left_wall > mapParams_.width ? mapParams_.width : left_wall;
//  //clip top
//  up_wall = up_wall > mapParams_.height ? mapParams_.height : up_wall;
//  down_wall = down_wall > mapParams_.height ? mapParams_.height : down_wall;
//  //clip bottom
//  up_wall = up_wall < 0 ? 0 : up_wall; down_wall = down_wall < 0 ? 0 : down_wall;
//
//  for (int i = down_wall; i < up_wall; i++)
//  {
//    for (int j = left_wall; j < right_wall; j++)
//    {
//      if (map_data_[i * mapParams_.width + j] > 0) {
//        std::cout << "Occupied" << std::endl;
//        float obstacle_x = mapParams_.origin_x + i * mapParams_.resolution - particle->getX();
//        float obstacle_y = mapParams_.origin_y + j * mapParams_.resolution - particle->getY();
//        float obstacle_theta = atan2(obstacle_y, obstacle_x);
////        Eigen::Quaternionf q(2, 0, 1, -3);
//        Eigen::Quaternionf obstacle_quaternion(cos(obstacle_theta/2),0.0,0.0,sin(obstacle_theta/2));
//
//        Eigen::Quaternionf particle_quaternion(cos(particle->getTheta()/2),0.0,0.0, sin(particle->getTheta()/2));
//
//        Eigen::Quaternionf  obstacle_scan_quaternion = obstacle_quaternion * particle_quaternion.inverse();
//
//        std::cout <<"Angle to turn is: " << 2*acos(obstacle_scan_quaternion.w()) <<std::endl;
//
//
//      } else {
////        std::cout << "Not Occupied" << std::endl;
//      }
//    }
//  }


  bool isInAWall = false;

  float x = particle->getX();
  float y = particle->getY();
  int x_coord = round( (x - mapParams_.origin_x) / mapParams_.resolution);
  int y_coord = round( (y - mapParams_.origin_y) / mapParams_.resolution);
  clipToMap(x_coord,y_coord);

  if (map_data_[y_coord * mapParams_.width + x_coord] > 0)
  {
    isInAWall = true;
  }


  if (isInAWall)
  {
    for (int i = 0; i < n; i++) {
      scanRanges.push_back(std::numeric_limits<float>::quiet_NaN());
    }
    return;
  }

  for (int i = 0; i < n; i++)
  {

    bool obstacle_is_set = false;

    for (float distance = laserScanParams_.range_min; distance < laserScanParams_.range_max; distance += mapParams_.resolution/2)
    {
      if (obstacle_is_set) {continue;}
      float x = particle->getX() + distance * cos( particle->getTheta() + laserScanParams_.angle_min + i * laserScanParams_.angle_increment);
      float y = particle->getY() + distance * sin( particle->getTheta() + laserScanParams_.angle_min + i * laserScanParams_.angle_increment);
      int x_coord = round( (x - mapParams_.origin_x) / mapParams_.resolution);
      int y_coord = round( (y - mapParams_.origin_y) / mapParams_.resolution);
      clipToMap(x_coord,y_coord);


      if (map_data_[y_coord * mapParams_.width + x_coord] > 0)
      {
        scanRanges.push_back(distance);
        obstacle_is_set = true;
      }

    }

    if (!obstacle_is_set)
    {
      scanRanges.push_back(std::numeric_limits<float>::quiet_NaN());
    }

  }


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


void ParticleFilter::clipToMap(int &x, int &y)
{
    //clip left
  x = x < 0 ? 0 : x;
  //clip right
  x = x > mapParams_.width ? mapParams_.width : x;
  //clip top
  y = y > mapParams_.height ? mapParams_.height : y;
  //clip bottom
  y = y < 0 ? 0 : y;
}


void ParticleFilter::GetParticleWeights(const sensor_msgs::LaserScanConstPtr& scan_msg) {
  std::vector<geometry_msgs::Point> points;

  for (int i = 0; i < particles_.size(); ++i) {
    // Convert sensor measurement to points in global map
    ConvertSensorMeasurementToPoints(particles_.at(i), scan_msg, points);

    // Get correlation value of particle and map
    int correlation = CorrelationParticleMap(points);

    float particle_weight = static_cast<float>(correlation);

    // Clean the particle weights such that particles are highly unlikely to be outside of the map
    CleanWeightOfParticle(particles_.at(i), particle_weight);
  }
}

void ParticleFilter::ConvertSensorMeasurementToPoints(
    boost::shared_ptr<WheelBot>& particle,
    const sensor_msgs::LaserScanConstPtr& scan_msg,
    std::vector<geometry_msgs::Point>& points) {
  for (int i = 0; i < scan_msg->ranges.size(); ++i) {
    // Range and angle of laser scan point in local camera frame
    float range = scan_msg->ranges.at(i);
    float angle = scan_msg->angle_min + i*scan_msg->angle_increment;

    // If range is within [range_min range_max] of the scan
    if (range > scan_msg->range_min && range < scan_msg->range_max) {
      geometry_msgs::Point point_scan, translation, rotation, pos;

      // Calculate polar rotation of the scan point in camera frame
      point_scan.x = range * cosf(angle);
      point_scan.y = range * sinf(angle);
      point_scan.z = 0.0;

      // Calculate rotational part
      rotation.x = point_scan.x * cosf(particle->getTheta()) - point_scan.y * sinf(particle->getTheta());
      rotation.y = point_scan.x * sinf(particle->getTheta()) + point_scan.y * cosf(particle->getTheta());
      rotation.z = 0.0;

      // Calculate translational part
      translation.x = particle->getX();
      translation.y = particle->getY();
      translation.z = 0.0;

      // Get final coordinates of point in global map
      pos.x = rotation.x + translation.x;
      pos.y = rotation.y + translation.y;
      pos.z = 0.0;

      points.push_back(pos);
    }
  }
}

int ParticleFilter::CorrelationParticleMap(const std::vector<geometry_msgs::Point>& points) {
  int correlation = 0;

  for (int i = 0; i < points.size(); ++i) {
    int ind_i = int(points.at(i).x/mapParams_.resolution);
    int ind_j = int(points.at(i).y/mapParams_.resolution);

    int map_index = ind_i + ind_j*mapParams_.width;

    if (ind_i > 0 && ind_i < mapParams_.height && ind_j > 0 && ind_j < mapParams_.width ) {
      if (map_data_.at(map_index) == 100) {
        correlation += 1;
      }
    }
  }

  return correlation;
}

void ParticleFilter::CleanWeightOfParticle(boost::shared_ptr<WheelBot>& particle,
                                           float& particle_weight) {
  int ind_i = int(particle->getX()/mapParams_.resolution);
  int ind_j = int(particle->getY()/mapParams_.resolution);

  int map_index = ind_i + ind_j*mapParams_.width;

  if (particle->getX() > mapParams_.width*mapParams_.resolution || particle->getX() < 0.0 ||
      particle->getY() > mapParams_.height*mapParams_.resolution || particle->getY() < 0.0 ||
      map_data_.at(map_index) == 100 || particle_weight == 0) {
    // Assign low correlation/ particle weight
    particle_weight = 0.0001;
  }
}

