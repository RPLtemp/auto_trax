#include "particle_filter.hpp"

ParticleFilter::ParticleFilter()
{
  max_weight_particle_ = boost::shared_ptr<WheelBot> (new WheelBot());
  Resampling::RougheningParams roughening_params;
  roughening_params.th_roughening_ = 0.0;
  roughening_params.xy_roughening_ = 0.0;

  resampler_ = boost::make_shared<StochasticUniversalResampling>(roughening_params);
}

ParticleFilter::ParticleFilter(int nParticles) : nParticles_(nParticles)
{
}

ParticleFilter::~ParticleFilter()
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
                                                         0.1, 0.1, 0.1 )));
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
////        std::cout << "Occupied" << std::endl;
//        float obstacle_x = mapParams_.origin_x + i * mapParams_.resolution - particle->getX();
//        float obstacle_y = mapParams_.origin_y + j * mapParams_.resolution - particle->getY();
//        float obstacle_theta = atan(obstacle_y/ obstacle_x);
////        Eigen::Quaternionf q(2, 0, 1, -3);
//        Eigen::Quaternionf obstacle_quaternion(1.0,0.0,0.0,0.0);
//
//        Eigen::Quaternionf particle_quaternion(0.0,0.0,0.0,1.0);
//
//        Eigen::Quaternionf  obstacle_scan_quaternion = particle_quaternion * obstacle_quaternion.inverse();
//
//        float angle_to_turn = 2*acos(obstacle_scan_quaternion.w());
//        angle_to_turn = atan2(sin(angle_to_turn), cos(angle_to_turn));
////        std::cout << "Angle to turn: " << angle_to_turn << std::endl;
//
//
//      } else {
////        std::cout << "Not Occupied" << std::endl;
//      }
//    }
//  }

  scanRanges.clear();
  int n = laserScanParams_.ranges_size;




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

    for (float distance = laserScanParams_.range_min; distance < laserScanParams_.range_max; distance += mapParams_.resolution)
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

void ParticleFilter::setLaserScanParameters(float angle_min, float angle_max,
                                                   float angle_increment, float range_min,
                                                   float range_max, int ranges_size)
{
  ParticleLaserScanParams params;
  params.angle_min       = angle_min;
  params.angle_max       = angle_max;
  params.angle_increment = angle_increment;
  params.range_min       = range_min;
  params.range_max       = range_max;
  params.ranges_size       = ranges_size;


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
    particles_.at(i)->addX(delta_y * cos(particles_.at(i)->getTheta()));
    particles_.at(i)->addY(delta_y * sin(particles_.at(i)->getTheta()));
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
    pose.orientation.z = sin(particles_.at(i)->getTheta()/2);
    pose.orientation.w = cos(particles_.at(i)->getTheta()/2);

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
  double max_weight = std::numeric_limits<double>::min();
  float min_observation_error = std::numeric_limits<float>::max();

  for (int i = 0; i < particles_.size(); ++i) {
    std::vector<geometry_msgs::Point> points;

    // Convert sensor measurement to points in global map
    WheelBot particle = *(particles_.at(i));

    int correlation;
    if (isObstacle(particle.getX(), particle.getY()))
    {
      correlation = 0;
    } else {
      ConvertSensorMeasurementToPoints(particle, scan_msg, points);
      // Get correlation value of particle and map
      correlation = CorrelationParticleMap(points);
    }


    // get particle local view
    std::vector<float> ranges;
    extract_particle_local_scan(particles_.at(i), ranges);
//     call agreement between particle view and laser scan measurement if a threshold of 5cm is met
    int modelObsCorrelation = 0;
    float modelObsError = 0.0;
    for (int range_point = 0; range_point < ranges.size(); range_point++)
    {
//      if (!isnan(ranges[range_point]) && !isnan(scan_msg->ranges[range_point]))
//      {
//        modelObsError += (ranges[range_point] - scan_msg->ranges[range_point]) *
//                         (ranges[range_point] - scan_msg->ranges[range_point]);
//      }
      if ( abs(ranges[range_point] - scan_msg->ranges[range_point]) < 0.10
           && !isnan(ranges[range_point])
           && !isnan(scan_msg->ranges[range_point]) )
      {
        modelObsCorrelation++;
      }
    }
//    double particle_weight = static_cast<double>( modelObsError <= 0.0 ? std::numeric_limits<float>::quiet_NaN() : -log(modelObsError));
//    particle_weight = particle_weight < 0.0 ? 0.0 : particle_weight;

    double particle_weight = static_cast<double>(modelObsCorrelation + correlation);

//    std::cout << "particle at " << i << " modelobserror: " << modelObsError <<std::endl;
//    std::cout << "particleWeight: " <<  particle_weight <<std::endl;


//    if (modelObsError < min_observation_error && !isnan(particle_weight) )
//    {
//      min_observation_error = modelObsError;
//      *max_weight_particle_ = *particles_.at(i);
//    }

    // Keep the particle with the highest weight
    if (particle_weight > max_weight) {
      max_weight = particle_weight;
      *max_weight_particle_ = *particles_.at(i);
    }

    particles_.at(i)->setWeight(particle_weight);
    resampler_->addParticle(particles_.at(i));

    // Clean the particle weights such that particles are highly unlikely to be outside of the map
    CleanWeightOfParticle(particle, particle_weight);
  }

}

void ParticleFilter::ConvertSensorMeasurementToPoints(
WheelBot particle,
const sensor_msgs::LaserScanConstPtr& scan_msg,
std::vector<geometry_msgs::Point>& points) {
for (int i = 0; i < scan_msg->ranges.size(); ++i) {
// Range and angle of laser scan point in local camera frame
float range = scan_msg->ranges.at(i);
float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

// If range is within [range_min range_max] of the scan
if (range > scan_msg->range_min && range < scan_msg->range_max) {
  geometry_msgs::Point point_scan, translation, rotation, pos;

  // Calculate polar rotation of the scan point in camera frame
  point_scan.x = range * cosf(angle);
  point_scan.y = range * sinf(angle);
  point_scan.z = 0.0;

  // Calculate rotational part
  rotation.x = point_scan.x * cosf(particle.getTheta()) - point_scan.y * sinf(particle.getTheta());
  rotation.y = point_scan.x * sinf(particle.getTheta()) + point_scan.y * cosf(particle.getTheta());
  rotation.z = 0.0;

  // Calculate translational part
  translation.x = particle.getX();
  translation.y = particle.getY();
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
    if ( isObstacle(points.at(i).x,points.at(i).y) )
    {
      correlation += 1;
    }
//
//  int ind_j = round((points.at(i).x - mapParams_.origin_x)/mapParams_.resolution);
//int ind_i = round((points.at(i).y - mapParams_.origin_y)/mapParams_.resolution);
//
//
//int map_index = ind_i * mapParams_.width + ind_j;
//
//if (ind_i > 0 && ind_i < mapParams_.height && ind_j > 0 && ind_j < mapParams_.width ) {
//  if (map_data_.at(map_index) > 0) {
//    correlation += 1;
//  }
  }
  return correlation;

}


void ParticleFilter::CleanWeightOfParticle(WheelBot particle,
                                       double& particle_weight) {
int ind_i = int(particle.getX()/mapParams_.resolution);
int ind_j = int(particle.getY()/mapParams_.resolution);

int map_index = ind_i + ind_j*mapParams_.width;

if (particle.getX() > mapParams_.width * mapParams_.resolution || particle.getX() < 0.0 ||
  particle.getY() > mapParams_.height * mapParams_.resolution || particle.getY() < 0.0 ||
  map_data_.at(map_index) == 100 || particle_weight == 0) {
// Assign low correlation/ particle weight
particle_weight = 0.0001;
}
}

boost::shared_ptr<WheelBot> ParticleFilter::Resample() {
particles_ = resampler_->resample();

return max_weight_particle_;
}

//Takes in a point in world coordinates
bool ParticleFilter::isObstacle(float x, float y)
{
  bool result = false;
  int ind_j = round(( x - mapParams_.origin_x)/mapParams_.resolution);
  int ind_i = round(( y - mapParams_.origin_y)/mapParams_.resolution);
  int map_index = ind_i + ind_j*mapParams_.width;
  if (ind_i > 0 && ind_i < mapParams_.height && ind_j > 0 && ind_j < mapParams_.width ) {
    if (map_data_.at(map_index) > 0) {
      result = true;
      return result;
    }
  }
}


