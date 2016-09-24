#pragma once
#include <vector>
#include <wheel_bot.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>

struct ParticleLaserScanParams {
  float angle_min, angle_max, angle_increment, range_min, range_max;
};

struct MapParams {
  float resolution;
  int width, height;
  float origin_x, origin_y, origin_theta;
};

class ParticleFilter{
public:
  ParticleFilter();
  ParticleFilter(int nParticles);
  void setNParticles(int nParticles);
  void initializeLaserScanParameters(float angle_min, float angle_max,
                                     float angle_increment, float range_min,
                                     float range_max);
  void initializeMapParameters(float resolution,
                               int width, int height,
                               float origin_x, float origin_y, float origin_theta);
  void setMap(std::vector<int>& map);
  void spawnParticles();
  void spawnParticles(WheelBot pose);
  void clipToMap(int &x, int &y);
  void show(int n);
  void extract_particle_local_scan(boost::shared_ptr<WheelBot>& particle, std::vector<float>& scanRanges);
  boost::shared_ptr<WheelBot> getParticle(int i);

  void propagate(float delta_x, float delta_y);

  geometry_msgs::PoseArray particlesToMarkers();


  ParticleLaserScanParams getLaserScanParams() {return laserScanParams_;}

  // Get the weight of a particle due to the correlation of scan scene and map
  void GetParticleWeights(const sensor_msgs::LaserScanConstPtr& a_scan_msg);

  // Convert the sensor measurement to points in the global map (Get the laser scan points in the global frame of one particle)
  void ConvertSensorMeasurementToPoints(boost::shared_ptr<WheelBot>& particle,
                                        const sensor_msgs::LaserScanConstPtr& a_scan_msg,
                                        std::vector<geometry_msgs::Point>& a_points);

  // Get the correlation of the particle and the map
  int CorrelationParticleMap(const std::vector<geometry_msgs::Point>& a_points);

  // Clean particles outside of the map and set weight diminishing small
  void CleanWeightOfParticle(boost::shared_ptr<WheelBot>& particle,
                             float& particle_weight);


private:
  void propagate();
  void perturb();
  void resample();

private:
  int nParticles_;
  std::vector<boost::shared_ptr<WheelBot>> particles_;

  ParticleLaserScanParams laserScanParams_;
  MapParams mapParams_;
  std::vector<int> map_data_;
};

struct ParticleVisualProperties{
  float length,width,height;
};
