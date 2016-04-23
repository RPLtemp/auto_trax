//
// Created by marius on 16.04.16.
//

#ifndef MBZIRC_TASK3_DETECTION_PARAMETER_BAG_H
#define MBZIRC_TASK3_DETECTION_PARAMETER_BAG_H

#include <string>

struct ParameterBag
{
  // Parameter specific to parameter bag
  std::string node_name;

  std::string subscribed_rostopic_img;
  int queue_size_subscriber_img;

  std::string subscribed_rostopic_imu;
  int queue_size_subscriber_imu;

  std::string subscribed_rostopic_calib;
  int queue_size_subscriber_calib;

  std::string subscribed_rostopic_cam;
  int queue_size_subscriber_cam;

  std::string image_path;
};

#endif //MBZIRC_TASK3_DETECTION_PARAMETER_BAG_H
