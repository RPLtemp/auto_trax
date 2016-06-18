//
// Created by marius on 16.04.16.
//

#ifndef CATKINPKG_FRAMEWORK_PARAMETER_BAG_H
#define CATKINPKG_FRAMEWORK_PARAMETER_BAG_H

#include <string>

struct ParameterBag
{
  // Parameter specific to parameter bag
  std::string node_name;

  std::string subscribed_rostopic_scan;
  int queue_size_subscriber_scan;

  std::string pub_rostopic_dist;
  int queue_size_pub_dist;
};

#endif //CATKINPKG_FRAMEWORK_PARAMETER_BAG_H
