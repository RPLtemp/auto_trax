//
// Created by frank on 10.07.16.
//

#ifndef AUTO_TRAX_PATH_PLANNING_OC_GRID_PARAMETERS_H
#define AUTO_TRAX_PATH_PLANNING_OC_GRID_PARAMETERS_H

#include <string>

struct OCGridParameterBag
{
  std::string node_name;

  std::string subscribed_rostopic_scan_summary;
  int queue_size_subscriber_scan_summary;

  std::string published_rostopic_oc_grid;
  int queue_size_pub_oc_grid;

};

#endif //AUTO_TRAX_PATH_PLANNING_OC_GRID_PARAMETERS_H
