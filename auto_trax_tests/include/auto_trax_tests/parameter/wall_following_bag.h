//
// Created by marius on 16.04.16.
//

#ifndef AUTO_TRAX_TESTS_WALL_FOLLOWING_BAG_H
#define AUTO_TRAX_TESTS_WALL_FOLLOWING_BAG_H

#include <string>

struct WallFollowingTestBag
{
  // Parameter specific to parameter bag
  std::string node_name;

  std::string subscribed_rostopic_scan;
  int queue_size_subscriber_scan;

  std::string pub_rostopic_dist;
  int queue_size_pub_dist;
};

#endif //AUTO_TRAX_TESTS_WALL_FOLLOWING_BAG_H
