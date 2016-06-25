//
// Created by marius on 16.04.16.
//

#ifndef AUTO_TRAX_PID_PID_BAG_H
#define AUTO_TRAX_PID_PID_BAG_H

#include <string>

struct PidBag
{
  float Kp, Ki, Kd;
  float upper_limit, lower_limit;
  float windup_limit;
  float cutoff_frequency;
};

#endif //AUTO_TRAX_PID_PID_BAG_H
