//
// Created by marius on 12.06.16.
//

#ifndef AUTO_TRAX_CONTROLLER_PID_H
#define AUTO_TRAX_CONTROLLER_PID_H

#include "auto_trax_controller/parameter/pid_bag.h"

namespace auto_trax {

class PID {
 public:
  PID(PidBag pid_param);
  virtual ~PID();

  double GetControlEffort();

 private:
  PidBag pid_param_;
};

} // namespace auto_trax

#endif //AUTO_TRAX_CONTROLLER_PID_H
