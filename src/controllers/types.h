#ifndef EMB_MOTOR_CONTROLLER_TYPES_H
#define EMB_MOTOR_CONTROLLER_TYPES_H

#include <utility>

namespace motor_controller
{
struct PIDValue
{
  double p{ 0.0 };
  double i{ 0.0 };
  double d{ 0.0 };

  PIDValue operator * (const PIDValue& rhs) const;
};
}

#endif  // EMB_MOTOR_CONTROLLER_TYPES_H
