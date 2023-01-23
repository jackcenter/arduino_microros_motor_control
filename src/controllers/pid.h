#ifndef EMB_MOTOR_CONTROLLER_PID_H
#define EMB_MOTOR_CONTROLLER_PID_H

#include <limits>
#include <utility>

#include "types.h"

namespace motor_controller
{
namespace
{
  using DoubleRange = std::pair<double,double>;
  using State = double;
  using Input = double;
  using Seconds = double;
}

struct PIDOpitons
{
  PIDValue gains{ 0.0, 0.0, 0.0 };
  DoubleRange input_range{ 0.0, 0.0 };
  DoubleRange deadband_range{ 0.0, 0.0 };
};

class PID
{
public:
  PID(const PIDOpitons& options);

  Input cycle(State& x_ref, State& x_hat, Seconds update_time);

  PIDValue getInputComponents() const;

protected:
  Input read() const;
  void update(State& x_ref, State& x_hat, Seconds update_time);

private:
  PIDOpitons options_;
  PIDValue input_components_{ 0.0, 0.0, 0.0 };
  PIDValue error_components_{ 0.0, 0.0, 0.0 };
  std::numeric_limits<Seconds>::quiet_NaN() update_time_{};
};
}

#endif  // EMB_MOTOR_CONTROLLER_PID_H