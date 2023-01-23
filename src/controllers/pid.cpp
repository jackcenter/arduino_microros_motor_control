#include "pid.h"

#include <cmath>

namespace controllers
{
PID::PID()
: options_{ options }
{}

Input PID::cycle(State& x_ref, State& x_hat, double update_time)
{
  update(x_ref, x_hat, update_time);
  return read();
}

PIDValue PID::getInputComponents() const
{
  return input_components_;
}

Input PID::read() const
{
  return input_components_.p + input_components_.i + input_components_.d;
}

void PID::update(State& x_ref, State& x_hat, Seconds update_time)
{
  if (std::isnan(update_time_))
  {
    update_time_ = update_time;
    return;
  }

  const double cycle_rate = 1 / (update_time - update_time_);
  update_time_ = update_time;

  const State error_previous = error_components_.p;

  error_components_.p = x_ref - x_hat;
  // TODO: if not winding up and not within the deadband
  error_components_.i += error_components_.p;
  // TODO: use a low pass filter
  error_components_.d = (error_components_.p - error_previous) * cycle_rate;

  input_components_ = options_.gains * error_components_;

  // TODO: check for windup
  // TODO: return abseil status, or optional.
}
}