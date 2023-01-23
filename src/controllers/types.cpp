#include "types.h"

namespace controllers
{
PIDValue PIDValue::operator * (const PIDValue& rhs) const
{
  return PIDValue{ p * rhs.p, i * rhs.i, d * rhs.d };
}
}
