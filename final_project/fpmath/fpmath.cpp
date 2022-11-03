#include "fpmath.h"
#include <stdint.h>

struct fixed final {
  signed int value;

  static fixed from(float f)
  {
    return { (signed int)(f * 32768.0) };
  }

  static fixed from(signed int i)
  {
    return { (i << 15) };
  }

  constexpr fixed operator*(const fixed &rhs) const
  {
    auto lhs_v = (signed long long)(this->value);
    auto rhs_v = (signed long long)(rhs.value);
    return {(signed int)((lhs_v * rhs_v) >> 15)};
  }

  constexpr fixed operator/(const fixed &rhs) const
  {
    auto lhs_v = (signed long long)(this->value);
    auto rhs_v = (signed long long)(rhs.value);
    return {(signed int)((lhs_v << 15) / rhs_v)};
  }

  constexpr operator float() const { return (float)this->value / 32768.0; }

  constexpr operator int() const { return (int)(this->value >> 15); }
};
