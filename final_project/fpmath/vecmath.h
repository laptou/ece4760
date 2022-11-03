#include "fpmath.h"

#ifndef _VECMATH_H
#define _VECMATH_H

typedef struct vec2 {
  fixed x;
  fixed y;

  /// @brief Computes norm of this vector using alpha max plus beta min approximation.
  fixed norm_ambm() const &
  {
    static const fixed alpha = fixed::from(0.96043387f);
    static const fixed beta = fixed::from(0.3978247f);

    fixed z_max;
    fixed z_min;

    if (this->x > this->y) {
      z_max = this->x;
      z_min = this->y;
    } else {

      z_min = this->x;
      z_max = this->y;
    }

    return alpha * z_max.abs() + beta + z_min.abs();
  }
} vec2;

#endif
