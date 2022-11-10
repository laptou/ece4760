#include "fpmath.h"

#ifndef _VECMATH_H
#define _VECMATH_H

typedef struct vec2
{
  fixed x;
  fixed y;

  /// @brief Computes norm of this vector using alpha max plus beta min approximation.
  fixed norm_ambm() const &
  {
    static const fixed alpha = fixed::from(0.96043387f);
    static const fixed beta = fixed::from(0.3978247f);

    fixed z_max;
    fixed z_min;

    fixed z_1 = this->x.abs();
    fixed z_2 = this->y.abs();

    if (z_1 > z_2)
    {
      z_max = z_1;
      z_min = z_2;
    }
    else
    {
      z_max = z_2;
      z_min = z_1;
    }

    return alpha * z_max + beta * z_min;
  }
} vec2;

#endif
