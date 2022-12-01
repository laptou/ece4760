#ifndef _NOTES_H
#define _NOTES_H

#include <string>
#include "fpmath/fpmath.h"

typedef struct
{
  fixed freq;
  std::string name;
} note;

// Returns the closest note to freq between lhs and rhs.
const note &get_closest_note(const note &lhs, const note &rhs, fixed freq);

// Returns element closest to target in arr[]
const note &find_closest_note(fixed freq);
#endif