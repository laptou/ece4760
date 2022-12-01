#ifndef _NOTES_H
#define _NOTES_H

#include <string>
#include "fpmath/fpmath.h"

enum class note
{
  A,
  B,
  C,
  D,
  E,
  F,
  G,
};

typedef struct absolute_note
{
  fixed freq;
  int octave;
  note value;
  bool sharp;

  const std::string str() const &
  {
    if (this == NULL)
    {
      return "N/A";
    }

    std::string tmp;

    switch (this->value)
    {
    case note::A:
      tmp += 'A';
      break;
    case note::B:
      tmp += 'B';
      break;
    case note::C:
      tmp += 'C';
      break;
    case note::D:
      tmp += 'D';
      break;
    case note::E:
      tmp += 'E';
      break;
    case note::F:
      tmp += 'F';
      break;
    case note::G:
      tmp += 'G';
      break;
    }

    if (this->sharp)
    {
      tmp += '#';
    }

    tmp += std::to_string(this->octave);

    return tmp;
  }
} absolute_note;

// Returns the closest absolute_note to freq between lhs and rhs.
const absolute_note &get_closest_note(const absolute_note &lhs, const absolute_note &rhs, fixed freq);

// Returns element closest to target in arr[]
const absolute_note &find_closest_note(fixed freq);
#endif