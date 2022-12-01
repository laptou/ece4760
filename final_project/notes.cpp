#include "notes.h"

absolute_note notelist[] = {
    {fixed::from(493), 4, note::B, false},
    {fixed::from(523), 5, note::C, false},
    {fixed::from(554), 5, note::C, true},
    {fixed::from(587), 5, note::D, false},
    {fixed::from(622), 5, note::D, true},
    {fixed::from(659), 5, note::E, false},
    {fixed::from(698), 5, note::F, false},
    {fixed::from(739), 5, note::F, true},
    {fixed::from(783), 5, note::G, false},
    {fixed::from(830), 5, note::G, true},
    {fixed::from(880), 5, note::A, false},
    {fixed::from(932), 5, note::A, true},
    {fixed::from(987), 5, note::B, false},
    {fixed::from(1046), 6, note::C, false},
    {fixed::from(1108), 6, note::C, true},
    {fixed::from(1174), 6, note::D, false},
    {fixed::from(1244), 6, note::D, true},
    {fixed::from(1318), 6, note::E, false},
    {fixed::from(1396), 6, note::F, false},
    {fixed::from(1479), 6, note::F, true},
    {fixed::from(1567), 6, note::G, false},
    {fixed::from(1661), 6, note::G, true},
    {fixed::from(1760), 6, note::A, false},
    {fixed::from(1864), 6, note::A, true},
    {fixed::from(1975), 6, note::B, false},
    {fixed::from(2093), 7, note::C, false},
    {fixed::from(2217), 7, note::C, true}};

// Returns the closest absolute_note to freq between lhs and rhs.
const absolute_note &get_closest_note(const absolute_note &lhs, const absolute_note &rhs, fixed freq)
{
  if (freq - lhs.freq >= rhs.freq - freq)
    return rhs;
  else
    return lhs;
}

// Returns element closest to target in arr[]
const absolute_note &find_closest_note(fixed freq)
{
  auto arr = notelist;
  auto n = sizeof(notelist) / sizeof(notelist[0]);

  // Corner cases
  // left-side case
  if (freq <= arr[0].freq)
    return arr[0];
  // right-side case
  if (freq >= arr[n - 1].freq)
    return arr[n - 1];

  // Doing binary search
  int i = 0, j = n, mid = 0;
  while (i < j)
  {
    mid = (i + j) / 2;

    if (arr[mid].freq == freq)
      return arr[mid];

    /* If freq is less than array element,
        then search in left */
    if (freq < arr[mid].freq)
    {

      // If target is greater than previous
      // to mid, return closest of two
      if (mid > 0 && freq > arr[mid - 1].freq)
        return get_closest_note(arr[mid - 1],
                                arr[mid], freq);
      j = mid;
    }
    /* Repeat for left half */

    // If freq is greater than mid
    else
    {
      if (mid < n - 1 && freq < arr[mid + 1].freq)
        return get_closest_note(arr[mid],
                                arr[mid + 1], freq);
      // update i
      i = mid + 1;
    }
  }

  // Only single element left after search
  return arr[mid];
}