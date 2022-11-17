#include <string>
#include "fpmath/fpmath.h"

typedef struct
{
  fixed freq;
  std::string name;
} note;

note notelist[] = {
    {fixed::from(493), "<C5"},
    {fixed::from(523), "C5"},
    {fixed::from(554), "C#5"},
    {fixed::from(587), "D5"},
    {fixed::from(622), "D#5"},
    {fixed::from(659), "E5"},
    {fixed::from(698), "F5"},
    {fixed::from(739), "F#5"},
    {fixed::from(783), "G5"},
    {fixed::from(830), "G#5"},
    {fixed::from(880), "A5"},
    {fixed::from(932), "A#5"},
    {fixed::from(987), "B5"},
    {fixed::from(1046), "C6"},
    {fixed::from(1108), "C#6"},
    {fixed::from(1174), "D6"},
    {fixed::from(1244), "D#6"},
    {fixed::from(1318), "E6"},
    {fixed::from(1396), "F6"},
    {fixed::from(1479), "F#6"},
    {fixed::from(1567), "G6"},
    {fixed::from(1661), "G#6"},
    {fixed::from(1760), "A6"},
    {fixed::from(1864), "A#6"},
    {fixed::from(1975), "B6"},
    {fixed::from(2093), "C7"},
    {fixed::from(2217), ">C7"}};

// int notelist[] = {493, 523, 554, 587, 622, 659, 698, 739, 783, 830, 880, 932, 987, 1046, 1108, 1174, 1244, 1318, 1396, 1479, 1567, 1661, 1760, 1864, 1975, 2093, 2217};
// int notelist_n = sizeof(notelist) / sizeof(notelist[0]);

// Method to compare which one is the more close. We find the closest by taking
// the difference between the target and both values. It assumes that val2 is
// greater than lhs and target lies between these two.
const note &get_closest_note(const note &lhs, const note &rhs, fixed freq)
{
  if (freq - lhs.freq >= rhs.freq - freq)
    return rhs;
  else
    return lhs;
}

// Returns element closest to target in arr[]
const note &find_closest_note(fixed freq)
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
