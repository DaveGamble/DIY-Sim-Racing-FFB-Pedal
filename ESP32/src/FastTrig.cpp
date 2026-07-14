//
//    FILE: FastTrig.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.6
// PURPOSE: Arduino library for a faster approximation of sin() and cos()
//    DATE: 2011-08-18
//     URL: https://github.com/RobTillaart/FastTrig
//          https://forum.arduino.cc/index.php?topic=69723.0
#include "FastTrig.h"
#include <stdint.h>
#include <math.h>

//  91 x 2 bytes ==> 182 bytes. use 65535.0 as divider

float isin(float f)
{
  const uint16_t sinTable16[] = {
    0, 1145, 2289, 3435, 4572, 5716, 6853, 7989, 9125, 10255, 11385, 12508, 13631, 14745, 15859, 16963, 18067, 19165, 20253, 21342, 22417,
    23489, 24553, 25610, 26659, 27703, 28731, 29755, 30773, 31777, 32772, 33756, 34734, 35697, 36649, 37594, 38523, 39445, 40350, 41247, 42131,
    42998, 43856, 44701, 45528, 46344, 47147, 47931, 48708, 49461, 50205, 50933, 51646, 52342, 53022, 53686, 54334, 54969, 55579, 56180, 56760,
    57322, 57866, 58394, 58908, 59399, 59871, 60327, 60768, 61184, 61584, 61969, 62330, 62677, 63000, 63304, 63593, 63858, 64108, 64334, 64545,
    64731, 64903, 65049, 65177, 65289, 65377, 65449, 65501, 65527, 65535, 65535
  };
  bool negative = (f < 0);
  if (negative) f = -f;

  long whole = f;
  float remain = f - whole;
  if (whole >= 360) whole %= 360;
  int y = whole;  //  16 bit math is faster than 32 bit
  if (y >= 180) { y -= 180; negative = !negative; }
  if (y >= 90) { y = 180 - y; if (remain) {remain = 1.f - remain; y--;} }

  //  float value improves ~4% on average error for ~60 bytes.
  float value = sinTable16[y] + (sinTable16[y + 1] - sinTable16[y]) * remain;   //  == * remain / 256
  return value * (negative ? -0.0000152590219f : 0.0000152590219f);
}
