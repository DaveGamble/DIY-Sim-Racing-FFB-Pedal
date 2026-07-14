//
//    FILE: FastTrig.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.6
// PURPOSE: Arduino library for a faster approximation of sin() and cos()
//    DATE: 2011-08-18
//     URL: https://github.com/RobTillaart/FastTrig
//          https://forum.arduino.cc/index.php?topic=69723.0


#include "FastTrig.h"


const float _PI_  = 3.14159265;
const float _PI_DIV_2_ = _PI_ / 2;


//  91 x 2 bytes ==> 182 bytes
//  use 65535.0 as divider
uint16_t sinTable16[] = {
  0,
1145, 2289, 3435, 4572, 5716, 6853, 7989, 9125, 10255, 11385,
12508, 13631, 14745, 15859, 16963, 18067, 19165, 20253, 21342, 22417,
23489, 24553, 25610, 26659, 27703, 28731, 29755, 30773, 31777, 32772,
33756, 34734, 35697, 36649, 37594, 38523, 39445, 40350, 41247, 42131,
42998, 43856, 44701, 45528, 46344, 47147, 47931, 48708, 49461, 50205,
50933, 51646, 52342, 53022, 53686, 54334, 54969, 55579, 56180, 56760,
57322, 57866, 58394, 58908, 59399, 59871, 60327, 60768, 61184, 61584,
61969, 62330, 62677, 63000, 63304, 63593, 63858, 64108, 64334, 64545,
64731, 64903, 65049, 65177, 65289, 65377, 65449, 65501, 65527, 65535,
65535
};

///////////////////////////////////////////////////////
//
//  GONIO LOOKUP
//
float isin(float f)
{
  bool negative = (f < 0);
  if (negative)
  {
    f = -f;
    negative = true;
  }

  long whole = f;
  uint8_t remain = (f - whole) * 256;

  if (whole >= 360)
  {
    whole %= 360;
    //  possible faster for 360-720
    //  if (whole >= 720) whole %= 360;
    //  else whole -= 360;
  }

  int y = whole;  //  16 bit math is faster than 32 bit

  if (y >= 180)
  {
    y -= 180;
    negative = !negative;
  }

  if (y >= 90)
  {
    y = 180 - y;
    if (remain != 0)
    {
      remain = 256 - remain;
      y--;
    }
  }

  //  float value improves ~4% on average error for ~60 bytes.
  uint16_t value = sinTable16[y];

  //  interpolate if needed
  if (remain > 0)
  {
    value = value + ((sinTable16[y + 1] - value) / 8 * remain) / 32;   //  == * remain / 256
  }
  float g = value * 0.0000152590219f;  //  = / 65535.0
  if (negative) return -g;
  return g;
}


float icos(float x)
{
  //  prevent modulo math if x in 0..360
  return isin(x - 270.0);  //  better than x + 90;
}

///////////////////////////////////////////////////////
//
//  INVERSE GONIO LOOKUP
//
float iasin(float f)
{
  bool negative = (f < 0);
  if (negative)
  {
    f = -f;
    negative = true;
  }
  uint16_t value = round(f * 65535);
  uint8_t lo = 0;
  uint8_t hi = 90;

  while (hi - lo > 1)
  {
    uint8_t mi = (lo + hi) / 2;
    if (sinTable16[mi] == value)
    {
      if (negative) return -mi;
      return mi;
    }
    if (sinTable16[mi] < value) lo = mi;
    else hi = mi;
  }
  float delta = value - sinTable16[lo];
  uint16_t range = sinTable16[hi] - sinTable16[lo];
  delta /= range;
  if (negative) return -(lo + delta);
  return (lo + delta);
}


float iacos(float f)
{
  return 90 - iasin(f);
}



inline float atanHelper(float x)
{
  float x2 = x * x;
  return (((0.079331f * x2) - 0.288679f) * x2 + 0.995354f) * x;

  //  an even more accurate alternative, less fast
  //  return ((((-0.0389929f * x2) + 0.1462766f) * x2 - 0.3211819f) * x2 + 0.9992150f) * x;
}

float atanFast(float x)
{
  //  remove two test will limit the input range but makes it even faster.
  if ( x > 1)  return (_PI_DIV_2_) - atanHelper(1.0 / x);
  if ( x < -1) return (-_PI_DIV_2_) - atanHelper(1.0 / x);
  return atanHelper(x);
}





float atan2Fast(float y, float x)
{
  //  catch singularity.
  if ((x == 0) && (y == 0)) return NAN;

  if (x >= 0)
  {
    if (y >= 0)
    {
      if (fabs(y) >= fabs(x)) return _PI_DIV_2_ - atanFast(x / y);
      return atanFast(y / x);
    }
    if (fabs(y) >= fabs(x)) return -_PI_DIV_2_ - atanFast(x / y);
    return atanFast(y / x);
  }
  else
  {
    if (y >= 0)
    {
      if (fabs(y) >= fabs(x)) return _PI_DIV_2_ - atanFast(x / y);
      return _PI_ + atanFast(y / x);
    }
    if (fabs(y) >= fabs(x)) return -_PI_DIV_2_ - atanFast(x / y);
    return -_PI_ + atanFast(y / x);
  }
}

//  -- END OF FILE --

