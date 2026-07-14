#pragma once
//
//    FILE: FastTrig.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.6
// PURPOSE: Arduino library for a faster approximation of sin() and cos()
//    DATE: 2011-08-18
//     URL: https://github.com/RobTillaart/FastTrig
//          https://forum.arduino.cc/index.php?topic=69723.0


#ifdef ESP_PLATFORM
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#else
#include "Arduino.h"
#endif


#define FAST_TRIG_LIB_VERSION             (F("0.3.6"))


#ifdef __cplusplus
extern "C"
{
#endif


///////////////////////////////////////////////////////
//
//  GONIO LOOKUP
//
float isin(float f);
float icos(float x);

///////////////////////////////////////////////////////
//
//  INVERSE GONIO LOOKUP
//
float iacos(float f);

//  PLACEHOLDER (might be obsolete due to atanFast() formula.
//float iatan(float f);
//  fast atan() formula, in fact a modified Taylor expansion
//  input = -1 .. 1
//float atanFast(float f);
//  atan2Fast() folds and mirrors => calls atanFast() + offset.
float atan2Fast(float y, float x);


#ifdef __cplusplus
}
#endif


//  -- END OF FILE --
