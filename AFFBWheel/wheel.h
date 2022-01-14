#pragma once

#include "axis.h"
#include "FfbEngine.h"
#include "WHID.h"
#include "hidDescriptor.h"

#define AXIS_ACC 0
#define AXIS_BRAKE 1
#define AXIS_CLUTCH 2
#define AXIS_AUX1 3
#define AXIS_AUX2 4

//Input Report 
typedef struct
{
  int16_t axes[6];
  uint32_t buttons;
} wheelData;

class Wheel_ 
{
  public:
    AxisWheel* axisWheel;
    Axis* analogAxes[5];
    uint32_t buttons;
    Wheel_();
    int16_t update();
    FfbEngine ffbEngine;
  private:
};
