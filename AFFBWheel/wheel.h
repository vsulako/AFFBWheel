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
#define AXIS_AUX3 5
#define AXIS_AUX4 6

//Input Report 
typedef struct
{
  int16_t axes[8]={-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768};
  uint32_t buttons;
#ifdef HATSWITCH
  uint8_t hat;
#endif
} wheelData;

//Reports for GUI
typedef struct
{
  uint8_t command;
  int16_t arg;
  uint8_t data[28];
} GUI_Report;

typedef struct
{
  char id[6]="AFFBW";
  char ver[12];
} GUI_Report_Version;

typedef struct
{
  int32_t rawValue;
  int16_t value;
  int16_t range;
  int16_t velocity;
  int16_t acceleration;
} GUI_Report_SteerAxis;
typedef struct
{
  int16_t rawValue;
  int16_t value;
  int16_t axisMin;
  int16_t axisMax;
  int16_t center;
  int16_t deadzone;
  uint8_t autoLimit;
  uint8_t hasCenter;
  uint8_t outputDisabled;
  uint8_t bitTrim;
} GUI_Report_AnalogAxis;
typedef struct
{
  int32_t buttons;
  int8_t centerButton;
  int8_t debounce;
} GUI_Report_Buttons;

typedef struct
{
  uint16_t maxvd;
  uint16_t maxvf;
  uint16_t maxacc;

  uint16_t minForce;
  uint16_t maxForce;
  uint16_t cutForce;

  uint8_t ffbBD;

  uint16_t endstopOffset;
  uint16_t endstopWidth;
} GUI_Report_Settings;


class Wheel_ 
{
  public:
    AxisWheel* axisWheel;
    Axis* analogAxes[7];
    uint32_t buttons;
    Wheel_();
    void update();
    FfbEngine ffbEngine;
    
    GUI_Report USB_GUI_Report;
  private:
    uint8_t getHatSwitch();
};
