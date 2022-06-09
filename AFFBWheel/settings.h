#pragma once
#include <Arduino.h>
#include "effNames.h"

struct SettingsAxis{
  int16_t axisMin;
  int16_t axisMax;
  int16_t axisCenter;
  int16_t axisDZ;
  int8_t  axisOutputDisabled;
  int8_t  axisBitTrim;
};

//settings in global variables
struct SettingsData
{
    int16_t gain[13];
    
    int8_t centerButton;
    uint8_t debounce;
    
    int16_t minForce;
    int16_t maxForce;
    int16_t cutForce;

    int16_t endstopOffset;
    int16_t endstopWidth;    
};    

//all settings
class SettingsEEPROM{
  public:
    SettingsAxis axes[7];
    SettingsData data;

    uint16_t range;
    uint8_t ffbBD;

    int16_t maxVelocityDamper;
    int16_t maxVelocityFriction;
    int16_t maxAcceleration;

    uint8_t checksum;

    void print();
    
    uint8_t calcChecksum();
};
