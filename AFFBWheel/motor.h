#pragma once

#include <Arduino.h>
#include "config.h"
#include <digitalWriteFast.h>

class Motor
{
  public:
    uint8_t bitDepth;
    void begin();
    void setForce(int16_t value);
    void setBitDepth(uint8_t value);
  private:
    uint8_t bitShift;
};
