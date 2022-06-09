#pragma once
#include <Arduino.h>

#include "config.h"
#include "movavg.h"


/*
 * Steering axis
 */
class AxisWheel
{
  public:
    int32_t rawValue; //raw steer position (2^STEER_BITDEPTH counts per turn)
    int32_t absValue; //=raw constrained and smoothed
    int16_t value;    //=output value (16bit)

    int32_t axisMax;
    int16_t range;

    int32_t lastPosition;
    int16_t velocity;
    int16_t acceleration;
    
    uint16_t lastUs;
    
    MovingAverage32* filterPosition;
    MovingAverage16* filterVelocity;
    MovingAverage16* filterAcceleration;
    
    AxisWheel();
    void setValue(int32_t rawValue_);
    void setRange(uint16_t _deg);
    void center();
  private:
    float rangeFactor;
};


/*
 * Analog axes
 */
class Axis
{
  public:
    int16_t rawValue;
    int16_t value=0;
    bool autoLimit=false;
    bool autoCenter=true;
    int16_t axisMin;
    int16_t axisMax;
    int16_t axisCenterN;
    int16_t axisCenterP;
    
    bool outputDisabled=false;
    int8_t bitTrim=0;
    
    Axis(uint8_t smoothLevel);
    void setValue(int16_t rawValue_);
    void setCenter(int16_t center);
    void setDZ(int16_t dz);
    void setLimits(int16_t _min, int16_t _max, bool _auto=false);
    void setAutoLimits(bool _auto);
    void updateRangeFactor();
    MovingAverage16* filter;

    int16_t getCenter();
    int16_t getDZ();
 private:
    float rangeFactorNeg, rangeFactorPos;
    
};
