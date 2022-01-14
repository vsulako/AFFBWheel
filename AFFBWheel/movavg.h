#pragma once
#include <Arduino.h>

/*
 * Вычисление скользящего среднего.
 * Moving average calculation
 */
class Median3{
  public:
    int32_t values[3];
    uint8_t index;
    int32_t setValue(int32_t _value);
};

class MovingAverage32
{
   public:
       int32_t *values;
       int32_t total;
       uint8_t index;
       uint8_t level;
       uint8_t size;
       MovingAverage32(uint8_t _lv);
       int32_t setValue(int32_t _value);
       int32_t getValue();
       void reset();
};

class MovingAverage16
{
   public:
       int16_t *values;
       int32_t total;
       uint8_t index;
       uint8_t level;
       uint8_t size;
       MovingAverage16(uint8_t _size);
       int16_t setValue(int16_t _value);
       int16_t getValue();
       void reset();
};
