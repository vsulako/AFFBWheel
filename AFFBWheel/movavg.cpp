#include "movavg.h"

MovingAverage32::MovingAverage32(uint8_t _lv)
{
  level=_lv;
  size=1<<_lv;
  
  values=(int32_t*) calloc(size, sizeof(int32_t));
  index=0;
  total=0;
}
int32_t MovingAverage32::setValue(int32_t _value)
{
  total-=values[index];
  total+=_value;
  values[index]=_value;

  index++;
  if (index==size)
    index=0;

  return total>>level;
}
int32_t MovingAverage32::getValue()
{
  return total>>level;
}
void MovingAverage32::reset()
{
  memset(values, 0, size*sizeof(*values));
  total=0;
}

MovingAverage16::MovingAverage16(uint8_t _lv)
{
  level=_lv;
  size=1<<_lv;
  values=(int16_t*) calloc(size, sizeof(int16_t));
  index=0;
  total=0;
}
int16_t MovingAverage16::setValue(int16_t _value)
{
  total-=values[index];
  total+=_value;
  values[index]=_value;

  index++;
  if (index==size)
    index=0;

  return total>>level;
}
int16_t MovingAverage16::getValue()
{
  return total>>level;
}
void MovingAverage16::reset()
{
  memset(values, 0, size*sizeof(*values));
  total=0;
}
