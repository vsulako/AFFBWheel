#include "axis.h"

AxisWheel::AxisWheel()
{
  filterPosition=new MovingAverage32(MA_LEVEL_WHEEL_POSITION);
  filterVelocity=new MovingAverage16(MA_LEVEL_WHEEL_VELOCITY);
  filterAcceleration=new MovingAverage16(MA_LEVEL_WHEEL_ACCELERATION);
}

void AxisWheel::setValue(int32_t rawValue_)
{
  #ifdef STEER_TM_RATIO_ENABLED
     rawValue=rawValue_ * ((float)STEER_TM_RATIO_MUL / (float)STEER_TM_RATIO_DIV);
  #else
     rawValue= rawValue_;
  #endif
  
  int32_t val=constrain(rawValue, -axisMax, axisMax);

  value=val * rangeFactor;
  
  //Velocity and acceleration calculation for damper/friction/inertia effects
  //These parameters do not depend on wheel range

  absValue = filterPosition->setValue(val);
  
  int16_t tmpUs=micros();
  int16_t td=tmpUs-lastUs;
  lastUs=tmpUs;

  int16_t newVelocity;
  newVelocity=filterVelocity->setValue(((absValue-lastPosition)<<15)/td);
  lastPosition=absValue;
  
  acceleration=filterAcceleration->setValue(((int32_t)(newVelocity-velocity)<<15)/td);
  velocity=newVelocity;
}

void AxisWheel::setRange(uint16_t _deg)
{
    range=_deg;
    rangeFactor=((int32_t)1<<(16-STEER_BITDEPTH))*360.0/range;
    axisMax=(((int32_t)1<<(STEER_BITDEPTH-1)))*range/360-1;
}
void AxisWheel::center()
{
  value=0;
  lastPosition=0;
  velocity=0;
  acceleration=0;
  filterPosition->reset();
  filterVelocity->reset();
  filterAcceleration->reset();
}

//Остальные оси
Axis::Axis(uint8_t smoothLevel)
{
    filter=new MovingAverage16(smoothLevel);
}

void Axis::setValue(int16_t rawValue_)
{
  rawValue=rawValue_;

  if (autoLimit)
  {
      if (rawValue<axisMin)
         setLimits(rawValue, axisMax, true);
      if (rawValue>axisMax)
         setLimits(axisMin, rawValue, true);
  }

  value=constrain(rawValue, axisMin, axisMax);
  value=filter->setValue(value);

  if (bitTrim)
  {
     if (value>=0)
        value=value>>bitTrim<<bitTrim;
     else
        value=-(-value>>bitTrim<<bitTrim);
  }

  if (value<axisCenterN)
    value= (value - axisCenterN ) * rangeFactorNeg;
  else
  if (value>axisCenterP)
    value= (value - axisCenterP ) * rangeFactorPos;
  else
    value=0;
}

int16_t Axis::getCenter()
{
  return ((int32_t)axisCenterN+axisCenterP)>>1;
}
int16_t Axis::getDZ()
{
  return (axisCenterP-axisCenterN)>>1;
}
void Axis::setCenter(int16_t center)
{
    autoCenter=false;
    int16_t dz=getDZ();
    axisCenterN=center-dz;
    axisCenterP=center+dz;
    updateRangeFactor();
}

void Axis::setDZ(int16_t dz)
{
    autoCenter=false;
    int16_t center=getCenter();
    axisCenterN=center-dz;
    axisCenterP=center+dz;
    updateRangeFactor();
}
void Axis::setLimits(int16_t _min, int16_t _max, bool _auto)
{
    axisMin=_min;
    axisMax=_max;
      
    if (!_auto)
       autoLimit=false;

    updateRangeFactor();
}
void Axis::setAutoLimits(bool _auto)
{
  if (_auto)
  {
    axisMin=rawValue;
    axisMax=rawValue;
    autoCenter=true;
    updateRangeFactor(); 
  }
  autoLimit=_auto;    
}

void Axis::updateRangeFactor()
{
    if ((axisCenterN<axisMin) || (axisCenterP>axisMax))
      autoCenter=true;
      
    if (autoCenter)
    {
      axisCenterN=axisCenterP=(axisMin+axisMax)>>1;
    }
  
    rangeFactorNeg=32767.0/(axisCenterN-axisMin);
    rangeFactorPos=32767.0/(axisMax-axisCenterP);
}
