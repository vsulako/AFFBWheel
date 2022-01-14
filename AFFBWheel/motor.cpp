#include "motor.h"

void Motor::begin()
{
  //Phase and Frequency Correct PWM, TOP=ICR1
  TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
  
  setBitDepth(DEFAULT_FFB_BITDEPTH);
  
  OCR1A=0;
  OCR1B=0;
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

//values -16383..16383
void Motor::setForce(int16_t force)
{
  force=constrain(force,-16383,16383);

  if (force>0)
  {
    OCR1A=force>>bitShift;
    OCR1B=0;
  }
  else 
  if (force<0)  
  {
    OCR1A=0;
    OCR1B=-(force>>bitShift);
  }
  else
  {
    OCR1A=0;
    OCR1B=0;
  }
}

void Motor::setBitDepth(uint8_t value)
{
  value=constrain(value,1,14);
  bitDepth=value;
  bitShift=14-bitDepth;
  ICR1=1<<bitDepth;
}
