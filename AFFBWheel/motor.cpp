#include "motor.h"

  
void Motor::begin()
{
  //Phase and Frequency Correct PWM, TOP=ICR3
  TCCR3A=(1<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
  TCCR3B=(0<<ICNC3) | (0<<ICES3) | (1<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (1<<CS30);

  //Output PWM to pin 5
  pinMode(5, OUTPUT);  
  
  setBitDepth(DEFAULT_FFB_BITDEPTH);
  
  OCR3A=0;

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWriteFast(9, 0);
  digitalWriteFast(10, 0);
}

//values -16383..16383
void Motor::setForce(int16_t force)
{
  force=constrain(force,-16383,16383);

  if (force>0)
  {
    digitalWriteFast(9, 1);
    digitalWriteFast(10, 0);
    
    OCR3A=(1+force)>>bitShift;
  }
  else 
  if (force<0)  
  {
    digitalWriteFast(9, 0);
    digitalWriteFast(10, 1);
    
    OCR3A=(1-force)>>bitShift;
  }
  else
  {
    digitalWriteFast(10, 0);
    digitalWriteFast(9, 0);

    OCR3A=0;
  }
}

void Motor::setBitDepth(uint8_t value)
{
  value=constrain(value,1,14);
  bitDepth=value;
  bitShift=14-bitDepth;
  ICR3=1<<bitDepth;
}
