#include "motor.h"

#ifdef USE_MCP4725

  #include "bb_i2c.h"
  MCP4725_BBI2C mcp4725;
  
#endif

void Motor::begin()
{
  #ifdef MODE_PWMDIR
  
    #ifdef PWM_INVERT
      #define PWM_COM1A0 1
    #else
      #define PWM_COM1A0 0
    #endif
  
    //Phase and Frequency Correct PWM, TOP=ICR1   1 pwm channel on pin 9
    TCCR1A=(1<<COM1A1) | (PWM_COM1A0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);

  #else
  
    //Phase and Frequency Correct PWM, TOP=ICR1   2 pwm channels, pins 9-10
    TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);

  #endif

  
  setBitDepth(DEFAULT_FFB_BITDEPTH);
  
  OCR1A=0;
  OCR1B=0;
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  #ifdef MOTOR_ENABLE_PIN
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWriteFast(MOTOR_ENABLE_PIN, 0);
  #endif

#ifdef USE_MCP4725
  #ifdef MCP4725_ADDR
    mcp4725.begin(MCP4725_ADDR);
  #else
    mcp4725.begin();
  #endif
#endif
}

//values -16383..16383
void Motor::setForce(int16_t force)
{
  byte buffer[3]; 
  
  force=constrain(force,-16383,16383);

  #ifdef MOTOR_ENABLE_PIN
  if (force!=0)
    digitalWriteFast(MOTOR_ENABLE_PIN, 1)
  else
    digitalWriteFast(MOTOR_ENABLE_PIN, 0);
  #endif

  #ifdef MODE_PWMDIR

      #ifdef DIR_INVERT
         #define DIR_POS 0
         #define DIR_NEG 1
      #else
         #define DIR_POS 1
         #define DIR_NEG 0
      #endif

      if (force>0)
      {
        OCR1A=(1+force)>>bitShift;
        digitalWriteFast(10, DIR_POS);
        #ifdef USE_MCP4725
        mcp4725.writeDAC(force>>2);
        #endif
      }
      else 
      if (force<0)  
      {
        OCR1A=(1-force)>>bitShift;
        digitalWriteFast(10, DIR_NEG);
        #ifdef USE_MCP4725
        mcp4725.writeDAC((-force)>>2);
        #endif
      }
      else
      {
        OCR1A=0;
      }
      
  #else
  
      if (force>0)
      {
        OCR1A=(1+force)>>bitShift;
        OCR1B=0;
      }
      else 
      if (force<0)  
      {
        OCR1A=0;
        OCR1B=(1-force)>>bitShift;
      }
      else
      {
        OCR1A=0;
        OCR1B=0;
      }
      
  #endif

  
}

void Motor::setBitDepth(uint8_t value)
{
  value=constrain(value,1,14);
  bitDepth=value;
  bitShift=14-bitDepth;
  ICR1=1<<bitDepth;
}
