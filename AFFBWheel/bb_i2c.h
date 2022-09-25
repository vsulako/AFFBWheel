#pragma once

#include "config.h"
#include <Arduino.h>
#include <digitalWriteFast.h>

#define MCP23017_DEFAULT_ADDR  0x20
#define MCP23017_RIOCON        0x0a
#define MCP23017_RIOCON2       0x05
#define MCP23017_RGPPUA        0x0c
#define MCP23017_RGPIOA        0x12

#define ADS1015_DEFAULT_ADDR   0x48
#define ADS1015_RCONV          0x00
#define ADS1015_RCONF          0x01
#define ADS1015_CONFIG         0b1000010111100011  //no conversion, channel 1, ref 2.048v, single-shot, 3300 sps, comparator disabled
#define ADS1015_CONFIG_CH0     (ADS1015_CONFIG | ((1)<<12))
#define ADS1015_CONFIG_CH1     (ADS1015_CONFIG | ((2)<<12))
#define ADS1015_CONFIG_CH2     (ADS1015_CONFIG | ((3)<<12))

#define AS5600_ADDR   0x36
#define AS5600_RRAWANGLE 0x0C
#define AS5600_RANGLE 0x0E
#define AS5600_RCONF  0x07
#define AS5600_CONFIG  0b0001101100000000 //no low power mode, hysteresis off, output digital PWM, PWM 115hz, slow filter 2x(11), fast filter 9LSB(011), watchdog off.

#define PCF8574 1
#define PCF8575 2
#define PCF857x_DEFAULT_ADDR  0x20

#define ADS7828_DEFAULT_ADDR   0x48

//bitbang I2C communication.
//hardware i2c does not allow to reach max available speed, so bitbang is used
class BB_I2C
{
  public:
    int8_t addrR;
    int8_t addrW;

    void setAddr(uint8_t addr);
    
    virtual void writeByte(uint8_t data);
    virtual void readByte(uint8_t* pData, bool ack=true);

    void writeRegister(uint8_t reg, uint8_t data);
    void writeRegister16(uint8_t reg, uint16_t data);

    void requestReadRegister(uint8_t reg);
    
    void read(uint8_t* pData);
    void read16(uint16_t* pData);

    int8_t read();
    int16_t read16();
};

class BB_I2C_S1: public BB_I2C
{
  public:
    void writeByte(uint8_t data);
    void readByte(uint8_t* pData, bool ack=true);  
};

class MCP23017_BBI2C: public BB_I2C
{
  public:
    void begin(uint8_t addr=MCP23017_DEFAULT_ADDR);
};

class ADS1015_BBI2C: public BB_I2C
{
  public:
    void begin(uint8_t addr=ADS1015_DEFAULT_ADDR);
    void requestADC(uint8_t channel);
};

class AS5600_BBI2C: public BB_I2C
{
  public:
    void begin(uint8_t addr=AS5600_ADDR);
    int16_t readAngle();
};

class PCF857x_BBI2C: public BB_I2C_S1
{
  public:
    void begin(uint8_t addr=PCF857x_DEFAULT_ADDR);
};

class ADS7828_BBI2C: public BB_I2C
{
  public:
    void begin(uint8_t addr=ADS7828_DEFAULT_ADDR);
    int16_t readADC(uint8_t channel);
};
