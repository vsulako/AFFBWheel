#include "bb_i2c.h"

#define I2C_SDA_LOW   {pinModeFast(I2C_PIN_SDA, OUTPUT);__builtin_avr_delay_cycles(1);}
#define I2C_SDA_HIGH  {pinModeFast(I2C_PIN_SDA, INPUT);__builtin_avr_delay_cycles(1);}
#define I2C_SCL_LOW   {pinModeFast(I2C_PIN_SCL, OUTPUT);__builtin_avr_delay_cycles(1);}
#define I2C_SCL_HIGH  {pinModeFast(I2C_PIN_SCL, INPUT);__builtin_avr_delay_cycles(1);}

#define I2C_IS_SDA_HIGH (digitalReadFast(I2C_PIN_SDA))

#define I2C_START {I2C_SDA_LOW;__builtin_avr_delay_cycles(4);I2C_SCL_LOW;}
#define I2C_STOP  {I2C_SCL_HIGH;I2C_SDA_HIGH;}

void BB_I2C::setAddr(uint8_t addr)
{
  addrW=(addr<<1);
  addrR=addrW | 1;
}

void BB_I2C::writeByte(uint8_t data)
{
  uint8_t b;

  b=0b10000000;
  do
  {
    if (data & b)
    {
      I2C_SDA_HIGH;
    }
    else
    {
      I2C_SDA_LOW;
    }
    I2C_SCL_HIGH;
    __builtin_avr_delay_cycles(4);
    I2C_SCL_LOW;
  }while(b>>=1);

  __builtin_avr_delay_cycles(1);

  //ACK
  I2C_SCL_HIGH;
  __builtin_avr_delay_cycles(4);
  I2C_SCL_LOW;
  I2C_SDA_LOW;
}

void BB_I2C::readByte(uint8_t* pData, bool ack)
{
    uint8_t b;

    *pData=0;

    I2C_SDA_HIGH;

    b=0b10000000;
    do
    {
      I2C_SCL_HIGH;
      __builtin_avr_delay_cycles(1);
      if (I2C_IS_SDA_HIGH)
        *pData |= b;
      I2C_SCL_LOW;
    }while(b>>=1);

  if (ack)
  {
    I2C_SDA_LOW;   //ACK
    I2C_SCL_HIGH;
    __builtin_avr_delay_cycles(4);
    I2C_SCL_LOW;
  }
  else
  {
    I2C_SCL_HIGH;  //NACK
    I2C_SCL_LOW;
    I2C_SDA_LOW;
  }
}

void BB_I2C::writeRegister(uint8_t reg, uint8_t data)
{
    I2C_START;
    writeByte(addrW);
    writeByte(reg);
    writeByte(data);
    I2C_STOP;
}

void BB_I2C::writeRegister16(uint8_t reg, uint16_t data)
{
    I2C_START;
    writeByte(addrW);
    writeByte(reg);
    writeByte((uint8_t)(data>>8));
    writeByte((uint8_t)data);
    I2C_STOP;
}

void BB_I2C::requestReadRegister(uint8_t reg)
{
    I2C_START;
    writeByte(addrW);
    writeByte(reg);
    I2C_STOP;
}

void BB_I2C::read(uint8_t* pData)
{
    I2C_START;
    writeByte(addrR);
    readByte(pData, false);
    I2C_STOP;
}
void BB_I2C::read16(uint16_t* pData)
{
    I2C_START;
    writeByte(addrR);
    readByte(((uint8_t*)pData)+1, true);
    readByte((uint8_t*)pData, false);
    I2C_STOP;
}

int8_t BB_I2C::read()
{
    uint8_t data;
    read(&data);
    return data;
}
int16_t BB_I2C::read16()
{
    uint16_t data;
    read16(&data);
    return data;
}

void MCP23017_BBI2C::begin(uint8_t addr)
{
    setAddr(addr);
    
    //just in case of IOCON.BANK=1, clear IOCON/GPINTENB to set IOCON.BANK=0
    writeRegister(MCP23017_RIOCON2, 0);

    //Byte mode, BANK=0
    writeRegister(MCP23017_RIOCON, 0b00110000);

    //turn on pullup
    writeRegister16(MCP23017_RGPPUA, 0xFFFF);
        
    //request read
    requestReadRegister(MCP23017_RGPIOA);
}

void ADS1015_BBI2C::begin(uint8_t addr)
{
    setAddr(addr);
}

void ADS1015_BBI2C::requestADC(uint8_t channel)
{
  switch(channel)
  {
    case 0:
      writeRegister16(ADS1015_RCONF, ADS1015_CONFIG_CH0);
      break;
    case 1:
      writeRegister16(ADS1015_RCONF, ADS1015_CONFIG_CH1);
      break;
    case 2:
      writeRegister16(ADS1015_RCONF, ADS1015_CONFIG_CH2);
      break;
  }
  requestReadRegister(ADS1015_RCONV);
}

void AS5600_BBI2C::begin(uint8_t addr)
{
    setAddr(addr);
    
    writeRegister16(AS5600_RCONF, AS5600_CONFIG);

    requestReadRegister(AS5600_RRAWANGLE);
}
int16_t AS5600_BBI2C::readAngle()
{
    int16_t val=read16();
    requestReadRegister(AS5600_RRAWANGLE);
    return val;
}
