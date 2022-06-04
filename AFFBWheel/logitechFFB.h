#pragma once

#include <Arduino.h>
#include "HIDReportType.h"
#include "FfbReportHandler.h"

#define LOGITECH_CMD_DOWNLOADFORCE     0x00
#define LOGITECH_CMD_DOWNLOADPLAYFORCE 0x01
#define LOGITECH_CMD_PLAYFORCE         0x02    
#define LOGITECH_CMD_STOPFORCE         0x03    
#define LOGITECH_CMD_REFRESHFORCE      0x0c   
 
#define LOGITECH_CMD_SETDEFAULTSPRING  0x0e
#define LOGITECH_CMD_DEFAULTSPRINGON   0x04
#define LOGITECH_CMD_DEFAULTSPRINGOFF  0x05

#define LOGITECH_CMD_NORMALMODE        0x08
#define LOGITECH_CMD_SETLED            0x09
#define LOGITECH_CMD_SETWATCHDOG       0x0a
#define LOGITECH_CMD_RAWMODE           0x0b
#define LOGITECH_CMD_FIXEDLOOP         0x0d
#define LOGITECH_CMD_SETDEADBAND       0x0f
#define LOGITECH_CMD_EXTENDEDCMD       0xf8

#define LOGITECH_EXTCMD_DFPPRO          0x01
#define LOGITECH_EXTCMD_WRANGE200       0x02
#define LOGITECH_EXTCMD_WRANGE900       0x03
#define LOGITECH_EXTCMD_DEVMODE         0x09
#define LOGITECH_EXTCMD_REVIDENTITY     0x0a
#define LOGITECH_EXTCMD_SWG25DETACH     0x10
#define LOGITECH_EXTCMD_SWG25NODETACH   0x11
#define LOGITECH_EXTCMD_RPMLEDS         0x12
#define LOGITECH_EXTCMD_WRANGE          0x81

#define LOGITECH_FORCE_DEFAULTSPRING        0x00

#define LOGITECH_FORCE_SPRING               0x01
#define LOGITECH_FORCE_DAMPER               0x02
#define LOGITECH_FORCE_AUTOCENTSPRING       0x03
#define LOGITECH_FORCE_HIRESSPRING          0x0b
#define LOGITECH_FORCE_HIRESDAMPER          0x0c
#define LOGITECH_FORCE_HIRESACSPRING        0x0d
#define LOGITECH_FORCE_FRICTION             0x0e
#define LOGITECH_FORCE_VARIABLE             0x08

//never used
#define LOGITECH_FORCE_CONSTANT             0x00  
#define LOGITECH_FORCE_RAMP                 0x09
#define LOGITECH_FORCE_SAWTOOTHUP           0x04
#define LOGITECH_FORCE_SAWTOOTHDOWN         0x05
#define LOGITECH_FORCE_TRAPEZOID            0x06
#define LOGITECH_FORCE_RECTANGLE            0x07
#define LOGITECH_FORCE_SQUARE               0x0a

class LogitechFFB
{
  public:
  
    static void downloadForce(uint8_t* data, uint8_t slot, volatile TEffectState* effect);
    
    static int16_t convertForce(uint8_t f);
    static int16_t convertPos(uint8_t f);
    static int16_t KFactor(uint8_t K, bool hires);
    
    static void setForceSpring(uint8_t* data, volatile TEffectState* effect);
    static void setForceDamper(uint8_t* data, volatile TEffectState* effect);
    static void setForceFriction(uint8_t* data, volatile TEffectState* effect);
    static void setForceVariable(uint8_t* data, uint8_t slot, volatile TEffectState* effect);
    static void setForceST(uint8_t* data, volatile TEffectState* effect);
    static void setForceTrapezoid(uint8_t* data, volatile TEffectState* effect);
    static void setForceRectangle(uint8_t* data, volatile TEffectState* effect);
    static void setForceSquare(uint8_t* data, volatile TEffectState* effect);
};
