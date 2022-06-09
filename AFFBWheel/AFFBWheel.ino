/*
MIT License

Copyright (c) 2022 Sulako

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <SPI.h>
#include <EEPROM.h>
#include <digitalWriteFast.h>         //https://github.com/NicksonYap/digitalWriteFast
#include <avdweb_AnalogReadFast.h>    //https://github.com/avandalen/avdweb_AnalogReadFast

#include "config.h"
#include "wheel.h"
#include "motor.h"
#include "settings.h"

#define AFFB_VER "1.0.2"

//global variables
Wheel_ wheel;
Motor motor;
SettingsData settings;

int16_t force;

bool timing=false;
bool fvaOut=false;

#ifdef APB
bool apb_out=false;
#endif
#ifdef ASHIFTER
bool ashifter_out=false;
#endif

uint16_t timerInfo;
uint16_t loopCount;

int8_t axisInfo=-1;

bool centerButtonState=false;

uint32_t tempButtons;
uint8_t debounceCount=0;

//constants and definitions for analog axes
#if ((PEDALS_TYPE == PT_INTERNAL) || (PEDALS_TYPE == PT_HC164))
  #define DEFAULT_AA_MIN 0
  #define DEFAULT_AA_MAX 1023
#endif

#if PEDALS_TYPE == PT_ADS1015
  #include "bb_i2c.h"
  #define DEFAULT_AA_MIN -32767
  #define DEFAULT_AA_MAX 32767

  ADS1015_BBI2C ads1015;
  int8_t ADS1015_axis=0;
  static const int8_t ADS1015_channels[]={ADS1015_CH_ACC, ADS1015_CH_BRAKE, ADS1015_CH_CLUTCH};
#endif

#if ((PEDALS_TYPE==PT_MCP3204_4W) || (PEDALS_TYPE==PT_MCP3204_SPI))
  #define DEFAULT_AA_MIN 0
  #define DEFAULT_AA_MAX 4095
#endif

//constants and definions for buttons

#if BUTTONS_TYPE == BT_MCP23017
  #include "bb_i2c.h"
  MCP23017_BBI2C mcp23017_1;
  MCP23017_BBI2C mcp23017_2;
#endif


void load(bool defaults=false);

//------------------------ steering wheel sensor ----------------------------
#if STEER_TYPE == ST_TLE5010
  #include <TLE5010.h>              //https://github.com/vsulako/TLE5010
  #include "multiturn.h"
  
  TLE5010_SPI sensor(TLE5010_PIN_CS);
  MultiTurn MT;

  #define SETUP_WHEEL_SENSOR setupTLE();
  #define GET_WHEEL_POS (-MT.setValue(getWheelPos()))
  #define CENTER_WHEEL MT.zero();

  inline int16_t getWheelPos(){
    SPI.begin();
    int16_t v=sensor.readInteger()>>(16-STEER_BITDEPTH);
    SPI.end();
    return v;
  }
  
  void setupTLE()
  {
     //4MHz on pin 5 for TLE5010
      TCCR3A=(0<<COM3A1) | (1<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
      TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) | (0<<CS32) | (0<<CS31) | (1<<CS30);
      OCR3A=1;
      pinMode(5, OUTPUT);

      sensor.begin();
      SPI.end();

      sensor.atan2FuncInt=atan2_fix; 
  
      delayMicroseconds(10000); //let sensor start
  
      GET_WHEEL_POS;
  }
#endif

#if STEER_TYPE == ST_ENCODER
  #include <Encoder.h>        //https://github.com/PaulStoffregen/Encoder
  Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);

  #define SETUP_WHEEL_SENSOR
  #define GET_WHEEL_POS (((int32_t)encoder.read() << STEER_BITDEPTH) / ENCODER_PPR)
  #define CENTER_WHEEL encoder.write(0);

#endif

#if STEER_TYPE == ST_AS5600
  #include "bb_i2c.h"
  #include "multiturn.h"
  
  AS5600_BBI2C AS5600;
  MultiTurn MT;

  #define SETUP_WHEEL_SENSOR setupAS5600();
  #define GET_WHEEL_POS (MT.setValue((AS5600.readAngle()-2048)<<(STEER_BITDEPTH-12)))
  #define CENTER_WHEEL MT.zero();

  void setupAS5600()
  {
     AS5600.begin();
  }
#endif
//-------------------------------------------------------------------------------------


void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(50);

  //Steering axis sensor setup
  SETUP_WHEEL_SENSOR;

  //set up analog axes
  #ifdef AA_PULLUP
    analogReference(INTERNAL);  //2.56v reference to get more resolution.
  #endif

  #if (PEDALS_TYPE==PT_MP_HC164)
    pinMode(MP_HC164_PIN_SCK, OUTPUT);
  #endif

  #if (PEDALS_TYPE==PT_MCP3204_4W)
    pinModeFast(MCP3204_4W_PIN_SCK, OUTPUT);
    
    #if (MCP3204_4W_PIN_MOSI!=MCP3204_4W_PIN_MISO)
      pinModeFast(MCP3204_4W_PIN_MOSI, OUTPUT);
      pinModeFast(MCP3204_4W_PIN_MISO, INPUT);
    #endif
  #endif

  #if (PEDALS_TYPE==PT_MCP3204_SPI)
    pinMode(MCP3204_PIN_CS, OUTPUT);
  #endif

  #if (PEDALS_TYPE == PT_ADS1015)
    ads1015.begin();
  #endif

  //setup buttons
  #if (BUTTONS_TYPE == BT_74HC165)
    pinMode(HC165_PIN_DATA1, INPUT_PULLUP);
    pinMode(HC165_PIN_DATA2, INPUT_PULLUP);
    pinMode(HC165_PIN_SCK, OUTPUT);
    #ifdef HC165_PIN_PL
      pinMode(HC165_PIN_PL, OUTPUT);
    #endif
  #endif

  #if (BUTTONS_TYPE == BT_MCP23017)
    mcp23017_1.begin(MCP23017_ADDR1);
    mcp23017_2.begin(MCP23017_ADDR2);
  #endif

  //motor setup
  motor.begin();
  
  //load settings
  load();

  center();

  while(true)
    mainLoop();
}

void mainLoop() {

  uint16_t t[5];

  //Gathering data and measuring time
  if (timing)
      {
        t[0]=micros();
        wheel.axisWheel->setValue(GET_WHEEL_POS);
        t[1]=micros();
        readAnalogAxes();
        t[2]=micros();
        readButtons();
        t[3]=micros();
        //Send data and receive FFB commands
        wheel.update();
        t[4]=micros();
        processFFB();
        t[5]=micros();
      }
   else
      {
        wheel.axisWheel->setValue(GET_WHEEL_POS);
        readAnalogAxes();
        readButtons();
        processUsbCmd();
        wheel.update();
        processFFB();
      }

  loopCount++;

  if ((uint16_t)(millis()-timerInfo)>1000)
  {
      if (timing)
      {

         Serial.print(F("S: "));
         Serial.print(t[1]-t[0]);
         Serial.print(F(" A: "));
         Serial.print(t[2]-t[1]);
         Serial.print(F(" B: "));
         Serial.print(t[3]-t[2]);
         Serial.print(F(" U: "));
         Serial.print(t[4]-t[3]);
         Serial.print(F(" F: "));
         Serial.print(t[5]-t[4]);
         Serial.print(F(" loop/sec:"));
         Serial.print(loopCount);
         Serial.println();
          
         loopCount=0;
         timerInfo=millis();
      }
  }
  
  processSerial();
}

//Processing endstop and force feedback
void processFFB()
{
    int32_t excess=0;
    if (wheel.axisWheel->rawValue > wheel.axisWheel->axisMax)
       excess=wheel.axisWheel->rawValue - wheel.axisWheel->axisMax;
    if (wheel.axisWheel->rawValue < -wheel.axisWheel->axisMax)
       excess=wheel.axisWheel->rawValue + wheel.axisWheel->axisMax;
    if (excess)
    {
      int32_t absExcess=abs(excess);
      if (absExcess<settings.endstopWidth)
      {
        force = settings.endstopOffset + (absExcess * (16383 - settings.endstopOffset) / settings.endstopWidth);
      }
      else
        force = 16383;

      if (excess<0)
        force = -force;
        
      if (settings.gain[GAIN_ENDSTOP]!=1024)
          force=applyGain(force, settings.gain[GAIN_ENDSTOP]);
    }
    else
    {
      
      force=wheel.ffbEngine.calculateForce(wheel.axisWheel);
    }
    
    force=applyForceLimit(force);
    motor.setForce(force);
}


//scaling force to minForce & maxForce and cut at cutForce
int16_t applyForceLimit(int16_t force)
{
    if (force==0)
      return 0;

    if ((settings.minForce!=0) || (settings.maxForce<16383))
    {
        if (abs(force)<1024)//slope
        {
          int32_t v = (((settings.maxForce-settings.minForce)) >> 4) + settings.minForce;
          force =  (force * v) >> 10; 
        }
        else
            force = (int16_t)((int32_t)force * (settings.maxForce-settings.minForce) >> 14) + sign(force)*settings.minForce;
    }

    if (settings.cutForce>=16383)
      return force;
    else
      return constrain(force, -settings.cutForce, settings.cutForce);
}


/*
communicating with GUI:
*/
void processUsbCmd()
{
  USB_GUI_Command* usbCmd=&wheel.ffbEngine.ffbReportHandler->usbCommand;

  //clear output report
  memset(&wheel.USB_GUI_Report, 0, sizeof(wheel.USB_GUI_Report));
  
  void* data=wheel.USB_GUI_Report.data;
  if (usbCmd->command)
  {
      //return data only for read commands
      if (usbCmd->command<10)
      {
        wheel.USB_GUI_Report.command=usbCmd->command;
        wheel.USB_GUI_Report.arg=usbCmd->arg[0];
      }
      
      switch(usbCmd->command)
      {
          //get data
          case 1: //return string "AFFBW "+version
              strcpy_P(((GUI_Report_Version*)data)->id, PSTR("AFFBW"));
              strcpy_P(((GUI_Report_Version*)data)->ver, PSTR(AFFB_VER));
            break;
          case 2: //return steering axis data
              ((GUI_Report_SteerAxis*)data)->rawValue=wheel.axisWheel->rawValue;
              ((GUI_Report_SteerAxis*)data)->value=wheel.axisWheel->value;
              ((GUI_Report_SteerAxis*)data)->range=wheel.axisWheel->range;
              ((GUI_Report_SteerAxis*)data)->velocity=wheel.axisWheel->velocity;
              ((GUI_Report_SteerAxis*)data)->acceleration=wheel.axisWheel->acceleration;
            break;
          case 3: //return analog axis data
              ((GUI_Report_AnalogAxis*)data)->rawValue=wheel.analogAxes[usbCmd->arg[0]]->rawValue;
              ((GUI_Report_AnalogAxis*)data)->value=wheel.analogAxes[usbCmd->arg[0]]->value;
              ((GUI_Report_AnalogAxis*)data)->axisMin=wheel.analogAxes[usbCmd->arg[0]]->axisMin;
              ((GUI_Report_AnalogAxis*)data)->axisMax=wheel.analogAxes[usbCmd->arg[0]]->axisMax;
              ((GUI_Report_AnalogAxis*)data)->center=wheel.analogAxes[usbCmd->arg[0]]->getCenter();
              ((GUI_Report_AnalogAxis*)data)->deadzone=wheel.analogAxes[usbCmd->arg[0]]->getDZ();
              ((GUI_Report_AnalogAxis*)data)->autoLimit=wheel.analogAxes[usbCmd->arg[0]]->autoLimit;
              ((GUI_Report_AnalogAxis*)data)->hasCenter=!wheel.analogAxes[usbCmd->arg[0]]->autoCenter;
              
              ((GUI_Report_AnalogAxis*)data)->outputDisabled=wheel.analogAxes[usbCmd->arg[0]]->outputDisabled;
              ((GUI_Report_AnalogAxis*)data)->bitTrim=wheel.analogAxes[usbCmd->arg[0]]->bitTrim;
            break;
          case 4: //return buttons data
              ((GUI_Report_Buttons*)data)->buttons=wheel.buttons;
              ((GUI_Report_Buttons*)data)->centerButton=settings.centerButton;
              ((GUI_Report_Buttons*)data)->debounce=settings.debounce;
            break;
          case 5: //return gains
              memcpy(data, settings.gain, sizeof(settings.gain));
            break;
          case 6: //return remaining settings
              //GUI_Report_Settings* repSettings=(GUI_Report_Settings*)(wheel.USB_GUI_Report.data);
              
              ((GUI_Report_Settings*)data)->maxvd=round(16384.0/wheel.ffbEngine.maxVelocityDamperC);
              ((GUI_Report_Settings*)data)->maxvf=round(16384.0/wheel.ffbEngine.maxVelocityFrictionC);
              ((GUI_Report_Settings*)data)->maxacc=round(16384.0/wheel.ffbEngine.maxAccelerationInertiaC);

              ((GUI_Report_Settings*)data)->minForce=settings.minForce;
              ((GUI_Report_Settings*)data)->maxForce=settings.maxForce;
              ((GUI_Report_Settings*)data)->cutForce=settings.cutForce;

              ((GUI_Report_Settings*)data)->ffbBD=motor.bitDepth;

              ((GUI_Report_Settings*)data)->endstopOffset=settings.endstopOffset;
              ((GUI_Report_Settings*)data)->endstopWidth=settings.endstopWidth;
            break;
            

          // set 
          case 10://set range for steering axis
              wheel.axisWheel->setRange(usbCmd->arg[0]);
            break;
          case 11://set limits for analog axis
              wheel.analogAxes[usbCmd->arg[0]]->setLimits(usbCmd->arg[1], usbCmd->arg[2]); 
            break;
          case 12://set center for analog axis
              wheel.analogAxes[usbCmd->arg[0]]->setCenter(usbCmd->arg[1]);
            break;
          case 13://set deadzone for analog axis
              wheel.analogAxes[usbCmd->arg[0]]->setDZ(usbCmd->arg[1]);
            break; 
          case 14://set autolimits for analog axis
              wheel.analogAxes[usbCmd->arg[0]]->setAutoLimits(usbCmd->arg[1]>0);
            break; 
            
          case 15://set center button
              settings.centerButton=usbCmd->arg[0];
            break; 
          case 16://set debounce value
              settings.debounce=usbCmd->arg[0];
            break; 

          case 17://set gain
              settings.gain[usbCmd->arg[0]]=usbCmd->arg[1];
            break;
            
          case 18://set misc settings
              switch (usbCmd->arg[0])
              {
                  case 0:
                     wheel.ffbEngine.maxVelocityDamperC=16384.0/usbCmd->arg[1];
                     break;
                  case 1:
                     wheel.ffbEngine.maxVelocityFrictionC=16384.0/usbCmd->arg[1];
                     break;
                  case 2:
                     wheel.ffbEngine.maxAccelerationInertiaC=16384.0/usbCmd->arg[1];
                     break;
                  case 3:
                     settings.minForce=usbCmd->arg[1];
                     break;
                  case 4:
                     settings.maxForce=usbCmd->arg[1];
                     break;
                  case 5:
                     settings.cutForce=usbCmd->arg[1];
                     break;
                  case 6:
                     motor.setBitDepth(usbCmd->arg[1]);
                     break;
                  case 7:
                     settings.endstopOffset=usbCmd->arg[1];
                     settings.endstopWidth=usbCmd->arg[2];
                     break;
              }
            break;
          case 19://set outputDisabled and bittrim for analog axis
              wheel.analogAxes[usbCmd->arg[0]]->outputDisabled=(usbCmd->arg[1]>0);
              wheel.analogAxes[usbCmd->arg[0]]->bitTrim=usbCmd->arg[2];
            break;  

          //commands
          case 20: //load settings from EEPROM
              load();
            break;
          case 21: //save settings to EEPROM
              save();
            break;
          case 22: //load defaults
              load(true);
            break;
          case 23://center wheel
              center();
            break;
      }
    }
    usbCmd->command=0;
}


//------------------------- Reading all analog axes ----------------------------------
void readAnalogAxes()
{

#if (PEDALS_TYPE==PT_INTERNAL)
  #ifdef AA_PULLUP_LINEARIZE
      wheel.analogAxes[AXIS_ACC]->setValue(pullup_linearize(analogReadFast(PIN_ACC)));
      wheel.analogAxes[AXIS_BRAKE]->setValue(pullup_linearize(analogReadFast(PIN_BRAKE)));
      wheel.analogAxes[AXIS_CLUTCH]->setValue(pullup_linearize(analogReadFast(PIN_CLUTCH)));
  #else
      wheel.analogAxes[AXIS_ACC]->setValue(analogReadFast(PIN_ACC));
      wheel.analogAxes[AXIS_BRAKE]->setValue(analogReadFast(PIN_BRAKE));
      wheel.analogAxes[AXIS_CLUTCH]->setValue(analogReadFast(PIN_CLUTCH));
  #endif
#endif

#if (PEDALS_TYPE==PT_HC164)
  digitalWriteFast(MP_HC164_PIN_SCK, 1);

  wheel.analogAxes[AXIS_ACC]->setValue(analogReadFast(MP_HC164_PIN_ADATA));

  digitalWriteFast(MP_HC164_PIN_SCK, 0);
  digitalWriteFast(MP_HC164_PIN_SCK, 1);

  wheel.analogAxes[AXIS_BRAKE]->setValue(analogReadFast(MP_HC164_PIN_ADATA));
  
  digitalWriteFast(MP_HC164_PIN_SCK, 0);
  digitalWriteFast(MP_HC164_PIN_SCK, 1);

  wheel.analogAxes[AXIS_CLUTCH]->setValue(analogReadFast(MP_HC164_PIN_ADATA));

  digitalWriteFast(MP_HC164_PIN_SCK, 0);
#endif

#if (PEDALS_TYPE==PT_MCP3204_4W)
  wheel.analogAxes[AXIS_ACC]->setValue(MCP3204_BB_read(MCP3204_CH_ACC));
  wheel.analogAxes[AXIS_BRAKE]->setValue(MCP3204_BB_read(MCP3204_CH_BRAKE));
  wheel.analogAxes[AXIS_CLUTCH]->setValue(MCP3204_BB_read(MCP3204_CH_CLUTCH));
#endif

#if (PEDALS_TYPE==PT_MCP3204_SPI)
  SPI.begin();
  wheel.analogAxes[AXIS_ACC]->setValue(MCP3204_SPI_read(MCP3204_CH_ACC));
  wheel.analogAxes[AXIS_BRAKE]->setValue(MCP3204_SPI_read(MCP3204_CH_BRAKE));
  wheel.analogAxes[AXIS_CLUTCH]->setValue(MCP3204_SPI_read(MCP3204_CH_CLUTCH));
  SPI.end();
#endif

#if (PEDALS_TYPE == PT_ADS1015)
  ADS1015_read();
#endif

  //additional axes
  #ifdef AA_PULLUP_LINEARIZE
    
      #ifdef PIN_AUX1
        wheel.analogAxes[AXIS_AUX1]->setValue(pullup_linearize(analogReadFast(PIN_AUX1)));
      #endif
      #ifdef PIN_AUX2
        wheel.analogAxes[AXIS_AUX2]->setValue(pullup_linearize(analogReadFast(PIN_AUX2)));
      #endif
      #ifdef PIN_AUX3
        wheel.analogAxes[AXIS_AUX3]->setValue(pullup_linearize(analogReadFast(PIN_AUX3)));
      #endif
      #ifdef PIN_AUX4
        wheel.analogAxes[AXIS_AUX4]->setValue(pullup_linearize(analogReadFast(PIN_AUX4)));
      #endif
  #else
      #ifdef PIN_AUX1
        wheel.analogAxes[AXIS_AUX1]->setValue(analogReadFast(PIN_AUX1));
      #endif
      #ifdef PIN_AUX2
        wheel.analogAxes[AXIS_AUX2]->setValue(analogReadFast(PIN_AUX2));
      #endif
      #ifdef PIN_AUX3
        wheel.analogAxes[AXIS_AUX3]->setValue(analogReadFast(PIN_AUX3));
      #endif
      #ifdef PIN_AUX4
        wheel.analogAxes[AXIS_AUX4]->setValue(analogReadFast(PIN_AUX4));
      #endif
  #endif

  if (fvaOut)
  {
    wheel.analogAxes[AXIS_AUX3]->value=(wheel.axisWheel->velocity << 1) * wheel.ffbEngine.maxVelocityDamperC;
    wheel.analogAxes[AXIS_AUX4]->value=(wheel.axisWheel->acceleration << 1)* wheel.ffbEngine.maxAccelerationInertiaC;
    wheel.analogAxes[AXIS_AUX2]->value=force<<1;
  }
}

#ifdef AA_PULLUP_LINEARIZE
//Linearizing axis values, when using internal adc + pullup.
int16_t pullup_linearize(int16_t val)
{
  //Assuming VCC=5v, ADCreference=2.56v, 0 < val < 1024
  //val = val * 1024 * (R_pullup / R_pot) / (1024 * 5 / 2.56 - val)
  //if Rpullup = R_pot...
  //val = val * 1024 / (2000 - val)
  //division is slow
  //piecewise linear approximation, 16 steps
  static const int16_t a[] ={0, 33, 70, 108, 150, 195, 243, 295, 352, 414, 481, 556, 638, 729, 831, 945};
  static const uint8_t b[] ={33, 37, 38, 42,  45,  48,  52,  57,  62,  67,  75,  82,  91, 102, 114, 129};

  uint8_t i=(val>>6);
 
  return a[i] + (((val % 64) * b[i]) >> 6);
}
#endif

int16_t  MCP3204_SPI_read(uint8_t channel)
{
  int16_t val;
  
  digitalWriteFast(MCP3204_PIN_CS, 0);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0b00000110);
  val=SPI.transfer16(channel<<14) & 0x0FFF;
  SPI.endTransaction();
  digitalWriteFast(MCP3204_PIN_CS, 1);
  
  return val;
}


int16_t MCP3204_BB_read(uint8_t channel)
{
    #if (MCP3204_4W_PIN_MOSI==MCP3204_4W_PIN_MISO)
    pinModeFast(MCP3204_4W_PIN_MOSI, OUTPUT); 
    #endif
       
    digitalWriteFast(MCP3204_4W_PIN_MOSI, 1);
    
    //start bit
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);

    //single
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);

    //d2 
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);

    //d1 and d0
    if (channel & 0b00000010)
      {
        digitalWriteFast(MCP3204_4W_PIN_MOSI, 1);
      }
    else
      {
        digitalWriteFast(MCP3204_4W_PIN_MOSI, 0);
      }
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);

    if (channel & 0b00000001)
      {
        digitalWriteFast(MCP3204_4W_PIN_MOSI, 1);
      }
    else
      {
        digitalWriteFast(MCP3204_4W_PIN_MOSI, 0);
      }
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);

    //skip 2 clock
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);
    
    digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
    digitalWriteFast(MCP3204_4W_PIN_SCK, 0);

    #if (MCP3204_4W_PIN_MOSI==MCP3204_4W_PIN_MISO)
    pinModeFast(MCP3204_4W_PIN_MISO, INPUT);
    #endif
    
    //read 12bit answer
    int16_t data=0;
    int16_t b=0b0000100000000000;
    do{
      
       if (digitalReadFast(MCP3204_4W_PIN_MISO))
          data |= b;
      
       digitalWriteFast(MCP3204_4W_PIN_SCK, 1);
       digitalWriteFast(MCP3204_4W_PIN_SCK, 0);
    }while(b>>=1);
    
    return data;
}

#if (PEDALS_TYPE == PT_ADS1015)
void ADS1015_read()
{
  wheel.analogAxes[ADS1015_axis]->setValue(ads1015.read16());
  
  ADS1015_axis++;
  if (ADS1015_axis==3)
    ADS1015_axis=0;

  ads1015.requestADC(ADS1015_channels[ADS1015_axis]);
}
#endif
//-----------------------------------end analog axes------------------------------

//-----------------------------------reading buttons------------------------------
void readButtons()
{
  uint32_t buttons=0;
  bool changed; 
  uint8_t i;

  uint8_t* d;
  if (settings.debounce)
  {
    d=(uint8_t *)&buttons;
    changed=false;
  }
  else
  {
    wheel.buttons=0;
    d=(uint8_t *)&wheel.buttons;
    changed=true;
  }
  
#if BUTTONS_TYPE == BT_74HC165

  #ifdef HC165_PIN_PL
    digitalWriteFast(HC165_PIN_PL,1);  
  #endif  
   
  i=0x80;
  do
  {  
      digitalWriteFast(HC165_PIN_SCK, 1);
      digitalWriteFast(HC165_PIN_SCK, 0);  
           
      if (!digitalReadFast(HC165_PIN_DATA1))
         d[0]|=i; 
      if (!digitalReadFast(HC165_PIN_DATA2))
         d[2]|=i; 
  } while(i>>=1);
  i=0x80;
  do
  {   
      digitalWriteFast(HC165_PIN_SCK, 1);
      digitalWriteFast(HC165_PIN_SCK, 0);
      
      if (!digitalReadFast(HC165_PIN_DATA1))
         d[1]|=i; 
      if (!digitalReadFast(HC165_PIN_DATA2))
         d[3]|=i; 
  } while(i>>=1);

  #ifdef HC165_PIN_PL
    digitalWriteFast(HC165_PIN_PL,0);  
  #endif


#endif

#if BUTTONS_TYPE == BT_MCP23017
  mcp23017_1.read16((uint16_t *)d);
  mcp23017_2.read16((uint16_t *)(d+2));
  *((uint32_t *)d)=~*((uint32_t *)d);
#endif

//analog pin to buttons
#ifdef APB
  static const uint8_t apb_values[] ={APB_VALUES};
  static const uint8_t apb_btns[]  ={APB_BTNS};
  uint8_t apb_val=analogReadFast(APB_PIN)>>2;

  if (apb_out)
  {
    Serial.print(F("APB: "));
    Serial.println(apb_val);
  }
  
  for(i=0;i<APB_BTN_COUNT;i++)
      bitWrite(*((uint32_t *)d), apb_btns[i]-1, ((apb_val>apb_values[i]-APB_TOLERANCE) && (apb_val<apb_values[i]+APB_TOLERANCE)));
#endif


  //debounce
  if (settings.debounce)
  {
    if (tempButtons!=buttons)
    {
      debounceCount=0;
      tempButtons=buttons;
    }
    else
      if (debounceCount<settings.debounce)
        debounceCount++;
      else
      {
        wheel.buttons=buttons;
        changed=true;
      }
  }

  //center button processing
  if (changed)
  if (settings.centerButton!=-1)
  {
      bool state=(bitRead(*((uint32_t *)d), settings.centerButton));
      if ((!centerButtonState) && state) //avoid multiple triggering
          center();
      centerButtonState=state;
      bitClear(wheel.buttons, settings.centerButton);
  }

//analog shifter
#ifdef ASHIFTER
  uint8_t x=analogReadFast(ASHIFTER_PINX)>>2;
  uint8_t y=analogReadFast(ASHIFTER_PINY)>>2;
  uint8_t g;

  if (ashifter_out)
  {
    Serial.print(F("Analog shifter X:"));
    Serial.print(x);
    Serial.print(" Y:");
    Serial.println(y);
  }
  
  //clear bits
  #if ASHIFTER_POS == 8
  wheel.buttons&=~((uint32_t)0xff << (ASHIFTER_1ST_BTN-1));
  #endif
  #if ASHIFTER_POS == 6
  wheel.buttons&=~((uint32_t)0x3f << (ASHIFTER_1ST_BTN-1));
  #endif

  //set bits
  if (y<ASHIFTER_Y1)
    g=0b00000001;
  else
  if (y>ASHIFTER_Y2)
    g=0b00000010;
  else
    return;

  if (x>ASHIFTER_X1)
    g<<=2;
  if (x>ASHIFTER_X2)
    g<<=2;
  
  #if ASHIFTER_POS == 8
  if (x>ASHIFTER_X3)
    g<<=2;
  #endif
  
 wheel.buttons|=((uint32_t)g<<(ASHIFTER_1ST_BTN-1));
#endif

}
//---------------------------------------- end buttons ----------------------------------------------

//Centering wheel
void center()
{
  CENTER_WHEEL;
  wheel.axisWheel->center();
  Serial.println(F("Centered"));
}

//Serial port - commands and output.
void processSerial()
{

  //output axis data
  if (axisInfo==0)
  {
   
    Serial.print(F("Axis#0 Raw:"));
    Serial.print(wheel.axisWheel->rawValue);
    Serial.print(F("\tAbs: "));
    Serial.print(wheel.axisWheel->absValue);
    Serial.print(F("\tMax: "));
    Serial.print(wheel.axisWheel->axisMax);
    Serial.print(F("\tValue: "));
    Serial.print(wheel.axisWheel->value);
    Serial.print(F(" ("));
    Serial.print((int8_t)(wheel.axisWheel->value * (100.0 / 65534)) + 50);
    Serial.print(F("%)\tV:"));
    Serial.print(wheel.axisWheel->velocity);
    Serial.print(F("\tA:"));
    Serial.print(wheel.axisWheel->acceleration);
    Serial.print(F("\tFFB:"));
    Serial.println(force);

  }
  else
  if ((axisInfo>0)&&(axisInfo<=7))
  {
      Serial.print(F("Axis#"));
      Serial.print(axisInfo);
      Serial.print(F("\tRaw: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->rawValue);
      Serial.print(F("\tAutoLimit: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->autoLimit);
      Serial.print(F("\tAutoCenter: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->autoCenter);
      Serial.print(F("\tDeadZone: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->getDZ());
      Serial.print(F("\tMin: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->axisMin);
      Serial.print(F("\tCenter: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->getCenter());
      Serial.print(F("\tMax: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->axisMax);
      Serial.print(F("\tValue: "));
      Serial.print(wheel.analogAxes[axisInfo-1]->value);
      Serial.print(F(" ("));
      Serial.print((int8_t)(wheel.analogAxes[axisInfo-1]->value * (100.0 / 65534)) + 50);
      Serial.println(F("%)"));
   
 }

  if (Serial.available())
  {
      char cmd[16];
      uint8_t cmdLength;
      int32_t arg1,arg2,arg3;
    
      arg1=-32768;
      arg2=-32768;
      arg3=-32768;

      cmdLength=Serial.readBytesUntil(' ', cmd, 15);
      cmd[cmdLength]=0;

      if (Serial.available())
        arg1=Serial.parseInt(SKIP_WHITESPACE);
      if (Serial.available())
        arg2=Serial.parseInt(SKIP_WHITESPACE);
      if (Serial.available())
        arg3=Serial.parseInt(SKIP_WHITESPACE);

     if (strcmp_P(cmd, PSTR("fvaout"))==0)
     {
        fvaOut=!fvaOut;
        Serial.print(F("Force-velocity-acc output "));
        if (fvaOut)
          Serial.println(F("on"));
        else
          Serial.println(F("off"));
     }

      if (strcmp_P(cmd, PSTR("timing"))==0)
      {
        timing=!timing;
      }
      
      //center
      if (strcmp_P(cmd, PSTR("center"))==0)
          center();
          
      if (strcmp_P(cmd, PSTR("load"))==0)
        load();

      if (strcmp_P(cmd, PSTR("defaults"))==0)
        load(true);
      
      if (strcmp_P(cmd, PSTR("save"))==0)
        save();
        
     if (strcmp_P(cmd, PSTR("centerbtn"))==0)
     {
        if ((arg1>=0)&&(arg1<=32))
        {
          settings.centerButton=arg1-1;
        }
        Serial.print(F("Center button: "));
        Serial.println(settings.centerButton+1);
     }

     if (strcmp_P(cmd, PSTR("range"))==0)
     {
        if (arg1>0)
        {
          wheel.axisWheel->setRange(arg1);
        }
        Serial.print(F("Wheel range: "));
        Serial.println(wheel.axisWheel->range);
     }

     if (strcmp_P(cmd, PSTR("maxvd"))==0)
     {
        if (arg1>0)
        {
          wheel.ffbEngine.maxVelocityDamperC=16384.0/arg1;
        }
        Serial.print(F("max velocity damper: "));
        Serial.println(round(16384.0/wheel.ffbEngine.maxVelocityDamperC));
     }
     
     if (strcmp_P(cmd, PSTR("maxvf"))==0)
     {
        if (arg1>0)
        {
          wheel.ffbEngine.maxVelocityFrictionC=16384.0/arg1;
        }
        Serial.print(F("max velocity friction: "));
        Serial.println(round(16384.0/wheel.ffbEngine.maxVelocityFrictionC));
     }
     
     if (strcmp_P(cmd, PSTR("maxacc"))==0)
     {
        if (arg1>0)
        {
          wheel.ffbEngine.maxAccelerationInertiaC=16384.0/arg1;
        }
        Serial.print(F("max acceleration: "));
        Serial.println(round(16384.0/wheel.ffbEngine.maxAccelerationInertiaC));
     }

     if (strcmp_P(cmd, PSTR("forcelimit"))==0)
     {
        if ((arg1>=0)&&(arg1<=16383))
           settings.minForce=arg1;
        if ((arg2>=0)&&(arg2<=16383))
           settings.maxForce=arg2;
        if ((arg3>=0)&&(arg3<=16383))
           settings.cutForce=arg3;           

        Serial.print(F("MinForce: "));
        Serial.print(settings.minForce);
        Serial.print(F(" MaxForce: "));
        Serial.print(settings.maxForce);
        Serial.print(F(" CutForce: "));
        Serial.println(settings.cutForce);
     }

     if (strcmp_P(cmd, PSTR("gain"))==0)          
     {
         if ((arg1>=0)&&(arg1<=12))
         {
            if ((arg2>=0) && (arg2<=32767))
            {
                settings.gain[arg1]=arg2;
            }
            
            Serial.print(F("Gain "));
            Serial.print(arg1);
            Serial.print(" ");
            printEffect(arg1);
            Serial.print(F(": "));
            Serial.println(settings.gain[arg1]);
         }
     }

     //axisinfo <axis>
     if (strcmp_P(cmd, PSTR("axisinfo"))==0)
     {
       if ((arg1>=0)&&(arg1<=7))
           axisInfo=arg1;
       else
           axisInfo=-1;
     }
     
     //limits <axis> <min> <max>
     if (strcmp_P(cmd, PSTR("limit"))==0)
     if ((arg1>=1) && (arg1<=7))
     {
        if ((arg2>-32768)&&(arg3>-32768))
          wheel.analogAxes[arg1-1]->setLimits(arg2, arg3);
          
        Serial.print(F("Limits axis#"));
        Serial.print(arg1);
        Serial.print(F(": "));
        Serial.print(wheel.analogAxes[arg1-1]->axisMin);
        Serial.print(" ");
        Serial.println(wheel.analogAxes[arg1-1]->axisMax);
     }
     
     //axiscenter <axis> <pos>
     if (strcmp_P(cmd, PSTR("axiscenter"))==0)
     if ((arg1>=1) && (arg1<=7))
     {
        if (arg2>-32768)
          wheel.analogAxes[arg1-1]->setCenter(arg2);
        Serial.print(F("Axis#"));
        Serial.print(arg1);
        Serial.print(F(" center:"));
        Serial.println(wheel.analogAxes[arg1-1]->getCenter());
     }

     //axisdz <axis> <pos>
     if (strcmp_P(cmd, PSTR("axisdz"))==0)
     if ((arg1>=1) && (arg1<=7))
     {
        if (arg2>-32768)
          wheel.analogAxes[arg1-1]->setDZ(arg2);
        Serial.print(F("Axis#"));
        Serial.print(arg1);
        Serial.print(F(" Deadzone:"));
        Serial.println(wheel.analogAxes[arg1-1]->getDZ());
     }     

      //axisdisable <axis> <pos>
     if (strcmp_P(cmd, PSTR("axisdisable"))==0)
     if ((arg1>=1) && (arg1<=7))
     {
        wheel.analogAxes[arg1-1]->outputDisabled=!wheel.analogAxes[arg1-1]->outputDisabled;
        
        Serial.print(F("Axis#"));
        Serial.print(arg1);
        if (wheel.analogAxes[arg1-1]->outputDisabled)
          Serial.println(F(" disabled"));
        else
          Serial.println(F(" enabled"));
     }    
     
     //axistrim <axis> <level>
     if (strcmp_P(cmd, PSTR("axistrim"))==0)
     if ((arg1>=1) && (arg1<=7))
     {
        if ((arg2>=0) && (arg2<8))
          wheel.analogAxes[arg1-1]->bitTrim=arg2;
        
        Serial.print(F("Axis#"));
        Serial.print(arg1);
        Serial.print(F(" trim:"));
        Serial.println(wheel.analogAxes[arg1-1]->bitTrim);
     }   
     
     //autolimits <axis>
     if (strcmp_P(cmd, PSTR("autolimit"))==0)
     if ((arg1>=1) && (arg1<=7))
     {
          wheel.analogAxes[arg1-1]->setAutoLimits(!wheel.analogAxes[arg1-1]->autoLimit);
          Serial.print(F("Axis #"));
          Serial.print(arg1);
          Serial.print(F(" autolimit"));
          if (wheel.analogAxes[arg1-1]->autoLimit)
            Serial.println(F(" on"));
          else
            Serial.println(F(" off"));
     }

     //FFB PWM bitdepth/frequency
     if (strcmp_P(cmd, PSTR("ffbbd"))==0)
     {
       if (arg1>0)
          motor.setBitDepth(arg1);
       Serial.print(F("FFB Bitdepth:"));
       Serial.print(motor.bitDepth);
       Serial.print(F(" Freq:"));
       Serial.println(16000000 / ((uint16_t)1<<(motor.bitDepth+1)));
     }

     //Debounce
     if (strcmp_P(cmd, PSTR("debounce"))==0)
     {
       if (arg1>=0)
          settings.debounce=arg1;
       Serial.print(F("Debounce:"));
       Serial.println(settings.debounce);
     }

     //Endstop
     if (strcmp_P(cmd, PSTR("endstop"))==0)
     {
       if (arg1>=0)
          settings.endstopOffset=arg1;
       if (arg2>=0)
          settings.endstopWidth=arg2;
       Serial.print(F("Endstop: offset:"));
       Serial.print(settings.endstopOffset);
       Serial.print(F(" width:"));
       Serial.println(settings.endstopWidth);
     }

#ifdef APB
     if (strcmp_P(cmd, PSTR("apbout"))==0)
     {
        apb_out=!apb_out;
     }
#endif     

#ifdef ASHIFTER
     if (strcmp_P(cmd, PSTR("ahsout"))==0)
     {
        ashifter_out=!ashifter_out;
     }
#endif
  }
}

//load and save settings
void load(bool defaults)
{
  SettingsEEPROM settingsE;
  uint8_t i;
  uint8_t checksum;
  EEPROM.get(0, settingsE);
  
  checksum=settingsE.calcChecksum();
  
  //Loading defaults
  if (defaults || (settingsE.checksum!=checksum))
  {

    Serial.println(F("Loading defaults"));
    
    for(i=0;i<13;i++)
      settingsE.data.gain[i]=1024;
      
    settingsE.range=WHEEL_RANGE_DEFAULT;

    settingsE.data.centerButton = -1; //no center button
    
    for(i=0;i<7;i++)
    {
      if (i<3)
      {
        settingsE.axes[i].axisMin = DEFAULT_AA_MIN;
        settingsE.axes[i].axisMax = DEFAULT_AA_MAX;
      }
      else
      {
        settingsE.axes[i].axisMin = 0;
        settingsE.axes[i].axisMax = 1023;
      }
      settingsE.axes[i].axisCenter=-32768; //no center
      settingsE.axes[i].axisDZ=0;

      settingsE.axes[i].axisOutputDisabled=0;
      settingsE.axes[i].axisBitTrim=0;
    }
    settingsE.data.debounce=0;
    settingsE.data.minForce=0;
    settingsE.data.maxForce=16383;
    settingsE.data.cutForce=16383;
    settingsE.ffbBD=DEFAULT_FFB_BITDEPTH;
    
    settingsE.maxVelocityDamper=DEFAULT_MAX_VELOCITY;
    settingsE.maxVelocityFriction=DEFAULT_MAX_VELOCITY;
    settingsE.maxAcceleration=DEFAULT_MAX_ACCELERATION;

    settingsE.data.endstopOffset=DEFAULT_ENDSTOP_OFFSET;
    settingsE.data.endstopWidth=DEFAULT_ENDSTOP_WIDTH;
  }
  
  settingsE.print();

  settings=settingsE.data;

  for(i=0;i<7;i++)
  {
    wheel.analogAxes[i]->setLimits(settingsE.axes[i].axisMin, settingsE.axes[i].axisMax);
    wheel.analogAxes[i]->setCenter(settingsE.axes[i].axisCenter);
    if (!wheel.analogAxes[i]->autoCenter)
       wheel.analogAxes[i]->setDZ(settingsE.axes[i].axisDZ);

    wheel.analogAxes[i]->bitTrim=settingsE.axes[i].axisBitTrim;
    wheel.analogAxes[i]->outputDisabled=settingsE.axes[i].axisOutputDisabled;
  }
    
  wheel.axisWheel->setRange(settingsE.range);
  
  wheel.ffbEngine.maxVelocityDamperC=16384.0/settingsE.maxVelocityDamper;
  wheel.ffbEngine.maxVelocityFrictionC=16384.0/settingsE.maxVelocityFriction;
  wheel.ffbEngine.maxAccelerationInertiaC=16384.0/settingsE.maxAcceleration;

  Serial.println(F("Settings loaded"));
}


void save()
{
    SettingsEEPROM settingsE;
    uint8_t i;

    settingsE.data=settings;

    settingsE.range=wheel.axisWheel->range;

    for(i=0;i<7;i++)
    {
      settingsE.axes[i].axisMin = wheel.analogAxes[i]->axisMin;
      settingsE.axes[i].axisMax = wheel.analogAxes[i]->axisMax;
      if (!wheel.analogAxes[i]->autoCenter)
        settingsE.axes[i].axisCenter = wheel.analogAxes[i]->getCenter();
      else
        settingsE.axes[i].axisCenter = -32768;
      settingsE.axes[i].axisDZ = wheel.analogAxes[i]->getDZ();

      settingsE.axes[i].axisBitTrim=wheel.analogAxes[i]->bitTrim;
      settingsE.axes[i].axisOutputDisabled=wheel.analogAxes[i]->outputDisabled;
    }

    settingsE.maxVelocityDamper=round(16384.0/wheel.ffbEngine.maxVelocityDamperC);
    settingsE.maxVelocityFriction=round(16384.0/wheel.ffbEngine.maxVelocityFrictionC);
    settingsE.maxAcceleration=round(16384.0/wheel.ffbEngine.maxAccelerationInertiaC);
    
    settingsE.checksum=settingsE.calcChecksum();
    
    EEPROM.put(0,settingsE);
    
    Serial.println(F("Settings saved"));
}
