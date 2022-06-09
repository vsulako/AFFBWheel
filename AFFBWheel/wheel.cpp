#include "wheel.h"

Wheel_::Wheel_(void)
{
  static HIDSubDescriptor node(wheelHIDDescriptor, sizeof(wheelHIDDescriptor));
  HID().AppendDescriptor(&node);
  
  ffbEngine.SetFfb(&HID().ffbReportHandler);

  axisWheel=new AxisWheel();

  analogAxes[AXIS_ACC]=new Axis(MA_LEVEL_AXIS_ACC);
  analogAxes[AXIS_BRAKE]=new Axis(MA_LEVEL_AXIS_BRAKE);
  analogAxes[AXIS_CLUTCH]=new Axis(MA_LEVEL_AXIS_CLUTCH);
  analogAxes[AXIS_AUX1]=new Axis(MA_LEVEL_AXIS_AUX1);
  analogAxes[AXIS_AUX2]=new Axis(MA_LEVEL_AXIS_AUX2);
  analogAxes[AXIS_AUX3]=new Axis(MA_LEVEL_AXIS_AUX3);
  analogAxes[AXIS_AUX4]=new Axis(MA_LEVEL_AXIS_AUX4);
}


void Wheel_::update(void) 
{
  
  wheelData data;

  data.axes[0]=axisWheel->value;

  int8_t i;
  for(i=0;i<7;i++)
  {
    if (!analogAxes[i]->outputDisabled)
      data.axes[i+1]=analogAxes[i]->value;
  }
  /*
  data.axes[1]=analogAxes[AXIS_ACC]->value;
  data.axes[2]=analogAxes[AXIS_BRAKE]->value;
  data.axes[3]=analogAxes[AXIS_CLUTCH]->value;

  data.axes[4]=analogAxes[AXIS_AUX1]->value;
  data.axes[5]=analogAxes[AXIS_AUX2]->value;
  data.axes[6]=analogAxes[AXIS_AUX3]->value;
  data.axes[7]=analogAxes[AXIS_AUX4]->value;
*/
#ifdef HATSWITCH
  data.hat=getHatSwitch();
#endif
  
  data.buttons=buttons;

  HID().RecvFfbReport();
  HID().SendReport(0x01, &data, sizeof(data));

  if (USB_GUI_Report.command)
  {
    HID().SendReport(16, &USB_GUI_Report, sizeof(USB_GUI_Report));
    USB_GUI_Report.command=0;
  }
}



//Обработка Hatswitch
/* Направления */
#define HAT_UP          0b00000000  //(0)
#define HAT_UP_RIGHT    0b00000001  //(1)
#define HAT_RIGHT       0b00000010  //(2)
#define HAT_DOWN_RIGHT  0b00000011  //(3)
#define HAT_DOWN        0b00000100  //(4)
#define HAT_DOWN_LEFT   0b00000101  //(5)
#define HAT_LEFT        0b00000110  //(6)
#define HAT_UP_LEFT     0b00000111  //(7)
#define HAT_CENTER      0b00001000  //(8)
  
uint8_t Wheel_::getHatSwitch()
{
  uint8_t hat=0;
  
  bitWrite(hat, 0, bitRead(buttons, HAT_BTN_UP-1));
  bitWrite(hat, 1, bitRead(buttons, HAT_BTN_DOWN-1));
  bitWrite(hat, 2, bitRead(buttons, HAT_BTN_LEFT-1));
  bitWrite(hat, 3, bitRead(buttons, HAT_BTN_RIGHT-1));

  #ifdef HAT_CLR_BTNS
  bitClear(buttons, HAT_BTN_UP-1);
  bitClear(buttons, HAT_BTN_DOWN-1);
  bitClear(buttons, HAT_BTN_LEFT-1);
  bitClear(buttons, HAT_BTN_RIGHT-1);
  #endif
  
  switch (hat)
  {
     case 0b00000001:
        hat = HAT_UP;
        break;
     case 0b00000010:
        hat = HAT_DOWN;
        break;
     case 0b00000100:
        hat = HAT_LEFT;
        break;
    case 0b00001000:
        hat = HAT_RIGHT;
        break;
     case 0b00000101:
        hat = HAT_UP_LEFT;
        break;
     case 0b00001001:
        hat = HAT_UP_RIGHT;
        break;
     case 0b00000110:
        hat = HAT_DOWN_LEFT;
        break;
    case 0b00001010:
        hat = HAT_DOWN_RIGHT;
        break;    
    default:
        hat = HAT_CENTER;
  };

  return hat;
}
