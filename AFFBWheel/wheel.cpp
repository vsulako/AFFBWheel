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
}


int16_t Wheel_::update(void) 
{
  
  wheelData data;

  data.axes[0]=axisWheel->value;

  data.axes[1]=analogAxes[AXIS_ACC]->value;
  data.axes[2]=analogAxes[AXIS_BRAKE]->value;
  data.axes[3]=analogAxes[AXIS_CLUTCH]->value;

  data.axes[4]=analogAxes[AXIS_AUX1]->value;
  data.axes[5]=analogAxes[AXIS_AUX2]->value;

  data.buttons=buttons;

  HID().RecvFfbReport();
  HID().SendReport(0x01, &data, sizeof(data));
}
