#pragma once

/* HID descriptor */
static const uint8_t wheelHIDDescriptor[] PROGMEM = {
  0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,          // USAGE (Joystick)
  0xA1, 0x01,          // COLLECTION (Application)
    //================================Input Report======================================//
    //0x09, 0x01,             // USAGE (Pointer)
    // WheelReport
    //0x85, 0x01,             // REPORT_ID (1)
    0xA1, 0x00,             // COLLECTION (Physical)
    
      //8 Axis for steering wheel, accelerator, brake, clutch, handbrake and spare
      0x05, 0x01,               // USAGE_PAGE (Generic Desktop)
      0xa1, 0x00,               // COLLECTION (Physical)
      0x09, 0x30,                   // USAGE (X)  -     g25 wheel
      0x09, 0x32,                   // USAGE (Z)  -     g25 z
      0x09, 0x31,                   // USAGE (Y)        g25 comb
      0x09, 0x33,                   // USAGE (Rx)       g25 rx
      0x09, 0x34,                   // USAGE (Ry)       g25 ry
      0x09, 0x35,                   // USAGE (Rz)       g25 brake
      0x09, 0x36,                   // USAGE (Slider)   g25 clutch
      0x09, 0x37,                   // USAGE (Dial)     g25 acc
      
      0x16, 0x00, 0x80,             // LOGICAL_MINIMUM (-32768)
      0x26, 0xFF, 0x7F,             // LOGICAL_MAXIMUM (32767)
      0x75, 0x10,                   // REPORT_SIZE (16)
      0x95, 0x08,                   // REPORT_COUNT (8)
      0x81, 0x02,                   // INPUT (Data,Var,Abs)
      0xc0,                     // END_COLLECTION 
  
      //32 buttons
      0x05, 0x09,             // USAGE_PAGE (Button)
      0x19, 0x01,             // Usage Minimum (1),
      0x29, 0x20,             // Usage Maximum (32)
      0x15, 0x00,             // LOGICAL_MINIMUM (0)
      0x25, 0x01,             // Logical Maximum (1),
      0x35, 0x00,             // PHYSICAL_MINIMUM (0) 
      0x45, 0x01,             // Physical Maximum (1),
      0x95, 0x20,             // Report Count (32 fields),
      0x75, 0x01,             // Report Size (1 bit),
      0x81, 0x02,             // INPUT (Data,Var,Abs) 

      //Hat switch
      0x05, 0x01,             //        Usage Page (Desktop),
      0x95, 0x01,             //        Report Count (1 field),
      0x75, 0x04,             //        Report Size (4 bit),
      0x25, 0x07,             //        Logical Maximum (7),
      0x46, 0x3B, 0x01,       //        Physical Maximum (315),
      //0x65, 0x14,             //        Unit (Degrees),
      0x09, 0x39,             //        Usage (Hat Switch),
      0x81, 0x42,             //        >>>> Input (Variable)

      //padding 4bits
      0x75, 0x04,             // REPORT_SIZE (06)
      0x95, 0x01,             // REPORT_COUNT (01)
      0x81, 0x03,             // Input (Constant, Variable)
      
    0xc0,                   // END_COLLECTION

        //Logitech FFB
        0xA1, 0x00,       //    Collection (Logical),
            //0x85, 0x01,            //   REPORT_ID (1)
            0x05, 0x01,       //        Usage Page (Desktop),   
            0x09, 0x02,       //        Usage (02h),
            0x75, 0x08,       //        Report Size (8 bit),   
            0x95, 0x07,       //        Report Count (7 fields),
            
            0x14,             //        Logical Minimum (0),
            0x26, 0xFF, 0x00, //        Logical Maximum (255),
            0x34,             //        Physical Minimum (0),
            0x46, 0xFF, 0x00, //        Physical Maximum (255),
            
            0x91, 0x02,       //            <<<< Output (Variable), *** FFB & Control ***
        0xC0,             //    End Collection, 

  0xC0, // END COLLECTION ()
};

/*
 Драйвер G25 что-то делает с осями:
  последняя по порядку ось, если это не Y, переименовывается в Accelerator 
  ось Rz переименовывается в Brake
  ось Slider переименовывается в Clutch
  ось Y переименовывается в Combined Pedals
  3я по порядку ось становится невидимой в некоторых программах
  а иногда бывает иначе, зависимость не ясна

наблюдения:

порядковый номер оси, обозначение в дескрипторе, как видится в VKB JoyTester, как видится в свойствах джойстика
d=Dial, s=slider, w=Wheel, z? - ось невидима

0     x     x     w
1     d     y     d
2     rz    rz    brake
3     s     z?    clutch
4     y     d     comb
5     rx    rx    rx 
6     ry    ry    ry
7     z     z     acc

0     x     x     w
1     z     y     z
2     rz    rz    brake
3     s     z?    clutch
4     y     s2    comb
5     rx    rx    rx
6     ry    ry    ry
7     d     s1    acc

0     x     x     w
1     y     y     comb
2     z     rz    z
3     rx    z?    rx
4     ry    s2    ry 
5     rz    rx    brake
6     s     ry    clutch
7     d     s1    acc

0     x     x     w
1     rz    rz    brake
2     s     z?    s
3     y     d     
4     z     y     clutch
5     rx    rx    rx
6     ry    ry    ry
7     d     s     acc


0     rx    x     rx
1     ry    rz    ry
2     x     z?    wheel
3     rz    d     brake
4     s     y     s
5     y     rx    comb
6     z     ry    clutch
7     d     s     acc

0     x     x     w
1     rz    rz    brake
2     y     z?    comb
3     s     d     s
4     rx    y     rx
5     ry    rx    ry
6     z     ry    clutch
7     d     s     acc

0     x     x     w
1     y     rz    comb
2     z     z?    z
3     rx    d     rx
4     ry    y     ry
5     rz    rx    brake
6     s     ry    clutch
7     d     s     acc


0     d     x     w
1     s     rz    comb
2     rz    z?    z
3     ry    d     rx
4     rx    y     ry
5     z     rx    brake
6     y     ry    clutch
7     x     s     acc

0     x     x     w
1     d     z?    d
2     s     d     clutch
3     rz    rz    brake
4     ry    ry    ry
5     rx    rx    rx
6     z     y     acc
7     y     s     comb

0     x     x     w
1     rx    rx    rx
2     ry    ry    ry
3     rz    rz    brake
4     d     z?    d
5     s     d     clutch
6     z     y     acc
7     y     s     comb

0     x           w
1     rz          brake
2     rx          rx
3     ry          ry
4     s           s
5     d           clutch
6     y           comb
7     z           acc

1     z     y     z
2     y           comb
3     rx    rx    rx
4     ry    ry    ry
5     rz    rz    brake
6     s     d     clutch
7     d     s     acc

 */
