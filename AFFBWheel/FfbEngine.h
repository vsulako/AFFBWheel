/*
  Force Feedback Joystick Math
  Joystick model specific code for calculating force feedback.
  Copyright 2016  Jaka Simonic
  Copyright 2019  hoantv
  Copyright 2021  Sulako
  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.
  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/
#pragma once

#include <Arduino.h>
#include "HIDReportType.h"
#include "FfbReportHandler.h"
#include "movavg.h"
#include "axis.h"
#include "settings.h"
#include "trig_fixed.h"

#define GAIN_TOTAL        0x00
#define GAIN_CONSTANT     USB_EFFECT_CONSTANT
#define GAIN_RAMP         USB_EFFECT_RAMP
#define GAIN_SQUARE       USB_EFFECT_SQUARE
#define GAIN_SINE         USB_EFFECT_SINE
#define GAIN_TRIANGLE     USB_EFFECT_TRIANGLE
#define GAIN_SAWTOOTHDOWN USB_EFFECT_SAWTOOTHDOWN
#define GAIN_SAWTOOTHUP   USB_EFFECT_SAWTOOTHUP
#define GAIN_SPRING       USB_EFFECT_SPRING
#define GAIN_DAMPER       USB_EFFECT_DAMPER
#define GAIN_INERTIA      USB_EFFECT_INERTIA
#define GAIN_FRICTION     USB_EFFECT_FRICTION
#define GAIN_ENDSTOP      0x0c

#define sign(x) ((x > 0) - (x < 0))

extern SettingsData settings;

class FfbEngine {
  public:
    FfbEngine();
    void SetFfb(FfbReportHandler* reporthandler);
    FfbReportHandler* ffbReportHandler;
    
    int16_t calculateForce(AxisWheel* axis);
    
    int16_t constantForce(volatile TEffectState*  effect);
    int16_t rampForce(volatile TEffectState*  effect);
    int32_t periodicForce(volatile TEffectState*  effect);
    int16_t envelope(volatile TEffectState* effect);
    
    int16_t sinefix(volatile TEffectState*  effect, int16_t magnitude);

    int16_t square(volatile TEffectState*  effect, int16_t magnitude);
    int16_t triangle(volatile TEffectState*  effect, int32_t magnitude);
    int16_t stdown(volatile TEffectState*  effect, int32_t magnitude);
    int16_t stup(volatile TEffectState*  effect, int32_t magnitude);
    
    int16_t springForce(volatile TEffectState*  effect, int16_t position);
    int16_t damperForce(volatile TEffectState*  effect, int16_t velocity);
    int16_t inertiaForce(volatile TEffectState*  effect, AxisWheel* axis);
    int16_t frictionForce(volatile TEffectState*  effect, int16_t velocity);
    
    float maxVelocityDamperC, maxVelocityFrictionC, maxAccelerationInertiaC;
    
  private:
    int16_t prevTime;
};

int16_t applyGain(int32_t force, int16_t gain);
