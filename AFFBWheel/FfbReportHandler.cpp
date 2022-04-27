/*
  Force Feedback Joystick
  Joystick model specific code for handling force feedback data.
  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen
  Copyright 2016  Jaka Simonic    (telesimke [at] gmail [dot] com)
  Copyright 2019  Hoan Tran (tranvanhoan206 [at] gmail [dot] com)
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

#include "FfbReportHandler.h"
FfbReportHandler::FfbReportHandler() {
  nextEID = 1;
  devicePaused = 0;
  deviceGain=255;
}

FfbReportHandler::~FfbReportHandler() {
  FreeAllEffects();
}

uint8_t FfbReportHandler::GetNextFreeEffect(void)
{
  if (nextEID == MAX_EFFECTS)
    return 0;

  uint8_t id = nextEID++;

  // Find the next free effect ID for next time
  //nextEID=0;
  while (gEffectStates[nextEID].state != 0)
  {
    if (nextEID >= MAX_EFFECTS)
      break;  // the last spot was taken
    nextEID++;
  }

  gEffectStates[id].state = MEFFECTSTATE_ALLOCATED;

  return id;
}

void FfbReportHandler::StopAllEffects(void)
{
  for (uint8_t id = 0; id <= MAX_EFFECTS; id++)
    StopEffect(id);
}

void FfbReportHandler::StartEffect(uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state = MEFFECTSTATE_PLAYING;
  gEffectStates[id].elapsedTime = 0;
}

void FfbReportHandler::StopEffect(uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state &= ~MEFFECTSTATE_PLAYING;
  pidBlockLoad.ramPoolAvailable += SIZE_EFFECT;
}

void FfbReportHandler::FreeEffect(uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state = 0;
  if (id < nextEID)
    nextEID = id;
}

void FfbReportHandler::FreeAllEffects(void)
{
  nextEID = 1;
  memset((void*)&gEffectStates, 0, sizeof(gEffectStates));
  pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

void FfbReportHandler::FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t* data)
{
  if (data->operation == 1)
  { // Start
    if (data->loopCount > 0) gEffectStates[data->effectBlockIndex].duration *= data->loopCount;
    if (data->loopCount == 0xFF) gEffectStates[data->effectBlockIndex].duration = USB_DURATION_INFINITE;
    StartEffect(data->effectBlockIndex);
  }
  else if (data->operation == 2)
  { // StartSolo

    // Stop all first
    StopAllEffects();

    // Then start the given effect
    StartEffect(data->effectBlockIndex);
  }
  else if (data->operation == 3)
  { // Stop

    StopEffect(data->effectBlockIndex);
  }
  else
  {
  }
}

void FfbReportHandler::FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t* data)
{
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF)
  { // all effects
    FreeAllEffects();
  }
  else
  {
    FreeEffect(eid);
  }
}

void FfbReportHandler::FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t* data)
{
  uint8_t control = data->control;

  if (control == 0x01)
  { // 1=Enable Actuators
    pidState.status |= 2;
  }
  else if (control == 0x02)
  { // 2=Disable Actuators
    pidState.status &= ~(0x02);
  }
  else if (control == 0x03)
  { // 3=Stop All Effects
    StopAllEffects();
  }
  else if (control == 0x04)
  { //  4=Reset
    FreeAllEffects();
  }
  else if (control == 0x05)
  { // 5=Pause
    devicePaused = 1;
  }
  else if (control == 0x06)
  { // 6=Continue
    devicePaused = 0;
  }
  else if (control & (0xFF - 0x3F))
  {
  }
}

void FfbReportHandler::FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t* data)
{
   deviceGain = data->gain;
}

void FfbReportHandler::FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t* data)
{
}
void FfbReportHandler::FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t* data)
{
}
void FfbReportHandler::FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t* data)
{
}

void FfbReportHandler::FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t* data)
{
  volatile TEffectState* effect = &gEffectStates[data->effectBlockIndex];

  effect->duration = data->duration;
  effect->directionX = data->directionX;
  effect->directionY = data->directionY;
  effect->effectType = data->effectType;
  effect->gain = data->gain;
  effect->enableAxis = data->enableAxis;

  
  if (effect->period==0)
    effect->period=1;
    
  switch (effect->effectType)
  {
      case USB_EFFECT_RAMP:
        effect->periodC = 2.0 / effect->duration;
        break;
      case USB_EFFECT_SINE:
        effect->periodC = 65535.0 / effect->period;
        break;
      case USB_EFFECT_TRIANGLE: 
        effect->periodC = 4.0 / effect->period;
        break;
      case USB_EFFECT_SAWTOOTHUP:
      case USB_EFFECT_SAWTOOTHDOWN:
        effect->periodC = 2.0 / effect->period;
        break;
  }

  if (effect->fadeTime || effect->attackTime)
  {
    if (effect->fadeTime > effect->duration - effect->attackTime)
      effect->fadeTime = effect->duration - effect->attackTime;
  
     effect->fadeTimeC=1.0/effect->fadeTime;
     effect->attackTimeC=1.0/effect->attackTime;
  }
 
}

void FfbReportHandler::SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->attackLevel = abs(data->attackLevel);
  effect->fadeLevel = abs(data->fadeLevel);
  effect->attackTime = data->attackTime;
  effect->fadeTime = data->fadeTime;
  
  if (effect->fadeTime || effect->attackTime)
  {
    if (effect->duration)
    if (effect->fadeTime > effect->duration - effect->attackTime)
      effect->fadeTime = effect->duration - effect->attackTime;
  
    effect->attackTimeC=1.0/effect->attackTime;
    effect->fadeTimeC=1.0/effect->fadeTime;
  }

}

void FfbReportHandler::SetCondition(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect)
{
  if ((data->parameterBlockOffset & 0x0F) !=0) //ignore Y axis
    return;
  effect->cpOffset = data->cpOffset;
  effect->positiveCoefficient = data->positiveCoefficient;
  effect->negativeCoefficient = data->negativeCoefficient;
  effect->positiveSaturation = data->positiveSaturation;
  effect->negativeSaturation = data->negativeSaturation;
  effect->deadBand = data->deadBand;

}

void FfbReportHandler::SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect)
{

  effect->magnitude = data->magnitude;
  effect->offset = data->offset;

  effect->period = data->period;
  effect->halfPeriod = data->period>>1;
  
  effect->periodTime=(uint32_t)data->period * data->phase * (1.0 / 36000);

  switch (effect->effectType)
  {
      case USB_EFFECT_SINE:
        effect->periodC = 65535.0 / data->period;
        break;
      case USB_EFFECT_TRIANGLE: 
        effect->periodC = 4.0 / data->period;
        break;
      case USB_EFFECT_SAWTOOTHUP:
      case USB_EFFECT_SAWTOOTHDOWN:
        effect->periodC = 2.0 / data->period;
        break;
  }

}

void FfbReportHandler::SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->magnitude = data->magnitude;
}

void FfbReportHandler::SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->offset=((int32_t)data->endMagnitude + data->startMagnitude) >> 1;
  effect->magnitude=((int32_t)data->endMagnitude - data->startMagnitude) >> 1;
}

void FfbReportHandler::FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData)
{
  pidBlockLoad.reportId = 6;
  pidBlockLoad.effectBlockIndex = GetNextFreeEffect();

  if (pidBlockLoad.effectBlockIndex == 0)
  {
    pidBlockLoad.loadStatus = 2;    // 1=Success,2=Full,3=Error
  }
  else
  {
    pidBlockLoad.loadStatus = 1;    // 1=Success,2=Full,3=Error

    volatile TEffectState* effect = &gEffectStates[pidBlockLoad.effectBlockIndex];

    memset((void*)effect, 0, sizeof(TEffectState));
    effect->state = MEFFECTSTATE_ALLOCATED;
    pidBlockLoad.ramPoolAvailable -= SIZE_EFFECT;
  }
}

uint8_t* FfbReportHandler::FfbOnPIDPool()
{
  FreeAllEffects();

  pidPoolReport.reportId = 7;
  pidPoolReport.ramPoolSize = MEMORY_SIZE;
  pidPoolReport.maxSimultaneousEffects = MAX_EFFECTS;
  pidPoolReport.memoryManagement = 3;
  return (uint8_t*)&pidPoolReport;
}

uint8_t* FfbReportHandler::FfbOnPIDBlockLoad()
{
  return (uint8_t*)&pidBlockLoad;
}

uint8_t* FfbReportHandler::FfbOnPIDStatus()
{
  return (uint8_t*)&pidState;
}


void FfbReportHandler::FfbOnUsbData(uint8_t* data, uint16_t len)
{
  uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.
  switch (data[0])    // reportID
  {
    case 1:
      FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t*)data);
      break;
    case 2:
      SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case 3:
      SetCondition((USB_FFBReport_SetCondition_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case 4:
      SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case 5:
      SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case 6:
      SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case 7:
      //FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*)data);
      break;
    case 8:
      //FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*)data);
      break;
    case 9:
      break;
    case 10:
      FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*)data);
      break;
    case 11:
      FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t*)data);
      break;
    case 12:
      FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*)data);
      break;
    case 13:
      FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*)data);
      break;
    case 14:
      //FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*)data);
      break;
    case 15:
      //commands from GUI
      usbCommand=*((USB_GUI_Command*)data);
      break;
    default:
      break;
  }
};
