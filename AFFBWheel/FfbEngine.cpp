/*
  Force Feedback Joystick Math
  Joystick model specific code for calculating force feedback.
  Copyright 2016  Jaka Simonic
  Copyright 2019  hoantv
  Copyright 2022  Sulako
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

#include "FfbEngine.h"
#include "HIDReportType.h"

FfbEngine::FfbEngine() {

}

void FfbEngine::SetFfb(FfbReportHandler* reporthandler) {
  ffbReportHandler = reporthandler;
}


//returns 15-bit force value in range [-16384..16384]
int16_t FfbEngine::calculateForce(AxisWheel* axis)
{
  int32_t totalForce = 0;
  int16_t tmpForce;

  volatile TEffectState* effect;

  int16_t _millis=millis();
  int16_t timeDiff=_millis-prevTime;
  prevTime=_millis;

  for (uint8_t id = 0; id <= MAX_EFFECTS; id++)
  {
    effect = &ffbReportHandler->gEffectStates[id];
    tmpForce=0;

    //stop effect if it reached duration
    if ((effect->state & MEFFECTSTATE_PLAYING) && !ffbReportHandler->devicePaused)
    {
      
      if ((effect->duration != USB_DURATION_INFINITE) && (effect->elapsedTime+timeDiff > effect->duration))
      {
        ffbReportHandler->StopEffect(id);
      }
      else
      {
        if (effect->duration != USB_DURATION_INFINITE)
          effect->elapsedTime += timeDiff;
          
        switch (effect->effectType)
        {
          case USB_EFFECT_CONSTANT:
            tmpForce = constantForce(effect);
            break;
          case USB_EFFECT_RAMP:
            effect->periodTime += timeDiff;
            tmpForce = rampForce(effect);
            break;
          case USB_EFFECT_SQUARE:
          case USB_EFFECT_SINE:
          case USB_EFFECT_TRIANGLE:
          case USB_EFFECT_SAWTOOTHDOWN:
          case USB_EFFECT_SAWTOOTHUP:
            effect->periodTime += timeDiff;
            tmpForce = periodicForce(effect);
            break;
          case USB_EFFECT_SPRING:
            tmpForce = springForce(effect, axis->value);
            break;
          case USB_EFFECT_DAMPER:
            tmpForce = damperForce(effect, axis->velocity);
            break;
          case USB_EFFECT_INERTIA:
            tmpForce = inertiaForce(effect, axis);
            break;
          case USB_EFFECT_FRICTION:
            tmpForce = frictionForce(effect, axis->velocity);
            break;
        }
        
        //applying effect gains
        if ((effect->gain==0)||(settings.gain[effect->effectType]==0))
          tmpForce = 0;
        else
        {
          
          if (effect->gain!=255)
            tmpForce = ((int32_t)tmpForce * (effect->gain+1)) >> 8;

          if (settings.gain[effect->effectType]!=1024)
            tmpForce = applyGain(tmpForce, settings.gain[effect->effectType]);

          tmpForce = constrain(tmpForce, -16383, 16383);
        }

        totalForce+=tmpForce;
        
      }  
    }
  }

  //actuators disabled
  if (!(ffbReportHandler->pidState.status & 2 ) || (!totalForce))
    return 0;

  //applying global gains
  if ((ffbReportHandler->deviceGain==0) || (settings.gain[GAIN_TOTAL]==0))
    return 0;
  else
  {
    if (ffbReportHandler->deviceGain!=255)
      totalForce = (totalForce * (ffbReportHandler->deviceGain +1)) >> 8;

    if (settings.gain[GAIN_TOTAL]!=1024)
      totalForce = applyGain(totalForce, settings.gain[GAIN_TOTAL]);
  }

  return constrain(totalForce, -16383, 16383);
}

int16_t FfbEngine::constantForce(volatile TEffectState*  effect)
{
    return  envelope(effect);
}

int16_t FfbEngine::rampForce(volatile TEffectState*  effect)
{
  int16_t magnitude=envelope(effect);
  
  if (effect->duration==USB_DURATION_INFINITE)
  {
    return effect->offset - magnitude;
  }
  else
  {
    return effect->offset + (int32_t)((int32_t)magnitude * effect->elapsedTime * effect->periodC ) - magnitude;
  }
}

int32_t FfbEngine::periodicForce(volatile TEffectState* effect)
{
  while (effect->periodTime>effect->period)
    effect->periodTime-=effect->period;
    
  int16_t magnitude=envelope(effect);
  
  switch (effect->effectType)
  {
    case USB_EFFECT_SQUARE:
      return effect->offset + square(effect, magnitude);
    case USB_EFFECT_SINE:
      return effect->offset + sinefix(effect, magnitude);
    case USB_EFFECT_TRIANGLE:
      return effect->offset + triangle(effect, magnitude);
    case USB_EFFECT_SAWTOOTHDOWN:
      return effect->offset + stdown(effect, magnitude);
    case USB_EFFECT_SAWTOOTHUP:
      return effect->offset - stdown(effect, magnitude);
  }

  return 0;
}

int16_t FfbEngine::envelope(volatile TEffectState* effect)
{
    if (effect->attackTime)
    if (effect->elapsedTime < effect->attackTime)
    {
       return sign(effect->magnitude) *(effect->attackLevel + (int32_t)(abs(effect->magnitude) - (int16_t)effect->attackLevel) * (int16_t)effect->elapsedTime * effect->attackTimeC);
    }

    if (effect->duration!=USB_DURATION_INFINITE)
    if (effect->fadeTime)
    if (effect->elapsedTime > effect->duration - effect->fadeTime)
    {
       return sign(effect->magnitude) * (effect->fadeLevel + (int32_t)(abs(effect->magnitude) - (int16_t)effect->fadeLevel) * (int16_t)(effect->duration-effect->elapsedTime) * effect->fadeTimeC);
    }

    return effect->magnitude;
}
  
int16_t FfbEngine::square(volatile TEffectState*  effect, int16_t magnitude)
{
    if (effect->periodTime < effect->halfPeriod)
      return magnitude;
    else
      return -magnitude;
}

int16_t FfbEngine::sinefix(volatile TEffectState*  effect, int16_t magnitude)
{
    return (int32_t)magnitude * sin_fix(effect->periodTime * effect->periodC) >>14;
}

int16_t FfbEngine::triangle(volatile TEffectState*  effect, int32_t magnitude)
{
    if (effect->periodTime < effect->halfPeriod)
      return -magnitude + (int32_t)((int32_t)magnitude * effect->periodTime * effect->periodC);
    else
      return 3 * magnitude - (int32_t)((int32_t)magnitude * effect->periodTime * effect->periodC);
}

int16_t FfbEngine::stdown(volatile TEffectState*  effect, int32_t magnitude)
{
    return magnitude - (int32_t)((int32_t)magnitude * effect->periodTime * effect->periodC);
} 

int16_t FfbEngine::springForce(volatile TEffectState*  effect, int16_t position) 
{
  int32_t  tempForce;

  position = position >>1;

  if (position < (effect->cpOffset - (int16_t)effect->deadBand)) 
  {
    tempForce = ((position - (effect->cpOffset - (int32_t)effect->deadBand)) * effect->negativeCoefficient) >> 14;
    tempForce = constrain(tempForce, -(int16_t)effect->negativeSaturation, effect->negativeSaturation);
  }
  else 
  if (position > (effect->cpOffset + (int16_t)effect->deadBand)) 
  {
    tempForce = ((position - (effect->cpOffset + (int32_t)effect->deadBand)) * effect->positiveCoefficient) >> 14;
    tempForce = constrain(tempForce, -(int16_t)effect->positiveSaturation, effect->positiveSaturation);
  }
  else
    return 0;
  
  return tempForce;
}


/*
 * damper, inertia, friction 
 * 
 * Про эти эффекты внятного описания найти не удалось
 * возможно, реализация неверна
 * 
 * Could not find any decent information about these effects
 * implementation may be incorrect.
 */

/*
 * damper creates force agains velocity. 
 * Maximum value for velocity is needed for scaling, so it is set with maxVelocityDamper
 */
int16_t FfbEngine::damperForce(volatile TEffectState*  effect, int16_t velocity) 
{
    int32_t  tempForce;

    velocity= velocity * maxVelocityDamperC;
    
    if (velocity < (effect->cpOffset - (int16_t)effect->deadBand)) 
    {
        tempForce = ((velocity - (effect->cpOffset - (int32_t)effect->deadBand)) * effect->negativeCoefficient) >>14;
        tempForce = constrain(tempForce, -(int16_t)effect->negativeSaturation, effect->negativeSaturation);
    }
    else 
    if (velocity > (effect->cpOffset + (int16_t)effect->deadBand)) 
    {
        tempForce = ((velocity - (effect->cpOffset + (int32_t)effect->deadBand)) * effect->positiveCoefficient)>>14;
        tempForce = constrain(tempForce, -(int16_t)effect->positiveSaturation, effect->positiveSaturation);
    }
    else
      return 0;

    return tempForce;
}


/*
 * Inertia effect should oppose acceleration.
 * maximum value for acceleration is set with maxAccelerationInertia
*/
int16_t FfbEngine::inertiaForce(volatile TEffectState*  effect, AxisWheel* axis) 
{
    int32_t  tempForce;
    
    //inertia effect works only for deceleration
    if (sign(axis->acceleration)==sign(axis->velocity))
      return 0;
      
    int16_t acceleration = axis->acceleration * maxAccelerationInertiaC;
   
    if (acceleration < (effect->cpOffset - (int16_t)effect->deadBand)) 
    {
        tempForce = ((acceleration - (effect->cpOffset - (int32_t)effect->deadBand)) * effect->negativeCoefficient)>>14;

        //Saturation ignored if is 0
        if (effect->negativeSaturation)
          tempForce = constrain(tempForce, -(int16_t)effect->negativeSaturation, effect->negativeSaturation);
    }
    else 
    if (acceleration > (effect->cpOffset + (int16_t)effect->deadBand)) 
    {
        tempForce = ((acceleration - (effect->cpOffset + (int32_t)effect->deadBand)) * effect->positiveCoefficient)>>14;
      
        if (effect->positiveSaturation)
          tempForce = constrain(tempForce, -(int16_t)effect->positiveSaturation, effect->positiveSaturation);
    }
    else
      return 0;

    return tempForce;
}



/*
 * Friction effect must create constant force against movement.
 * At tiny values of velocity constant force causes overshoot and wheel jerking, so a small slope is added
 * again, maximum value for velocity is set with maxVelocityFriction
 * 
 * cpOffset and saturation ignored
 */
int16_t FfbEngine::frictionForce(volatile TEffectState*  effect, int16_t velocity) 
{
       
    if (velocity==0)
      return 0;
   
    velocity = velocity * maxVelocityFrictionC;

    int16_t coefficient,tVelocity;

    if (velocity>0)
    {
      coefficient=effect->positiveCoefficient;
      tVelocity=velocity-effect->deadBand;
    }
    else
    {
      coefficient=-effect->negativeCoefficient;
      tVelocity=-velocity-effect->deadBand;
    }

    if (tVelocity > 256)
    {
      return coefficient;
    }
    else
    if (tVelocity>0)
    {
      return ((int32_t)tVelocity * coefficient) >> 8;
    }

    return 0;
}

int16_t applyGain(int32_t force, int16_t gain)
{
    force=force * gain;
    force=constrain(force, -16383L<<10, 16383L<<10);
    return force >> 10;
}
