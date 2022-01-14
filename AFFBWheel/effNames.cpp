#include "effNames.h"

void printEffect(uint8_t e)
{
  switch(e)
  {
    case 0:
      Serial.print(F("Total"));
      break;
    case 1:
      Serial.print(F("Const"));
      break;
    case 2:
      Serial.print(F("Ramp"));
      break;
    case 3:
      Serial.print(F("Square"));
      break;
    case 4:
      Serial.print(F("Sine"));
      break;
    case 5:
      Serial.print(F("Triangle"));
      break;
    case 6:
      Serial.print(F("ST Down"));
      break;
    case 7:
      Serial.print(F("ST Up"));
      break;
    case 8:
      Serial.print(F("Spring"));
      break;
    case 9:
      Serial.print(F("Damper"));
      break;
    case 10:
      Serial.print(F("Inertia"));
      break;
    case 11:
      Serial.print(F("Friction"));
      break;
    case 12:
      Serial.print(F("Endstop"));
      break;    
  }
}
