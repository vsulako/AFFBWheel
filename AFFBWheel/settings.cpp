#include "settings.h"

uint8_t  SettingsEEPROM::calcChecksum()
{
  uint8_t i, checksum;
  
  checksum=~((uint8_t*)this)[0];
  for(i=1;i<sizeof(SettingsEEPROM)-1;i++)
    checksum=checksum ^ ~((uint8_t*)this)[i];
  return checksum;
}

void SettingsEEPROM::print()
{
  uint8_t i;

  Serial.print(F("Range: "));
  Serial.println(range);
  
  Serial.println(F("Gains:"));
  for(i=0;i<13;i++)
  {
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    printEffect(i);
    Serial.print(F(": "));
    Serial.print(data.gain[i]);
    Serial.println();
  }
  
  for(i=0;i<7;i++)
  {
    Serial.print(F("Axis "));
    Serial.print(i+1);
    Serial.print(F(" Min:"));
    Serial.print(axes[i].axisMin);
    Serial.print(F(" Max:"));
    Serial.print(axes[i].axisMax);
    Serial.print(F(" Center:"));
    Serial.print(axes[i].axisCenter);
    Serial.print(F(" DZ:"));
    Serial.println(axes[i].axisDZ);
  }

  Serial.print(F("Center button: "));
  Serial.println(data.centerButton+1);

  Serial.print(F("Debounce: "));
  Serial.println(data.debounce);
  
  Serial.print(F("minForce: "));
  Serial.println(data.minForce);
  Serial.print(F("maxForce: "));
  Serial.println(data.maxForce);
  Serial.print(F("cutForce: "));
  Serial.println(data.cutForce);

  Serial.print(F("FFB Bitdepth: "));
  Serial.println(ffbBD);

  Serial.print(F("FFB maxVelocityDamper: "));
  Serial.println(maxVelocityDamper);
  Serial.print(F("FFB maxVelocityFriction: "));
  Serial.println(maxVelocityFriction);
  Serial.print(F("FFB maxAcceleration: "));
  Serial.println(maxAcceleration);
  
  Serial.print(F("Checksum: "));
  Serial.println(checksum);
}
