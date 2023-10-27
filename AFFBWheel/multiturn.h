#pragma once
#include "Arduino.h"
#include <stdint.h>
#include "config.h"

class MultiTurn
  {
      private:
        int16_t turns;
        int16_t prevVal;
        int16_t zeroPosition=0;        
      public:
        int32_t setValue(int16_t value);
        int32_t getValue();
        void zero();
        void setPosition(int32_t value);
  };
