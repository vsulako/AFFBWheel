#include "multiturn.h"


#define MT_PPR_Q (1<<(STEER_BITDEPTH-2))

int32_t MultiTurn::setValue(int16_t value)
  {
    int16_t val=value;

    if ((prevVal>MT_PPR_Q) && (val<-MT_PPR_Q))
      turns++;
    if ((val>MT_PPR_Q) && (prevVal<-MT_PPR_Q))
      turns--;
      
    prevVal=val; 

    return ((int32_t)turns<<STEER_BITDEPTH) + prevVal - zeroPosition;
  }

int32_t MultiTurn::getValue()
{
  return ((int32_t)turns<<STEER_BITDEPTH) + prevVal - zeroPosition;
}
  
void MultiTurn::zero()
{
  zeroPosition=prevVal;
  turns=0;
}
