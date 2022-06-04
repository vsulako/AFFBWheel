#include "logitechFFB.h"

  void LogitechFFB::downloadForce(uint8_t* data, uint8_t slot, volatile TEffectState* effect)
  {
      effect->gain = 255;
      effect->duration = USB_DURATION_INFINITE;
//      effect->state= MEFFECTSTATE_ALLOCATED;
      
        switch (data[1])
        {
            case LOGITECH_FORCE_VARIABLE:
                 setForceVariable(data, slot, effect);
                break;
          
            case LOGITECH_FORCE_CONSTANT:
            
                effect->effectType=USB_EFFECT_CONSTANT;
                effect->magnitude=convertForce(data[slot+1]);
                Serial.print(F("Constant magn="));
                Serial.print(effect->magnitude);
                Serial.println();
                
                break;
            case LOGITECH_FORCE_SPRING:
            case LOGITECH_FORCE_AUTOCENTSPRING:
            case LOGITECH_FORCE_HIRESSPRING:
            case LOGITECH_FORCE_HIRESACSPRING:  
                setForceSpring(data, effect);   //Springs
                break;
            case LOGITECH_FORCE_DAMPER:
            case LOGITECH_FORCE_HIRESDAMPER:  //Damper
                setForceDamper(data, effect);
                break;
            case LOGITECH_FORCE_SAWTOOTHUP:
            case LOGITECH_FORCE_SAWTOOTHDOWN:
                setForceST(data, effect);
                break;
            case LOGITECH_FORCE_TRAPEZOID:
                setForceTrapezoid(data, effect);
                break;
            case LOGITECH_FORCE_RECTANGLE:
                setForceRectangle(data, effect);
                break;
            case LOGITECH_FORCE_SQUARE:
                setForceSquare(data, effect);
                break;
            case LOGITECH_FORCE_FRICTION:  //Friction
                setForceFriction(data, effect);
                break;
        }
  }


  int16_t LogitechFFB::KFactor(uint8_t K, bool hires)
  {
      static const int8_t _k[] = {1,2,3,4,6,8,12,16};
 
      if (hires)
           return  (K+1)<<10;
      else
           return ((int16_t)_k[K]<<10);
  }
  int16_t LogitechFFB::convertForce(uint8_t f)
  {
      /*
       сила направлена обратно: 16383 - максимум усилия против часовой
       0..127-128..255 => -16383..0..16383
      */
      if (f<127)
        return (-((127-f)<<7) + 127); 
      if (f>128)
        return ((f-128)<<7) + 127;
      return 0;
  }
  int16_t LogitechFFB::convertPos(uint8_t f)
  {
      /*
       255 и 16383 - максимум поворота по часовой
       0..127-128..255 => -16383..0..16383
       127 => -127
       128 => 127
       0 должен быть между 127 и 128.
      */
      
      if (f<=127)
        return (-((127-f)<<7) + 127); 
      if (f>=128)
        return ((f-128)<<7) + 127;
      return 0;
  }

  

  void LogitechFFB::setForceSpring(uint8_t* data, volatile TEffectState* effect)
  {
      uint8_t D1,D2,K1,K2,S1,S2,Clip;
      bool hires=false;

      switch(data[1])
      {
          case LOGITECH_FORCE_SPRING:
          case LOGITECH_FORCE_HIRESSPRING:
              D1=data[2];
              D2=data[3];
              K1=data[4]&0b00001111;
              K2=data[4]>>4;
              S1=data[5]&0b00000001;
              S2=data[5]&0b00010000;
              Clip=data[6];

              if (D1>D2)  //на всякий случай
              {
                  D1=data[3];
                  D2=data[2];
              }
              break;
          case LOGITECH_FORCE_AUTOCENTSPRING:
          case LOGITECH_FORCE_HIRESACSPRING: 
          case LOGITECH_FORCE_DEFAULTSPRING:
              D1=126;
              D2=129;
              K1=data[2]&0b00001111;
              K2=data[3]&0b00001111;
              S1=0;
              S2=0;
              Clip=data[4];
              break;
          
      }
         
      if ((data[1]==LOGITECH_FORCE_HIRESSPRING)||(data[1]==LOGITECH_FORCE_HIRESACSPRING))
              hires=true;
      
      Serial.print("Spring D1:");
      Serial.print(D1);
      Serial.print(" D2:");
      Serial.print(D2);
      Serial.print(" K1:");
      Serial.print(K1);
      Serial.print(" K2:");
      Serial.print(K2);
      Serial.print(" S1:");
      Serial.print(S1);
      Serial.print(" S2:");
      Serial.print(S2);
      Serial.print(" Clip");
      Serial.print(Clip);
      Serial.print(" hires:");
      Serial.print(hires);
      Serial.println();
      
          /*  
              1) границы мертвой зоны D1 и D2 драйвер всегда устанавливает в 0 (левый максимум), что не имеет смысла.
              поэтому в этом случае считаем что это центрующие пружины
              2) условное значение силы в любую сторону по протоколу не более 127, но ограничитель Clip ставится от 0 до 255.
          */
          if ((!D1)&&(!D2))
          {
              D1=127;
              D2=128;
          }

          
          //трансляция в обычный эффект.
          effect->effectType=USB_EFFECT_SPRING;
      
          int16_t _D1=convertPos(D1);
          int16_t _D2=convertPos(D2);

          int16_t _Clip=((int16_t)Clip<<6) | 0x3f; //масштабирование  0-255 => 0-16383
          
          effect->cpOffset = (_D2 + _D1)>>1;
          effect->deadBand = (_D2 - _D1)>>1;
          
          effect->positiveSaturation = _Clip;
          effect->negativeSaturation = _Clip;

          effect->negativeCoefficient = (S1?-1:1) * KFactor(K1, hires);
          effect->positiveCoefficient = (S2?-1:1) * KFactor(K2, hires);

           Serial.print("cpOff:");
           Serial.print(effect->cpOffset);

           Serial.print("db:");
           Serial.print(effect->deadBand);

           Serial.print("negSat:");
           Serial.print(effect->negativeSaturation);
           Serial.print("posSat:");
           Serial.print(effect->positiveSaturation);

           Serial.print("negC:");
           Serial.print(effect->negativeCoefficient);
           Serial.print("posC:");
           Serial.print(effect->positiveCoefficient);

           Serial.println();
  }

  void LogitechFFB::setForceDamper(uint8_t* data, volatile TEffectState* effect)
  {
        uint8_t K1,K2,S1,S2;
        bool hires=false;
  
        K1=data[2]&0b00001111;
        K2=data[4]&0b00001111;
        S1=data[3]&0b00000001;
        S2=data[5]&0b00000001;
        if (data[1]==LOGITECH_FORCE_HIRESDAMPER)
            hires=true;
            
      Serial.print(F("Damper "));
      Serial.print(" K1:");
      Serial.print(K1);
      Serial.print(" K2:");
      Serial.print(K2);
      Serial.print(" S1:");
      Serial.print(S1);
      Serial.print(" S2:");
      Serial.print(S2);
      Serial.print(" hires:");
      Serial.print(hires);
      Serial.println();
      
        effect->effectType=USB_EFFECT_DAMPER;
    
        effect->cpOffset = 0;
        effect->deadBand = 0;
        
        effect->positiveSaturation = 16384;
        effect->negativeSaturation = 16384;

        effect->negativeCoefficient = (S1?-1:1) * KFactor(K1, hires);
        effect->positiveCoefficient = (S2?-1:1) * KFactor(K2, hires);

       Serial.print("cpOff:");
       Serial.print(effect->cpOffset);

       Serial.print("db:");
       Serial.print(effect->deadBand);

       Serial.print("negSat:");
       Serial.print(effect->negativeSaturation);
       Serial.print("posSat:");
       Serial.print(effect->positiveSaturation);

       Serial.print("negC:");
       Serial.print(effect->negativeCoefficient);
       Serial.print("posC:");
       Serial.print(effect->positiveCoefficient);

       Serial.println();        
  }

    void LogitechFFB::setForceFriction(uint8_t* data, volatile TEffectState* effect)
  {
        uint8_t K1,K2,S1,S2,Clip;
  
        K1=data[2];
        K2=data[3];
        Clip=data[4];
        S2=data[5]&0b00010000;
        S1=data[5]&0b00000001;
        
        Serial.print("Friction ");
        Serial.print(" K1:");
        Serial.print(K1);
        Serial.print(" K2:");
        Serial.print(K2);
        Serial.print(" S1:");
        Serial.print(S1);
        Serial.print(" S2:");
        Serial.print(S2);
        Serial.print(" Clip:");
        Serial.print(Clip);
        Serial.println();
      
        effect->effectType=USB_EFFECT_FRICTION;
    
        effect->cpOffset = 0;
        effect->deadBand = 0;

        int16_t _Clip=((int16_t)Clip<<6) | 0x3f;
        
        effect->positiveSaturation = effect->negativeSaturation = _Clip;

        effect->negativeCoefficient = (S1?-1:1) * KFactor(K1, false);
        effect->positiveCoefficient = (S2?-1:1) * KFactor(K2, false);

       Serial.print("cpOff:");
       Serial.print(effect->cpOffset);

       Serial.print("db:");
       Serial.print(effect->deadBand);

       Serial.print("negSat:");
       Serial.print(effect->negativeSaturation);
       Serial.print("posSat:");
       Serial.print(effect->positiveSaturation);

       Serial.print("negC:");
       Serial.print(effect->negativeCoefficient);
       Serial.print("posC:");
       Serial.print(effect->positiveCoefficient);
  }

  void LogitechFFB::setForceVariable(uint8_t* data, uint8_t slot, volatile TEffectState* effect)
  {
      uint8_t L,T,S,D;

      /*
          на деле всегда устанавливается только параметр L1, L2 всегда 128, остальные 0
          то есть сила Variable работает как постоянная
      */
       switch (slot)
       {
          case 1:
              L=data[2];
              T=data[4]>>4;
              S=data[4]&0b00001111;
              D=data[6]&0b00000001;
              break;
          case 3:
              L=data[3];
              T=data[5]>>4;
              S=data[5]&0b00001111;
              D=data[6]&0b00010000;
       }

        Serial.print(F("Variable "));
        Serial.print(" L:");
        Serial.print(L);
        Serial.print(" T:");
        Serial.print(T);
        Serial.print(" S:");
        Serial.print(S);
        Serial.print(" D:");
        Serial.print(D);
        Serial.println();       

      //Если T или S равны 0, это равносильно Constant
      if ((!T)||(!S))
      {
          effect->effectType=USB_EFFECT_CONSTANT;
          effect->magnitude=convertForce(L);

          Serial.print("magn: ");
          Serial.print(effect->magnitude);
          Serial.println();
          
          return;
      }

      //эффект - начинает с L, и растет/снижается пока не упрется в край, и дальше там остается.
      effect->effectType=USB_EFFECT_LOGITECH_VARIABLE;
      
      effect->offset=convertForce(L);

      if (D)
      {
        effect->period=(int16_t)L * T * 2 / S;
        effect->magnitude=-16383 - effect->offset;
        effect->periodC=(float)effect->magnitude / effect->period;
      }
      else
      {
        effect->period=(int16_t)(255-L) * T * 2 / S;
        effect->magnitude=16383 - effect->offset;
        effect->periodC=(float)effect->magnitude / effect->period;
      }
      
      Serial.print("per: ");
      Serial.print(effect->period);
      Serial.print("off: ");
      Serial.print(effect->offset);
      Serial.print("magn: ");
      Serial.print(effect->magnitude);
      Serial.println();
  }

  void LogitechFFB::setForceST(uint8_t* data, volatile TEffectState* effect)
  {
      uint8_t  L1,L2,Inc,T3,L0;
    
      L1=data[2];
      L2=data[3];
      Inc=data[6]&0b00001111;
      T3=data[6]>>4;
      L0=data[4];
  
      //коррекция на всякий случай
      if (L1<L2)
      {
          L2=data[2];
          L1=data[3];
      }
        
        Serial.print(F("Sawtooth L0:"));
        
        Serial.print(L0);
        Serial.print(" L1:");
        Serial.print(L1);
        Serial.print(" L2:");
        Serial.print(L2);
        Serial.print(" Inc:");
        Serial.print(Inc);
        Serial.print(" T3:");
        Serial.print(T3);
        Serial.println();   
          
      if (data[1]==0x05)
      {
          effect->effectType=USB_EFFECT_SAWTOOTHDOWN;
          effect->periodTime=(L1-L0) * T3 *2 / Inc;
      }
      else
      {
          effect->effectType=USB_EFFECT_SAWTOOTHUP;
          
          effect->periodTime=(L0-L2) * T3 *2 / Inc;
      }

      int16_t _L1=convertForce(L1);
      int16_t _L2=convertForce(L2);
      
      effect->magnitude = ( _L1-_L2 ) >>1;
      effect->offset = (_L1 + _L2 ) >>1;

      effect->period = (int16_t)(L1-L2) * T3 * 2 / Inc;
      effect->periodC = 2.0 / effect->period;
/*
      Serial.print("period: ");
      Serial.print(effect->period);
      Serial.print("period: ");
      Serial.print(effect->periodTime);
      Serial.print("offset: ");
      Serial.print(effect->offset);
      Serial.print("magnitude: ");
      Serial.print(effect->magnitude);
      Serial.println();
*/      
  }

  void LogitechFFB::setForceTrapezoid(uint8_t* data, volatile TEffectState* effect)
  {
      uint8_t  L1,L2,T1,T2,T3,S;
    
    L1=data[2];
    L2=data[3];
    T1=data[4];
    T2=data[5];
    T3=data[6]>>4;
    S=data[6]&0b00001111;

    //коррекция на всякий случай
    if (L1<L2)
    {
        L2=data[2];
        L1=data[3];
    }
        
        Serial.print(F("trapezoid L1:"));
        
        Serial.print(L1);
        Serial.print(" L2:");
        Serial.print(L2);
        Serial.print(" T1:");
        Serial.print(T1);
        Serial.print(" T2:");
        Serial.print(T2);
        Serial.print(" T3:");
        Serial.print(T3);
        Serial.print(" S:");
        Serial.print(S);
        Serial.println();   
        

        effect->effectType=USB_EFFECT_LOGITECH_TRAPEZOID;
        effect->periodTime=0;


      int16_t _L1=convertForce(L1);
      int16_t _L2=convertForce(L2);
      
      effect->magnitude = _L1-_L2;
      effect->offset = _L2;

      effect->halfPeriod = (int16_t)(L1-L2) * T3 * 2 / S;
      effect->period = (effect->halfPeriod + T1 + T2) * 2;
      
      effect->deadBand=  (int16_t)T1 * 2;
      effect->cpOffset=  (int16_t)T2 * 2;
/*  
      Serial.print("period: ");
      Serial.print(effect->period);
      Serial.print("offset: ");
      Serial.print(effect->offset);
      Serial.print("magnitude: ");
      Serial.print(effect->magnitude);
      Serial.println();
*/      
  }
  
void LogitechFFB::setForceRectangle(uint8_t* data, volatile TEffectState* effect)
  {
      uint8_t  L1,L2,T1,T2,P;
    
    L1=data[2];
    L2=data[3];
    T1=data[4];
    T2=data[5];
    P=data[6];

    //коррекция на всякий случай
    if (L1<L2)
    {
        L2=data[2];
        L1=data[3];
    }
        
        Serial.print(F("rectangle L1:"));
        Serial.print(L1);
        Serial.print(" L2:");
        Serial.print(L2);
        Serial.print(" T1:");
        Serial.print(T1);
        Serial.print(" T2:");
        Serial.print(T2);
        Serial.print(" P:");
        Serial.print(P);
        Serial.println();   
          

        effect->effectType=USB_EFFECT_SQUARE;
        effect->periodTime=P*2;

      int16_t _L1=convertForce(L1);
      int16_t _L2=convertForce(L2);
      
      effect->magnitude = (_L1 - _L2) >> 1;
      effect->offset = (_L1 + _L2) >> 1;

      effect->period = (int16_t)(T1 + T2) * 2;
      effect->halfPeriod = T1 * 2;

      /*
      
      Serial.print("period: ");
      Serial.print(effect->period);
      Serial.print("period: ");
      Serial.print(effect->halfPeriod);
      Serial.print("offset: ");
      Serial.print(effect->offset);
      Serial.print("magnitude: ");
      Serial.print(effect->magnitude);
      Serial.println();
      */
  }


void LogitechFFB::setForceSquare(uint8_t* data, volatile TEffectState* effect)
  {
      uint8_t  Amp,N;
      uint16_t T;
      
      Amp=data[2];
      T =(uint16_t)data[4]<<8 & data[3];
      N=data[6];

      Serial.print(F("Square  Amp:"));
      Serial.print(Amp);
      Serial.print(" N:");
      Serial.print(N);
      Serial.print(" T:");
      Serial.print(T);
      Serial.println();   
      
      effect->effectType=USB_EFFECT_SQUARE;
      effect->periodTime=0;

      effect->magnitude = ((int16_t)Amp << 6) | 0x7F;
      effect->offset = 0;

      effect->halfPeriod = min(T, 16383);
      effect->period = effect->halfPeriod << 1;
      /*    

      Serial.print("period: ");
      Serial.print(effect->period);
      Serial.print("period: ");
      Serial.print(effect->halfPeriod);
      Serial.print("offset: ");
      Serial.print(effect->offset);
      Serial.print("magnitude: ");
      Serial.print(effect->magnitude);
      Serial.println();
      */
  }
  
