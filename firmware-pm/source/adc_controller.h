#ifndef ADC_CONTROLLER_H
#define ADC_CONTROLLER_H

#include <stdint.h>

class CADCController {
public:

   enum class EChannel {
      ADC0 = 0x00,
      ADC1 = 0x01,
      ADC2 = 0x02,
      ADC3 = 0x03,
      ADC4 = 0x04,
      ADC5 = 0x05,
      ADC6 = 0x06,
      ADC7 = 0x07,
      TEMP = 0x08,
      AREF = 0x0E,
      GND = 0x0F
   };

public:
   uint8_t GetValue(EChannel e_channel);

   static CADCController& GetInstance();

private:

   /* singleton instance */
   static CADCController m_cADCControllerInstance;

   CADCController();
};

#endif
