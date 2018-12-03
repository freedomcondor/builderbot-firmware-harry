#ifndef RF_CONTROLLER_H
#define RF_CONTROLLER_H

#include <stdint.h>

class CRFController {
public:
   enum class ENumberOfSamples : uint8_t {
      N1   = 0,
      N2   = 1,
      N4   = 2,
      N8   = 3,
      N16  = 4,
      N32  = 5,
      N64  = 6,
      N128 = 7
   };
   
   enum class EDeviceType {
      VCNL4000,
      VCNL4010
   } m_eDeviceType;

   bool Probe();

   void Configure(ENumberOfSamples e_num_samples = ENumberOfSamples::N32,
                  uint8_t un_led_current = 20);

   uint16_t ReadProximity();

   uint16_t ReadAmbient();

};

#endif

