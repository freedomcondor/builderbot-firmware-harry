#ifndef PCA9633_MODULE_H
#define PCA9633_MODULE_H

#include <stdint.h>

class CPCA9633Module {
public:

   CPCA9633Module(uint8_t un_device_address) :
      m_unDeviceAddress(un_device_address) {}

   void Init();

   enum class ELEDMode : uint8_t {
      OFF  = 0x00,
      ON   = 0x01,
      PWM  = 0x02,
      BLINK = 0x03,
   };

   static void ResetDevices();

   void SetLEDMode(uint8_t un_led, ELEDMode e_mode);

   void SetLEDBrightness(uint8_t un_led, uint8_t un_val);

   void SetGlobalBlinkRate(uint8_t un_period, uint8_t un_duty_cycle);

private:
   uint8_t m_unDeviceAddress;

   enum class ERegister : uint8_t {
      MODE1          = 0x00,
      MODE2          = 0x01,
      PWM0           = 0x02,
      PWM1           = 0x03,
      PWM2           = 0x04,
      PWM3           = 0x05,
      GRPPWM         = 0x06,
      GRPFREQ        = 0x07,
      LEDOUT         = 0x08,
      SUBADR1        = 0x09,
      SUBADR2        = 0x0A,
      SUBADR3        = 0x0B,
      ALLCALLADR     = 0x0C
   };
};

#endif
