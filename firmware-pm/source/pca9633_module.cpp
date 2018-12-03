
#include "pca9633_module.h"

#include <firmware.h>

#define PCA9633_RST_ADDR  0x03 
#define PCA9633_RST_BYTE1 0xA5
#define PCA9633_RST_BYTE2 0x5A

#define PCA9633_LEDOUTX_MASK 0x03

void CPCA9633Module::ResetDevices() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(PCA9633_RST_ADDR);
   CFirmware::GetInstance().GetTWController().Write(PCA9633_RST_BYTE1);
   CFirmware::GetInstance().GetTWController().Write(PCA9633_RST_BYTE2);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);   
}

void CPCA9633Module::Init() {
   /* Wake up the internal oscillator, disable group addressing and auto-increment */
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::MODE1));
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   /* Enable group blinking */
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::MODE2));
   CFirmware::GetInstance().GetTWController().Write(0x25);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   /* Default blink configuration 1s period, 50% duty cycle */
   SetGlobalBlinkRate(0x18, 0x80);

   /* Default all leds to full brightness */
   for(uint8_t unLED = 0; unLED < 4; unLED++) {
      SetLEDBrightness(unLED, 0xFF);
   }
}

void CPCA9633Module::SetLEDMode(uint8_t un_led, ELEDMode e_mode) {
   /* read current register value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::LEDOUT));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(m_unDeviceAddress, 1, true);
   uint8_t unRegisterVal = CFirmware::GetInstance().GetTWController().Read();
   /* clear and set target bits in unRegisterVal */
   unRegisterVal &= ~(PCA9633_LEDOUTX_MASK << ((un_led % 4) * 2));
   unRegisterVal |= (static_cast<uint8_t>(e_mode) << ((un_led % 4) * 2));
   /* write back */
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::LEDOUT));
   CFirmware::GetInstance().GetTWController().Write(unRegisterVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

void CPCA9633Module::SetLEDBrightness(uint8_t un_led, uint8_t un_val) {
   /* get the register responsible for LED un_led */
   uint8_t unRegisterAddr = static_cast<uint8_t>(ERegister::PWM0) + un_led;
   /* write value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(unRegisterAddr);
   CFirmware::GetInstance().GetTWController().Write(un_val);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

void CPCA9633Module::SetGlobalBlinkRate(uint8_t un_period, uint8_t un_duty_cycle) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::GRPFREQ));
   CFirmware::GetInstance().GetTWController().Write(un_period);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unDeviceAddress);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::GRPPWM));
   CFirmware::GetInstance().GetTWController().Write(un_duty_cycle);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}
