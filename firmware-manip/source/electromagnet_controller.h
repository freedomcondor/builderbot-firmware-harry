#ifndef ELECTROMAGNET_CONTROLLER_H
#define ELECTROMAGNET_CONTROLLER_H

#define COILS_REG_EN 0x08
#define COILS_CTRL_A 0x01
#define COILS_CTRL_B 0x02

#include <avr/io.h>
#include <adc_controller.h>

class CElectromagnetController {

public:

   enum EDischargeMode {
      CONSTRUCTIVE,
      DESTRUCTIVE,
      DISABLE
   };

CElectromagnetController() {
      /* Initially disable the regulator for charging the capacitors */
      PORTC |= COILS_REG_EN;
      DDRC &= ~COILS_REG_EN;

      /* Initially put the coil driver in sleep mode */
      PORTB &= ~(COILS_CTRL_A | COILS_CTRL_B);
      DDRB |= (COILS_CTRL_A | COILS_CTRL_B);
   }

   uint8_t GetAccumulatedVoltage() {
      return CADCController::GetInstance().GetValue(CADCController::EChannel::ADC7);
   }

   void SetChargeEnable(bool b_charge_enable) {
      if(b_charge_enable) {
         DDRC |= COILS_REG_EN;
      } else {
         DDRC &= ~COILS_REG_EN;
      }
   }

   void SetDischargeMode(EDischargeMode e_discharge_mode) {
      /* Default, dissconnect the coils, power down the driver */
      PORTB &= ~(COILS_CTRL_A | COILS_CTRL_B);
      /* If constructive or destructive mode was selected, enable the driver */
      switch(e_discharge_mode) {
      case EDischargeMode::CONSTRUCTIVE:
         PORTB |= COILS_CTRL_B;
         break;
      case EDischargeMode::DESTRUCTIVE:
         PORTB |= COILS_CTRL_A;
         break;
      case EDischargeMode::DISABLE:
         break;
      }    
   }

};

#endif
