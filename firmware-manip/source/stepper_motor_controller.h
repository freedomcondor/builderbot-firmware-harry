#ifndef STEPPER_MOTOR_CONTROLLER_H
#define STEPPER_MOTOR_CONTROLLER_H

/* Port D */
#define STM_CHA_MASK  0x20
#define STM_CHB_MASK  0x40

/* Port C */
#define MTR_REG_EN    0x04

#include <stdint.h>

class CStepperMotorController {

public:

   enum ERotationDirection {
      FORWARD,
      REVERSE
   };

   CStepperMotorController();
   
   void SetHalfPeriod(uint8_t un_half_period) {
      m_unHalfPeriod = un_half_period;
   }

   uint8_t GetHalfPeriod() {
      return m_unHalfPeriod;
   }
      
   void SetRotationDirection(ERotationDirection e_rotation_direction) {
      m_eRotationDirection = e_rotation_direction;
   }

   ERotationDirection GetRotationDirection() {
      return m_eRotationDirection;
   }

   void Enable();

   void Disable();

   void UpdateWaveform();
   
   bool IsWaveformActive();
   
private:

   uint8_t m_unHalfPeriod;

   ERotationDirection m_eRotationDirection;
};


#endif
