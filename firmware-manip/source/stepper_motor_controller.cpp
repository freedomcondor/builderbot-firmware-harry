
#include "stepper_motor_controller.h"

#include <avr/io.h>
#include <firmware.h>

/***********************************************************/
/***********************************************************/

CStepperMotorController::CStepperMotorController() :
   m_unHalfPeriod(0),
   m_eRotationDirection(ERotationDirection::FORWARD) {
   /* Stop the counter (prescaler to zero) */
   TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
   /* Set the count to zero */
   TCNT0 = 0;
   /* Put the counter in CTC mode and connect the output ports in toggle mode */
   TCCR0A |= ((1 << WGM01) | (1 << COM0A0) | (1 << COM0B0));
   /* Set the output drivers in output mode */
   DDRD |= (STM_CHA_MASK | STM_CHB_MASK);
   /* Initially disable the regulator */
   PORTC &= ~MTR_REG_EN;
   /* Set the regulator enable signal as output */
   DDRC |= MTR_REG_EN;
}

/***********************************************************/
/***********************************************************/

void CStepperMotorController::Enable() {
   /* Enable the regulator */
   PORTC |= MTR_REG_EN;
}

/***********************************************************/
/***********************************************************/

void CStepperMotorController::Disable() {
   /* Disable the regulator */
   PORTC &= ~MTR_REG_EN;
   /* Stop the counter (prescaler to zero) */
   TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
   /* Set the count to zero */
   TCNT0 = 0;
}

/***********************************************************/
/***********************************************************/
   
void CStepperMotorController::UpdateWaveform() {
   /* Stop the counter (prescaler to zero) */
   TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
   /* Set the count to zero */
   TCNT0 = 0;
   /* if the speed is zero, there is no need to do anything further */
   if(m_unHalfPeriod != 0) {
      uint8_t unPort = PIND;
      bool bChAEqualsChB = (((unPort ^ (unPort << 1)) & STM_CHB_MASK) == 0);
      /* In order to avoid skipping a step in the motor excitation sequence,
         it is required in two situations to manually toggle channel A */
      if((m_eRotationDirection == ERotationDirection::REVERSE && bChAEqualsChB) ||
         (m_eRotationDirection == ERotationDirection::FORWARD && !bChAEqualsChB)) {
         /* Toggle channel A */
         TCCR0B |= (1 << FOC0A);
      }
      /* set the half period for the excitation sequence */
      OCR0A = m_unHalfPeriod;
      OCR0B = m_unHalfPeriod >> 1;
      /* Re-enable the counter (prescaler to 1024) */
      TCCR0B |= ((1 << CS02) | (1 << CS00));
   }
}

/***********************************************************/
/***********************************************************/

bool CStepperMotorController::IsWaveformActive() {
   return (TCCR0B & ((1 << CS02) | (1 << CS01) | (1 << CS00)) != 0);
}


