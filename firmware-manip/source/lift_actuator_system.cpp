
#include "lift_actuator_system.h"

#include <avr/interrupt.h>

#include <firmware.h>

#define PORTD_LTSW_TOP_IRQ 0x10
#define PORTD_LTSW_BTM_IRQ 0x80

#define LIFT_ACTUATOR_DEFAULT_STEPS 2000
#define MINIMUM_HALF_PERIOD 30
#define DEFAULT_HALF_PERIOD 35
#define MAXIMUM_HALF_PERIOD 40

#define LIFT_ACTUATOR_RANGE_MM 140
#define LIFT_ACTUATOR_SPEED_MIN_MMPERSEC 10
#define LIFT_ACTUATOR_SPEED_MAX_MMPERSEC 25

#define TIMER0_PRESCALE_VAL 1024UL

#define BIT_PATTERN_ALT 0xAAAAAAAA
#define BIT_PATTERN_ALL_ON 0xFFFFFFFF
#define BIT_PATTERN_ALL_OFF 0x00000000

#define POSITION_CTRL_ERROR_THESHOLD 2

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CLiftActuatorSystem() :
   m_eSystemState(ESystemState::INACTIVE),
   m_nMaxPosition(LIFT_ACTUATOR_DEFAULT_STEPS),
   m_cLimitSwitchInterrupt(this, PCINT2_vect_num),
   m_cStepCounterInterrupt(this, TIMER0_COMPA_vect_num),
   m_cPositionController(this),
   m_cSpeedController(this) {
   
   /* enable the system interrupts */
   m_cLimitSwitchInterrupt.Enable();
   m_cStepCounterInterrupt.Enable();
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::SetSpeed(int8_t n_speed) {
   bool bIsNegative = (n_speed < 0);
   uint8_t unMagnitude = bIsNegative ? -n_speed : n_speed;

   if((unMagnitude >= LIFT_ACTUATOR_SPEED_MIN_MMPERSEC) &&
      (unMagnitude <= LIFT_ACTUATOR_SPEED_MAX_MMPERSEC)) {
      
      uint8_t unHalfPeriod = 
            (LIFT_ACTUATOR_RANGE_MM * F_CPU / (TIMER0_PRESCALE_VAL * m_nMaxPosition)) / unMagnitude;
      
      CStepperMotorController::ERotationDirection eRotationDirection = 
            bIsNegative ? CStepperMotorController::ERotationDirection::REVERSE : 
                          CStepperMotorController::ERotationDirection::FORWARD;
      
      m_cSpeedController.SetTargetSpeed(unHalfPeriod, eRotationDirection);
   }
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::SetPosition(uint8_t un_position) {
   /* Validate the input */
   if(un_position <= LIFT_ACTUATOR_RANGE_MM) {
      m_cPositionController.SetTargetPosition(
         static_cast<int32_t>(m_nMaxPosition) * un_position / LIFT_ACTUATOR_RANGE_MM);        
   }
}

/***********************************************************/
/***********************************************************/

uint8_t CLiftActuatorSystem::GetPosition() {
   int16_t nPositionInSteps = m_cStepCounterInterrupt.GetPosition();
   int32_t nPosition = static_cast<int32_t>(nPositionInSteps) * LIFT_ACTUATOR_RANGE_MM / m_nMaxPosition;
   return (nPosition > UINT8_MAX ? UINT8_MAX : (nPosition < 0 ? 0 : nPosition));
}

/***********************************************************/
/***********************************************************/

/* Reminder: this method can be called from an interrupt context */
void CLiftActuatorSystem::ProcessEvent(ESystemEvent e_system_event) {
   /* since this method can be called from both an interrupt and non-interrupt 
      context, we must clear the interrupt flag so it is only invoked once */
   uint8_t unSREG = SREG;
   cli();
   /* check the event type */
   switch(e_system_event) {
   case ESystemEvent::LIMIT_SWITCH_PRESSED:
      m_cStepperMotorController.Disable();
      /* handle the calibration states */
      switch(m_eSystemState) {
      case ESystemState::CALIBRATION_SRCH_BTM:
         if(m_cLimitSwitchInterrupt.GetUpperSwitchState()) {
            m_cStepCounterInterrupt.Enable();
            m_eSystemState = ESystemState::CALIBRATION_SRCH_TOP;
         }
         break;
      case ESystemState::CALIBRATION_SRCH_TOP:
         if(m_cLimitSwitchInterrupt.GetLowerSwitchState()) {
            m_nMaxPosition = m_cStepCounterInterrupt.GetPosition();
            m_eSystemState = ESystemState::INACTIVE;
         }
         break;
      default:
         m_eSystemState = ESystemState::INACTIVE;
         break;
      }
      break;
   case ESystemEvent::STOP:
      m_cStepperMotorController.Disable();
      m_eSystemState = ESystemState::INACTIVE;
      break;
   case ESystemEvent::START_CALIBRATION:
      m_eSystemState = ESystemState::CALIBRATION_SRCH_BTM;
      break;
   case ESystemEvent::START_POSITION_CTRL:
      m_eSystemState = ESystemState::ACTIVE_POSITION_CTRL;
      break;
   case ESystemEvent::START_SPEED_CTRL:
      m_eSystemState = ESystemState::ACTIVE_SPEED_CTRL;
      break;
   }
   /* restore SREG, re-enable interrupts if disabled */
   SREG = unSREG;
}

/***********************************************************/
/***********************************************************/

/* Note: this method is never called from an interrupt context, it is polled from CFirmware::Exec */
void CLiftActuatorSystem::Step() {
   /* since m_eSystemState and the motor configuration can be modified from an 
      interrupt context, we must clear the interrupt flag so this doesn't happen. */
   uint8_t unSREG = SREG;
   cli();

   uint8_t unTargetHalfPeriod =
      m_cStepperMotorController.GetHalfPeriod();
   CStepperMotorController::ERotationDirection eRotationDirection = 
      m_cStepperMotorController.GetRotationDirection();
   
   switch(m_eSystemState) {
   case ESystemState::INACTIVE:
      break;
   case ESystemState::CALIBRATION_SRCH_BTM:
      unTargetHalfPeriod = DEFAULT_HALF_PERIOD;
      eRotationDirection = CStepperMotorController::ERotationDirection::REVERSE;
      break;
   case ESystemState::CALIBRATION_SRCH_TOP:
      unTargetHalfPeriod = DEFAULT_HALF_PERIOD;
      eRotationDirection = CStepperMotorController::ERotationDirection::FORWARD;
      break;
   case ESystemState::ACTIVE_SPEED_CTRL:
      unTargetHalfPeriod = m_cSpeedController.GetHalfPeriod();
      eRotationDirection = m_cSpeedController.GetRotationDirection();
      break;
   case ESystemState::ACTIVE_POSITION_CTRL:
      m_cPositionController.Step();
      unTargetHalfPeriod = m_cPositionController.GetHalfPeriod();
      eRotationDirection = m_cPositionController.GetRotationDirection();
      break;
   }
   /* If the system is not inactive, update the stepper motor control and enable */
   if(m_eSystemState != ESystemState::INACTIVE) {
      /* only start the motor in the given direction if the respective limit switch is not active */
      if(((eRotationDirection == CStepperMotorController::ERotationDirection::REVERSE) &&
         (m_cLimitSwitchInterrupt.GetUpperSwitchState() == true)) ||
         ((eRotationDirection == CStepperMotorController::ERotationDirection::FORWARD) &&
         (m_cLimitSwitchInterrupt.GetLowerSwitchState() == true))) {
         /* Assuming we are not calibrating - disable the motor and move to the inactive state */
         ProcessEvent(ESystemEvent::LIMIT_SWITCH_PRESSED);
      }
      else {     
         /* only update the waveform if the */
         if(m_cStepperMotorController.IsWaveformActive() == false ||
            unTargetHalfPeriod != m_cStepperMotorController.GetHalfPeriod() ||
            eRotationDirection != m_cStepperMotorController.GetRotationDirection()) {
            m_cStepperMotorController.SetHalfPeriod(unTargetHalfPeriod);
            m_cStepperMotorController.SetRotationDirection(eRotationDirection);
            m_cStepperMotorController.UpdateWaveform();
         }
         m_cStepperMotorController.Enable();
      }
   }
   /* restore SREG, re-enable interrupts if disabled */
   SREG = unSREG;
}

/***********************************************************/
/***********************************************************/


CLiftActuatorSystem::CLimitSwitchInterrupt::CLimitSwitchInterrupt(CLiftActuatorSystem* pc_lift_actuator_system,
                                                                  uint8_t un_intr_vect_num) : 
   m_pcLiftActuatorSystem(pc_lift_actuator_system) {
   Register(this, un_intr_vect_num);

   PORTD |= (PORTD_LTSW_TOP_IRQ | PORTD_LTSW_BTM_IRQ);
   DDRD &= ~(PORTD_LTSW_TOP_IRQ | PORTD_LTSW_BTM_IRQ);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CLimitSwitchInterrupt::Enable() {
   /* Enable port change interrupts for external events */ 
   PCMSK2 |= ((1 << PCINT20)  | (1 << PCINT23));
   /* Enable the port change interrupt group PCINT[23:16] */
   PCICR |= (1 << PCIE2);
   /* Run the service routine to initialise the state */
   ServiceRoutine();
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CLimitSwitchInterrupt::Disable() {
   /* Disable port change interrupts for external events */ 
   PCMSK2 &= ~((1 << PCINT20)  | (1 << PCINT23));
   /* Disable the port change interrupt group PCINT[23:16] */
   PCICR &= ~(1 << PCIE2);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CLimitSwitchInterrupt::ServiceRoutine() {
   /* initialize the debounce variables to an alternating bit pattern */
   uint32_t unUpperSwitchDebounce = BIT_PATTERN_ALT;
   uint32_t unLowerSwitchDebounce = BIT_PATTERN_ALT;
   /* take a snapshot of the original switch states */
   bool bUpperSwitchPrevState = m_bUpperSwitchState;
   bool bLowerSwitchPrevState = m_bLowerSwitchState;
   /* debounce the input */
   for(;;) {
      uint8_t unPortSample = PIND;
      unUpperSwitchDebounce = 
         (unUpperSwitchDebounce << 1) | ((unPortSample & PORTD_LTSW_TOP_IRQ) ? 0x01 : 0x00);   
      unLowerSwitchDebounce = 
         (unLowerSwitchDebounce << 1) | ((unPortSample & PORTD_LTSW_BTM_IRQ) ? 0x01 : 0x00);
      /* break once the inputs are stable */   
      if((unUpperSwitchDebounce == BIT_PATTERN_ALL_OFF || unUpperSwitchDebounce == BIT_PATTERN_ALL_ON) &&
         (unLowerSwitchDebounce == BIT_PATTERN_ALL_OFF || unLowerSwitchDebounce == BIT_PATTERN_ALL_ON)) {
         break;  
      }
   }
   /* update the state variables */
   m_bUpperSwitchState = (unUpperSwitchDebounce == BIT_PATTERN_ALL_ON);
   m_bLowerSwitchState = (unLowerSwitchDebounce == BIT_PATTERN_ALL_ON);  
   /* generate an event, but only if a switch was pressed */
   if((m_bUpperSwitchState && (m_bUpperSwitchState != bUpperSwitchPrevState)) ||
      (m_bLowerSwitchState && (m_bLowerSwitchState != bLowerSwitchPrevState))) {
      m_pcLiftActuatorSystem->ProcessEvent(CLiftActuatorSystem::ESystemEvent::LIMIT_SWITCH_PRESSED);
   }
}

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CStepCounterInterrupt::CStepCounterInterrupt(CLiftActuatorSystem* pc_lift_actuator_system,
                                                                  uint8_t un_intr_vect_num) : 
   m_pcLiftActuatorSystem(pc_lift_actuator_system),
   m_nPosition(0) {
   Register(this, un_intr_vect_num);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CStepCounterInterrupt::Enable() {
   /* Set the position to zero */
   m_nPosition = 0;
   /* Enable output compare match interrupt on channel A */ 
   TIMSK0 |= (1 << OCIE0A);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CStepCounterInterrupt::Disable() {
   /* Disable output compare match interrupt on channel A */ 
   TIMSK0 &= ~(1 << OCIE0A);
}

/***********************************************************/
/***********************************************************/

int16_t CLiftActuatorSystem::CStepCounterInterrupt::GetPosition() {
   uint8_t unSREG = SREG;
   cli();
   int16_t nVal = m_nPosition;
   SREG = unSREG;
   return nVal;
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CStepCounterInterrupt::ServiceRoutine() {
   uint8_t unPort = PIND;
   if(((unPort ^ (unPort << 1)) & STM_CHB_MASK) == 0) {
      m_nPosition++;
   }
   else {
      m_nPosition--;
   }
}

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CPositionController::CPositionController(CLiftActuatorSystem* pc_lift_actuator_system) :
   m_pcLiftActuatorSystem(pc_lift_actuator_system),
   m_nTargetPosition(0) {
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CPositionController::Step() {
   /* calculate the error */
   int32_t nError = m_nTargetPosition -
      m_pcLiftActuatorSystem->m_cStepCounterInterrupt.GetPosition();
   if(nError < POSITION_CTRL_ERROR_THESHOLD && 
      nError > -POSITION_CTRL_ERROR_THESHOLD) {
      m_pcLiftActuatorSystem->ProcessEvent(CLiftActuatorSystem::ESystemEvent::STOP);
   }
   else {
      /* take the absolute values, saturating the parameters at their limits */
      if(nError < 0) {
         int32_t nOutput = nError * (-MAXIMUM_HALF_PERIOD + MINIMUM_HALF_PERIOD) / 
            m_pcLiftActuatorSystem->m_nMaxPosition - MAXIMUM_HALF_PERIOD;
         m_unTargetHalfPeriod = (nOutput < -MAXIMUM_HALF_PERIOD) ? MAXIMUM_HALF_PERIOD : 
            (nOutput > -MINIMUM_HALF_PERIOD) ? MINIMUM_HALF_PERIOD : -nOutput;
         m_eTargetRotationDirection = CStepperMotorController::ERotationDirection::REVERSE;
      }
      else {
         int32_t nOutput = nError * (MINIMUM_HALF_PERIOD - MAXIMUM_HALF_PERIOD) / 
            m_pcLiftActuatorSystem->m_nMaxPosition + MAXIMUM_HALF_PERIOD;
         m_unTargetHalfPeriod = (nOutput > MAXIMUM_HALF_PERIOD) ? MAXIMUM_HALF_PERIOD : 
            (nOutput < MINIMUM_HALF_PERIOD) ? MINIMUM_HALF_PERIOD : nOutput;
         m_eTargetRotationDirection = CStepperMotorController::ERotationDirection::FORWARD;
      }
   }        
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CPositionController::SetTargetPosition(int16_t n_target_position) {
   m_nTargetPosition = n_target_position;
}


/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CSpeedController::CSpeedController(CLiftActuatorSystem* pc_lift_actuator_system) :
   m_pcLiftActuatorSystem(pc_lift_actuator_system) {
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CSpeedController::SetTargetSpeed(uint8_t un_half_period,
                                                           CStepperMotorController::ERotationDirection e_rotation_direction) {
   m_unTargetHalfPeriod = un_half_period;
   m_eTargetRotationDirection = e_rotation_direction;
}

      
