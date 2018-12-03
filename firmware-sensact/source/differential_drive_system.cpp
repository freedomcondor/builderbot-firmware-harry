
#include "differential_drive_system.h"

#include <firmware.h>

/* Port B Pins - Power and faults */
#define DRV8833_EN     0x01
#define DRV8833_FAULT  0x02

/* Port C Pins - Encoder input */
#define ENC_RIGHT_CHA  0x01
#define ENC_RIGHT_CHB  0x02
#define ENC_LEFT_CHA   0x04
#define ENC_LEFT_CHB   0x08

/* Port D Pins - Motor output */
#define LEFT_CTRL_PIN  0x04
#define RIGHT_CTRL_PIN 0x08
#define RIGHT_MODE_PIN 0x10
#define RIGHT_PWM_PIN  0x20
#define LEFT_PWM_PIN   0x40
#define LEFT_MODE_PIN  0x80

/****************************************/
/****************************************/


CDifferentialDriveSystem::CDifferentialDriveSystem() :
   m_cShaftEncodersInterrupt(this, PCINT1_vect_num),
   m_cPIDControlStepInterrupt(this, TIMER1_COMPA_vect_num),
   m_nLeftSteps(0),
   m_nRightSteps(0) {

   /* Initialise pins in a disabled, coasting state */
   PORTB &= ~(DRV8833_EN);
   PORTD &= ~(LEFT_MODE_PIN  |
              LEFT_CTRL_PIN  |
              LEFT_PWM_PIN   |
              RIGHT_MODE_PIN |
              RIGHT_CTRL_PIN |
              RIGHT_PWM_PIN);
   /* set the direction of the output pins to output */
   DDRB |= (DRV8833_EN);
   DDRD |= (LEFT_MODE_PIN  |
            LEFT_CTRL_PIN  |
            LEFT_PWM_PIN   |
            RIGHT_MODE_PIN |
            RIGHT_CTRL_PIN |
            RIGHT_PWM_PIN);

   /* Setup up timer 0 for PWM */
   /* Select phase correct, non-inverting PWM mode on channel A & B */
   TCCR0A |= (1 << WGM00);
   /* Set precaler to 1 */
   TCCR0B |= (1 << CS00);

   /* Initialize left and right motor duty cycle to zero */
   OCR0A = 0;
   OCR0B = 0;

   /* CTC Mode , with precaler set to 64, OCR1A = 2039 (61.275Hz update frequency) */
   TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
   OCR1A = 2039;
   
   /* Enable port change interrupts for right encoder A/B
      and left encoder A/B respectively */
   PCMSK1 |= (1 << PCINT8)  | (1 << PCINT9) |
             (1 << PCINT10) | (1 << PCINT11);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::SetTargetVelocity(int16_t n_left_velocity, int16_t n_right_velocity) {
   m_cPIDControlStepInterrupt.SetTargetVelocity(n_left_velocity, n_right_velocity);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::SetPIDParams(float f_Kp, float f_Ki, float f_Kd) {
   m_cPIDControlStepInterrupt.SetPIDParams(f_Kp, f_Ki, f_Kd);
}

/****************************************/
/****************************************/

int16_t CDifferentialDriveSystem::GetLeftVelocity() {
   int16_t nVelocity;
   uint8_t unSREG = SREG;
   cli();
   nVelocity = m_nLeftStepsOut;
   SREG = unSREG;
   return nVelocity;
}

/****************************************/
/****************************************/

int16_t CDifferentialDriveSystem::GetRightVelocity() {
   int16_t nVelocity;
   uint8_t unSREG = SREG;
   cli();
   nVelocity = m_nRightStepsOut;
   SREG = unSREG;
   return nVelocity;
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::Enable() {
   /* Enable the shaft encoder interrupt */
   m_cShaftEncodersInterrupt.Enable();
   /* Enable the PID controller interrupt */
   m_cPIDControlStepInterrupt.Enable();
   /* Enable the motor driver */
   PORTB |= (DRV8833_EN);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::Disable() {
   /* Disable the motor driver */
   PORTB &= ~(DRV8833_EN);
   /* Disable the PID controller interrupt */
   m_cPIDControlStepInterrupt.Disable();
   /* Disable the shaft encoder interrupt */
   m_cShaftEncodersInterrupt.Disable();
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::ConfigureLeftMotor(CDifferentialDriveSystem::EBridgeMode e_mode,
                                                  uint8_t un_duty_cycle) {
   switch(e_mode) {
   case EBridgeMode::FORWARD_PWM_FD:
      PORTD |= LEFT_CTRL_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      break;
   case EBridgeMode::FORWARD_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD &= ~LEFT_CTRL_PIN;
      PORTD |= LEFT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_FD:
      PORTD &= ~LEFT_CTRL_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD |= LEFT_CTRL_PIN;
      PORTD |= LEFT_MODE_PIN;
      break;
   case EBridgeMode::COAST:
      PORTD &= ~(LEFT_PWM_PIN | LEFT_CTRL_PIN | LEFT_MODE_PIN);
      break;
   case EBridgeMode::FORWARD:
      PORTD |= LEFT_PWM_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      PORTD |= LEFT_CTRL_PIN;
      break;
   case EBridgeMode::REVERSE:
      PORTD |= LEFT_PWM_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      PORTD &= ~LEFT_CTRL_PIN;
      break;
   case EBridgeMode::BRAKE:
      PORTD |= (LEFT_PWM_PIN | LEFT_CTRL_PIN | LEFT_MODE_PIN);
      break;
   }

   if(e_mode == EBridgeMode::COAST   ||
      e_mode == EBridgeMode::REVERSE ||
      e_mode == EBridgeMode::FORWARD ||
      e_mode == EBridgeMode::BRAKE) {
      /* disconnect PWM from the output pin */
      OCR0A = 0;
      TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
   }
   else {
      /* connect PWM to the output pin */
      OCR0A = un_duty_cycle;
      TCCR0A |= (1 << COM0A1);
   }
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::ConfigureRightMotor(CDifferentialDriveSystem::EBridgeMode e_mode,
                                                   uint8_t un_duty_cycle) {
   switch(e_mode) {
   case EBridgeMode::FORWARD_PWM_FD:
      PORTD &= ~RIGHT_CTRL_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      break;
   case EBridgeMode::FORWARD_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD |= RIGHT_CTRL_PIN;
      PORTD |= RIGHT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_FD:
      PORTD |= RIGHT_CTRL_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD &= ~RIGHT_CTRL_PIN;
      PORTD |= RIGHT_MODE_PIN;
      break;
   case EBridgeMode::COAST:
      PORTD &= ~(RIGHT_PWM_PIN | RIGHT_CTRL_PIN | RIGHT_MODE_PIN);
      break;
   case EBridgeMode::FORWARD:
      PORTD |= RIGHT_PWM_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      PORTD &= ~RIGHT_CTRL_PIN;
      break;
   case EBridgeMode::REVERSE:
      PORTD |= RIGHT_PWM_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      PORTD |= RIGHT_CTRL_PIN;
      break;
   case EBridgeMode::BRAKE:
      PORTD |= (RIGHT_PWM_PIN | RIGHT_CTRL_PIN | RIGHT_MODE_PIN);
      break;
   }

   if(e_mode == EBridgeMode::COAST   ||
      e_mode == EBridgeMode::REVERSE ||
      e_mode == EBridgeMode::FORWARD ||
      e_mode == EBridgeMode::BRAKE) {
      /* disconnect PWM from the output pin */
      OCR0B = 0;
      TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
   }
   else {
      /* connect PWM to the output pin */
      OCR0B = un_duty_cycle;
      TCCR0A |= (1 << COM0B1);
   }
}

/****************************************/
/****************************************/

CDifferentialDriveSystem::CShaftEncodersInterrupt::CShaftEncodersInterrupt(
   CDifferentialDriveSystem* pc_differential_drive_system,
   uint8_t un_intr_vect_num) :
   m_pcDifferentialDriveSystem(pc_differential_drive_system),
   m_unPortLast(0) {
   Register(this, un_intr_vect_num);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CShaftEncodersInterrupt::Enable() {
   /* clear variables */
   m_unPortLast = 0;
   m_pcDifferentialDriveSystem->m_nLeftSteps = 0;
   m_pcDifferentialDriveSystem->m_nRightSteps = 0;
   /* enable interrupt */
   PCICR |= (1 << PCIE1);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CShaftEncodersInterrupt::Disable() {
   /* disable interrupt */
   PCICR &= ~(1 << PCIE1);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CShaftEncodersInterrupt::ServiceRoutine() {
   uint8_t unPortSnapshot = PINC;
   uint8_t unPortDelta = m_unPortLast ^ unPortSnapshot;
   /* This intermediate value determines whether the motors are moving
      backwards or forwards. The expression uses Reed-Muller logic and
      leverages the symmetry of the encoder inputs */
   uint8_t unIntermediate = (~unPortSnapshot) ^ (m_unPortLast >> 1);
   /* check the left encoder */
   if(unPortDelta & (ENC_LEFT_CHA | ENC_LEFT_CHB)) {
      if(unIntermediate & ENC_LEFT_CHA) {
         m_pcDifferentialDriveSystem->m_nLeftSteps--;
      }
      else {
         m_pcDifferentialDriveSystem->m_nLeftSteps++;
      }
   }
   /* check the right encoder */
   if(unPortDelta & (ENC_RIGHT_CHA | ENC_RIGHT_CHB)) {
      if(unIntermediate & ENC_RIGHT_CHA) {
         m_pcDifferentialDriveSystem->m_nRightSteps++;
      }
      else {
         m_pcDifferentialDriveSystem->m_nRightSteps--;
      }
   }
   m_unPortLast = unPortSnapshot;
}

/****************************************/
/****************************************/

CDifferentialDriveSystem::CPIDControlStepInterrupt::CPIDControlStepInterrupt(
   CDifferentialDriveSystem* pc_differential_drive_system,
   uint8_t un_intr_vect_num) :
   m_pcDifferentialDriveSystem(pc_differential_drive_system),
   m_nLeftTarget(0),
   m_nLeftLastError(0),
   m_nLeftErrorIntegral(0.0f),
   m_nRightTarget(0),
   m_nRightLastError(0),
   m_nRightErrorIntegral(0.0f),
   m_fLeftOutput(0),
   m_fRightOutput(0),

   /* most stable one, but not powerful enough */
   //m_fKp(0.25f),
   //m_fKi(0.0f),
   //m_fKd(0.6f) {

   /* works best in the office
	* use this one first */
   m_fKp(0.75f),
   m_fKi(0.0f),
   m_fKd(0.35f) {

   /* works in the robot room, 
	* but this will generate a very bad oscillation when robot is held in the air */
   //m_fKp(1.0f),
   //m_fKi(0.0f),
   //m_fKd(0.25f) {

   Register(this, un_intr_vect_num);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::Enable() {
   /* clear intermediate variables */
   m_nLeftLastError = 0;
   m_nLeftErrorIntegral = 0;
   m_nRightLastError = 0;
   m_nRightErrorIntegral = 0;
   m_nLeftTarget = 0;
   m_nRightTarget = 0;
   /* enable interrupt */
   TIMSK1 |= (1 << OCIE1A);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::Disable() {
   /* disable interrupt */
   TIMSK1 &= ~(1 << OCIE1A);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::SetPIDParams(float f_Kp, float f_Ki, float f_Kd) {
   uint8_t unSREG = SREG;
   cli();
   m_fKp = f_Kp;
   m_fKi = f_Ki;
   m_fKd = f_Kd;
   SREG = unSREG;
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::SetTargetVelocity(int16_t n_left_velocity, int16_t n_right_velocity) {
   uint8_t unSREG = SREG;
   cli();
   m_nLeftTarget = n_left_velocity;
   m_nRightTarget = n_right_velocity;
   SREG = unSREG;
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::ServiceRoutine() {
   /* Calculate left PID intermediates */
   int16_t nLeftError = m_nLeftTarget - m_pcDifferentialDriveSystem->m_nLeftSteps;
   /* Accumulate the integral component */
   m_nLeftErrorIntegral += nLeftError;
   /* Calculate the derivate component */
   int16_t nLeftErrorDerivative = (nLeftError - m_nLeftLastError);
   m_nLeftLastError = nLeftError;
   /* Calculate output value */
   m_fLeftOutput +=
      (m_fKp * nLeftError) +
      (m_fKi * m_nLeftErrorIntegral) +
      (m_fKd * nLeftErrorDerivative);
   /* Limit output */
   m_fLeftOutput = (m_fLeftOutput < float(UINT8_MAX)) ? m_fLeftOutput : float(UINT8_MAX);
   m_fLeftOutput = (m_fLeftOutput >-float(UINT8_MAX)) ? m_fLeftOutput :-float(UINT8_MAX);
   /* store the sign of the output */
   bool bLeftNegative = (m_fLeftOutput < 0.0f);
   /* take the absolute value */
   float fLeftOutput = (bLeftNegative ? -m_fLeftOutput : m_fLeftOutput);
   /* saturate into the uint8_t range */
   uint8_t unLeftDutyCycle = uint8_t(fLeftOutput);

   /* Calculate right PID intermediates */
   int16_t nRightError = m_nRightTarget - m_pcDifferentialDriveSystem->m_nRightSteps;
   /* Accumulate the integral component */
   m_nRightErrorIntegral += nRightError;
   /* Calculate the derivate component */
   int16_t nRightErrorDerivative = (nRightError - m_nRightLastError);
   m_nRightLastError = nRightError;
   /* Calculate output value */
   m_fRightOutput +=
      (m_fKp * nRightError) +
      (m_fKi * m_nRightErrorIntegral) +
      (m_fKd * nRightErrorDerivative);
   /* Limit output */
   m_fRightOutput = (m_fRightOutput < float(UINT8_MAX)) ? m_fRightOutput : float(UINT8_MAX);
   m_fRightOutput = (m_fRightOutput >-float(UINT8_MAX)) ? m_fRightOutput :-float(UINT8_MAX);
   /* store the sign of the output */
   bool bRightNegative = (m_fRightOutput < 0.0f);
   /* take the absolute value */
   float fRightOutput = (bRightNegative ? -m_fRightOutput : m_fRightOutput);
   /* saturate into the uint8_t range */
   uint8_t unRightDutyCycle = uint8_t(fRightOutput);

   /* Update right motor */
   m_pcDifferentialDriveSystem->ConfigureRightMotor(
      bRightNegative ? CDifferentialDriveSystem::EBridgeMode::REVERSE_PWM_FD :
                       CDifferentialDriveSystem::EBridgeMode::FORWARD_PWM_FD,
      unRightDutyCycle);

   /* Update left motor */
   m_pcDifferentialDriveSystem->ConfigureLeftMotor(
      bLeftNegative  ? CDifferentialDriveSystem::EBridgeMode::REVERSE_PWM_FD :
                       CDifferentialDriveSystem::EBridgeMode::FORWARD_PWM_FD,
      unLeftDutyCycle);

   /* copy the step counters for velocity measurements */
   m_pcDifferentialDriveSystem->m_nRightStepsOut = m_pcDifferentialDriveSystem->m_nRightSteps;
   m_pcDifferentialDriveSystem->m_nLeftStepsOut = m_pcDifferentialDriveSystem->m_nLeftSteps; 
   /* clear the step counters */
   m_pcDifferentialDriveSystem->m_nRightSteps = 0;
   m_pcDifferentialDriveSystem->m_nLeftSteps = 0;
}

/****************************************/
/****************************************/









