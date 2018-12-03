#ifndef LIFT_ACTUATOR_SYSTEM_H
#define LIFT_ACTUATOR_SYSTEM_H

#include <stepper_motor_controller.h>
#include <electromagnet_controller.h>
#include <interrupt.h>

class CLiftActuatorSystem {

public:
   enum class ESystemState : uint8_t {
      /* Inactive means the stepper motor is disabled */
      INACTIVE = 0,
      /* Active means the stepper motor is running */
      ACTIVE_POSITION_CTRL = 1,
      ACTIVE_SPEED_CTRL = 2,
      /* Calibration search bottom/top */
      CALIBRATION_SRCH_TOP = 3,
      CALIBRATION_SRCH_BTM = 4,
   };
   
   enum class ESystemEvent : uint8_t {
      STOP,
      LIMIT_SWITCH_PRESSED,
      START_CALIBRATION,
      START_POSITION_CTRL,
      START_SPEED_CTRL,
   };

public:
   CLiftActuatorSystem();
   
   ESystemState GetSystemState() {
      return m_eSystemState;
   }
   
   void ProcessEvent(ESystemEvent e_system_event);
   
   void Step();
   
   /* SetSpeed and SetPosition validate the input and convert units */
   /* to start speed or position control, the step function must be 
      called with the associated event */

   /* speed is in mm/sec - for performance reasons this value is 
      saturates into the range of 10mm/sec and 25mm/sec */
   void SetSpeed(int8_t n_speed);

   /* position is in mm */
   void SetPosition(uint8_t un_position);
   
   /* position is in mm */      
   uint8_t GetPosition();
 
   bool GetUpperLimitSwitchState() {
      return m_cLimitSwitchInterrupt.GetUpperSwitchState();
   }

   bool GetLowerLimitSwitchState() {
      return m_cLimitSwitchInterrupt.GetLowerSwitchState();
   }
   
   CElectromagnetController& GetElectromagnetController() {
      return m_cElectromagnetController;
   }
   
private:

   volatile ESystemState m_eSystemState;

   CStepperMotorController m_cStepperMotorController;
   CElectromagnetController m_cElectromagnetController;
   
   /* Max position of the end effector in steps */
   int16_t m_nMaxPosition;
      
   /* Interrupt for monitoring the limit switches */   
   class CLimitSwitchInterrupt : public CInterrupt {
   public:
      CLimitSwitchInterrupt(CLiftActuatorSystem* pc_lift_actuator_system, 
                           uint8_t un_intr_vect_num);
      void Enable();
      void Disable();
      bool GetUpperSwitchState() {
         return m_bUpperSwitchState;
      }
      bool GetLowerSwitchState() {
         return m_bLowerSwitchState;
      }
   private:  
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      bool m_bUpperSwitchState;
      bool m_bLowerSwitchState;
      void ServiceRoutine();
   } m_cLimitSwitchInterrupt;
   
 
   /* Interrupt for tracking (and when in position control mode, controlling) 
      the position of the end effector */
   class CStepCounterInterrupt : public CInterrupt {
   public:
      CStepCounterInterrupt(CLiftActuatorSystem* pc_lift_actuator_system, 
                            uint8_t un_intr_vect_num);
      void Enable();
      void Disable();      
      int16_t GetPosition();
   private:  
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      /* Note: representation of position is in cycles not mm */
      volatile int16_t m_nPosition;
      void ServiceRoutine();

   } m_cStepCounterInterrupt;
   
   /* Closed loop position control */
   class CPositionController {
   public:
      CPositionController(CLiftActuatorSystem* pc_lift_actuator_system);
      void Step();
      void SetTargetPosition(int16_t n_target_position);
      uint8_t GetHalfPeriod() {
         return m_unTargetHalfPeriod;
      }
      CStepperMotorController::ERotationDirection GetRotationDirection() {
         return m_eTargetRotationDirection;
      }    
   private:
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      int16_t m_nTargetPosition;
      CStepperMotorController::ERotationDirection m_eTargetRotationDirection;
      uint8_t m_unTargetHalfPeriod;
   } m_cPositionController;
   
   /* Open loop speed control */
   class CSpeedController {
   public:
      CSpeedController(CLiftActuatorSystem* pc_lift_actuator_system);
      void SetTargetSpeed(uint8_t un_half_period, 
                          CStepperMotorController::ERotationDirection e_rotation_direction);
      uint8_t GetHalfPeriod() {
         return m_unTargetHalfPeriod;
      }
      CStepperMotorController::ERotationDirection GetRotationDirection() {
         return m_eTargetRotationDirection;
      }
   private:
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      CStepperMotorController::ERotationDirection m_eTargetRotationDirection;
      uint8_t m_unTargetHalfPeriod;
   } m_cSpeedController;

   /* Subclasses are friends of CLiftActuatorSystem */
   friend class CLimitSwitchInterrupt;
   friend class CStepCounterInterrupt;
   friend class CPositionController;
   friend class CSpeedController;
};

#endif
