#ifndef FIRMWARE_H
#define FIRMWARE_H

//#define DEBUG

/* AVR Headers */
#include <avr/io.h>
#include <avr/interrupt.h>

/* debug */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Firmware Headers */
#include <usb_interface_system.h>
#include <power_management_system.h>
#include <packet_control_interface.h>

#include <adc_controller.h>
#include <huart_controller.h>
#include <timer.h>
#include <tw_controller.h>

class CFirmware {
public:
      
   static CFirmware& GetInstance() {
      return _firmware;
   }

   uint8_t GetId();

   void SetFilePointer(FILE* ps_huart) {
      m_psHUART = ps_huart;
   }


   CHUARTController& GetHUARTController() {
      return m_cHUARTController;
   }

   CTWController& GetTWController() {
      return m_cTWController;
   }

   CTimer& GetTimer() {
      return m_cTimer;
   }

   void Exec();

   void TestPMICs();
      
private:

   /* private constructor */
   CFirmware() :
      m_cTimer(TCCR2A,
               TCCR2A | (1 << WGM21) | (1 << WGM20),
               TCCR2B,
               TCCR2B | (1 << CS22),
               TIMSK2,
               TIMSK2 | (1 << TOIE2),
               TIFR2,
               TCNT2,
               TIMER2_OVF_vect_num),
      m_cHUARTController(CHUARTController::instance()),
      m_cTWController(CTWController::GetInstance()),
      m_cPacketControlInterface(m_cHUARTController),
      m_cPowerEventInterrupt(this, PCINT1_vect_num),
      m_eSwitchState(ESwitchState::RELEASED),
      m_unSwitchPressedTime(0),
      m_bSwitchSignal(false),
      m_bUSBSignal(false),
      m_bSystemPowerSignal(false),
      m_bActuatorPowerSignal(false) {

      /* Enable interrupts */
      sei();

   }

   /* Hardware objects */
   CTimer m_cTimer;

   /* ATMega328P Controllers */
   /* TODO remove singleton and reference from HUART */
   CHUARTController& m_cHUARTController;
 
   CTWController& m_cTWController;

   CPacketControlInterface m_cPacketControlInterface;

   CPowerManagementSystem m_cPowerManagementSystem;

   class CPowerEventInterrupt : public CInterrupt {
   public:
      CPowerEventInterrupt(CFirmware* pc_firmware, 
                           uint8_t un_intr_vect_num);

      void Enable();

      void Disable();
   private:  
      CFirmware* m_pcFirmware;
      uint8_t m_unPortLast;
      void ServiceRoutine();
   } m_cPowerEventInterrupt;

   friend CPowerEventInterrupt;

   enum class ESwitchState {
      PRESSED,
      RELEASED,
   } m_eSwitchState = ESwitchState::RELEASED;

   uint32_t m_unSwitchPressedTime;
   
   /* Signals */
   bool m_bSwitchSignal;
   bool m_bUSBSignal;
   bool m_bSystemPowerSignal;
   bool m_bActuatorPowerSignal;

   static CFirmware _firmware;

public: // TODO, don't make these public
    /* File structs for fprintf */
   FILE* m_psHUART;

};

#endif
