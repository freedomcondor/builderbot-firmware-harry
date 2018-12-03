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
#include <huart_controller.h>
#include <tw_controller.h>
#include <nfc_controller.h>
#include <timer.h>
#include <tw_channel_selector.h>
#include <lift_actuator_system.h>
#include <packet_control_interface.h>
#include <rf_controller.h>

#define PWR_MON_MASK   0x03
#define PWR_MON_PGOOD  0x02
#define PWR_MON_CHG    0x01

/* NFC Reset and Interrupt Signals on Port D */
#define NFC_INT        0x04
#define NFC_RST        0x08

class CFirmware {
public:
      
   static CFirmware& GetInstance() {
      return _firmware;
   }

   void SetFilePointer(FILE* ps_huart) {
      m_psHUART = ps_huart;
   }

   CHUARTController& GetHUARTController() {
      return m_cHUARTController;
   }

   CTWController& GetTWController() {
      return m_cTWController;
   }

   CLiftActuatorSystem& GetLiftActuatorSystem() {
      return m_cLiftActuatorSystem;
   }

   CTimer& GetTimer() {
      return m_cTimer;
   }

   void Exec();
      
private:

   /* Test Routines */
   void TestPMIC();
   void TestDestructiveField();
   void TestConstructiveField();

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
      m_cPacketControlInterface(m_cHUARTController) {     

      /* Enable interrupts */
      sei();

      /* Configure the BQ24075 monitoring pins */
      /* TODO: Move this logic into a power manager class */
      DDRC &= ~PWR_MON_MASK;  // set as input
      PORTC &= ~PWR_MON_MASK; // disable pull ups
   }
   
   CTimer m_cTimer;

   /* ATMega328P Controllers */
   /* TODO remove singleton and reference from HUART */
   //CHUARTController& m_cHUARTController;
   CHUARTController& m_cHUARTController;
 
   CTWController& m_cTWController;

   CTWChannelSelector m_cTWChannelSelector;

   CNFCController m_cNFCController;

   CLiftActuatorSystem m_cLiftActuatorSystem;

   CPacketControlInterface m_cPacketControlInterface;

   static CFirmware _firmware;

public: // TODO, don't make these public
    /* File structs for fprintf */
   FILE* m_psHUART;



};

#endif
