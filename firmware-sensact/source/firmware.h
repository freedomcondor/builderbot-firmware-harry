#ifndef FIRMWARE_H
#define FIRMWARE_H

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
#include <packet_control_interface.h>

#include <differential_drive_system.h>
#include <accelerometer_system.h>

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

   void Exec();

private:

   /* private constructor */
   CFirmware() :
      m_cHUARTController(CHUARTController::instance()),
      m_cTWController(CTWController::GetInstance()),
      m_cPacketControlInterface(m_cHUARTController) {     

      /* Enable interrupts */
      sei();
   }

   /* ATMega328P Controllers */
   CHUARTController& m_cHUARTController;
   CTWController& m_cTWController;
   
   /* Modules */
   CPacketControlInterface m_cPacketControlInterface;

   /* Systems */
   CDifferentialDriveSystem m_cDifferentialDriveSystem;
   CAccelerometerSystem m_cAccelerometerSystem;

   static CFirmware _firmware;

public: // TODO, don't make these public
    /* File structs for fprintf */
   FILE* m_psHUART;
};

#endif
