
#include "rf_controller.h"

#include <firmware.h>

#define VCNL40X0_ADDRESS  0x13

#define VCNL40X0_R0_PROXIMITY_START_MASK 0x08
#define VCNL40X0_R0_AMBIENT_START_MASK   0x10
#define VCNL40X0_R0_PROXIMITY_READY_MASK 0x20
#define VCNL40X0_R0_AMBIENT_READY_MASK   0x40
#define VCNL40X0_R4_AUTOCOMP_MASK 0x08

#define VCNL40X0_MAX_CURRENT 200

#define VCNL4000_REVISION 0x11
#define VCNL4010_REVISION 0x21

#define VCNL4000_PROX_FREQ 0x89
#define VCNL4000_PROX_FREQ_VAL 0x03

#define VCNL4000_PROX_MOD 0x8A
#define VCNL4000_PROX_MODVAL 0x81

#define VCNL4010_PROX_FREQMOD 0x8F
#define VCNL4010_PROX_FREQMOD_VAL 0x01

enum class ERegister : uint8_t {
   COMMAND = 0x80,
   PRODUCT_ID = 0x81,
   LED_CURRENT = 0x83,
   AMBIENT_PARAMETERS = 0x84,
   AMBIENT_RES_H = 0x85,
   AMBIENT_RES_L = 0x86,
   PROXIMITY_RES_H = 0x87,
   PROXIMITY_RES_L = 0x88,
};

/***********************************************************/
/***********************************************************/

bool CRFController::Probe() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PRODUCT_ID));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(VCNL40X0_ADDRESS, 1, true);
   
   uint8_t unDeviceId = CFirmware::GetInstance().GetTWController().Read();
   
   if(unDeviceId == VCNL4000_REVISION) {
      m_eDeviceType = EDeviceType::VCNL4000;
      return true;
   }
   else if(unDeviceId == VCNL4010_REVISION) {
      m_eDeviceType = EDeviceType::VCNL4010;
      return true;
   }
   else {
      return false;
   }
}

/***********************************************************/
/***********************************************************/

void CRFController::Configure(ENumberOfSamples e_num_samples, 
                              uint8_t un_led_current) {
   uint8_t unCurrentParameter = 
      ((un_led_current > VCNL40X0_MAX_CURRENT) ? VCNL40X0_MAX_CURRENT : un_led_current) / 10;
   
   /* Write the LED current parameter */
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::LED_CURRENT));
   CFirmware::GetInstance().GetTWController().Write(unCurrentParameter);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   /* Write the ambient parameters */
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::AMBIENT_PARAMETERS));
   CFirmware::GetInstance().GetTWController().Write(VCNL40X0_R4_AUTOCOMP_MASK | 
                                                   static_cast<uint8_t>(e_num_samples));
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   if(m_eDeviceType == EDeviceType::VCNL4000) {
      /* Write the sampling frequency parameter */
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);
      CFirmware::GetInstance().GetTWController().Write(VCNL4000_PROX_FREQ);
      CFirmware::GetInstance().GetTWController().Write(VCNL4000_PROX_FREQ_VAL);
      CFirmware::GetInstance().GetTWController().EndTransmission(true);

      /* Write the reccomended modulator adjust parameter */
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);
      CFirmware::GetInstance().GetTWController().Write(VCNL4000_PROX_MOD);
      CFirmware::GetInstance().GetTWController().Write(VCNL4000_PROX_MODVAL);
      CFirmware::GetInstance().GetTWController().EndTransmission(true);
   }
   else if(m_eDeviceType == EDeviceType::VCNL4010) {
      /* Write the reccomended modulator adjust parameter */
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);
      CFirmware::GetInstance().GetTWController().Write(VCNL4010_PROX_FREQMOD);
      CFirmware::GetInstance().GetTWController().Write(VCNL4010_PROX_FREQMOD_VAL);
      CFirmware::GetInstance().GetTWController().EndTransmission(true);
   }
   
   
}

/***********************************************************/
/***********************************************************/

uint16_t CRFController::ReadProximity() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
   CFirmware::GetInstance().GetTWController().Write(VCNL40X0_R0_PROXIMITY_START_MASK);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   do {
      CFirmware::GetInstance().GetTimer().Delay(10);
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
      CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
      CFirmware::GetInstance().GetTWController().EndTransmission(false);
      CFirmware::GetInstance().GetTWController().Read(VCNL40X0_ADDRESS, 1, true);
   } while ((CFirmware::GetInstance().GetTWController().Read() & VCNL40X0_R0_PROXIMITY_READY_MASK) == 0);

   uint8_t punResult[2];

   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PROXIMITY_RES_H));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(VCNL40X0_ADDRESS, 2, true);
   
   punResult[0] = CFirmware::GetInstance().GetTWController().Read();
   punResult[1] = CFirmware::GetInstance().GetTWController().Read();
   
   return (punResult[0] << 8) | punResult[1];
}

/***********************************************************/
/***********************************************************/

uint16_t CRFController::ReadAmbient() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
   CFirmware::GetInstance().GetTWController().Write(VCNL40X0_R0_AMBIENT_START_MASK);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   do {
      CFirmware::GetInstance().GetTimer().Delay(10);
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
      CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
      CFirmware::GetInstance().GetTWController().EndTransmission(false);
      CFirmware::GetInstance().GetTWController().Read(VCNL40X0_ADDRESS, 1, true);
   } while ((CFirmware::GetInstance().GetTWController().Read() & VCNL40X0_R0_AMBIENT_READY_MASK) == 0);

   uint8_t punResult[2];

   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL40X0_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::AMBIENT_RES_H));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(VCNL40X0_ADDRESS, 2, true);
   
   punResult[0] = CFirmware::GetInstance().GetTWController().Read();
   punResult[1] = CFirmware::GetInstance().GetTWController().Read();
   
   return (punResult[0] << 8) | punResult[1];

}
