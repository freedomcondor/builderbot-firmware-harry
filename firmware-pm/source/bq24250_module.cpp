
#include "bq24250_module.h"

#include <stdint.h>

#include <firmware.h>

#define BQ24250_ADDR 0x6A

#define R0_STAT_MASK 0x30
#define R0_FAULT_MASK 0x0F
#define R0_WDEN_MASK 0x40
#define R0_WDFAULT_MASK 0x80

#define R1_ILIMIT_MASK 0x70
#define R1_HIZ_MASK 0x01
#define R1_RST_MASK 0x80
#define R1_CHGEN_MASK 0x02

/***********************************************************/
/***********************************************************/

void CBQ24250Module::DumpRegister(uint8_t un_addr) {

   CTWController::GetInstance().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(un_addr);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);
   fprintf(CFirmware::GetInstance().m_psHUART,
           "Register 0x%02x : 0x%02x\r\n",
           un_addr,
           CFirmware::GetInstance().GetTWController().Read());
}

/***********************************************************/
/***********************************************************/

void CBQ24250Module::SetRegisterValue(uint8_t un_addr, uint8_t un_mask, uint8_t un_value) {
   /* read old value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(un_addr);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);
   uint8_t unRegister = CFirmware::GetInstance().GetTWController().Read();
   /* clear bits to be updated */
   unRegister &= ~un_mask;
   /* shift the value into the correct position */
   while((un_mask & 0x01) == 0) {
      un_mask >>= 1;
      un_value <<= 1;
   }
   /* set the updated bits */
   unRegister |= un_value;
   /* write back the value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(un_addr);
   CFirmware::GetInstance().GetTWController().Write(unRegister);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

uint8_t CBQ24250Module::GetRegisterValue(uint8_t un_addr, uint8_t un_mask) {
   /* read old value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(un_addr);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);
   uint8_t unRegister = CFirmware::GetInstance().GetTWController().Read();
   /* clear unwanted bits */
   unRegister &= un_mask;
   /* shift value down to the correction position  */
   while((un_mask & 0x01) == 0) {
      un_mask >>= 1;
      unRegister >>= 1;
   }
   /* return the result */
   return unRegister;
}

/***********************************************************/
/***********************************************************/

CBQ24250Module::EInputLimit CBQ24250Module::GetInputLimit() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x01);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);

   uint8_t unRegister = CFirmware::GetInstance().GetTWController().Read();

   if((unRegister & R1_HIZ_MASK) == 0) {
      unRegister &= R1_ILIMIT_MASK;
      switch(unRegister >> 4) {
      case 0:
         return EInputLimit::L100;
         break;
      case 1:
         return EInputLimit::L150;
         break;
      case 2:
         return EInputLimit::L500;
         break;
      case 3:
         return EInputLimit::L900;
         break;
      case 4:
         return EInputLimit::L1500;
         break;
      case 5:
         return EInputLimit::L2000;
         break;
      case 6:
         return EInputLimit::LEXT;
         break;
      case 7:
         return EInputLimit::LPTM;
         break;
      default:
         return EInputLimit::LHIZ;
      }
   }
   else {
      return EInputLimit::LHIZ;
   }
}

/***********************************************************/
/***********************************************************/

void CBQ24250Module::SetInputLimit(EInputLimit eInputLimit) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x01);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);

   uint8_t unRegister = CFirmware::GetInstance().GetTWController().Read();

   /* clear the current value and assure reset is clear */
   unRegister &= ~R1_ILIMIT_MASK;
   unRegister &= ~R1_HIZ_MASK;
   unRegister &= ~R1_RST_MASK;

   switch(eInputLimit) {
   case EInputLimit::L100:
      unRegister |= (0x00 << 4);
      break;
   case EInputLimit::L150:
      unRegister |= (0x01 << 4);
      break;
   case EInputLimit::L500:
      unRegister |= (0x02 << 4);
      break;
   case EInputLimit::L900:
      unRegister |= (0x03 << 4);
      break;
   case EInputLimit::L1500:
      unRegister |= (0x04 << 4);
      break;
   case EInputLimit::L2000:
      unRegister |= (0x05 << 4);
      break;
   case EInputLimit::LEXT:
      unRegister |= (0x06 << 4);
      break;
   case EInputLimit::LPTM:
      unRegister |= (0x07 << 4);
      break;
   case EInputLimit::LHIZ:
      unRegister |= R1_HIZ_MASK;
      break;
   }

   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x01);
   CFirmware::GetInstance().GetTWController().Write(unRegister);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24250Module::ResetWatchdogTimer() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().Write(0x40);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   // DEBUG
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);
   CFirmware::GetInstance().GetTWController().Read();
}

/***********************************************************/
/***********************************************************/

void CBQ24250Module::SetChargingEnable(bool b_enable) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x01);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);

   uint8_t unRegister = CFirmware::GetInstance().GetTWController().Read();

   /* assure reset is clear */
   unRegister &= ~R1_RST_MASK;

   /* enable/disable charging */
   if(b_enable) {
      unRegister &= ~R1_CHGEN_MASK;
   }
   else {
      unRegister |= R1_CHGEN_MASK;
   }

   /* write back */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x01);
   CFirmware::GetInstance().GetTWController().Write(unRegister);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

}

/***********************************************************/
/***********************************************************/

void CBQ24250Module::Synchronize() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24250_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24250_ADDR, 1, true);

   uint8_t unRegister = CFirmware::GetInstance().GetTWController().Read();

   /* update the device state variable */
   switch((unRegister & R0_STAT_MASK) >> 4) {
   case 0x00:
      eDeviceState = EDeviceState::READY;
      break;
   case 0x01:
      eDeviceState = EDeviceState::CHARGING;
      break;
   case 0x02:
      eDeviceState = EDeviceState::DONE;
      break;
   case 0x03:
      eDeviceState = EDeviceState::FAULT;
      break;
   }  
   
   /* udpate the fault variable */
   switch(unRegister & R0_FAULT_MASK) {
   case 0x00:
      eFault = EFault::NONE;
      break;
   case 0x01:
      eFault = EFault::INPUT_OVER_VOLTAGE;
      break;
   case 0x02:
      eFault = EFault::INPUT_UNDER_VOLTAGE;
      break;
   case 0x03:
      eFault = EFault::SLEEP;
      break;
   case 0x04:
      eFault = EFault::BATT_THERMAL_SHDN;
      break;
   case 0x05:
      eFault = EFault::BATT_OVER_VOLTAGE;
      break;
   case 0x06:
      eFault = EFault::DEV_THERMAL_SHDN;
      break;
   case 0x07:
      eFault = EFault::DEV_TIMER_FAULT;
      break;
   case 0x08:
      eFault = EFault::BATT_DISCONNECTED;
      break;
   case 0x09:
      eFault = EFault::ISET_SHORTED;
      break;
   case 0x0A:
      eFault = EFault::INPUT_FAULT;
      break;
   default:
      eFault = EFault::UNDEFINED;
      break;
   }

   /* Update the watchdog variables */
   bWatchdogEnabled = ((unRegister & R0_WDEN_MASK) != 0);
   bWatchdogFault = ((unRegister & R0_WDFAULT_MASK) != 0);
}
