
#include "bq24161_module.h"

#include <firmware.h>

#define BQ24161_ADDR 0x6B

#define R0_ADDR 0x00
#define R1_ADDR 0x01
#define R2_ADDR 0x02
#define R3_ADDR 0x03
#define R4_ADDR 0x04
#define R5_ADDR 0x05

#define R0_WDT_RST_MASK 0x80
#define R0_STAT_MASK 0x70
#define R0_SUPPLY_MASK 0x08
#define R0_FAULT_MASK 0x07

#define R1_ADAPTER_STAT_MASK 0xC0
#define R1_USB_STAT_MASK 0x30
#define R1_OTG_LOCKOUT_MASK 0x08
#define R1_BATT_STAT_MASK 0x06
#define R1_NOBATT_OP_MASK 0x01

#define R2_RST_MASK 0x80
#define R2_USB_INPUT_LIMIT_MASK 0x70
#define R2_CHG_EN_MASK 0x02
#define R2_TERM_EN_MASK 0x04

#define R3_ADP_INPUT_LIMIT_MASK 0x02

#define TERM_CURRENT_BASE 50
#define TERM_CURRENT_OFFSET 50
#define CHRG_CURRENT_BASE 75
#define CHRG_CURRENT_OFFSET 550
#define REG_VOLTAGE_BASE 20
#define REG_VOLTAGE_OFFSET 3500

/***********************************************************/
/***********************************************************/

void CBQ24161Module::ResetWatchdogTimer() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R0_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);

   uint8_t unRegVal = CFirmware::GetInstance().GetTWController().Read();

   unRegVal |= R0_WDT_RST_MASK;

   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R0_ADDR);
   CFirmware::GetInstance().GetTWController().Write(unRegVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::DumpRegister(uint8_t un_addr) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(un_addr);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);
   fprintf(CFirmware::GetInstance().m_psHUART,
           "Register 0x%02x : 0x%02x\r\n",
           un_addr,
           CFirmware::GetInstance().GetTWController().Read());
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::EInputLimit CBQ24161Module::GetInputLimit(ESource e_source) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R2_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 2, true);

   uint8_t punRegVals[2] = {
      CFirmware::GetInstance().GetTWController().Read(),
      CFirmware::GetInstance().GetTWController().Read()
   };

   switch(e_source) {
   case ESource::USB:
      punRegVals[0] &= R2_USB_INPUT_LIMIT_MASK;
      switch(punRegVals[0] >> 4) {
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
         return EInputLimit::L800;
         break;
      case 4:
         return EInputLimit::L900;
         break;
      case 5:
         return EInputLimit::L1500;
         break;
      default:
         return EInputLimit::L0;
         break;
      }
      break;
   case ESource::ADAPTER:
      return ((punRegVals[1] & R3_ADP_INPUT_LIMIT_MASK) == 0) ? 
         EInputLimit::L1500 : EInputLimit::L2500;
      break;
   default:
      return EInputLimit::L0;
      break;
   }
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::SetInputLimit(ESource e_source, EInputLimit e_input_limit) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R2_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 2, true);

   uint8_t punRegVals[2] = {
      CFirmware::GetInstance().GetTWController().Read(),
      CFirmware::GetInstance().GetTWController().Read()
   };

   /* clear the reset bit, always set on read */
   punRegVals[0] &= ~R2_RST_MASK;

   switch(e_source) {
   case ESource::USB:
      /* clear the USB input limit bits */
      punRegVals[0] &= ~R2_USB_INPUT_LIMIT_MASK;

      switch(e_input_limit) {
      case EInputLimit::L100:
         punRegVals[0] |= (0 << 4);
         break;
      case EInputLimit::L150:
         punRegVals[0] |= (1 << 4);
         break;
      case EInputLimit::L500:
         punRegVals[0] |= (2 << 4);
         break;
      case EInputLimit::L800:
         punRegVals[0] |= (3 << 4);
         break;
      case EInputLimit::L900:
         punRegVals[0] |= (4 << 4);
         break;
      case EInputLimit::L1500:
         punRegVals[0] |= (5 << 4);
         break;
      default:
         /* by default apply the lowest setting 100mA */
         punRegVals[0] |= (0 << 4);
         break;
      }
      break;
   case ESource::ADAPTER:
      if(e_input_limit == EInputLimit::L2500) {
         punRegVals[1] |= R3_ADP_INPUT_LIMIT_MASK;
      }
      else {
         /* else apply the lowest setting 1500mA */
         punRegVals[1] &= ~R3_ADP_INPUT_LIMIT_MASK;
      }
      break;
   default:
      break;
   }

   /* write back */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x02);
   CFirmware::GetInstance().GetTWController().Write(punRegVals[0]);
   CFirmware::GetInstance().GetTWController().Write(punRegVals[1]);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::SetChargingEnable(bool b_enable) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R2_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);

   uint8_t unRegVal = CFirmware::GetInstance().GetTWController().Read();

   /* clear the reset bit, always set on read */
   unRegVal &= ~R2_RST_MASK;
   /* set the charge enable flag with respect to b_enable */
   if(b_enable == true) {
      unRegVal &= ~R2_CHG_EN_MASK;
   }
   else {
      unRegVal |= R2_CHG_EN_MASK;   
   }
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R2_ADDR);
   CFirmware::GetInstance().GetTWController().Write(unRegVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::SetNoBattOperationEnable(bool b_enable) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R1_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);

   uint8_t unRegVal = CFirmware::GetInstance().GetTWController().Read();

   /* set the no battery operation flag with respect to b_enable */
   if(b_enable == true) {
      unRegVal |= R1_NOBATT_OP_MASK;
   }
   else {
      unRegVal &= ~R1_NOBATT_OP_MASK;
   }
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R1_ADDR);
   CFirmware::GetInstance().GetTWController().Write(unRegVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::SetBatteryRegulationVoltage(uint16_t un_batt_voltage_mv) {
   /* check if the requested voltage is in range */
   if(un_batt_voltage_mv < REG_VOLTAGE_OFFSET || 
      un_batt_voltage_mv > 4440)
      return;

   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R3_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);
   uint8_t unRegVal = CFirmware::GetInstance().GetTWController().Read();          
   /* decrement by the internal offset voltage */
   un_batt_voltage_mv -= REG_VOLTAGE_OFFSET;
   /* loop over the different increments to determine register programming */
   for(uint16_t unIdx = 0; unIdx < 6; unIdx++) {
      uint16_t unInc = (1 << (5 - unIdx)) * REG_VOLTAGE_BASE;
      /* set/clear the register bits for the current increment */
      if(un_batt_voltage_mv / unInc > 0) {
         un_batt_voltage_mv -= unInc;
         unRegVal |= (1 << ((5 - unIdx) + 2));
      }
      else {
         unRegVal &= ~(1 << ((5 - unIdx) + 2));
      }
   }
   /* Write value back to register */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R3_ADDR);
   CFirmware::GetInstance().GetTWController().Write(unRegVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::SetBatteryChargingCurrent(uint16_t un_batt_chrg_current_ma) {
   /* check if the requested current is in range */
   if(un_batt_chrg_current_ma < CHRG_CURRENT_OFFSET ||
      un_batt_chrg_current_ma > 2875)
      return;

   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R5_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);
   uint8_t unRegVal = CFirmware::GetInstance().GetTWController().Read();          
   /* decrement by the internal offset current */
   un_batt_chrg_current_ma -= CHRG_CURRENT_OFFSET;
   /* loop over the different increments to determine register programming */
   for(uint16_t unIdx = 0; unIdx < 5; unIdx++) {
      uint16_t unInc = (1 << (4 - unIdx)) * CHRG_CURRENT_BASE;
      /* set/clear the register bits for the current increment */
      if(un_batt_chrg_current_ma / unInc > 0) {
         un_batt_chrg_current_ma -= unInc;
         unRegVal |= (1 << ((4 - unIdx) + 3));
      }
      else {
         unRegVal &= ~(1 << ((4 - unIdx) + 3));
      }
   }
   /* Write value back to register */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R5_ADDR);
   CFirmware::GetInstance().GetTWController().Write(unRegVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::SetBatteryTerminationCurrent(uint16_t un_batt_term_current_ma) {
   /* check if the requested current is in range */
   if(un_batt_term_current_ma < TERM_CURRENT_OFFSET || 
      un_batt_term_current_ma > 2875)
      return;

   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R5_ADDR);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 1, true);
   uint8_t unRegVal = CFirmware::GetInstance().GetTWController().Read();          
   /* decrement by the internal offset current */
   un_batt_term_current_ma -= TERM_CURRENT_OFFSET;
   /* loop over the different increments to determine register programming */
   for(uint16_t unIdx = 0; unIdx < 3; unIdx++) {
      uint16_t unInc = (1 << (2 - unIdx)) * TERM_CURRENT_BASE;
      /* set/clear the register bits for the current increment */
      if(un_batt_term_current_ma / unInc > 0) {
         un_batt_term_current_ma -= unInc;
         unRegVal |= (1 << (2 - unIdx));
      }
      else {
         unRegVal &= ~(1 << (2 - unIdx));
      }
   }
   /* Write value back to register */
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(R5_ADDR);
   CFirmware::GetInstance().GetTWController().Write(unRegVal);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

void CBQ24161Module::Synchronize() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(BQ24161_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(BQ24161_ADDR, 2, true);

   uint8_t punRegisters[2];

   punRegisters[0] = CFirmware::GetInstance().GetTWController().Read();
   punRegisters[1] = CFirmware::GetInstance().GetTWController().Read();

   /* update the preferred source variable */
   ePreferredSource = ((punRegisters[0] & R0_SUPPLY_MASK) == 0) ?
      ESource::ADAPTER : ESource::USB;

   /* update the selected source & state variables */
   switch((punRegisters[0] & R0_STAT_MASK) >> 4) {
   case 0x00:
      eSelectedSource = ESource::NONE;
      eDeviceState = EDeviceState::STANDBY;
      break;
   case 0x01:
      eSelectedSource = ESource::ADAPTER;
      eDeviceState = EDeviceState::READY;
      break;
   case 0x02:
      eSelectedSource = ESource::USB;
      eDeviceState = EDeviceState::READY;
      break;
   case 0x03:
      eSelectedSource = ESource::ADAPTER;
      eDeviceState = EDeviceState::CHARGING;
      break;
   case 0x04:
      eSelectedSource = ESource::USB;
      eDeviceState = EDeviceState::CHARGING;
      break;
   case 0x05:
      eSelectedSource = ESource::NONE;
      eDeviceState = EDeviceState::DONE;
      break;
   case 0x06:
      eSelectedSource = ESource::NONE;
      eDeviceState = EDeviceState::FAULT;
      break;
   case 0x07:
      eSelectedSource = ESource::NONE;
      eDeviceState = EDeviceState::FAULT;
      break;
   }  
   
   /* udpate the fault variable */
   switch(punRegisters[0] & R0_FAULT_MASK) {
   case 0x00:
      eFault = EFault::NONE;
      break;
   case 0x01:
      eFault = EFault::DEV_THERMAL_SHDN;
      break;
   case 0x02:
      eFault = EFault::BATT_THERMAL_SHDN;
      break;
   case 0x03:
      eFault = EFault::WATCHDOG_TMR_EXPR;
      break;
   case 0x04:
      eFault = EFault::SAFETY_TMR_EXPR;
      break;
   case 0x05:
      eFault = EFault::ADAPTER_FAULT;
      break;
   case 0x06:
      eFault = EFault::USB_FAULT;
      break;
   case 0x07:
      eFault = EFault::BATT_FAULT;
      break;
   }

   /* update adapter input status variable */
   switch((punRegisters[1] & R1_ADAPTER_STAT_MASK) >> 6) {
   case 0x00:
      eAdapterInputState = EInputState::NORMAL;
      break;
   case 0x01:
      eAdapterInputState = EInputState::OVER_VOLTAGE;
      break;
   case 0x02:
      eAdapterInputState = EInputState::WEAK_SOURCE;
      break;
   case 0x03:
      eAdapterInputState = EInputState::UNDER_VOLTAGE;
      break;
   }  

   /* update USB input status variable */
   switch((punRegisters[1] & R1_USB_STAT_MASK) >> 4) {
   case 0x00:
      eUSBInputState = EInputState::NORMAL;
      break;
   case 0x01:
      eUSBInputState = EInputState::OVER_VOLTAGE;
      break;
   case 0x02:
      eUSBInputState = EInputState::WEAK_SOURCE;
      break;
   case 0x03:
      eUSBInputState = EInputState::UNDER_VOLTAGE;
      break;
   }

   /* update battery status variable */
   switch((punRegisters[1] & R1_BATT_STAT_MASK) >> 1) {
   case 0x00:
      eBatteryState = EBatteryState::NORMAL;
      break;
   case 0x01:
      eBatteryState = EBatteryState::OVER_VOLTAGE;
      break;
   case 0x02:
      eBatteryState = EBatteryState::DISCONNECTED;
      break;
   case 0x03:
      eBatteryState = EBatteryState::UNDEFINED;
      break;
   }
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::EFault CBQ24161Module::GetFault() {
   return eFault;
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::ESource CBQ24161Module::GetSelectedSource() {
   return eSelectedSource;
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::ESource CBQ24161Module::GetPreferredSource() {
   return ePreferredSource;
}

/***********************************************************/
/***********************************************************/
   
CBQ24161Module::EDeviceState CBQ24161Module::GetDeviceState() {
   return eDeviceState;
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::EInputState CBQ24161Module::GetInputState(ESource e_source) {
   switch(e_source) {
   case ESource::ADAPTER:
      return eAdapterInputState;
      break;
   case ESource::USB:
      return eUSBInputState;
      break;
   default:
      return EInputState::UNDER_VOLTAGE;
      break;
   }
}

/***********************************************************/
/***********************************************************/
   
CBQ24161Module::EBatteryState CBQ24161Module::GetBatteryState() {
   return eBatteryState;
}



