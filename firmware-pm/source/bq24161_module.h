#ifndef BQ24161_MODULE_H
#define BQ24161_MODULE_H

#include <stdint.h>

class CBQ24161Module {
public:

   enum class EFault : uint8_t {
      NONE = 0,
      DEV_THERMAL_SHDN = 1,
      BATT_THERMAL_SHDN = 2,
      WATCHDOG_TMR_EXPR = 3,
      SAFETY_TMR_EXPR = 4,
      ADAPTER_FAULT = 5,
      USB_FAULT = 6,
      BATT_FAULT = 7
   };

   enum class ESource : uint8_t {
      NONE = 0,
      ADAPTER = 1,
      USB = 2
   };   

   enum class EDeviceState : uint8_t {
      STANDBY = 0,
      READY = 1,
      CHARGING = 2,
      DONE = 3,
      FAULT = 4
   };

   enum class EInputState : uint8_t {
      NORMAL = 0,
      OVER_VOLTAGE = 1,
      WEAK_SOURCE = 2,
      UNDER_VOLTAGE = 3
   };

   enum class EInputLimit : uint8_t {
      L0 = 0,
      L100 = 1,
      L150 = 2,
      L500 = 3,
      L800 = 4,
      L900 = 5,
      L1500 = 6,
      L2500 = 7
   };

   enum class EBatteryState : uint8_t {
      NORMAL = 0,
      OVER_VOLTAGE = 1,
      DISCONNECTED = 2,
      UNDEFINED = 3
   };
   
   void Synchronize();

   void DumpRegister(uint8_t un_addr);

   void SetInputLimit(ESource e_source, EInputLimit e_input_limit);

   EInputLimit GetInputLimit(ESource e_source);

   void SetChargingEnable(bool b_enable);

   void SetNoBattOperationEnable(bool b_enable);

   void SetBatteryRegulationVoltage(uint16_t un_batt_voltage_mv);

   void SetBatteryChargingCurrent(uint16_t un_batt_chrg_current_ma);

   void SetBatteryTerminationCurrent(uint16_t un_batt_term_current_ma);

   void ResetWatchdogTimer();

   EFault GetFault();

   ESource GetSelectedSource();

   ESource GetPreferredSource();
   
   EDeviceState GetDeviceState();

   EInputState GetInputState(ESource e_source);
   
   EBatteryState GetBatteryState();

private:
   EFault eFault;
   ESource eSelectedSource;
   ESource ePreferredSource;
   EDeviceState eDeviceState;
   EInputState eAdapterInputState, eUSBInputState;
   EBatteryState eBatteryState;
};

#endif
