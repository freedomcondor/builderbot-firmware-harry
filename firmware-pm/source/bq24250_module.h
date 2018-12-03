#ifndef BQ24250_MODULE_H
#define BQ24250_MODULE_H

#include <stdint.h>

class CBQ24250Module {
public:

   enum class EFault : uint8_t {
      NONE = 0,
      INPUT_OVER_VOLTAGE = 1,
      INPUT_UNDER_VOLTAGE = 2,
      SLEEP = 3,
      BATT_THERMAL_SHDN = 4,
      BATT_OVER_VOLTAGE = 5,
      DEV_THERMAL_SHDN = 6,
      DEV_TIMER_FAULT = 7,
      BATT_DISCONNECTED = 8,
      ISET_SHORTED = 9,
      INPUT_FAULT = 10,
      UNDEFINED = 11
   };

   enum class EDeviceState : uint8_t {
      READY = 0,
      CHARGING = 1,
      DONE = 2,
      FAULT = 3
   };

   enum class EInputLimit : uint8_t {
      L100 = 0,
      L150 = 1,
      L500 = 2,
      L900 = 3,
      L1500 = 4,
      L2000 = 5,
      LEXT = 6,
      LPTM = 7,
      LHIZ = 8
   };


   void SetChargingEnable(bool b_enable);
   void SetInputLimit(EInputLimit e_input_limit);

   EInputLimit GetInputLimit();
   
   void DumpRegister(uint8_t un_addr);

   void SetRegisterValue(uint8_t un_addr, uint8_t un_mask, uint8_t un_value);
   uint8_t GetRegisterValue(uint8_t un_addr, uint8_t un_mask);

   void ResetWatchdogTimer();

   bool GetWatchdogEnabled() {
      return bWatchdogEnabled;
   }

   bool GetWatchdogFault() {
      return bWatchdogFault;
   }

   void Synchronize();

   EFault GetFault() {
      return eFault;
   }
    
   EDeviceState GetDeviceState() {
      return eDeviceState;
   }

private:
   EFault eFault;
   EDeviceState eDeviceState;
   bool bWatchdogEnabled;
   bool bWatchdogFault;
};

#endif

