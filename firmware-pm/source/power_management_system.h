#ifndef POWER_MANAGEMENT_SYSTEM_H
#define POWER_MANAGEMENT_SYSTEM_H

#include <stdint.h>
#include <bq24161_module.h>
#include <bq24250_module.h>
#include <pca9633_module.h>
#include <adc_controller.h>
#include <interrupt.h>

class CPowerManagementSystem {
public:
   CPowerManagementSystem();

   void Init();

   void SetSystemPowerOn(bool b_set_power_on);

   void SetActuatorPowerOn(bool b_set_power_on);

   void SetPassthroughPowerOn(bool b_set_power_on);

   void SetActuatorInputLimitOverride(CBQ24250Module::EInputLimit e_actuator_input_limit_override);

   bool IsSystemPowerOn();

   bool IsActuatorPowerOn();

   bool IsPassthroughPowerOn();
   
   bool IsSystemBatteryCharging();

   bool IsActuatorBatteryCharging();
   
   CBQ24161Module::EInputLimit GetSystemInputLimit();
   
   CBQ24250Module::EInputLimit GetActuatorInputLimit();
    
   CBQ24161Module::EInputState GetAdapterInputState();

   CBQ24161Module::EInputState GetUSBInputState();

   void Update();

private:
   CBQ24161Module m_cSystemPowerManager;
   CBQ24250Module m_cActuatorPowerManager;

   CPCA9633Module m_cBatteryStatusLEDs;
   CPCA9633Module m_cInputStatusLEDs;

   uint16_t m_unSystemBatteryVoltage;
   uint16_t m_unActuatorBatteryVoltage;

   CBQ24250Module::EInputLimit m_eActuatorInputLimitOverride;
};

#endif
