
#include "power_management_system.h"

#include <firmware.h>

/* Port D control pins */
#define PIN_ACTUATORS_EN 0x80
#define PIN_SYSTEM_EN 0x10
#define PIN_PASSTHROUGH_EN 0x40
#define PIN_VUSB50_L500_EN 0x20
#define PIN_SYSTEM_PWDN 0x08

/* Coefficient for converting ADC measurement to battery voltage.
   assumes a 1V1 reference and a 1M/330k voltage divider */
#define ADC_BATT_MV_COEFF 17u

/* TW addresses of configurable devices */
#define INPUT_STATUS_LEDS_ADDR 0x60
#define BATT_STATUS_LEDS_ADDR 0x61

/* Definitions for status LEDs */
#define ADP_LED_INDEX 0
#define USB_LP_LED_INDEX 1
#define USB_HP_LED_INDEX 2
#define USB_FP_LED_INDEX 3

#define BATT1_STAT_INDEX 0
#define BATT1_CHRG_INDEX 1
#define BATT2_STAT_INDEX 2
#define BATT2_CHRG_INDEX 3

/* Input voltages to PMICs mV */
#define SYS_INPUT_VOLTAGE 5000
#define ACT_INPUT_VOLTAGE 5000

/* Power requirements for various parts of the system in mW */
#define SYS_POWER_REQ 2500
#define ACT_POWER_REQ 15000
#define SYS_ACT_PASSTHROUGH_LOSS 50

/* Battery parameters (in mV, mA, mW) */
#define SYS_BATT_REG_VOLTAGE 4200L
#define SYS_BATT_INIT_CHG_VOLTAGE 4100
#define SYS_BATT_CHG_CURRENT 740
#define SYS_BATT_CHG_POWER ((SYS_BATT_CHG_CURRENT * SYS_BATT_REG_VOLTAGE) / 1000)
#define SYS_BATT_TRM_CURRENT 50
#define SYS_BATT_LOW_VOLTAGE 3200
#define SYS_BATT_NOTPRESENT_VOLTAGE 500

#define ACT_BATT_REG_VOLTAGE 4200L
#define ACT_BATT_INIT_CHG_VOLTAGE 4100
#define ACT_BATT_CHG_CURRENT 740
#define ACT_BATT_CHG_POWER ((ACT_BATT_CHG_CURRENT * ACT_BATT_REG_VOLTAGE) / 1000)
#define ACT_BATT_TRM_CURRENT 50
#define ACT_BATT_LOW_VOLTAGE 3200
#define ACT_BATT_NOTPRESENT_VOLTAGE 100

/***********************************************************/
/***********************************************************/

CPowerManagementSystem::CPowerManagementSystem() :
   m_cBatteryStatusLEDs(BATT_STATUS_LEDS_ADDR),
   m_cInputStatusLEDs(INPUT_STATUS_LEDS_ADDR),
   m_eActuatorInputLimitOverride(CBQ24250Module::EInputLimit::LHIZ) {}

/***********************************************************/
/***********************************************************/

void CPowerManagementSystem::Init() {
   /* Init base power configuration */
   SetSystemPowerOn(true);
   SetPassthroughPowerOn(true);
   SetActuatorPowerOn(false);

   /* Init output control pins - this overrides hardware pull ups / downs */
   DDRD |= (PIN_ACTUATORS_EN | PIN_SYSTEM_EN | PIN_PASSTHROUGH_EN);

   m_cActuatorPowerManager.SetInputLimit(CBQ24250Module::EInputLimit::LHIZ);

   m_cSystemPowerManager.SetChargingEnable(false);
   m_cActuatorPowerManager.SetChargingEnable(false);

   m_cSystemPowerManager.SetBatteryRegulationVoltage(SYS_BATT_REG_VOLTAGE);
   m_cSystemPowerManager.SetBatteryChargingCurrent(SYS_BATT_CHG_CURRENT);
   m_cSystemPowerManager.SetBatteryTerminationCurrent(SYS_BATT_TRM_CURRENT);

   CPCA9633Module::ResetDevices();
   m_cInputStatusLEDs.Init();
   m_cBatteryStatusLEDs.Init();

   Update();
}

/***********************************************************/
/***********************************************************/

void CPowerManagementSystem::SetSystemPowerOn(bool b_set_power_on) {
   if(b_set_power_on) {
      PORTD |= PIN_SYSTEM_EN;
   }
   else {
      PORTD &= ~PIN_SYSTEM_EN;
   }
}

/***********************************************************/
/***********************************************************/

void CPowerManagementSystem::SetActuatorPowerOn(bool b_set_power_on) {
   if(b_set_power_on) {
      PORTD |= PIN_ACTUATORS_EN;
   }
   else {
      PORTD &= ~PIN_ACTUATORS_EN;
   }
}

/***********************************************************/
/***********************************************************/

void CPowerManagementSystem::SetPassthroughPowerOn(bool b_set_power_on) {
   if(b_set_power_on) {
      PORTD |= PIN_PASSTHROUGH_EN;
   }
   else {
      PORTD &= ~PIN_PASSTHROUGH_EN;
   }
}

/***********************************************************/
/***********************************************************/

void CPowerManagementSystem::SetActuatorInputLimitOverride(CBQ24250Module::EInputLimit e_actuator_input_limit_override) {
   m_eActuatorInputLimitOverride = e_actuator_input_limit_override;
}

/***********************************************************/
/***********************************************************/

bool CPowerManagementSystem::IsSystemPowerOn() {
   return ((PORTD & PIN_SYSTEM_EN) != 0);
}

/***********************************************************/
/***********************************************************/

bool CPowerManagementSystem::IsActuatorPowerOn() {
   return ((PORTD & PIN_ACTUATORS_EN) != 0);
}

/***********************************************************/
/***********************************************************/

bool CPowerManagementSystem::IsPassthroughPowerOn() {
   return ((PORTD & PIN_PASSTHROUGH_EN) != 0);
}

/***********************************************************/
/***********************************************************/

void CPowerManagementSystem::Update() {
   /* Reset watchdogs and synchronise state with remote PMICs */
   m_cSystemPowerManager.ResetWatchdogTimer();
   m_cSystemPowerManager.Synchronize();
   m_cActuatorPowerManager.ResetWatchdogTimer();
   m_cActuatorPowerManager.Synchronize();

   /* check if the USB port is powered, and enable/disable the hub */
   if(m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::USB) ==
      CBQ24161Module::EInputState::NORMAL) {
      if(!CUSBInterfaceSystem::GetInstance().IsEnabled()) {
         CUSBInterfaceSystem::GetInstance().Enable();
      }
      /* Detect the available USB input power */
      if(CUSBInterfaceSystem::GetInstance().IsSuspended()) {
         m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L0);
      }
      else {
         switch(CUSBInterfaceSystem::GetInstance().GetUSBChargerType()) {   
         case CUSBInterfaceSystem::EUSBChargerType::DCP:
         case CUSBInterfaceSystem::EUSBChargerType::SE1S:
            m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L1500);
            break;
         case CUSBInterfaceSystem::EUSBChargerType::SE1H:
            m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L900);
            break;
         case CUSBInterfaceSystem::EUSBChargerType::CDP:
            if(CUSBInterfaceSystem::GetInstance().IsHighSpeedMode()) {
               m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L900);
            }
            else {
               m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L1500);
            }
            break;
         case CUSBInterfaceSystem::EUSBChargerType::SDP:
         case CUSBInterfaceSystem::EUSBChargerType::SE1L:
            m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L500);
            break;
         default:
            m_cSystemPowerManager.SetInputLimit(CBQ24161Module::ESource::USB, CBQ24161Module::EInputLimit::L0);
            break;
         }
      }
      m_cSystemPowerManager.Synchronize();
   }
   else {
      if(CUSBInterfaceSystem::GetInstance().IsEnabled()) {
         CUSBInterfaceSystem::GetInstance().Disable();
      }
   }
  
   /* Reflect the state of the system power sources on the LEDs */
   /* Adapter */
   switch(m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::ADAPTER)) {
   case CBQ24161Module::EInputState::NORMAL:
      m_cInputStatusLEDs.SetLEDMode(ADP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
      break;
   case CBQ24161Module::EInputState::UNDER_VOLTAGE:
      m_cInputStatusLEDs.SetLEDMode(ADP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
      break;
   default:
      /* Indicate error condition */
      m_cInputStatusLEDs.SetLEDMode(ADP_LED_INDEX, CPCA9633Module::ELEDMode::BLINK);
      break;
   }
   /* USB */
   switch(m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::USB)) {
   case CBQ24161Module::EInputState::NORMAL:
      switch(m_cSystemPowerManager.GetInputLimit(CBQ24161Module::ESource::USB)) {
      case CBQ24161Module::EInputLimit::L100:
      case CBQ24161Module::EInputLimit::L150:
         m_cInputStatusLEDs.SetLEDMode(USB_LP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
         m_cInputStatusLEDs.SetLEDMode(USB_HP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
         m_cInputStatusLEDs.SetLEDMode(USB_FP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
         break;
      case CBQ24161Module::EInputLimit::L500:
         m_cInputStatusLEDs.SetLEDMode(USB_LP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
         m_cInputStatusLEDs.SetLEDMode(USB_HP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
         m_cInputStatusLEDs.SetLEDMode(USB_FP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
         break;
      case CBQ24161Module::EInputLimit::L800:
      case CBQ24161Module::EInputLimit::L900:
      case CBQ24161Module::EInputLimit::L1500:
         m_cInputStatusLEDs.SetLEDMode(USB_LP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
         m_cInputStatusLEDs.SetLEDMode(USB_HP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
         m_cInputStatusLEDs.SetLEDMode(USB_FP_LED_INDEX, CPCA9633Module::ELEDMode::ON);
         break;
      default:
         /* Indicate error condition */
         m_cInputStatusLEDs.SetLEDMode(USB_LP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
         m_cInputStatusLEDs.SetLEDMode(USB_HP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
         m_cInputStatusLEDs.SetLEDMode(USB_FP_LED_INDEX, CPCA9633Module::ELEDMode::BLINK);
         break;
      }
      break;
   case CBQ24161Module::EInputState::UNDER_VOLTAGE:
      m_cInputStatusLEDs.SetLEDMode(USB_LP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
      m_cInputStatusLEDs.SetLEDMode(USB_HP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
      m_cInputStatusLEDs.SetLEDMode(USB_FP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
      break;
   default:
      /* Indicate error condition */
      m_cInputStatusLEDs.SetLEDMode(USB_LP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
      m_cInputStatusLEDs.SetLEDMode(USB_HP_LED_INDEX, CPCA9633Module::ELEDMode::OFF);
      m_cInputStatusLEDs.SetLEDMode(USB_FP_LED_INDEX, CPCA9633Module::ELEDMode::BLINK);
      break;
   }

   /* Determine power available to the system */
   uint16_t unAvailablePower = 0;
   /* create an ordered list of sources to be checked */
   CBQ24161Module::ESource peInputSourceList[3];
     
   switch(m_cSystemPowerManager.GetPreferredSource()) {
   case CBQ24161Module::ESource::ADAPTER:
      peInputSourceList[0] = CBQ24161Module::ESource::ADAPTER;
      peInputSourceList[1] = CBQ24161Module::ESource::USB;
      peInputSourceList[2] = CBQ24161Module::ESource::NONE;
      break;
   case CBQ24161Module::ESource::USB:
      peInputSourceList[0] = CBQ24161Module::ESource::USB;
      peInputSourceList[1] = CBQ24161Module::ESource::ADAPTER;
      peInputSourceList[2] = CBQ24161Module::ESource::NONE;
      break;
   default:
      peInputSourceList[0] = CBQ24161Module::ESource::NONE;
      peInputSourceList[1] = CBQ24161Module::ESource::NONE;
      peInputSourceList[2] = CBQ24161Module::ESource::NONE;
      break;
   }
   
   /* for each source check the input state and limit */
   for(CBQ24161Module::ESource eInputSource : peInputSourceList) {
      if(m_cSystemPowerManager.GetInputState(eInputSource) != CBQ24161Module::EInputState::NORMAL) {
         continue;
      }
      else {
         switch(m_cSystemPowerManager.GetInputLimit(eInputSource)) {
         case CBQ24161Module::EInputLimit::L0:
            unAvailablePower = 0 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L100:
            unAvailablePower = 100 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L150:
            unAvailablePower = 150 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L500:
            unAvailablePower = 500 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L800:
            unAvailablePower = 800 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L900:
            unAvailablePower = 900 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L1500:
            unAvailablePower = 1500 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         case CBQ24161Module::EInputLimit::L2500:
            unAvailablePower = 2500 * (SYS_INPUT_VOLTAGE / 1000);
            break;
         }
         /* At this point we have a valid input source selected, terminate the search */
         if(unAvailablePower > 0) break;
      }
   }

   /* Read battery voltages */
   m_unSystemBatteryVoltage =
      CADCController::GetInstance().GetValue(CADCController::EChannel::ADC6) * ADC_BATT_MV_COEFF;
   m_unActuatorBatteryVoltage =
      CADCController::GetInstance().GetValue(CADCController::EChannel::ADC7) * ADC_BATT_MV_COEFF;

   /* Allocate power to the system if switched on */
   if(IsSystemPowerOn()) {
      if(unAvailablePower > SYS_POWER_REQ) {
         unAvailablePower -= SYS_POWER_REQ;
      }
      else {
         unAvailablePower = 0;
      }
   }

   /* Enable charging the system battery if at least a third of the charging power
      is available. BQ24161 automatically prioritizes system power. */
   if(m_cSystemPowerManager.GetBatteryState() != CBQ24161Module::EBatteryState::NORMAL ||
      m_cSystemPowerManager.GetFault() == CBQ24161Module::EFault::BATT_FAULT ||
      m_cSystemPowerManager.GetFault() == CBQ24161Module::EFault::BATT_THERMAL_SHDN) {
      /* Sometimes a fault is caused by the remote PMIC losing it's configuration */
      /* resend the configuration if this is the case */
      m_cSystemPowerManager.SetBatteryRegulationVoltage(SYS_BATT_REG_VOLTAGE);
      m_cSystemPowerManager.SetBatteryChargingCurrent(SYS_BATT_CHG_CURRENT);
      m_cSystemPowerManager.SetBatteryTerminationCurrent(SYS_BATT_TRM_CURRENT);
   
      /* If we are charging, disable it */
      if(m_cSystemPowerManager.GetDeviceState() == CBQ24161Module::EDeviceState::CHARGING) {
         m_cSystemPowerManager.SetChargingEnable(false);
      }
      m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
      /* Depending on the battery voltage, it is either not present or we have an actual fault */
      if(m_unSystemBatteryVoltage < SYS_BATT_NOTPRESENT_VOLTAGE) {
         /* Battery isn't present */
         m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::OFF);
      }
      else {
         /* indicate error condition */
         m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::BLINK);
      }
   }
   else { /* Battery is present and in a normal state */
      if(m_cSystemPowerManager.GetDeviceState() == CBQ24161Module::EDeviceState::CHARGING) {
         if(unAvailablePower > (SYS_BATT_CHG_POWER / 3)) {
            unAvailablePower -= (SYS_BATT_CHG_POWER / 3);
            m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::BLINK);
            m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
         }
         else {
            /* Terminate charging */
            m_cSystemPowerManager.SetChargingEnable(false);
            m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
            m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
         }
      }
      else { /* Battery is not currently charging */
         if(m_unSystemBatteryVoltage < SYS_BATT_INIT_CHG_VOLTAGE) {
            if(unAvailablePower > (SYS_BATT_CHG_POWER / 3)) {
               unAvailablePower -= (SYS_BATT_CHG_POWER / 3);
               /* Resend parameters due to the BQ24161 being a piece of garbage */
               m_cSystemPowerManager.SetBatteryRegulationVoltage(SYS_BATT_REG_VOLTAGE);
               m_cSystemPowerManager.SetBatteryChargingCurrent(SYS_BATT_CHG_CURRENT);
               m_cSystemPowerManager.SetBatteryTerminationCurrent(SYS_BATT_TRM_CURRENT);
               /* Enable the charging */
               m_cSystemPowerManager.SetChargingEnable(true);
               m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::BLINK);
               m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
            }
            else { /* There is not sufficient power available to charge the battery */
               if(m_unSystemBatteryVoltage > ACT_BATT_LOW_VOLTAGE) {
                  m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
                  m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
               }
               else { /* Indicate low battery condition */
                  m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
                  m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::BLINK);
               }
            }
         }
         else {
            /* Battery is present and already charged */
            m_cBatteryStatusLEDs.SetLEDMode(BATT1_CHRG_INDEX, CPCA9633Module::ELEDMode::ON);
            m_cBatteryStatusLEDs.SetLEDMode(BATT1_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
         }
      }
   }

   /* Deduct a passthrough loss from the available power to compensate for regulation losses
      and ensure system stability */
   if(unAvailablePower > SYS_ACT_PASSTHROUGH_LOSS) {
      unAvailablePower -= SYS_ACT_PASSTHROUGH_LOSS;
   }
   else {
      unAvailablePower = 0;
   }

   /* eActuatorInputLimit defines the remaining power that we forward to the actuator system */
   CBQ24250Module::EInputLimit eActuatorInputLimit;

   if(m_eActuatorInputLimitOverride == CBQ24250Module::EInputLimit::LHIZ) {
      /* Select eInputLimit, depending on the remaining power */
      if(unAvailablePower > 900 * (ACT_INPUT_VOLTAGE / 1000)) {
         eActuatorInputLimit = CBQ24250Module::EInputLimit::L900;
      }
      else if(unAvailablePower > 500 * (ACT_INPUT_VOLTAGE / 1000)) {
         eActuatorInputLimit = CBQ24250Module::EInputLimit::L500;
      }
      else if(unAvailablePower > 150 * (ACT_INPUT_VOLTAGE / 1000)) {
         eActuatorInputLimit = CBQ24250Module::EInputLimit::L150;
      }
      else if(unAvailablePower > 100 * (ACT_INPUT_VOLTAGE / 1000)) {
         eActuatorInputLimit = CBQ24250Module::EInputLimit::L100;
      }
      else {
         /* If the remaining power on the system line is less than 500mW there is no point
            forwarding it to the actuator system. Note, when in HIZ the BQ24250 still operates */
         eActuatorInputLimit = CBQ24250Module::EInputLimit::LHIZ;
         unAvailablePower = 0;
      }
   }
   else {
      eActuatorInputLimit = m_eActuatorInputLimitOverride;
   }

   /* Set the input limit */
   m_cActuatorPowerManager.SetInputLimit(eActuatorInputLimit);
   /* Allocate power to the actuators if switched on */
   if(IsActuatorPowerOn()) {
      if(unAvailablePower > ACT_POWER_REQ) {
         unAvailablePower -= ACT_POWER_REQ;
      }
      else {
         unAvailablePower = 0;
      }
   }

   /* Enable charging the actuator battery if at least a third of the charging power
      is available. BQ24250 automatically prioritizes actuator power. */
   switch(m_cActuatorPowerManager.GetDeviceState()) {
   case CBQ24250Module::EDeviceState::FAULT:
      /* Fault condition - terminate charging */
      m_cActuatorPowerManager.SetChargingEnable(false);
      if(m_cActuatorPowerManager.GetFault() == CBQ24250Module::EFault::BATT_OVER_VOLTAGE ||
         m_cActuatorPowerManager.GetFault() == CBQ24250Module::EFault::BATT_DISCONNECTED) {
         /* Battery fault */
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::BLINK);
      }
      else if(m_cActuatorPowerManager.GetFault() == CBQ24250Module::EFault::BATT_THERMAL_SHDN) {
         /* Terminate charging */
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
         /* Handle special case, since disconnected battery removes thermistor network
            generating a BATT_THERMAL_SHDN fault */
         if(m_unActuatorBatteryVoltage < ACT_BATT_NOTPRESENT_VOLTAGE) {
            m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::OFF);
         }
         else {
            /* We have an actual thermal fault */
            m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::BLINK);
         }
      }
      else {
         /* All other faults are considered charger faults */
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::BLINK);
         /* Indicate if the battery is present */
         if(m_unActuatorBatteryVoltage < ACT_BATT_NOTPRESENT_VOLTAGE) {
            m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::OFF);
         }
         else {
            m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
         }
      }
      break;
   case CBQ24250Module::EDeviceState::READY:
      if(m_unActuatorBatteryVoltage > ACT_BATT_NOTPRESENT_VOLTAGE) {
         /* Indicate battery is present */
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
         if(m_unActuatorBatteryVoltage < ACT_BATT_INIT_CHG_VOLTAGE &&
            unAvailablePower > (ACT_BATT_CHG_POWER / 3)) {
            unAvailablePower -= (ACT_BATT_CHG_POWER / 3);
            m_cActuatorPowerManager.SetChargingEnable(true);
            m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::BLINK);
         }
         else {
            m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
         }
      }
      else { 
         /* m_unActuatorBatteryVoltage <= ACT_BATT_NOTPRESENT_VOLTAGE */
         m_cActuatorPowerManager.SetChargingEnable(false);
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::OFF);
      }
      break;
   case CBQ24250Module::EDeviceState::CHARGING:
      m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
      if(unAvailablePower > (ACT_BATT_CHG_POWER / 3)) {
         unAvailablePower -= (ACT_BATT_CHG_POWER / 3);
         /* since we are in the charging state, there should be no need to execute
            m_cActuatorPowerManager.SetChargingEnable(true) here */
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::BLINK);
      }
      else {
         m_cActuatorPowerManager.SetChargingEnable(false);
         m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
      }
      break;
   case CBQ24250Module::EDeviceState::DONE:
      m_cActuatorPowerManager.SetChargingEnable(false);
      m_cBatteryStatusLEDs.SetLEDMode(BATT2_CHRG_INDEX, CPCA9633Module::ELEDMode::OFF);
      m_cBatteryStatusLEDs.SetLEDMode(BATT2_STAT_INDEX, CPCA9633Module::ELEDMode::ON);
      break;
   }
}

/***********************************************************/
/***********************************************************/

bool CPowerManagementSystem::IsSystemBatteryCharging() {
   return (m_cSystemPowerManager.GetDeviceState() == CBQ24161Module::EDeviceState::CHARGING);
}

/***********************************************************/
/***********************************************************/

bool CPowerManagementSystem::IsActuatorBatteryCharging() {
   return (m_cActuatorPowerManager.GetDeviceState() == CBQ24250Module::EDeviceState::CHARGING);
}

/***********************************************************/
/***********************************************************/

CBQ24250Module::EInputLimit CPowerManagementSystem::GetActuatorInputLimit() {
   return m_cActuatorPowerManager.GetInputLimit();
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::EInputLimit CPowerManagementSystem::GetSystemInputLimit() {
   CBQ24161Module::ESource eSource = m_cSystemPowerManager.GetSelectedSource();
   return m_cSystemPowerManager.GetInputLimit(eSource);
}

/***********************************************************/
/***********************************************************/
 
CBQ24161Module::EInputState CPowerManagementSystem::GetAdapterInputState() {
  return m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::ADAPTER);
}

/***********************************************************/
/***********************************************************/

CBQ24161Module::EInputState CPowerManagementSystem::GetUSBInputState() {
  return m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::USB);
}

/***********************************************************/
/***********************************************************/

/*
void CPowerManagementSystem::PrintStatus() {
   fprintf(CFirmware::GetInstance().m_psHUART, "<Power Domains>\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "system: %s\r\n",
           IsSystemPowerOn()?"on":"off");
   fprintf(CFirmware::GetInstance().m_psHUART, "actuators: %s\r\n",
           IsActuatorPowerOn()?"on":"off");
   fprintf(CFirmware::GetInstance().m_psHUART, "<Batteries>\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "Batt 1: %u\r\n",
           CADCController::GetInstance().GetValue(CADCController::EChannel::ADC6) * ADC_BATT_MV_COEFF);
   fprintf(CFirmware::GetInstance().m_psHUART, "Batt 2: %u\r\n",
           CADCController::GetInstance().GetValue(CADCController::EChannel::ADC7) * ADC_BATT_MV_COEFF);

   // System power manager
   m_cSystemPowerManager.Synchronize();
   fprintf(CFirmware::GetInstance().m_psHUART, "<System PMIC>\r\n");
   for(uint8_t i = 0; i < 7; i++) {
      m_cSystemPowerManager.DumpRegister(i);
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "---\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "state: ");
   switch(m_cSystemPowerManager.GetDeviceState()) {
   case CBQ24161Module::EDeviceState::STANDBY:
      fprintf(CFirmware::GetInstance().m_psHUART, "STBY");
      break;
   case CBQ24161Module::EDeviceState::READY:
      fprintf(CFirmware::GetInstance().m_psHUART, "RDY");
      break;
   case CBQ24161Module::EDeviceState::CHARGING:
      fprintf(CFirmware::GetInstance().m_psHUART, "CHG");
      break;
   case CBQ24161Module::EDeviceState::DONE:
      fprintf(CFirmware::GetInstance().m_psHUART, "DONE");
      break;
   case CBQ24161Module::EDeviceState::FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "FAULT");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "fault: ");
   switch(m_cSystemPowerManager.GetFault()) {
   case CBQ24161Module::EFault::NONE:
      fprintf(CFirmware::GetInstance().m_psHUART, "NA");
      break;
   case CBQ24161Module::EFault::DEV_THERMAL_SHDN:
      fprintf(CFirmware::GetInstance().m_psHUART, "DTHM_SHDN");
      break;
   case CBQ24161Module::EFault::BATT_THERMAL_SHDN:
      fprintf(CFirmware::GetInstance().m_psHUART, "BTHM_SHDN");
      break;
   case CBQ24161Module::EFault::WATCHDOG_TMR_EXPR:
      fprintf(CFirmware::GetInstance().m_psHUART, "WTMR_EXP");
      break;
   case CBQ24161Module::EFault::SAFETY_TMR_EXPR:
      fprintf(CFirmware::GetInstance().m_psHUART, "STMR_EXP");
      break;
   case CBQ24161Module::EFault::ADAPTER_FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "ADP");
      break;
   case CBQ24161Module::EFault::USB_FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "USB");
      break;
   case CBQ24161Module::EFault::BATT_FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "BATT");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "selected source: ");
   switch(m_cSystemPowerManager.GetSelectedSource()) {
   case CBQ24161Module::ESource::NONE:
      fprintf(CFirmware::GetInstance().m_psHUART, "NA");
      break;
   case CBQ24161Module::ESource::ADAPTER:
      fprintf(CFirmware::GetInstance().m_psHUART, "ADP");
      break;
   case CBQ24161Module::ESource::USB:
      fprintf(CFirmware::GetInstance().m_psHUART, "USB");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "ADP input: ");
   switch(m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::ADAPTER)) {
   case CBQ24161Module::EInputState::NORMAL:
      fprintf(CFirmware::GetInstance().m_psHUART, "OK");
      break;
   case CBQ24161Module::EInputState::OVER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "OV");
      break;
   case CBQ24161Module::EInputState::WEAK_SOURCE:
      fprintf(CFirmware::GetInstance().m_psHUART, "WS");
      break;
   case CBQ24161Module::EInputState::UNDER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "UV");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "USB input: ");
   switch(m_cSystemPowerManager.GetInputState(CBQ24161Module::ESource::USB)) {
   case CBQ24161Module::EInputState::NORMAL:
      fprintf(CFirmware::GetInstance().m_psHUART, "OK");
      break;
   case CBQ24161Module::EInputState::OVER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "OV");
      break;
   case CBQ24161Module::EInputState::WEAK_SOURCE:
      fprintf(CFirmware::GetInstance().m_psHUART, "WS");
      break;
   case CBQ24161Module::EInputState::UNDER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "UV");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "battery_state: ");
   switch(m_cSystemPowerManager.GetBatteryState()) {
   case CBQ24161Module::EBatteryState::NORMAL:
      fprintf(CFirmware::GetInstance().m_psHUART, "OK");
      break;
   case CBQ24161Module::EBatteryState::OVER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "OV");
      break;
   case CBQ24161Module::EBatteryState::DISCONNECTED:
      fprintf(CFirmware::GetInstance().m_psHUART, "DIS");
      break;
   case CBQ24161Module::EBatteryState::UNDEFINED:
      fprintf(CFirmware::GetInstance().m_psHUART, "UNDEF");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "input_limit: ");
   switch(m_cSystemPowerManager.GetInputLimit(
          m_cSystemPowerManager.GetSelectedSource())) {
   case CBQ24161Module::EInputLimit::L0:
      fprintf(CFirmware::GetInstance().m_psHUART, "0");
      break;
   case CBQ24161Module::EInputLimit::L100:
      fprintf(CFirmware::GetInstance().m_psHUART, "100");
      break;
   case CBQ24161Module::EInputLimit::L150:
      fprintf(CFirmware::GetInstance().m_psHUART, "150");
      break;
   case CBQ24161Module::EInputLimit::L500:
      fprintf(CFirmware::GetInstance().m_psHUART, "500");
      break;
   case CBQ24161Module::EInputLimit::L800:
      fprintf(CFirmware::GetInstance().m_psHUART, "800");
      break;
   case CBQ24161Module::EInputLimit::L900:
      fprintf(CFirmware::GetInstance().m_psHUART, "900");
      break;
   case CBQ24161Module::EInputLimit::L1500:
      fprintf(CFirmware::GetInstance().m_psHUART, "1500");
      break;
   case CBQ24161Module::EInputLimit::L2500:
      fprintf(CFirmware::GetInstance().m_psHUART, "2500");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");

   // Actuator power manager
   m_cActuatorPowerManager.Synchronize();
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "<Actuator PMIC>\r\n");
   for(uint8_t i = 0; i < 2; i++) {
      m_cActuatorPowerManager.DumpRegister(i);
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "---\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "state: ");
   switch(m_cActuatorPowerManager.GetDeviceState()) {
   case CBQ24250Module::EDeviceState::READY:
      fprintf(CFirmware::GetInstance().m_psHUART, "RDY");
      break;
   case CBQ24250Module::EDeviceState::CHARGING:
      fprintf(CFirmware::GetInstance().m_psHUART, "CHG");
      break;
   case CBQ24250Module::EDeviceState::DONE:
      fprintf(CFirmware::GetInstance().m_psHUART, "DONE");
      break;
   case CBQ24250Module::EDeviceState::FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "FAULT");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "fault: ");
   switch(m_cActuatorPowerManager.GetFault()) {
   case CBQ24250Module::EFault::NONE:
      fprintf(CFirmware::GetInstance().m_psHUART, "NA");
      break;
   case CBQ24250Module::EFault::INPUT_OVER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "IN_OV");
      break;
   case CBQ24250Module::EFault::INPUT_UNDER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "IN_UV");
      break;
   case CBQ24250Module::EFault::SLEEP:
      fprintf(CFirmware::GetInstance().m_psHUART, "SLP");
      break;
   case CBQ24250Module::EFault::BATT_THERMAL_SHDN:
      fprintf(CFirmware::GetInstance().m_psHUART, "BTHM_SHDN");
      break;
   case CBQ24250Module::EFault::BATT_OVER_VOLTAGE:
      fprintf(CFirmware::GetInstance().m_psHUART, "B_OV");
      break;
   case CBQ24250Module::EFault::DEV_THERMAL_SHDN:
      fprintf(CFirmware::GetInstance().m_psHUART, "DTHM_SHDN");
      break;
   case CBQ24250Module::EFault::DEV_TIMER_FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "DTMR_FAIL");
      break;
   case CBQ24250Module::EFault::BATT_DISCONNECTED:
      fprintf(CFirmware::GetInstance().m_psHUART, "NO_BATT");
      break;
   case CBQ24250Module::EFault::ISET_SHORTED:
      fprintf(CFirmware::GetInstance().m_psHUART, "ISET_FAIL");
      break;
   case CBQ24250Module::EFault::INPUT_FAULT:
      fprintf(CFirmware::GetInstance().m_psHUART, "IN_FAIL");
      break;
   case CBQ24250Module::EFault::UNDEFINED:
      fprintf(CFirmware::GetInstance().m_psHUART, "UNDEF");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART, "input_limit: ");
   switch(m_cActuatorPowerManager.GetInputLimit()) {
   case CBQ24250Module::EInputLimit::L100:
      fprintf(CFirmware::GetInstance().m_psHUART, "100");
      break;
   case CBQ24250Module::EInputLimit::L150:
      fprintf(CFirmware::GetInstance().m_psHUART, "150");
      break;
   case CBQ24250Module::EInputLimit::L500:
      fprintf(CFirmware::GetInstance().m_psHUART, "500");
      break;
   case CBQ24250Module::EInputLimit::L900:
      fprintf(CFirmware::GetInstance().m_psHUART, "900");
      break;
   case CBQ24250Module::EInputLimit::L1500:
      fprintf(CFirmware::GetInstance().m_psHUART, "1500");
      break;
   case CBQ24250Module::EInputLimit::L2000:
      fprintf(CFirmware::GetInstance().m_psHUART, "2000");
      break;
   case CBQ24250Module::EInputLimit::LEXT:
      fprintf(CFirmware::GetInstance().m_psHUART, "EXT");
      break;
   case CBQ24250Module::EInputLimit::LPTM:
      fprintf(CFirmware::GetInstance().m_psHUART, "PTM");
      break;
   case CBQ24250Module::EInputLimit::LHIZ:
      fprintf(CFirmware::GetInstance().m_psHUART, "HIZ");
      break;
   }
   fprintf(CFirmware::GetInstance().m_psHUART, "\r\n");
   fprintf(CFirmware::GetInstance().m_psHUART,
           "wd_en: %c\r\n",
           m_cActuatorPowerManager.GetWatchdogEnabled()?'t':'f');
   fprintf(CFirmware::GetInstance().m_psHUART,
           "wd_fault: %c\r\n",
           m_cActuatorPowerManager.GetWatchdogFault()?'t':'f');
}

*/

