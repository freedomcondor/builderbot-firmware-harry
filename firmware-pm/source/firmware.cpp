#include "firmware.h"

#include <pca9554_module.h>

/***********************************************************/
/***********************************************************/

#define PORTC_SWITCH_IRQ 0x01
#define PORTC_HUB_IRQ 0x02
#define PORTC_SYSTEM_POWER_IRQ 0x04
#define PORTC_ACTUATOR_POWER_IRQ 0x08

/***********************************************************/
/***********************************************************/

#define SYNC_PERIOD 5000
#define HARD_PWDN_PERIOD 750

/***********************************************************/
/***********************************************************/

/* initialisation of the static singleton */
CFirmware CFirmware::_firmware;

/* main function that runs the firmware */
int main(void)
{
   /* FILE structs for fprintf */
   FILE huart;

   /* Set up FILE structs for fprintf */                           
   fdev_setup_stream(&huart, 
                     [](char c_to_write, FILE* pf_stream) {
                        CFirmware::GetInstance().GetHUARTController().Write(c_to_write);
                        return 1;
                     },
                     [](FILE* pf_stream) {
                        return int(CFirmware::GetInstance().GetHUARTController().Read());
                     },
                     _FDEV_SETUP_RW);

   CFirmware::GetInstance().SetFilePointer(&huart);

   /* Execute the firmware */
   CFirmware::GetInstance().Exec();
   /* Terminate */
   return 0;
}

/***********************************************************/
/***********************************************************/

uint8_t CFirmware::GetId() {
   return ~CPCA9554Module<0x20>::GetInstance().GetRegister(CPCA9554Module<0x20>::ERegister::INPUT);
}

/***********************************************************/
/***********************************************************/

CFirmware::CPowerEventInterrupt::CPowerEventInterrupt(CFirmware* pc_firmware, 
                                                      uint8_t un_intr_vect_num) : 
   m_pcFirmware(pc_firmware) {
   Register(this, un_intr_vect_num);
}

/***********************************************************/
/***********************************************************/

void CFirmware::CPowerEventInterrupt::Enable() {
   /* Enable port change interrupts for external events */ 
   PCMSK1 |= ((1 << PCINT8)  | (1 << PCINT9) | 
              (1 << PCINT10) | (1 << PCINT11));
   /* Assume all interrupt sources except the switch could have fired */
   m_unPortLast = ~PINC | PORTC_SWITCH_IRQ;
   /* Enable the port change interrupt group PCINT[14:8] */
   PCICR |= (1 << PCIE1);
   /* Manually execute the service routine once */
   ServiceRoutine();
}

/***********************************************************/
/***********************************************************/

void CFirmware::CPowerEventInterrupt::Disable() {
   /* Disable port change interrupts for external events */ 
   PCMSK1 &= ~((1 << PCINT8)  | (1 << PCINT9) | 
               (1 << PCINT10) | (1 << PCINT11));
   /* Disable the port change interrupt group PCINT[14:8] */
   PCICR &= ~(1 << PCIE1);
}

/***********************************************************/
/***********************************************************/

void CFirmware::CPowerEventInterrupt::ServiceRoutine() {
   uint8_t unPortSnapshot = PINC;
   uint8_t unPortDelta = m_unPortLast ^ unPortSnapshot;

   /* Check power switch state */
   if(unPortDelta & PORTC_SWITCH_IRQ) {
      m_pcFirmware->m_bSwitchSignal = true;
      m_pcFirmware->m_eSwitchState = (unPortSnapshot & PORTC_SWITCH_IRQ) ?
         CFirmware::ESwitchState::RELEASED :
         CFirmware::ESwitchState::PRESSED;
   }
   /* Check USB hub system state */
   if(unPortDelta & PORTC_HUB_IRQ) {
      /* signal is true if it was a falling edge */
      m_pcFirmware->m_bUSBSignal = 
         m_pcFirmware->m_bUSBSignal ||
         ((unPortSnapshot & PORTC_HUB_IRQ) == 0);
   }
   /* Check system power manager state */
   if(unPortDelta & PORTC_SYSTEM_POWER_IRQ) {
      /* signal is true if it was a falling edge */
      m_pcFirmware->m_bSystemPowerSignal = 
         m_pcFirmware->m_bSystemPowerSignal ||
         ((unPortSnapshot & PORTC_SYSTEM_POWER_IRQ) == 0);
   }
   /* Check actuator power manager state */
   if(unPortDelta & PORTC_ACTUATOR_POWER_IRQ) {
      /* signal is true if it was a falling edge */
      m_pcFirmware->m_bActuatorPowerSignal = 
         m_pcFirmware->m_bActuatorPowerSignal ||
         ((unPortSnapshot & PORTC_ACTUATOR_POWER_IRQ) == 0);
   }
   m_unPortLast = unPortSnapshot;
}

/***********************************************************/
/***********************************************************/

void CFirmware::Exec() 
{
   uint32_t unLastSyncTime = 0;
   uint32_t unSwitchPressedTime = 0;
   bool bSyncRequiredSignal = false;

   m_cPowerManagementSystem.Init();
   m_cPowerEventInterrupt.Enable();

   for(;;) {
      /* Respond to interrupt signals */
      if(m_bSwitchSignal || m_bUSBSignal || m_bSystemPowerSignal || m_bActuatorPowerSignal) {
         if(m_bSwitchSignal) {
            m_bSwitchSignal = false;
            if(m_eSwitchState == ESwitchState::PRESSED) {
               unSwitchPressedTime = GetTimer().GetMilliseconds();
            }
         }
         if(m_bUSBSignal) {
            m_bUSBSignal = false;
            //bSyncRequiredSignal = true;
         }
         if(m_bSystemPowerSignal || m_bActuatorPowerSignal) {
            m_bSystemPowerSignal = false;
            m_bActuatorPowerSignal = false;
            /* assert the sync required signal */
            bSyncRequiredSignal = true;
         }
      }

      /* Check if an update is required */
      if((GetTimer().GetMilliseconds() - unLastSyncTime > SYNC_PERIOD) || bSyncRequiredSignal) {
         /* Update last sync time variable */
         unLastSyncTime = GetTimer().GetMilliseconds();
         /* Deassert the sync required signal */
         bSyncRequiredSignal = false;
         /* Run the update loop for the power mangement system */
         m_cPowerManagementSystem.Update();
      }

      /* Handle the switch state */
      if(m_eSwitchState == ESwitchState::PRESSED) {
         if(m_cPowerManagementSystem.IsSystemPowerOn()) {
            if(GetTimer().GetMilliseconds() - unSwitchPressedTime > HARD_PWDN_PERIOD) {
               /* hard power down */
               m_cPowerManagementSystem.SetActuatorPowerOn(false);
               m_cPowerManagementSystem.SetSystemPowerOn(false);
               /* Set m_eSwitchState to ESwitchState::RELEASED indicate that the switch
                  pressed event has been handled */
               m_eSwitchState = ESwitchState::RELEASED;
               /* Assert the sync required signal */
               bSyncRequiredSignal = true;
            }
            /* Soft power down */
            else {
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::REQ_SOFT_PWDN);
            }
         }
         else { /* !m_cPowerManagementSystem.IsSystemPowerOn() */
            m_cPowerManagementSystem.SetSystemPowerOn(true);
            /* Set m_eSwitchState to ESwitchState::RELEASED indicate that the switch
               pressed event has been handled */
            m_eSwitchState = ESwitchState::RELEASED;
            /* Assert the sync required signal */
            bSyncRequiredSignal = true;
         }
      }
      
      /* Process inbound packets */
      m_cPacketControlInterface.ProcessInput();

      if(m_cPacketControlInterface.GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
         CPacketControlInterface::CPacket cPacket = m_cPacketControlInterface.GetPacket();
         switch(cPacket.GetType()) {
         case CPacketControlInterface::CPacket::EType::GET_UPTIME:
            if(cPacket.GetDataLength() == 0) {
               uint32_t unUptime = m_cTimer.GetMilliseconds();
               uint8_t punTxData[] = {
                  uint8_t((unUptime >> 24) & 0xFF),
                  uint8_t((unUptime >> 16) & 0xFF),
                  uint8_t((unUptime >> 8 ) & 0xFF),
                  uint8_t((unUptime >> 0 ) & 0xFF)
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_UPTIME,
                                                    punTxData,
                                                    sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_BATT_LVL:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] = {
                  CADCController::GetInstance().GetValue(CADCController::EChannel::ADC6),
                  CADCController::GetInstance().GetValue(CADCController::EChannel::ADC7)         
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_BATT_LVL,
                                                    punTxData,
                                                    sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_PM_STATUS:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] = {
                  m_cPowerManagementSystem.IsSystemPowerOn(),
                  m_cPowerManagementSystem.IsActuatorPowerOn(),
                  m_cPowerManagementSystem.IsPassthroughPowerOn(),
                  m_cPowerManagementSystem.IsSystemBatteryCharging(),
                  m_cPowerManagementSystem.IsActuatorBatteryCharging(),
                  static_cast<uint8_t>(m_cPowerManagementSystem.GetSystemInputLimit()),
                  static_cast<uint8_t>(m_cPowerManagementSystem.GetActuatorInputLimit()),
                  static_cast<uint8_t>(m_cPowerManagementSystem.GetAdapterInputState()),
                  static_cast<uint8_t>(m_cPowerManagementSystem.GetUSBInputState()),
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_PM_STATUS,
                                                    punTxData,
                                                    sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_USB_STATUS:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] = {
                  CUSBInterfaceSystem::GetInstance().IsEnabled(),
                  CUSBInterfaceSystem::GetInstance().IsHighSpeedMode(),
                  CUSBInterfaceSystem::GetInstance().IsSuspended(),
                  static_cast<uint8_t>(CUSBInterfaceSystem::GetInstance().GetUSBChargerType()),
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_USB_STATUS,
                                                    punTxData,
                                                    sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_SYSTEM_POWER_ENABLE:
            /* Set the enable signal for the actuator power supply */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               m_cPowerManagementSystem.SetSystemPowerOn((punRxData[0] != 0) ? true : false);
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_ACTUATOR_POWER_ENABLE:
            /* Set the enable signal for the actuator power supply */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               m_cPowerManagementSystem.SetActuatorPowerOn((punRxData[0] != 0) ? true : false);
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE:
            /* Set the speed of the differential drive system */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               CBQ24250Module::EInputLimit e_input_limit = CBQ24250Module::EInputLimit::LHIZ;
               switch (punRxData[0]) {
               case 1:
                  e_input_limit = CBQ24250Module::EInputLimit::L100;
                  break;
               case 2:
                  e_input_limit = CBQ24250Module::EInputLimit::L150;
                  break;
               case 3:
                  e_input_limit = CBQ24250Module::EInputLimit::L500;
                  break;
               case 4:
                  e_input_limit = CBQ24250Module::EInputLimit::L900;
                  break;
               default:
                  /* case 0 or invalid is LHIZ (no override / auto mode) */
                  break;
               }
               m_cPowerManagementSystem.SetActuatorInputLimitOverride(e_input_limit);
            }
            break;
         default:
            /* unknown command */
            break;
         }
      }
   }
}

/***********************************************************/
/***********************************************************/

