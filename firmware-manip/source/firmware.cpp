#include "firmware.h"

#include <tw_channel_selector.h>


/***********************************************************/
/***********************************************************/

#define REPLY_BUFFER_LENGTH 8
#define I2C_TX_DATA_LENGTH 8

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
   /* Shutdown */
   return 0;
}

/***********************************************************/
/***********************************************************/

void CFirmware::Exec() {
   /* NFC Reset and Interrupt Signals */
   /* Enable pull up on IRQ line, drive one on RST line */
   PORTD |= (NFC_INT | NFC_RST);
   DDRD |= NFC_RST;
   DDRD &= ~NFC_INT;

   /* Select the interface board */
   m_cTWChannelSelector.Select(CTWChannelSelector::EBoard::Interfaceboard);

   /* Initialise NFC */
   bool bNFCInitSuccess = 
      m_cNFCController.Probe() &&
      m_cNFCController.ConfigureSAM() && 
      m_cNFCController.PowerDown();

   uint8_t punReplyBuffer[REPLY_BUFFER_LENGTH];
   uint8_t unRxBufferCount;
      
   for(;;) {
      /* step the lift actuator system state machine */
      m_cLiftActuatorSystem.Step();
      /* check the PCI for input */
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
                                                    4);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_BATT_LVL:
            if(cPacket.GetDataLength() == 0) {
               uint8_t unBattLevel = CADCController::GetInstance().GetValue(CADCController::EChannel::ADC6);
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_BATT_LVL,
                                                    &unBattLevel,
                                                    1);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_CHARGER_STATUS:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] {
                  uint8_t((PINC & PWR_MON_PGOOD) ? 0x00 : 0x01),
                  uint8_t((PINC & PWR_MON_CHG) ? 0x00 : 0x01)
               };
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_CHARGER_STATUS,
                  punTxData,
                  sizeof(punTxData));
            }
            break;            
         case CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_POSITION:
            /* Set the speed of the stepper motor */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               m_cLiftActuatorSystem.SetPosition(punRxData[0]);
               m_cLiftActuatorSystem.ProcessEvent(CLiftActuatorSystem::ESystemEvent::START_POSITION_CTRL);
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED:
            /* Set the speed of the stepper motor */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               int8_t nSpeed = reinterpret_cast<const int8_t&>(punRxData[0]);
               m_cLiftActuatorSystem.SetSpeed(nSpeed);
               m_cLiftActuatorSystem.ProcessEvent(CLiftActuatorSystem::ESystemEvent::START_SPEED_CTRL);
            }
            break;
         case CPacketControlInterface::CPacket::EType::CALIBRATE_LIFT_ACTUATOR:
            if(cPacket.GetDataLength() == 0) {
               m_cLiftActuatorSystem.ProcessEvent(CLiftActuatorSystem::ESystemEvent::START_CALIBRATION);
            }
            break;
         case CPacketControlInterface::CPacket::EType::EMER_STOP_LIFT_ACTUATOR:
            if(cPacket.GetDataLength() == 0) {
               m_cLiftActuatorSystem.ProcessEvent(CLiftActuatorSystem::ESystemEvent::STOP);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_POSITION:
            /* Set the speed of the stepper motor */
            if(cPacket.GetDataLength() == 0) {
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_POSITION,
                  m_cLiftActuatorSystem.GetPosition());
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE:
            /* Set the speed of the stepper motor */
            if(cPacket.GetDataLength() == 0) {
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_LIFT_ACTUATOR_STATE,
                  static_cast<uint8_t>(m_cLiftActuatorSystem.GetSystemState()));
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] {
                  uint8_t(m_cLiftActuatorSystem.GetUpperLimitSwitchState() ? 0x01 : 0x00),
                  uint8_t(m_cLiftActuatorSystem.GetLowerLimitSwitchState() ? 0x01 : 0x00)
               };
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE,
                  punTxData,
                  sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_EM_CHARGE_ENABLE:
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               if(punRxData[0] != 0) {
                  m_cLiftActuatorSystem.GetElectromagnetController().SetChargeEnable(true);
               } 
               else {
                  m_cLiftActuatorSystem.GetElectromagnetController().SetChargeEnable(false);
               }
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE:
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               switch(punRxData[0]) {
               case 0: 
                  m_cLiftActuatorSystem.GetElectromagnetController().SetDischargeMode(
                     CElectromagnetController::EDischargeMode::CONSTRUCTIVE);
                  break;
               case 1: 
                  m_cLiftActuatorSystem.GetElectromagnetController().SetDischargeMode(
                     CElectromagnetController::EDischargeMode::DESTRUCTIVE);
                  break;
               default:
                  m_cLiftActuatorSystem.GetElectromagnetController().SetDischargeMode(
                     CElectromagnetController::EDischargeMode::DISABLE);
                  break;
               }
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE:
            if(cPacket.GetDataLength() == 0) {
               uint8_t unAccumulatedVoltage = 
                  m_cLiftActuatorSystem.GetElectromagnetController().GetAccumulatedVoltage();
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE,
                  &unAccumulatedVoltage,
                  1);
            }
            break;
         case CPacketControlInterface::CPacket::EType::READ_SMBUS_BYTE:
            if(cPacket.GetDataLength() == 1) {
               uint8_t unAddress = cPacket.GetDataPointer()[0];
               m_cTWController.Read(unAddress, 1, true);
               punReplyBuffer[0] = m_cTWController.Read();
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::READ_SMBUS_BYTE,
                  punReplyBuffer,
                  1);
            }
            break;
         case CPacketControlInterface::CPacket::EType::WRITE_SMBUS_BYTE:
            if(cPacket.GetDataLength() == 2) {
               uint8_t unAddress = cPacket.GetDataPointer()[0];
               uint8_t unData = cPacket.GetDataPointer()[1];
               m_cTWController.BeginTransmission(unAddress);    
               m_cTWController.Write(unData);
               m_cTWController.EndTransmission(true);
            }
            break;
         case CPacketControlInterface::CPacket::EType::READ_SMBUS_BYTE_DATA:
            if(cPacket.GetDataLength() == 2) {
               uint8_t unAddress = cPacket.GetDataPointer()[0];
               uint8_t unRegister = cPacket.GetDataPointer()[1];
               m_cTWController.BeginTransmission(unAddress);    
               m_cTWController.Write(unRegister);
               m_cTWController.EndTransmission(false);
               m_cTWController.Read(unAddress, 1, true);
               punReplyBuffer[0] = m_cTWController.Read();
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::READ_SMBUS_BYTE_DATA,
                  punReplyBuffer,
                  1);
            }
            break;
         case CPacketControlInterface::CPacket::EType::WRITE_SMBUS_BYTE_DATA:
            if(cPacket.GetDataLength() == 3) {
               uint8_t unAddress = cPacket.GetDataPointer()[0];
               uint8_t unRegister = cPacket.GetDataPointer()[1];
               uint8_t unData = cPacket.GetDataPointer()[2];
               m_cTWController.BeginTransmission(unAddress);
               m_cTWController.Write(unRegister);
               m_cTWController.Write(unData);
               m_cTWController.EndTransmission(true);
            }
            break;
         case CPacketControlInterface::CPacket::EType::READ_SMBUS_WORD_DATA:
            if(cPacket.GetDataLength() == 2) {
               uint8_t unAddress = cPacket.GetDataPointer()[0];
               uint8_t unRegister = cPacket.GetDataPointer()[1];
               m_cTWController.BeginTransmission(unAddress);  
               m_cTWController.Write(unRegister);
               m_cTWController.EndTransmission(false);
               m_cTWController.Read(unAddress, 2, true);
               punReplyBuffer[0] = m_cTWController.Read();
               punReplyBuffer[1] = m_cTWController.Read();
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::READ_SMBUS_WORD_DATA,
                  punReplyBuffer,
                  2);
            }
            break;
         case CPacketControlInterface::CPacket::EType::READ_SMBUS_I2C_BLOCK_DATA:
            if(cPacket.GetDataLength() == 3) {
               uint8_t unAddress = cPacket.GetDataPointer()[0];
               uint8_t unRegister = cPacket.GetDataPointer()[1];
               uint8_t unCount = cPacket.GetDataPointer()[2];
               m_cTWController.BeginTransmission(unAddress);
               m_cTWController.Write(unRegister);
               m_cTWController.EndTransmission(false);
               m_cTWController.Read(unAddress, unCount, true);
               for(uint8_t unIndex = 0; unIndex < unCount; unIndex++) {
                  punReplyBuffer[unIndex] = m_cTWController.Read();
               }
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::READ_SMBUS_I2C_BLOCK_DATA,
                  punReplyBuffer,
                  unCount);
            }
            break;

         case CPacketControlInterface::CPacket::EType::WRITE_NFC:
            if(cPacket.HasData()) {
               if(m_cNFCController.P2PInitiatorInit()) {
                  unRxBufferCount = 
                     m_cNFCController.P2PInitiatorTxRx(cPacket.GetDataPointer(),
                                                       cPacket.GetDataLength(),
                                                       punReplyBuffer,
                                                       REPLY_BUFFER_LENGTH);
               }
               m_cNFCController.PowerDown();
            }
            break;
         default:            
            break;
         }
      }
   }
}

/***********************************************************/
/***********************************************************/
