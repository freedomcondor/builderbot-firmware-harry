
#include <firmware.h>

#include "packet_control_interface.h"

/***********************************************************/
/***********************************************************/

CPacketControlInterface::CPacket::EType CPacketControlInterface::CPacket::GetType() const {
   switch(m_unTypeId) {
   /* uptime and voltage */
   case 0x00:
      return EType::GET_UPTIME;
      break;
   case 0x01:
      return EType::GET_BATT_LVL;
      break;

   /* differential driving system */
   case 0x10:
      return EType::SET_DDS_ENABLE;
      break;
   case 0x11:
      return EType::SET_DDS_SPEED;
      break;
   case 0x13:
      return EType::GET_DDS_SPEED;
      break;
   case 0x14:
      return EType::SET_DDS_PARAMS;
      break;
   case 0x15:
      return EType::GET_DDS_PARAMS;

   /* power management */
   case 0x39:
      return EType::SET_SYSTEM_POWER_ENABLE;
      break;
   case 0x40:
      return EType::SET_ACTUATOR_POWER_ENABLE;
      break;
   case 0x41:
      return EType::SET_ACTUATOR_INPUT_LIMIT_OVERRIDE;
      break;
   case 0x42:
      return EType::SET_USBIF_ENABLE;
      break;
   case 0x43:
      return EType::REQ_SOFT_PWDN;
      break;
   case 0x44:
      return EType::GET_PM_STATUS;
      break;
   case 0x45:
      return EType::GET_USB_STATUS;
      break;

   /* manipulator */
   case 0x60:
      return EType::GET_CHARGER_STATUS;
      break;
   case 0x70:
      return EType::SET_LIFT_ACTUATOR_POSITION;
      break;      
   case 0x71:
      return EType::GET_LIFT_ACTUATOR_POSITION;
      break;      
   case 0x72:
      return EType::SET_LIFT_ACTUATOR_SPEED;
      break;
   case 0x73:
      return EType::GET_LIMIT_SWITCH_STATE;
      break;
   case 0x74:
      return EType::CALIBRATE_LIFT_ACTUATOR;
      break;
   case 0x75:
      return EType::EMER_STOP_LIFT_ACTUATOR;
      break;
   case 0x76:
      return EType::GET_LIFT_ACTUATOR_STATE;
      break;
   case 0x80:
      return EType::SET_EM_CHARGE_ENABLE;
      break;
   case 0x81:
      return EType::SET_EM_DISCHARGE_MODE;
      break;
   case 0x82:
      return EType::GET_EM_ACCUM_VOLTAGE;
      break;
   case 0x90:
      return EType::GET_RF_RANGE;
      break;
   case 0x91:
      return EType::GET_RF_AMBIENT;
      break;
   case 0xA0:
      return EType::READ_NFC;
      break;
   case 0xA1:
      return EType::WRITE_NFC;
      break;
   case 0xC0:
      return EType::READ_SMBUS_BYTE;
      break;
   case 0xC1:
      return EType::READ_SMBUS_BYTE_DATA;
      break;
   case 0xC2:
      return EType::READ_SMBUS_WORD_DATA;
      break;
   case 0xC3:
      return EType::READ_SMBUS_BLOCK_DATA;
      break;
   case 0xC4:
      return EType::READ_SMBUS_I2C_BLOCK_DATA;
      break;
   case 0xD0:
      return EType::WRITE_SMBUS_BYTE;
      break;
   case 0xD1:
      return EType::WRITE_SMBUS_BYTE_DATA;
      break;
   case 0xD2:
      return EType::WRITE_SMBUS_WORD_DATA;
      break;
   case 0xD3:
      return EType::WRITE_SMBUS_BLOCK_DATA;
      break;
   case 0xD4:
      return EType::WRITE_SMBUS_I2C_BLOCK_DATA;
      break;
   default:
      return EType::INVALID;
      break;
   }
}

/***********************************************************/
/***********************************************************/
      
bool CPacketControlInterface::CPacket::HasData() const {
   return (m_unDataLength != 0);
}

/***********************************************************/
/***********************************************************/

uint8_t CPacketControlInterface::CPacket::GetDataLength() const {
   return m_unDataLength;
}

/***********************************************************/
/***********************************************************/

const uint8_t* CPacketControlInterface::CPacket::GetDataPointer() const {
   return m_punData;
}

/***********************************************************/
/***********************************************************/

CPacketControlInterface::EState CPacketControlInterface::GetState() const {
   return m_eState;
}

/***********************************************************/
/***********************************************************/

void CPacketControlInterface::SendPacket(CPacket::EType e_type,
                                         const uint8_t* pun_tx_data,
                                         uint8_t un_tx_data_length) {

   uint8_t punTxBuffer[TX_COMMAND_BUFFER_LENGTH];
   uint8_t unTxBufferPointer = 0;
   /* Check if the data will fit into the buffer */
   if(un_tx_data_length + NON_DATA_SIZE > TX_COMMAND_BUFFER_LENGTH)
      return;

   punTxBuffer[unTxBufferPointer++] = PREAMBLE1;
   punTxBuffer[unTxBufferPointer++] = PREAMBLE2;
   punTxBuffer[unTxBufferPointer++] = static_cast<uint8_t>(e_type);
   punTxBuffer[unTxBufferPointer++] = un_tx_data_length;
   while(unTxBufferPointer < DATA_START_OFFSET + un_tx_data_length) {
      punTxBuffer[unTxBufferPointer] = pun_tx_data[unTxBufferPointer - DATA_START_OFFSET];
      unTxBufferPointer++;
   }
   punTxBuffer[unTxBufferPointer++] = ComputeChecksum(punTxBuffer, TX_COMMAND_BUFFER_LENGTH);
   punTxBuffer[unTxBufferPointer++] = POSTAMBLE1;
   punTxBuffer[unTxBufferPointer++] = POSTAMBLE2;

   for(uint8_t unIdx = 0; unIdx < unTxBufferPointer; unIdx++)
      m_cController.Write(punTxBuffer[unIdx]);
}

/***********************************************************/
/***********************************************************/

void CPacketControlInterface::Reset() {
   m_unRxBufferPointer = 0;
   m_unUsedBufferLength = 0;
   m_unReparseOffset = RX_COMMAND_BUFFER_LENGTH;
   m_eState = EState::SRCH_PREAMBLE1;
}

/***********************************************************/
/***********************************************************/

void CPacketControlInterface::AdjustRxBuffer() {
   /* search for the beginning of new packet */
   for(m_unReparseOffset = 1; m_unReparseOffset < m_unUsedBufferLength; m_unReparseOffset++)
      if(m_punRxBuffer[m_unReparseOffset] == PREAMBLE1)
         break;

   /* shift data down in buffer, move the new packet to the beginning of the buffer*/
   for(uint8_t unBufferIdx = m_unReparseOffset;
       unBufferIdx < m_unUsedBufferLength;
       unBufferIdx++)
      m_punRxBuffer[unBufferIdx - m_unReparseOffset] = m_punRxBuffer[unBufferIdx];

   m_unUsedBufferLength -= m_unReparseOffset;
   m_unRxBufferPointer = 0;
   m_eState = EState::SRCH_PREAMBLE1;
}

/***********************************************************/
/***********************************************************/

void CPacketControlInterface::ReceiveFrame(uint8_t *pun_data, uint8_t un_length) {
   /* check if the checksum are valid */
   uint8_t unCheckSum = ComputeChecksum(pun_data, RX_COMMAND_BUFFER_LENGTH);
   if(pun_data[un_length + CHECKSUM_OFFSET] == unCheckSum){
      /* At this point we assume we have a valid command */
      m_eState = EState::RECV_COMMAND;
      /* Populate the packet fields */
      m_cPacket = CPacket(m_punRxBuffer[TYPE_OFFSET],
                          m_punRxBuffer[DATA_LENGTH_OFFSET],
                          &m_punRxBuffer[DATA_START_OFFSET]);
   }
}

void CPacketControlInterface::ProcessInput() {
   uint8_t unRxByte = 0;
   uint8_t m_unReparseOffset = RX_COMMAND_BUFFER_LENGTH;

   if(m_eState == EState::RECV_COMMAND) {
      /* we received a command in the last call, prepare for the next one */
      m_eState = EState::SRCH_PREAMBLE1;
      AdjustRxBuffer();
   }

   while(m_eState != EState::RECV_COMMAND) {
      /* if the pointer goes to the end of the buffer, ask for more data, or quit*/
      if(m_unRxBufferPointer < m_unUsedBufferLength) {
         unRxByte = m_punRxBuffer[m_unRxBufferPointer];
         m_unRxBufferPointer++;
      }     
      else if(m_cController.Available()) {
         unRxByte = m_cController.Read();
         m_punRxBuffer[m_unRxBufferPointer++] = unRxByte;
         m_unUsedBufferLength++;
      }
      else {
         /* no new data coming, break while and quit this function */
         break;
      }

      switch(m_eState) {
      case EState::SRCH_PREAMBLE1:
         if(unRxByte != PREAMBLE1) {
            AdjustRxBuffer();
         }
         else {
            m_eState = EState::SRCH_PREAMBLE2;
         }
         break;
      case EState::SRCH_PREAMBLE2:
         if(unRxByte != PREAMBLE2) {
            AdjustRxBuffer();
         }
         else {
            m_eState = EState::SRCH_POSTAMBLE1;
         }
         break;
      case EState::SRCH_POSTAMBLE1:
         /* check pointer position with the packet length declared in the packet */
         if(m_unRxBufferPointer > DATA_LENGTH_OFFSET &&
            m_unRxBufferPointer == m_punRxBuffer[DATA_LENGTH_OFFSET] + NON_DATA_SIZE - 1) {
            if(unRxByte != POSTAMBLE1) {
               /* reached packet length declared in the packet, but the data is not a postamble*/
               AdjustRxBuffer();
            }
            else {
               /* unRxByte == POSTAMBLE1 */
               m_eState = EState::SRCH_POSTAMBLE2;
            }
         }
         break;
      case EState::SRCH_POSTAMBLE2:
         if(m_unRxBufferPointer > DATA_LENGTH_OFFSET &&
            m_unRxBufferPointer == m_punRxBuffer[DATA_LENGTH_OFFSET] + NON_DATA_SIZE) {
            if(unRxByte == POSTAMBLE2) {
               ReceiveFrame(m_punRxBuffer, m_unRxBufferPointer);
            }
         }
         break;
      default:
         break;
      }
   }
}

/***********************************************************/
/***********************************************************/

const char* CPacketControlInterface::StateToString(CPacketControlInterface::EState e_state) const {
   switch(e_state) {
   case EState::SRCH_PREAMBLE1:
      return "SRCH_PREAMBLE1";
      break;
   case EState::SRCH_PREAMBLE2:
      return "SRCH_PREAMBLE2";
      break;
   case EState::SRCH_POSTAMBLE1:
      return "SRCH_POSTAMBLE1";
      break;
   case EState::SRCH_POSTAMBLE2:
      return "SRCH_POSTAMBLE2";
      break;
   case EState::RECV_COMMAND:
      return "RECV_COMMAND";
      break;
   default:
      return "UNKNOWN STATE";
      break;
   }
}

/***********************************************************/
/***********************************************************/

const CPacketControlInterface::CPacket& CPacketControlInterface::GetPacket() const {
   return m_cPacket;
}

/***********************************************************/
/***********************************************************/

uint8_t CPacketControlInterface::ComputeChecksum(uint8_t* pun_buf_data, uint8_t un_buf_length) {
   uint8_t unChecksum = 0;
   for(uint8_t unIdx = TYPE_OFFSET;
       unIdx < ((DATA_START_OFFSET + pun_buf_data[DATA_LENGTH_OFFSET] > un_buf_length) ?
          un_buf_length : (DATA_START_OFFSET + pun_buf_data[DATA_LENGTH_OFFSET]));
       unIdx++) {
      unChecksum += pun_buf_data[unIdx];
   }
   return unChecksum;
}
