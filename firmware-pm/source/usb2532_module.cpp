
#include "usb2532_module.h"

#include <firmware.h>

#define HUB_CFG_ADDR 0x2D

#define HUB_CFG_ENABLE_STRINGS  0x01
#define HUB_CFG_ENABLE_REMAP    0x08
#define HUB_CFG_ENABLE_COMPOUND 0x08
#define HUB_CFG_MAP_P1P2        0x02
#define HUB_CFG_MAP_P2P1        0x10
#define HUB_CFG_MAP_PDIS        0x00
#define HUB_CFG_NRD_P1P2        0x06
#define HUB_CFG_START_CHGDET    0x01
#define HUB_CFG_ENABLE_ECHGDET  0x04

#define HUB_RT_ADDR 0x2C

#define HUB_RT_SELECT_PAGE1 0x00
#define HUB_RT_SELECT_PAGE2 0x40

/***********************************************************/
/***********************************************************/

void CUSB2532Module::Init() {
   const char m_pchManufacturer[] = "SCT Paderborn";
   const char m_pchProduct[] = "Duovero BeBot";
   /* Fetch the identification number and build the robot serial number */
   char pchSerial[6];
   snprintf(pchSerial, sizeof(pchSerial), "BB%03u", CFirmware::GetInstance().GetId());
   /* Enable port remapping and string support on hub */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::HUB_CFG3),
                      HUB_CFG_ENABLE_STRINGS | HUB_CFG_ENABLE_REMAP);
   /* Set language ID to US English (0x0409) */
   const uint8_t punLangId[] = {0x04, 0x09};
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::LANG_ID_H),
                      sizeof(punLangId),
                      punLangId);
   /* Set the string length fields (UTF16, no null-terminating character) */
   /* Manufacturer */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::MFR_STR_LEN),
                      sizeof(m_pchManufacturer) - 1);
   /* Product */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::PRD_STR_LEN),
                      sizeof(m_pchProduct) - 1);
   /* Serial number */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::SER_STR_LEN),
                      sizeof(pchSerial) - 1);
   uint8_t punBuffer[32];
   uint8_t unStringBytesWritten = 0;
   uint8_t unBufferIdx = 0;
   for(const char& ch_char : m_pchManufacturer) {
      if(ch_char != '\0') {
         /* convert UTF-8 to UTF-16 */
         punBuffer[unBufferIdx++] = ch_char;
         punBuffer[unBufferIdx++] = 0;
      }
   }
   unStringBytesWritten += WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::STRINGS) +
                                              unStringBytesWritten,
                                              unBufferIdx,
                                              punBuffer);
   unBufferIdx = 0;
   for(const char& ch_char : m_pchProduct) {

      if(ch_char != '\0') {
         /* convert UTF-8 to UTF-16 */
         punBuffer[unBufferIdx++] = ch_char;
         punBuffer[unBufferIdx++] = 0;
      }
   }
   unStringBytesWritten += WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::STRINGS) +
                                              unStringBytesWritten,
                                              unBufferIdx,
                                              punBuffer);
   unBufferIdx = 0;
   for(const char& ch_char : pchSerial) {   
      if(ch_char != '\0') {
         /* convert UTF-8 to UTF-16 */
         punBuffer[unBufferIdx++] = ch_char;
         punBuffer[unBufferIdx++] = 0;
      }
   }
   unStringBytesWritten += WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::STRINGS) +
                                              unStringBytesWritten,
                                              unBufferIdx,
                                              punBuffer);
   /* Swap ports 1/2 so that the FT231 enumerates first */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::HUB_PRT_REMAP12),
                      HUB_CFG_MAP_P1P2 | HUB_CFG_MAP_P2P1);
   /* Disable ports 3/4/CTRL as these are not used */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::HUB_PRT_REMAP34),
                      HUB_CFG_MAP_PDIS);
   /* Disable the hub controller */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::HUB_CTRL_REMAP),
                      HUB_CFG_MAP_PDIS);       
   /* Mark ports 1/2 as non-removable */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::NRD),
                      HUB_CFG_NRD_P1P2);       
   /* Device is a compound device */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::HUB_CFG2),
                      HUB_CFG_ENABLE_COMPOUND);
   /* Enable enhanced and SE1 battery charger detection */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::BC_CHG_MODE),
                      HUB_CFG_ENABLE_ECHGDET);
   /* Start charger detection */
   WriteConfiguration(static_cast<uint16_t>(EConfigurationRegister::UP_BC_DET),
                      HUB_CFG_START_CHGDET);
   /* exit configuration stage and connect the hub */
   WriteCommand(ECommand::HUB_ATTACH);
}

/***********************************************************/
/***********************************************************/

void CUSB2532Module::WriteRegister(ERuntimeRegister e_register, uint8_t un_value, bool b_on_pg2) {
   /* Select page */
   CFirmware::GetInstance().GetTWController().BeginTransmission(HUB_RT_ADDR);
   CFirmware::GetInstance().GetTWController().Write(
      static_cast<uint8_t>(ERuntimeRegister::SMBUS_PAGE));
   CFirmware::GetInstance().GetTWController().Write(b_on_pg2 ?
                                                    HUB_RT_SELECT_PAGE2 : HUB_RT_SELECT_PAGE1);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
   /* Write value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(HUB_RT_ADDR);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_register));
   CFirmware::GetInstance().GetTWController().Write(un_value);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

uint8_t CUSB2532Module::ReadRegister(ERuntimeRegister e_register, bool b_on_pg2) {
   /* Select page */
   CFirmware::GetInstance().GetTWController().BeginTransmission(HUB_RT_ADDR);
   CFirmware::GetInstance().GetTWController().Write(
      static_cast<uint8_t>(ERuntimeRegister::SMBUS_PAGE));
   CFirmware::GetInstance().GetTWController().Write(b_on_pg2 ?
                                                    HUB_RT_SELECT_PAGE2 : HUB_RT_SELECT_PAGE1);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
   /* Read value */
   CFirmware::GetInstance().GetTWController().BeginTransmission(HUB_RT_ADDR);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_register));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(HUB_RT_ADDR, 1, true);
   return CFirmware::GetInstance().GetTWController().Read();
}

/***********************************************************/
/***********************************************************/

uint8_t CUSB2532Module::WriteConfiguration(uint16_t un_address,
                                           uint8_t un_data_length,
                                           const uint8_t* pun_data) {

   uint8_t punPacketHeader[] = {
      0x00, /* write configuration register */
      un_data_length,
      (un_address >> 8) & 0xFF,
      (un_address >> 0) & 0xFF
   };

   /* Write configuration to memory */
   CFirmware::GetInstance().GetTWController().BeginTransmission(HUB_CFG_ADDR);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().Write(sizeof(punPacketHeader) + un_data_length);
   for(const uint8_t& un_byte : punPacketHeader) {
      CFirmware::GetInstance().GetTWController().Write(un_byte);
   }
   for(uint8_t unIdx = 0; unIdx < un_data_length; unIdx++) {
      CFirmware::GetInstance().GetTWController().Write(pun_data[unIdx]);
   }
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
   /* Transfer configuration to registers */
   WriteCommand(ECommand::EXEC_REG_OP);

   return un_data_length;
}

/***********************************************************/
/***********************************************************/

void CUSB2532Module::WriteCommand(CUSB2532Module::ECommand e_command) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(HUB_CFG_ADDR);
   CFirmware::GetInstance().GetTWController().Write((static_cast<uint16_t>(e_command) >> 8) & 0xFF);
   CFirmware::GetInstance().GetTWController().Write((static_cast<uint16_t>(e_command) >> 0) & 0xFF);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

