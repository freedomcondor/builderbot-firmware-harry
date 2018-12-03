#ifndef USB2532_MODULE_H
#define USB2532_MODULE_H

#include <stdint.h>

class CUSB2532Module {
public:
   enum class ERuntimeRegister : uint8_t {
      SMBUS_PAGE = 0xFF,
      UP_BC_DET = 0xE2,
      BC_CHG_MODE = 0xEC,
      CHG_DET_MSK = 0xED,
   };
  
   void Init();

   void WriteRegister(ERuntimeRegister e_register,
                      uint8_t un_value,
                      bool b_on_pg2 = false);

   uint8_t ReadRegister(ERuntimeRegister e_register,
                        bool b_on_pg2 = false);
   
private:
   enum class EConfigurationRegister : uint16_t {
      HUB_CFG1 = 0x3006,
      HUB_CFG2 = 0x3007,
      HUB_CFG3 = 0x3008,
      NRD = 0x3009,
      LANG_ID_H = 0x3011,
      LANG_ID_L = 0x3012,
      MFR_STR_LEN = 0x3013,
      PRD_STR_LEN = 0x3014,
      SER_STR_LEN = 0x3015,
      STRINGS = 0x3016,
      UP_BC_DET = 0x30E2,
      BC_CHG_MODE = 0x30EC,   
      HUB_PRT_REMAP12 = 0x30FB,
      HUB_PRT_REMAP34 = 0x30FC,
      HUB_CTRL_REMAP = 0x30FD
   };
   
   enum class ECommand : uint16_t {
      EXEC_REG_OP = 0x9937,
      HUB_ATTACH = 0xAA55,
      OTP_PROG = 0x9933,
      OTP_READ = 0x9934
   };

   uint8_t WriteConfiguration(uint16_t un_address,
                              uint8_t un_data_length,
                              const uint8_t* pun_data);

   uint8_t WriteConfiguration(uint16_t un_address,
                              uint8_t un_data) {
      return WriteConfiguration(un_address, 1, &un_data);
   }

   void WriteCommand(ECommand e_command);

};

#endif
