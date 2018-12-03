
#include "mcp23008_module.h"

#include <firmware.h>

CMCP23008Module::CMCP23008Module(uint8_t un_addr) {
   m_unAddr = un_addr;
}

uint8_t CMCP23008Module::ReadRegister(ERegister e_register) {
   CTWController::GetInstance().BeginTransmission(m_unAddr);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_register));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(m_unAddr, 1, true);
   return CFirmware::GetInstance().GetTWController().Read();
}

void CMCP23008Module::WriteRegister(ERegister e_register, uint8_t un_val) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(m_unAddr);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_register));
   CFirmware::GetInstance().GetTWController().Write(un_val);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}
