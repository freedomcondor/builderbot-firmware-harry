#ifndef MCP23008_MODULE_H
#define MCP23008_MODULE_H

#include <stdint.h>

class CMCP23008Module {

public:

   enum class ERegister : uint8_t {
      DIRECTION = 0x00,
      GPINTEN = 0x02,
      PORT = 0x09
   };

   CMCP23008Module(uint8_t un_addr);

   uint8_t ReadRegister(ERegister e_register);
   void WriteRegister(ERegister e_register, uint8_t un_val);

private:
   uint8_t m_unAddr;




};

#endif
