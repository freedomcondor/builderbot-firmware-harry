#ifndef PCA9554_MODULE_H
#define PCA9554_MODULE_H

#include <stdint.h>

template<uint8_t DEVICE_ADDR>
class CPCA9554Module {

public:

   enum class ERegister : uint8_t {
      INPUT  = 0x00, // R
      OUTPUT = 0x01, // R/W
      CONFIG = 0x03  // R/W
   };
   
public:

   static CPCA9554Module<DEVICE_ADDR>& GetInstance() {
      return m_cInstance;
   }

   uint8_t GetRegister(ERegister e_register) {
      CTWController::GetInstance().BeginTransmission(DEVICE_ADDR);
      CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_register));
      CFirmware::GetInstance().GetTWController().EndTransmission(false);
      CFirmware::GetInstance().GetTWController().Read(DEVICE_ADDR, 1, true);
      return CFirmware::GetInstance().GetTWController().Read();
   }
   
   void SetRegister(ERegister e_register, uint8_t un_val) {
      CFirmware::GetInstance().GetTWController().BeginTransmission(DEVICE_ADDR);
      CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_register));
      CFirmware::GetInstance().GetTWController().Write(un_val);
      CFirmware::GetInstance().GetTWController().EndTransmission(true);
   }

private:
   static CPCA9554Module<DEVICE_ADDR> m_cInstance;
   
   CPCA9554Module() {}
};

template<uint8_t DEVICE_ADDR>
CPCA9554Module<DEVICE_ADDR> CPCA9554Module<DEVICE_ADDR>::m_cInstance = CPCA9554Module<DEVICE_ADDR>();

#endif

