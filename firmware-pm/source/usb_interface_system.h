#ifndef USB_INTERFACE_SYSTEM_H
#define USB_INTERFACE_SYSTEM_H

#include <usb2532_module.h>
#include <mcp23008_module.h>

#include <stdint.h>

class CUSBInterfaceSystem {
public:
   enum class EUSBChargerType : uint8_t {
      DISABLED = 0, 
      WAIT = 1,
      DCP = 2, 
      CDP = 3, 
      SDP = 4, 
      SE1L = 5, 
      SE1H = 6, 
      SE1S = 7,
   };

public:
   static CUSBInterfaceSystem& GetInstance();

   bool IsHighSpeedMode();

   bool IsSuspended();

   bool IsEnabled();
   
   void Enable();
   
   void Disable();

   EUSBChargerType GetUSBChargerType();

private:
   CUSBInterfaceSystem();
   
   static CUSBInterfaceSystem m_cInstance;

   CMCP23008Module cMCP23008Module;
   CUSB2532Module cUSB2532Module;

};

#endif
