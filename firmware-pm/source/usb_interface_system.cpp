
#include "usb_interface_system.h"

#include <firmware.h>

/* Configuration and power management masks for the USB hub */
#define HUB_HS_IND      0x01 
#define HUB_CFG_STRAP1  0x02
#define HUB_SUSP_IND    0x04
#define HUB_CFG_STRAP2  0x08
#define HUB_TW_SDA_PU   0x10
#define HUB_TW_SCL_PU   0x20
#define HUB_TW_INT_EN   0x40
#define HUB_RST         0x80

#define UIS_EN_PIN  0x01
#define UIS_NRST_PIN 0x02

#define HUB_CHGDET_START 0x01
#define HUB_CHGDET_DONE  0x10
#define HUB_CHGDET_TYPE  0xE0
#define HUB_CHGDET_RES_SHIFT 5

#define HUB_CHGDET_MASK_SUSP 0x20
#define HUB_CHGDET_MASK_CONN 0x40
#define HUB_CHGDET_MASK_CONF 0x80

/***********************************************************/
/***********************************************************/

CUSBInterfaceSystem CUSBInterfaceSystem::m_cInstance;

/***********************************************************/
/***********************************************************/

CUSBInterfaceSystem& CUSBInterfaceSystem::GetInstance() {
   return m_cInstance;
}

/***********************************************************/
/***********************************************************/

CUSBInterfaceSystem::CUSBInterfaceSystem() :
   cMCP23008Module(0x21) {
   /* Init with power disabled and interface reset asserted */
   PORTB &= ~(UIS_NRST_PIN | UIS_EN_PIN); 
   DDRB |= UIS_NRST_PIN | UIS_EN_PIN;
}

/***********************************************************/
/***********************************************************/

bool CUSBInterfaceSystem::IsHighSpeedMode() {
   uint8_t unPort = cMCP23008Module.ReadRegister(CMCP23008Module::ERegister::PORT);
   return (((unPort & HUB_HS_IND) ^ ((unPort & HUB_CFG_STRAP1) >> 1)) != 0);
}

/***********************************************************/
/***********************************************************/

bool CUSBInterfaceSystem::IsSuspended() {
   uint8_t unPort = cMCP23008Module.ReadRegister(CMCP23008Module::ERegister::PORT);
   return (((unPort & HUB_SUSP_IND) ^ ((unPort & HUB_CFG_STRAP2) >> 1)) != 0);
}

/***********************************************************/
/***********************************************************/

bool CUSBInterfaceSystem::IsEnabled() {
   return ((PORTB & UIS_EN_PIN) != 0);
}

/***********************************************************/
/***********************************************************/

void CUSBInterfaceSystem::Enable() {
   /* Enable power and deassert interface reset */
   PORTB |= (UIS_EN_PIN);
   PORTB |= (UIS_NRST_PIN);
   /* Set up the power and configuration GPIO port */
   /* drive the two-wire pull resistors high, other outputs are low */
   uint8_t unPort = HUB_TW_SDA_PU | HUB_TW_SCL_PU;
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::PORT, unPort);
   /* set the direction bits for the outputs to override the pull up resistors*/
   /* note: low is output, high is input */
   uint8_t unOutputs = ~(HUB_CFG_STRAP1 | HUB_CFG_STRAP2 | HUB_TW_SDA_PU |
                         HUB_TW_SCL_PU | HUB_TW_INT_EN | HUB_RST);
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::DIRECTION, unOutputs);
   /* enable the two-wire interface */
   unPort |= HUB_TW_INT_EN;
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::PORT, unPort);
   /* release the hub reset signal */
   unPort |= HUB_RST;
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::PORT, unPort);
   /* Allow time for the embedded microcontroller to start */
   CFirmware::GetInstance().GetTimer().Delay(5);
   /* Configure the USB2532 */
   cUSB2532Module.Init();
   /* Enable the suspend and high-speed indicator interrupts */
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::GPINTEN,
                                 HUB_HS_IND | HUB_SUSP_IND);
}

/***********************************************************/
/***********************************************************/

void CUSBInterfaceSystem::Disable() {
   /* Disable the suspend and high-speed indicator interrupts */
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::GPINTEN, 0);
   /* leave the TW lines pulled up, but disconnect from the bus and assert reset */
   uint8_t unPort = HUB_TW_SDA_PU | HUB_TW_SCL_PU;
   cMCP23008Module.WriteRegister(CMCP23008Module::ERegister::PORT, unPort);
   /* Assert interface reset and disable power */
   PORTB &= ~(UIS_NRST_PIN);
   PORTB &= ~(UIS_EN_PIN);
}

/***********************************************************/
/***********************************************************/

CUSBInterfaceSystem::EUSBChargerType CUSBInterfaceSystem::GetUSBChargerType() {
   uint8_t unChgDetReg =
      cUSB2532Module.ReadRegister(CUSB2532Module::ERuntimeRegister::UP_BC_DET);
   
   if((unChgDetReg & HUB_CHGDET_DONE) == 0) {
      return EUSBChargerType::WAIT;
   }
   switch(unChgDetReg >> HUB_CHGDET_RES_SHIFT) {
   case 0x00:
      return EUSBChargerType::WAIT;
      break;
   case 0x01:
      return EUSBChargerType::DCP;
      break;
   case 0x02:
      return EUSBChargerType::CDP;
      break;
   case 0x03:
      return EUSBChargerType::SDP;
      break;
   case 0x04:
      return EUSBChargerType::SE1L;
      break;
   case 0x05:
      return EUSBChargerType::SE1H;
      break;
   case 0x06:
      return EUSBChargerType::SE1S;
      break;
   case 0x07:
      return EUSBChargerType::DISABLED;
      break;
   default:
      return EUSBChargerType::DISABLED;
      break;
   }
}

/***********************************************************/
/***********************************************************/
