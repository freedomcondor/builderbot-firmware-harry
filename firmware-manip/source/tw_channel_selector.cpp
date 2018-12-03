
#include "tw_channel_selector.h"

#include <firmware.h>

void CTWChannelSelector::Select(EBoard e_board, uint8_t un_mux_ch) {
   CFirmware::GetInstance().GetTWController().BeginTransmission(PCA9542A_I2C_ADDRESS);
   CFirmware::GetInstance().GetTWController().Write(((e_board == EBoard::Mainboard) ? 0x0 : 0x1) | PCA9542A_EN_MASK);
   CFirmware::GetInstance().GetTWController().EndTransmission();

   if(e_board != EBoard::Mainboard) {
      CFirmware::GetInstance().GetTWController().BeginTransmission(PCA9544A_I2C_ADDRESS);    
      CFirmware::GetInstance().GetTWController().Write((un_mux_ch & PCA9544A_SEL_MASK) | PCA9544A_EN_MASK);
      CFirmware::GetInstance().GetTWController().EndTransmission();
   }
}

/***********************************************************/
/***********************************************************/

void CTWChannelSelector::Reset() {
   /* select the interfaceboard */ 
   Select(EBoard::Interfaceboard);
   /* Disable the mux on the interfaceboard */
   CFirmware::GetInstance().GetTWController().BeginTransmission(PCA9544A_I2C_ADDRESS);
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission();
   /* Disable the mux on the mainboard */
   CFirmware::GetInstance().GetTWController().BeginTransmission(PCA9542A_I2C_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission();
}

/***********************************************************/
/***********************************************************/

