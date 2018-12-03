#ifndef TW_CHANNEL_SELECTOR
#define TW_CHANNEL_SELECTOR

#include <stdint.h>

#define PCA9542A_I2C_ADDRESS 0x71
#define PCA9544A_I2C_ADDRESS 0x70

#define PCA9544A_SEL_MASK 0x03

#define PCA9542A_EN_MASK 0x04
#define PCA9544A_EN_MASK 0x04

class CTWChannelSelector {
public:
   
   enum class EBoard {
      Mainboard,
      Interfaceboard
   };

   void Select(EBoard e_board, uint8_t un_mux_ch = 0x00);
   void Reset();
};

#endif
