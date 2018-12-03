#ifndef ACCELEROMETER_SYSTEM_H
#define ACCELEROMETER_SYSTEM_H

#include <stdint.h>

class CAccelerometerSystem {
public:
   struct SReading {
      int16_t X, Y, Z;
      int16_t Temp;
   };

   bool Init();

   SReading GetReading();

private:

   enum class ERegister : uint8_t {
      /* MPU6050 Registers */
      PWR_MGMT_1     = 0x6B, // R/W
      PWR_MGMT_2     = 0x6C, // R/W
      ACCEL_XOUT_H   = 0x3B, // R  
      ACCEL_XOUT_L   = 0x3C, // R  
      ACCEL_YOUT_H   = 0x3D, // R  
      ACCEL_YOUT_L   = 0x3E, // R  
      ACCEL_ZOUT_H   = 0x3F, // R  
      ACCEL_ZOUT_L   = 0x40, // R  
      TEMP_OUT_H     = 0x41, // R  
      TEMP_OUT_L     = 0x42, // R  
      WHOAMI         = 0x75  // R
   };
};

#endif
