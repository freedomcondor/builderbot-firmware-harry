
#include "accelerometer_system.h"

#include <firmware.h>

#define MPU6050_DEV_ADDR 0x68

/****************************************/
/****************************************/

bool CAccelerometerSystem::Init() {
   /* Probe */
   CFirmware::GetInstance().GetTWController().BeginTransmission(MPU6050_DEV_ADDR);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::WHOAMI));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(MPU6050_DEV_ADDR, 1, true);
         
   if(CFirmware::GetInstance().GetTWController().Read() != MPU6050_DEV_ADDR) 
      return false;

   /* select internal clock, disable sleep/cycle mode, enable temperature sensor*/
   CFirmware::GetInstance().GetTWController().BeginTransmission(MPU6050_DEV_ADDR);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PWR_MGMT_1));
   CFirmware::GetInstance().GetTWController().Write(0x00);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   return true;
}

/****************************************/
/****************************************/


CAccelerometerSystem::SReading CAccelerometerSystem::GetReading() {
   /* Buffer for holding accelerometer result */
   uint8_t punRes[8];

   CFirmware::GetInstance().GetTWController().BeginTransmission(MPU6050_DEV_ADDR);
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::ACCEL_XOUT_H));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(MPU6050_DEV_ADDR, 8, true);
   /* Read the requested 8 bytes */
   for(uint8_t i = 0; i < 8; i++) {
      punRes[i] = CFirmware::GetInstance().GetTWController().Read();
   }

   return SReading { 
      int16_t((punRes[0] << 8) | punRes[1]),
      int16_t((punRes[2] << 8) | punRes[3]),
      int16_t((punRes[4] << 8) | punRes[5]),
      (int16_t((punRes[6] << 8) | punRes[7]) + 12412) / 340};
}

/****************************************/
/****************************************/



