/* 10/16/2018 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The BMA400 is an inexpensive (~$2), three-axis, medium-resolution (12-bit), ultra-low power (800 nA low power mode) accelerometer 
 *  in a tiny 2 mm x 2 mm LGA12 package with 1024-byte FIFO, 
 *  two multifunction interrupts and widely configurable sample rate (15 - 800 Hz), full range (2 - 16 g), low power modes, 
 *  and interrupt detection behaviors. This accelerometer is nice choice for motion-based wake/sleep,
 *  tap detection, step counting, and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "BMA400.h"
#include "I2CDev.h"

BMA400::BMA400(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t BMA400::getChipID()
{
  uint8_t c = _i2c_bus->readByte(BMA400_ADDRESS, BMA400_CHIPID);
  return c;
}

uint8_t BMA400::getStatus()
{
  uint8_t c = _i2c_bus->readByte(BMA400_ADDRESS, BMA400_INT_STAT_0);
  return c;
}

float  BMA400::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
   // Possible accelerometer scales (and their register bit settings) are:
   // 2 Gs , 4 Gs , 8 Gs , and 16 Gs . 
    case AFS_2G:
          _aRes = 2.0f/2048.0f;// per data sheet
          return _aRes;
          break;
    case AFS_4G:
          _aRes = 4.0f/2048.0f;
          return _aRes;
          break;
    case AFS_8G:
          _aRes = 8.0f/2048.0f;
          return _aRes;
          break;
    case AFS_16G:
          _aRes = 16.0f/2048.0f;
          return _aRes;
          break;
  }
}

void BMA400::initBMA400(uint8_t Ascale, uint8_t SR, uint8_t power_Mode, uint8_t OSR, uint8_t acc_filter)
{
   /* Normal mode configuration */
   // set bandwidth to 0.2x sample rate (bit 7), OSR in low-power mode, power mode
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG0, 0x80 | OSR << 5 | power_Mode);
   delay(2); // wait 1.5 ms
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG1, Ascale << 6 | OSR << 4 | SR); // set full-scale range, oversampling and sample rate
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG2, acc_filter << 2);             // set accel filter
   
   /* Low power mode configuration */
// _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_AUTOLOWPOW_0, 0xFA);             // set auto low power time out to 10 second
// _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_AUTOLOWPOW_1, 0x04);             // switch to low power after time out (max 10.2 sec)
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_AUTOLOWPOW_1, 0x02);             // switch to low power when GEN1 interrupt triggered

   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_AUTOWAKEUP_1, 0x02);             // enable interrupt for auto wake up
   // enable interrupt on all three axes (bits 5 - 7)for auto wake up
   // use 4 samples (n + 1) for wakeup condition (bits 4 - 2), 4 samples = 160 ms at 25 Hz
   // reference updated whenever going into low power mode (bits 1, 0)
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_WKUP_INT_CONFIG0, 0x07 << 5 | 0x03 << 2 | 0x01); 
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_WKUP_INT_CONFIG1, 0x08); //  50 mg threshold for wake on any axis, n x 6.25 mgs at 2 g FS
 
   /* Interrupt configuration */
   // Map interrupts and set interrupt behavior
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_INT1_MAP, 0x01);              // map WKUP interrupt to INT1 (bit 0) 
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_INT2_MAP, 0x04);              // map GEN1 interrupt to INT2 (bit 2)
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_INT12_IO_CTRL, 0x20 | 0x02);  // set both interrupts push-pull, active HIGH 

   // Config GEN1 on INT2
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_GEN1INT_CONFIG0, 0xFA);       // all axes, fixed 100 Hz, reference every time, hysteresis 48 mg
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_GEN1INT_CONFIG1, 0x01);       // inactive motion interrupt configuration
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_GEN1INT_CONFIG2, 0x03);       // set threshold 24 mg (8 mg per count)
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_GEN1INT_CONFIG3, 0x00);       // measured over 1024 data samples
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_GEN1INT_CONFIG31,0x80);       // 128/100 ~ 1.28 seconds

  /* Enable interrupts */
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_INT_CONFIG0,  0x04);          // enable GEN1 interrupt (bit 2) 
}


void BMA400::activateNoMotionInterrupt()
{
  _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_INT_CONFIG0,  0x04);           // enable GEN1 (no_Motion) interrupt  
}


void BMA400::deactivateNoMotionInterrupt()
{
  _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_INT_CONFIG0,  0x00);           // disable GEN1 (no_Motion) interrupt  
}


void BMA400::CompensationBMA400(uint8_t Ascale, uint8_t SR, uint8_t power_Mode, uint8_t OSR, uint8_t acc_filter, float * offset)
{
     Serial.println("hold flat and motionless for bias calibration");
     delay(5000);
     
     _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG0, 0x80 | OSR << 5 | power_Mode);// set bandwidth to 0.2x sample rate, OSR in low-power mode, power mode
     delay(2);
     _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG1, Ascale << 6 | OSR << 4 | SR); // set full-scale range, oversampling and sample rate
     _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG2, acc_filter << 2);             // set accel filter

     int16_t temp[3] = {0, 0, 0};
     int32_t sum[3] = {0, 0, 0};

     for(uint8_t ii = 0; ii < 128; ii++)
     {
       readBMA400AccelData(temp);
       sum[0] += temp[0];
       sum[1] += temp[1];
       sum[2] += temp[2];
       delay(100);
     }
     
     offset[0] = float(sum[0])/128.0f;
     offset[1] = float(sum[1])/128.0f;
     offset[2] = float(sum[2])/128.0f;
     offset[0] *= _aRes;
     offset[1] *= _aRes;
     offset[2] *= _aRes;
     if(offset[2] > +0.5f) offset[2] = offset[2] - 1.0f;
     if(offset[2] < -0.5f) offset[2] = offset[2] + 1.0f;

     Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
     Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
     Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");
     /* end of accel calibration */
     }


   void BMA400::resetBMA400()
   {
    _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_CMD,0xB6); // software reset the BMA400
   }


   void BMA400::selfTestBMA400()
   {
   int16_t temp[3]={0, 0, 0};
   int16_t posX, posY, posZ, negX, negY, negZ;
   
   // initialize sensor for self test
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG0, 0x80 | osr3 << 5 | normal_Mode);// set bandwidth to 0.2x sample rate, OSR in low-power mode, power mode
   delay(2);
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG1, AFS_4G << 6 | osr3 << 4 | SR_100Hz); // set full-scale range to +/- 4 g
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_ACC_CONFIG2, acc_filt1 << 2);             // set accel filter
   float STres = 4000.0f/2048.0f; // mg/LSB for 4 g full scale
   delay(100);
   
   // positive axes test
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_SELF_TEST, 0x00 | 0x07); // positive axes
   delay(100);
   readBMA400AccelData(temp);
   posX = temp[0];
   posY = temp[1];
   posZ = temp[2];

   // negative axes test
   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_SELF_TEST, 0x08 | 0x07); // negative axes
   delay(100);
   readBMA400AccelData(temp);
   negX = temp[0];
   negY = temp[1];
   negZ = temp[2];

   Serial.print("x-axis self test = "); Serial.print((float)(posX - negX)*STres, 1); Serial.println("mg, should be > 2000 mg");
   Serial.print("y-axis self test = "); Serial.print((float)(posY - negY)*STres, 1); Serial.println("mg, should be > 1800 mg");
   Serial.print("z-axis self test = "); Serial.print((float)(posZ - negZ)*STres, 1); Serial.println("mg, should be > 800 mg");

   _i2c_bus->writeByte(BMA400_ADDRESS,BMA400_SELF_TEST, 0x00); // disable self test
/* end of self test*/
}


void BMA400::readBMA400AccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(BMA400_ADDRESS, BMA400_ACCD_X_LSB, 6, &rawData[0]);  // Read the 6 raw data registers into data array
  destination[0] = ( (rawData[1] & 0x0F) << 8) | rawData[0];     // Turn the MSB and LSB into a signed 12-bit value
  destination[1] = ( (rawData[3] & 0x0F) << 8) | rawData[2];  
  destination[2] = ( (rawData[5] & 0x0F) << 8) | rawData[4]; 
  if(destination[0] > 2047) destination[0] += -4096;
  if(destination[1] > 2047) destination[1] += -4096;
  if(destination[2] > 2047) destination[2] += -4096;
}


int16_t BMA400::readBMA400TempData()
{
  uint8_t temp = _i2c_bus->readByte(BMA400_ADDRESS, BMA400_TEMP_DATA);    // Read the raw data register  
  int16_t tmp = (int16_t) ( ((int16_t)temp << 8) | 0x00) >> 8;  // Turn into signed 8-bit temperature value
  return tmp;
  }
