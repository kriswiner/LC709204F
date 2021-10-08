/* 05/01/2021 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The LC709204F is a low-cost, low-power 1S LiPo battery fuel gauge.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
 
#ifndef LC709204F_h
#define LC709204F_h

#include "Arduino.h"
#include "I2CDev.h"
#include <Wire.h>

/*  Register Map LC709204F
//  https://www.onsemi.com/pdf/datasheet/lc709204f-d.pdf 
*/
#define LC709204F_TIME_TO_EMPTY           0x03
#define LC709204F_BEFORE_RSOC             0x04
#define LC709204F_TIME_TO_FULL            0x05
#define LC709204F_TSENSE1_THERMB          0x06
#define LC709204F_INITIAL_RSOC            0x07
#define LC709204F_CELL_TEMP               0x08
#define LC709204F_CELL_VOLTAGE            0x09
#define LC709204F_CURRENT_DIR             0x0A
#define LC709204F_APA                     0x0B
#define LC709204F_APT                     0x0C
#define LC709204F_RSOC                    0x0D
#define LC709204F_TSENSE2_THERMB          0x0E
#define LC709204F_ITE                     0x0F
#define LC709204F_IC_VERSION              0x11
#define LC709204F_CHANGE_PARAM            0x12
#define LC709204F_ALARM_LOW_RSOC          0x13
#define LC709204F_ALARM_LOW_CELL_VLT      0x14
#define LC709204F_IC_POWERMODE            0x15
#define LC709204F_STATUS_BIT              0x16
#define LC709204F_CYCLE_COUNT             0x17
#define LC709204F_BATTERY_STATUS          0x19
#define LC709204F_NUMBER_PARAM            0x1A
#define LC709204F_TERM_CURRENT_RATE       0x1C
#define LC709204F_EMPTY_CELL_VOLTAGE      0x1D
#define LC709204F_ITE_OFFSET              0x1E
#define LC709204F_ALARM_HIGH_CELL_VLT     0x1F
#define LC709204F_ALARM_LOW_TEMP          0x20
#define LC709204F_ALARM_HIGH_TEMP         0x21
#define LC709204F_TOTAL_RUN_TIME_L        0x24
#define LC709204F_TOTAL_RUN_TIME_H        0x25
#define LC709204F_ACC_TEMPERATURE_L       0x26
#define LC709204F_ACC_TEMPERATURE_H       0x27
#define LC709204F_ACC_RSOC_L              0x28
#define LC709204F_ACC_RSOC_H              0x29
#define LC709204F_MAX_CELL_VOLTAGE        0x2A
#define LC709204F_MIN_CELL_VOLTAGE        0x2B
#define LC709204F_MAX_CELL_TEMP_TSENSE1   0x2C
#define LC709204F_MIN_CELL_TEMP_TSENSE1   0x2D
#define LC709204F_AMB_TEMP_TSENSE2        0x30
#define LC709204F_STATE_OF_HEALTH         0x32
#define LC709204F_USER_ID_L               0x36
#define LC709204F_USER_ID_H               0x37

#define LC709204F_ADDRESS                 0x0B   // I2C device address


class LC709204F
{
  public: 
  LC709204F(I2Cdev* i2c_bus);
  uint16_t getChipID();
  void init();
  uint16_t getCellVoltage();
  uint16_t getRSOC();
  uint16_t getITE();
  uint16_t getStatus();
  void clearStatus();
  void sleep();
  void operate();
  uint16_t timetoEmpty();
  uint16_t stateofHealth();
  void setTemperature(uint16_t temperature);
  uint16_t getTemperature();
  
 private:
  I2Cdev* _i2c_bus;
};

#endif
