/* 05/01/2021 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The LC709204F is a low-cost, low-power 1S LiPo battery fuel gauge.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
 
#include "LC709204F.h"
#include "I2CDev.h"

LC709204F::LC709204F(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint16_t LC709204F::getChipID()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_NUMBER_PARAM, 2, &tmp[0]); // read number of the parameter, should be 0x1001
  uint16_t userid = (uint16_t) ((uint16_t)tmp[1] << 8) | tmp[0];
  return userid;
}


void LC709204F::init()
{
  uint8_t tmp[2] = {0, 0};
  tmp[0] = 0x15;
  tmp[1] = 0x15;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_APA, 2, &tmp[0]);                 // set APA for 100 mAH capacity
  tmp[0] = 0x00;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_CHANGE_PARAM,  2, &tmp[0]);       // set change of parameter for 3.7 V 1S LiPo
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_STATUS_BIT,  2, &tmp[0]);         // set to I2C mode

// Set up interrupts
  tmp[0] = 0xA0;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_ALARM_LOW_RSOC,  2, &tmp[0]);     // set to alarm when RSOC falls below 10%
  tmp[0] = 0x10;
  tmp[1] = 0x0E;  //0x0E10 = 3600
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_ALARM_LOW_CELL_VLT,  2, &tmp[0]); // set to alarm when battery voltage falls below 3.6 V
  tmp[0] = 0x3C;
  tmp[1] = 0x0C;  //0x0C3C = 3132 = 2732 + 400 so alarms at 40 C
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_ALARM_HIGH_TEMP,  2, &tmp[0]);    // set to alarm when temperature rises above 40 C

// Set power mode and clear battery status
  tmp[0] = 0x01;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_IC_POWERMODE,  2, &tmp[0]);       // set to operate mode
  tmp[0] = 0x00;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_BATTERY_STATUS,  2, &tmp[0]);     // reset battery status
}


void LC709204F::sleep()
{
  uint8_t tmp[2] = {0, 0};
  tmp[0] = 0x02;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_IC_POWERMODE,  2, &tmp[0]);       // set to sleep mode
}


void LC709204F::operate()
{
  uint8_t tmp[2] = {0, 0};
  tmp[0] = 0x01;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_IC_POWERMODE,  2, &tmp[0]);       // set to operate mode
}

uint16_t LC709204F::getCellVoltage()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_CELL_VOLTAGE, 2, &tmp[0]);
  uint16_t volt = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return volt;
}


uint16_t LC709204F::getRSOC()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_RSOC, 2, &tmp[0]);
  uint16_t rsoc = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return rsoc;
}


uint16_t LC709204F::getITE()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_ITE, 2, &tmp[0]);
  uint16_t ite = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return ite;
}


uint16_t LC709204F::getStatus()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_BATTERY_STATUS, 2, &tmp[0]);
  uint16_t stat = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return stat;
}


void LC709204F::clearStatus()
{
  uint8_t tmp[2] = {0, 0};
  tmp[0] = 0x00;
  tmp[1] = 0x00;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_BATTERY_STATUS,  2, &tmp[0]);     // reset battery status
}


uint16_t LC709204F::timetoEmpty()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_TIME_TO_EMPTY, 2, &tmp[0]);
  uint16_t time = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return time;
}


uint16_t LC709204F::stateofHealth()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_STATE_OF_HEALTH, 2, &tmp[0]);
  uint16_t health = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return health;
}


void LC709204F::setTemperature(uint16_t temperature)
{
  uint8_t tmp[2] = {0, 0};
  tmp[0] = temperature & 0x00FF;
  tmp[1] = (temperature & 0xFF00) >> 8;
  _i2c_bus->writeBytes(LC709204F_ADDRESS, LC709204F_CELL_TEMP,  2, &tmp[0]); // input cell (MCU) temperature in 0.1 K
}


uint16_t LC709204F::getTemperature()
{
  uint8_t tmp[2] = {0, 0};
  _i2c_bus->readBytes(LC709204F_ADDRESS, LC709204F_CELL_TEMP, 2, &tmp[0]);
  uint16_t cell = (uint16_t) ((uint16_t) tmp[1] << 8) | tmp[0];
  return cell;
}
