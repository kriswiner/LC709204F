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
  uint16_t userid = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_NUMBER_PARAM); // read number of the parameter, should be 0x1001
  return userid;
}


void LC709204F::init()
{
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_APA, 0x1515);                 // set APA for 100 mAH capacity
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_CHANGE_PARAM,  0x0000);       // set change of parameter for 3.7 V 1S LiPo
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_STATUS_BIT,    0x0000);       // set to I2C mode

// Set up interrupts
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_ALARM_LOW_RSOC,  0x00A0);     // set to alarm when RSOC falls below 10%
  //0x0E10 = 3600
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_ALARM_LOW_CELL_VLT,  0x0E10); // set to alarm when battery voltage falls below 3.6 V
  //0x0C3C = 3132 = 2732 + 400 so alarms at 40 C
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_ALARM_HIGH_TEMP,  0x0C3C);    // set to alarm when temperature rises above 40 C

// Set power mode and clear battery status
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_IC_POWERMODE,  0x0001);       // set to operate mode
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_BATTERY_STATUS,  0x0000);     // reset battery status
}


void LC709204F::sleep()
{
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_IC_POWERMODE,  0x0002);      // set to sleep mode
}


void LC709204F::operate()
{
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_IC_POWERMODE,  0x0001);       // set to operate mode
}


uint16_t LC709204F::getCellVoltage()
{
  uint16_t volt = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_CELL_VOLTAGE);
  return volt;
}


uint16_t LC709204F::getRSOC()
{
  uint16_t rsoc = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_RSOC);
  return rsoc;
}


uint16_t LC709204F::getITE()
{
  uint16_t ite = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_ITE);
  return ite;
}


uint16_t LC709204F::getStatus()
{
  uint16_t stat = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_BATTERY_STATUS);
  return stat;
}


void LC709204F::clearStatus()
{
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_BATTERY_STATUS,  0x0000);     // reset battery status
}


uint16_t LC709204F::timetoEmpty()
{
  uint16_t time = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_TIME_TO_EMPTY);
  return time;
}


uint16_t LC709204F::stateofHealth()
{
  uint16_t health = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_STATE_OF_HEALTH);
  return health;
}


void LC709204F::setTemperature(uint16_t temperature)
{
  _i2c_bus->writeBytes16(LC709204F_ADDRESS, LC709204F_CELL_TEMP, temperature); // input cell (MCU) temperature in 0.1 K
}


uint16_t LC709204F::getTemperature()
{
  uint16_t cell = _i2c_bus->readBytes16(LC709204F_ADDRESS, LC709204F_CELL_TEMP);
  return cell;
}

