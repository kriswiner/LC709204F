#include "IS31FL3194.h"

  IS31FL3194::IS31FL3194(I2Cdev* i2c_bus)
  {
  _i2c_bus = i2c_bus;
  }


  // Read the Chip ID register, this is a good test of communication
  uint8_t IS31FL3194::getChipID() 
  { 
  uint8_t c = _i2c_bus->readByte(IS31FL3194_ADDRESS, IS31FL3194_PRODUCT_ID); // Read PRODUCT_ID register for IS31FL3194
  return c;
  }


  void IS31FL3194::reset()
  {
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_RESET, 0xC5);
  }


  void IS31FL3194::powerDown()
  {
  uint8_t d = _i2c_bus->readByte(IS31FL3194_ADDRESS, IS31FL3194_OP_CONFIG);
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OP_CONFIG, d & ~(0x01)); //clear bit 0 to shut down
  }


  void IS31FL3194::powerUp()
  {
  uint8_t d = _i2c_bus->readByte(IS31FL3194_ADDRESS, IS31FL3194_OP_CONFIG);
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OP_CONFIG, d | 0x01); //set bit 0 to enable
  }


  void IS31FL3194::init()// configure rgb led function
  {
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OP_CONFIG, 0x01);     // normal operation in current mode
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT_CONFIG, 0x07);    // enable all three ouputs
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_CURRENT_BAND, 0x00);  // 10 mA max current
  _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_HOLD_FUNCTION, 0x00); // hold function disable
  }


  void IS31FL3194::ledBlink(uint8_t color, uint32_t duration)
  {
   if(color == green) {
   _out1 = 0x00;
   _out2 = 0xFF;
   _out3 = 0x00;
   }

   if(color == blue) {
   _out1 = 0xFF;
   _out2 = 0x00;
   _out3 = 0x00;
  }

   if(color == red) {
   _out1 = 0x00;
   _out2 = 0x00;
   _out3 = 0xFF;
   }

   if(color == cyan) {
   _out1 = 0x20;
   _out2 = 0x20;
   _out3 = 0x00;
   }

   if(color == magenta) {
   _out1 = 0x20;
   _out2 = 0x00;
   _out3 = 0x20;
   }

   if(color == yellow) {
   _out1 = 0x00;
   _out2 = 0x20;
   _out3 = 0x20;
   }

   
   // set rgb led current
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT1, _out1); //maximum current
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT2, _out2);
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT3, _out3);
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_COLOR_UPDATE, 0xC5); // write to color update register for changes to take effect
   delay(duration);
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT1, 0x00); //maximum current
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT2, 0x00);
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_OUT3, 0x00);
   _i2c_bus->writeByte(IS31FL3194_ADDRESS, IS31FL3194_COLOR_UPDATE, 0xC5); // write to color update register for changes to take effect
  }
