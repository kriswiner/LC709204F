/* LED Blink, for Ladybug
 
   This example code is in the public domain.
*/
#include <Arduino.h>
#include <RTC.h>
#include "BMA400.h"
#include "IS31FL3194.h"
#include "LC709204F.h"
#include "I2CDev.h"

#define myButton     39
#define myBuzzer     13
 
 // Type 1SC pin assignments
#define UART_RTS        4
#define UART_CTS        3
#define PMU_SHUTDOWN   A1
#define PMU_WAKEUP     11
#define SC_SWP         12
#define PMU_EXT_ALARM  10
#define PWM0           A0

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

// Battery voltage monitor definitions
float VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
volatile bool SerialDebug = true;
volatile bool alarmFlag = false;

//BMA400 definitions
#define BMA400_intPin1   5    // interrupt1 pin definitions, wake-up from STANDBY pin
#define BMA400_intPin2  A2    // interrupt2 pin definitions, data ready or sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      SR_15_5Hz, SRW_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode, sleep_Mode
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t Ascale = AFS_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = false;
bool BMA400_sleep_flag = false;
bool InMotion = false;

BMA400 BMA400(&i2c_0); // instantiate BMA400 accel class


// Configure the LC709204F battery fuel gauge
#define LC709204F_intPin      A4

uint32_t userID=0;
uint16_t batStatus = 0, RSOC = 0, timetoEmpty = 0, stateofHealth = 0, cellTemperature = 0;
float VBAT, ITE;
volatile bool LC709204F_alarm_flag = false;

LC709204F LC709204F(&i2c_0); // instantiate LC709204F class

IS31FL3194 IS31FL3194(&i2c_0); // instantiate IS31FL3194 led driver class


void setup() 
{
  if(SerialDebug) Serial2.begin(115200); // Debug output on Serial2
  delay(2000);
  if(SerialDebug) Serial2.println("Serial enabled!");
  
  pinMode(BMA400_intPin1, INPUT);     // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);
  pinMode(LC709204F_intPin, INPUT);   // set up  battery fuel gauge interrupt
  pinMode(myBuzzer, OUTPUT);

  // Test the buzzer
  tone(myBuzzer, 3780, 500);  
  delay(500);
  tone(myBuzzer, 3960, 500);  
  
//  Serial1.begin(115200); // Type 1 SC interface
//  delay(2000);
 
  STM32.getUID(UID);
  if(SerialDebug) {Serial2.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial2.print(UID[1], HEX); Serial2.println(UID[2], HEX);} 

   /* initialize wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(100);

  Serial2.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect BMA400 at 0x14 and BME280 at 0x77
  delay(100);
  
  /* Check internal STML082 and battery power configuration */
  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();
  
  // Internal STM32L4 functions
  Serial2.print("VDDA = "); Serial.print(VDDA, 2); Serial2.println(" V");
  Serial2.print("STM32L4 MCU Temperature = "); Serial2.println(Temperature, 2);
  Serial2.println(" "); 

  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial2.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial2.print("BMA400 "); Serial2.print("I AM "); Serial2.print(c, HEX); Serial2.print(" I should be "); Serial2.println(0x90, HEX);
  Serial2.println(" ");
  delay(100); 
  
  Serial2.println("IS31FL3194 RGB led driver...");
  byte d = IS31FL3194.getChipID();
  Serial2.print("IS31FL3194 "); Serial2.print("I AM "); Serial2.print(d, HEX); Serial2.print(" I should be "); Serial2.println(0xCE, HEX);
  Serial2.println(" ");
  delay(100); 

  // Read the LC709204F Chip ID register 
  Serial2.println("LC709204F LiPo battery fuel gauge...");
  uint16_t e = LC709204F.getChipID();  // Read USER_ID register for LC709204F
  Serial2.print("LC709204F "); Serial2.print("I AM 0x"); Serial2.print(e, HEX); Serial2.print(" I should be "); Serial2.println(0x1001, HEX);
  Serial2.println(" ");
  delay(100); 

  if(c == 0x90 && d == 0xCE && e == 0x1001) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial2.println("BMA400 and IS31FL3194 and LC709204F are online..."); Serial2.println(" ");
   
   aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);      
   BMA400.selfTestBMA400();                                             // perform sensor self test
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);                                                          // give some time to read the screen
   BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
   BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application 

   LC709204F.init();
   batStatus = LC709204F.getStatus();
   if(batStatus & 0x0080) Serial2.println("Fuel gauge initialized!");  
   if(batStatus & 0x0040) Serial2.println("Fuel gauge discharging!");  
   stateofHealth = LC709204F.stateofHealth();
   Serial2.print("Battery state of Health = "); Serial2.println(stateofHealth, HEX);
   delay(100);   

  
   IS31FL3194.reset();  // start by resetting device
   delay(100);  
   IS31FL3194.init(); // initialize IS31FL3194}
  }
   
  else 
  {
  if(c != 0x90) Serial2.println(" BMA400 not functioning!");
  if(d != 0xCE) Serial2.println(" IS31FL3194 not functioning!");
  if(e != 0x1001) Serial2.println(" LC709204F not functioning!");
  }

  /* Set the RTC time */
  SetDefaultRTC();
  
  // set alarm to update the RTC periodically
//  RTC.setAlarmTime(0, 0, 0);
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // attach wake-up   interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 
  attachInterrupt(LC709204F_intPin, myinthandler3, FALLING);  // attach interrupt for ALARM pin output of LC709204F 

  BMA400.getStatus(); // read status of interrupts to clear
  LC709204F.clearStatus();

} // end of setup


void loop() 
{
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   Serial2.println("** BMA400 is awake! **");
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 

   IS31FL3194.powerUp();
   IS31FL3194.ledBlink(red, 10); // blink red led for 10 milliseconds
   IS31FL3194.powerDown();
  }

  if(BMA400_sleep_flag)
  {
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   Serial2.println("** BMA400 is asleep! **");
   detachInterrupt(BMA400_intPin2);      // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 

   IS31FL3194.powerUp();
   IS31FL3194.ledBlink(green, 10); // blink green led for 10 milliseconds
   IS31FL3194.powerDown();
  }/* end of sleep/wake detect */


    // LC709204F alarm handling
  if(LC709204F_alarm_flag)
  {
     LC709204F_alarm_flag = false;

   batStatus = LC709204F.getStatus();
   LC709204F.clearStatus();
   if(batStatus & 0x0800) Serial2.println("Low Cell Voltage Alarm!");  
   if(batStatus & 0x0200) Serial2.println("Low RSOC Alarm!");  
   if(batStatus & 0x1000) Serial2.println("High temperature Alarm!");  
   stateofHealth = LC709204F.stateofHealth();
   Serial2.print("Battery state of Health = "); Serial2.println(stateofHealth, HEX);
  }

  
  /*RTC*/
  if (alarmFlag) { // update RTC output at the alarm
      alarmFlag = false;
    
    if(SerialDebug && InMotion) {
      
     BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

    // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - offset[1];   
     az = (float)accelCount[2]*aRes - offset[2]; 
     
    Serial2.println(" ");
    Serial2.print("ax = ");  Serial2.print((int)1000*ax);  
    Serial2.print(" ay = "); Serial2.print((int)1000*ay); 
    Serial2.print(" az = "); Serial2.print((int)1000*az); Serial2.println(" mg");
    Serial2.println(" ");
    }

   Temperature = STM32.getTemperature();
    VDDA = STM32.getVREF();
    // Battery fuel gauge measurements
//    LC709204F.operate();
    // Set cell temperature as MCU temperature in units of 0.1 K
    LC709204F.setTemperature(2732 +  ((int16_t) (10.0f * Temperature)) );
//    Serial.println(2732 +  ((int16_t) (10.0f * Temperature)) , HEX);
//    cellTemperature = LC709204F.getTemperature(); // write only in I2C mode

    VBAT = ((float) LC709204F.getCellVoltage() )/1000.0f;
    RSOC = LC709204F.getRSOC();
    timetoEmpty = LC709204F.timetoEmpty();
    ITE = ((float) LC709204F.getITE()) / 10.0f;
//    LC709204F.sleep(); // sleep current 1.3 uA, operate current ~2 uA
     
   if(SerialDebug) {
      Serial2.print("VDDA = "); Serial2.print(VDDA, 2); Serial2.println(" V");
      Serial2.print("VBAT = "); Serial2.print(VBAT, 2); Serial2.println(" V");
      Serial2.print("RSOC = "); Serial2.print(RSOC); Serial2.println(" %");
      Serial2.print("Time to Empty = "); Serial2.print(timetoEmpty); Serial2.println(" min");
      Serial2.print("Indicator to Empty = "); Serial2.print(ITE, 1); Serial2.println(" %");
      Serial2.print("STM32L4 MCU Temperature = "); Serial2.println(Temperature, 2);
//      Serial2.print("Cell Temperature = 0x"); Serial2.println(cellTemperature, HEX);
      Serial2.println(" ");
    }

    tempCount = BMA400.readBMA400TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial2.print("Accel temperature is ");  Serial2.print(temperature, 1);  Serial2.println(" degrees C"); // Print T values to tenths of s degree C        

  Serial2.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial2.print("0"); Serial2.print(Hours);} else Serial2.print(Hours);
  Serial2.print(":"); 
  if(Minutes < 10) {Serial2.print("0"); Serial2.print(Minutes);} else Serial2.print(Minutes); 
  Serial2.print(":"); 
  if(Seconds < 10) {Serial2.print("0"); Serial2.println(Seconds);} else Serial2.println(Seconds);  

  Serial2.print(Month); Serial2.print("/"); Serial2.print(Day); Serial2.print("/"); Serial2.println(Year);
  Serial2.println(" ");

  IS31FL3194.powerUp();
  IS31FL3194.ledBlink(blue, 10); // blink blue led for 10 milliseconds
  IS31FL3194.powerDown();

  tone(myBuzzer, 3960, 50);  
 } // end of RTC alarm section

 
//    STM32.stop();        // Enter STOP mode and wait for an interrupt
    STM32.sleep();        // Enter SLEEP mode and wait for an interrupt

} // end of main loop


/* Useful functions */
void myinthandler1()
{
  BMA400_wake_flag = true; 
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
}


void myinthandler3()
{
  LC709204F_alarm_flag = true;
  Serial.println("** LC709204F alarm! **");
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()                                                                                 // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];                                                                       // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for(uint8_t i=0; i<3; i++)
  {
    build_mo += Build_mo[i];
  }
  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
