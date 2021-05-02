/* 
   Pendant test code
   Copyright 2021 Tlera Corporation

   Includes driver and use example for BMA400 accelerometer demonstrating
   wake-on-motion and sleeop-on-no-motion functionality as well as a driver 
   for LC709204F LiPo battery fuel gauge showing configuration,
   wake-on-low voltage and wake-on-high-temperature.

   This sketch may be used without limitations with proper attribution

   This example code is in the public domain.
*/
#include "RTC.h"
#include "BMA400.h"
#include "LC709204F.h"
#include "I2Cdev.h"

// STM32L4 pin assignments
#define myLed          A3// blue led
#define myButton       39

// Type 1SC pin assignments
#define UART_RTS        4
#define UART_CTS        3
#define PMU_SHUTDOWN   A1
#define PMU_WAKEUP     11
#define SC_SWP         12
#define PMU_EXT_ALARM   8
#define PWM0            9

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

#define I2C_BUS    Wire              // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus//

bool SerialDebug = true;

uint16_t Hour = 1, Minute = 1, Second = 1, Millisec, Year = 1, Month = 1, Day = 1;
uint8_t hour = 12, minute = 0, second = 0, year = 1, month = 1, day = 1;
uint32_t subSeconds, milliseconds;
bool alarmFlag = false;

// battery voltage monitor definitions
float VDDA, VBUS, Temperature;


//BMA400 definitions
#define BMA400_intPin1   5    // interrupt1 pin definitions, wake-up from STANDBY pin
#define BMA400_intPin2   A2   // interrupt2 pin definitions, data ready or sleep interrupt

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
volatile bool BMA400_wake_flag = false;
volatile bool BMA400_sleep_flag = false;
volatile bool InMotion = false;

BMA400 BMA400(&i2c_0); // instantiate BMA400 class


// COonfigure the LC709204F battery fuel gauge
#define LC709204F_intPin      10

uint32_t userID=0;
uint16_t batStatus = 0, RSOC = 0, timetoEmpty = 0, stateofHealth = 0, cellTemperature = 0;
float VBAT, ITE;
volatile bool LC709204F_alarm_flag = false;

LC709204F LC709204F(&i2c_0); // instantiate LC709204F class


void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");
  
  /* configure IO pins */
  pinMode(LC709204F_intPin, INPUT);       // set up  battery fuel gauge interrupt
  pinMode(BMA400_intPin1, INPUT);         // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);

  /* initialize wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect BMA400 at 0x14 and LC709204F at 0x0B 
  delay(1000);
  
  // Set the RTC time
  SetDefaultRTC();
  
  /* Check internal STM32 configuration */
  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();
 
  // Internal STM32L4 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
  Serial.println(" ");
  
  // Read the BMA400 Chip ID register 
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM 0x"); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LC709204F Chip ID register 
  Serial.println("LC709204F LiPo battery fuel gauge...");
  uint16_t d = LC709204F.getChipID();  // Read USER_ID register for LC709204F
  Serial.print("LC709204F "); Serial.print("I AM 0x"); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x1001, HEX);
  Serial.println(" ");
  delay(1000); 

  if(c == 0x90 && d == 0x1001) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("BMA400 and LC709204F are online..."); Serial.println(" ");
   
   aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);      
   BMA400.selfTestBMA400();                                             // perform sensor self test
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);                                                          
   BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
   BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application                     

   LC709204F.init();
   batStatus = LC709204F.getStatus();
   if(batStatus & 0x0080) Serial.println("Fuel gauge initialized!");  
   if(batStatus & 0x0040) Serial.println("Fuel gauge discharging!");  
   stateofHealth = LC709204F.stateofHealth();
   Serial.print("Battery state of Health = "); Serial.println(stateofHealth, HEX);
   delay(1000);                                                         // give some time to read the screen
   
  }
  else 
  {
  if(c != 0x90) Serial.println(" BMA400 not functioning!");
  if(d != 0x1001) Serial.println(" LC709204F not functioning!");
  }

  // set alarm to update the RTC periodically
  RTC.setAlarmTime(0, 0, 0);
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // attach wake-up   interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 
  attachInterrupt(LC709204F_intPin, myinthandler3, FALLING);  // attach interrupt for ALARM pin output of LC709204F 

  BMA400.getStatus(); // read status of interrupts to clear
  LC709204F.clearStatus();
   
} /* end of setup */


void loop()
{
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 
   digitalWrite(myLed, HIGH);
  }

  if(BMA400_sleep_flag)
  {
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   detachInterrupt(BMA400_intPin2);      // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 
   digitalWrite(myLed, LOW);
  }/* end of sleep/wake detect */


  // LC709204F alarm handling
  if(LC709204F_alarm_flag)
  {
     LC709204F_alarm_flag = false;

   batStatus = LC709204F.getStatus();
   LC709204F.clearStatus();
   if(batStatus & 0x0800) Serial.println("Low Cell Voltage Alarm!");  
   if(batStatus & 0x0200) Serial.println("Low RSOC Alarm!");  
   if(batStatus & 0x1000) Serial.println("High temperature Alarm!");  
   stateofHealth = LC709204F.stateofHealth();
   Serial.print("Battery state of Health = "); Serial.println(stateofHealth, HEX);
  }

 
  /*RTC*/
  if (alarmFlag) { // update RTC output whenever there is a GNSS pulse
      alarmFlag = false;
    
    if(SerialDebug && InMotion) {
      
     BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

    // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - offset[1];   
     az = (float)accelCount[2]*aRes - offset[2]; 
     
    Serial.println(" ");
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.println(" ");
    }

    VDDA = STM32.getVREF();
    Temperature = STM32.getTemperature();

    // Battery fuel gauge measurements
    LC709204F.operate();
    //Set cell temperature as MCU temperature in units of 0.1 K
    LC709204F.setTemperature(2732 +  ((int16_t) (10.0f * Temperature)) );
//    Serial.println(2732 +  ((int16_t) (10.0f * Temperature)) , HEX);
//    cellTemperature = LC709204F.getTemperature(); // write only in I2C mode

    VBAT = ((float) LC709204F.getCellVoltage() )/1000.0f;
    RSOC = LC709204F.getRSOC();
    timetoEmpty = LC709204F.timetoEmpty();
    ITE = ((float) LC709204F.getITE()) / 10.0f;
    LC709204F.sleep(); // sleep current 1.3 uA, operate current ~2 uA
    
   if(SerialDebug) {
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
      Serial.print("RSOC = "); Serial.print(RSOC); Serial.println(" %");
      Serial.print("Time to Empty = "); Serial.print(timetoEmpty); Serial.println(" min");
      Serial.print("Indicator to Empty = "); Serial.print(ITE, 1); Serial.println(" %");
      Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
//      Serial.print("Cell Temperature = 0x"); Serial.println(cellTemperature, HEX);
      Serial.println(" ");
    }

    tempCount = BMA400.readBMA400TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    // Read RTC
    hour   = RTC.getHours();
    minute = RTC.getMinutes();
    second = RTC.getSeconds();
    Serial.print("RTC Time = ");
    if (hour < 10)   {
      Serial.print("0");
      Serial.print(hour);
    } else Serial.print(hour);
    Serial.print(":");
    if (minute < 10) {
      Serial.print("0");
      Serial.print(minute);
    } else Serial.print(minute);
    Serial.print(":");
    if (second < 10) {
      Serial.print("0");
      Serial.print(second);
    } else Serial.println(second);
  
    year = RTC.getYear();
    month = RTC.getMonth();
    day = RTC.getDay();
    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();

    digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW);
        
    } // end of alarm section
    
 
//    STM32.stop();        // Enter STOP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */
void myinthandler1()
{
  BMA400_wake_flag = true; 
  Serial.println("** BMA400 is awake! **");
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
  Serial.println("** BMA400 is asleep! **");
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
  hour    = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minute  = (build_time[3] - 48)*10 + build_time[4]  - 48;
  second  = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hour);
  RTC.setMinutes(minute);
  RTC.setSeconds(second);
}
