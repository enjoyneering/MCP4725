/***************************************************************************************************/
/*
   This is an Arduino library for MCP4725, 12-bit Digital-to-Analog Converter with EEPROM

   NOTE:
   - operating/reference voltage 2.7v - 5.5v
   - add 100μF & 0.1 μF bypass capacitors within 4mm to Vdd
   - output voltage from 0 to operating voltage
   - maximum output current 25mA
   - output impedance 1 Ohm
   - maximum output load 1000pF/0.001μF in parallel with 5 kOhm
   - voltage settling time 6 μsec - 10 μsec 
   - slew rate 0.55 V/μs
   - device has 14-bit EEPROM with on-chip charge pump circuit for fail-safe writing
   - estimated EEPROM endurance 1 million write cycles
   - if Vdd < 2v all circuits & output disabled, when Vdd
     increases above Vpor device takes a reset state & upload data from EEPROM

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/

   This chip uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA                    SCL                    Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
   Mega2560................................. 20                     21                     5v
   Due, SAM3X8E............................. 20                     21                     3.3v
   Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
   Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
   Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
   ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
   ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#ifndef MCP4725_h
#define MCP4725_h

/* 
The arduino toolchain includes library headers before it includes your sketch.
Unfortunately, you cannot #define something in a sketch & get it in the library.
*/
//#define MCP4725_DISABLE_SANITY_CHECK     //disable some sanity checks to increase speed, use with caution


#if defined(ARDUINO) && ((ARDUINO) >= 100) //arduino core v1.0 or later
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>                  //for Arduino AVR PROGMEM support
#elif defined(ESP8266)
#include <pgmspace.h>                      //for Arduino ESP8266 PROGMEM support
#elif defined(_VARIANT_ARDUINO_STM32_)
#include <avr/pgmspace.h>                  //for Arduino STM32 PROGMEM support
#endif


#include <Wire.h>

/* dac addresses */
typedef enum : uint8_t
{
  MCP4725A0_ADDR_A00         = 0x60,                              //i2c address, A0 = 0
  MCP4725A0_ADDR_A01         = 0x61,                              //i2c address, A0 = 1

  MCP4725A1_ADDR_A00         = 0x62,                              //i2c address, A0 = 0
  MCP4725A1_ADDR_A01         = 0x63,                              //i2c address, A0 = 1

  MCP4725A2_ADDR_A00         = 0x64,                              //i2c address, A0 = 0
  MCP4725A2_ADDR_A01         = 0x65                               //i2c address, A0 = 1
}
MCP4725Ax_ADDRESS;

/* dac register, command bits */
typedef enum : uint8_t
{
  MCP4725_FAST_MODE          = 0x00,                              //writes data to DAC register
  MCP4725_REGISTER_MODE      = 0x40,                              //writes data & configuration bits to DAC register
  MCP4725_EEPROM_MODE        = 0x60                               //writes data & configuration bits to DAC register & EEPROM
}
MCP4725_COMMAND_TYPE;

/* dac register, power down bits */
typedef enum : uint8_t
{
  MCP4725_POWER_DOWN_OFF     = 0x00,                              //power down off
  MCP4725_POWER_DOWN_1KOHM   = 0x01,                              //power down on, with 1.0 kOhm to ground
  MCP4725_POWER_DOWN_100KOHM = 0x02,                              //power down on, with 100 kOhm to ground
  MCP4725_POWER_DOWN_500KOHM = 0x03                               //power down on, with 500 kOhm to ground
}
MCP4725_POWER_DOWN_TYPE;

/* dac library specific command */
typedef enum : uint8_t
{
  MCP4725_READ_SETTINGS      = 1,                                 //read 1 byte,  settings data
  MCP4725_READ_DAC_REG       = 3,                                 //read 3 bytes, DAC register data
  MCP4725_READ_EEPROM        = 5                                  //read 5 bytes, EEPROM data
}
MCP4725_READ_TYPE;

/* dac general call command */
#define MCP4725_GENERAL_CALL_ADDRESS 0x00                         //general call address
#define MCP4725_GENERAL_CALL_RESET   0x06                         //general call hard reset command
#define MCP4725_GENERAL_WAKE_UP      0x09                         //general call wake-up command

/* dac mics. */
#define MCP4725_RESOLUTION           12                           //resolution 12-bit
#define MCP4725_STEPS                pow(2, (MCP4725_RESOLUTION)) //quantity of DAC steps 2^12-bits = 4096
#define MCP4725_EEPROM_WRITE_TIME    25                           //non-volatile memory write time, maximum 50 msec

#define MCP4725_REFERENCE_VOLTAGE    5.00                         //supply-reference votltage
#define MCP4725_MAX_VALUE            MCP4725_STEPS - 1
#define MCP4725_ERROR                0xFFFF                       //returns 65535, if communication error is occurred



class MCP4725
{
 public:
  MCP4725(MCP4725Ax_ADDRESS = MCP4725A0_ADDR_A00, float refV = MCP4725_REFERENCE_VOLTAGE);

  #if defined(ESP8266)
  bool     begin(uint8_t sda = SDA, uint8_t scl = SCL);
  #else
  bool     begin(void);
  #endif

  void     setReferenceVoltage(float value);
  float    getReferenceVoltage(void);

  bool     setValue(uint16_t value, MCP4725_COMMAND_TYPE = MCP4725_FAST_MODE, MCP4725_POWER_DOWN_TYPE = MCP4725_POWER_DOWN_OFF);
  bool     setVoltage(float voltage, MCP4725_COMMAND_TYPE = MCP4725_FAST_MODE, MCP4725_POWER_DOWN_TYPE = MCP4725_POWER_DOWN_OFF);

  uint16_t getValue(void);
  float    getVoltage(void);
  uint16_t getStoredValue(void);
  float    getStoredVoltage(void);

  uint16_t getPowerType(void);
  uint16_t getStoredPowerType(void);

  void     reset(void);
  void     wakeUP(void);


 private:
  MCP4725Ax_ADDRESS _i2cAddress;
  float             _refVoltage;
  uint16_t          _bitsPerVolt;

  bool     getEepromBusyFlag(void);
  bool     writeComand(uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType);
  uint16_t readRegister(MCP4725_READ_TYPE dataType);
};

#endif
