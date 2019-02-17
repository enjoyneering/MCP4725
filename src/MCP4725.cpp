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

#include "MCP4725.h"


/**************************************************************************/
/*
    MCP4725()

    Constructor
*/
/**************************************************************************/ 
MCP4725::MCP4725(MCP4725Ax_ADDRESS addr, float refV)
{
   _i2cAddress = addr;

   setReferenceVoltage(refV); //set _refVoltage & _bitsPerVolt variables
}

/**************************************************************************/
/*
    begin()

    Initialize & configure i2c

    NOTE:
    - function Wire.endTransmission() returns:
      - 0 success
      - 1 data too long to fit in transmit data16
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/ 
#if defined(ESP8266)
bool MCP4725::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  Wire.setClock(100000UL);                           //experimental! ESP8266 i2c bus speed: 50kHz..400kHz/50000..400000, default 100000
  Wire.setClockStretchLimit(230);                    //experimental! default 230Ојsec
#else
bool MCP4725::begin()
{
  Wire.begin();
  Wire.setClock(100000UL);                           //experimental! AVR i2c bus speed: AVR 31kHz..400kHz/31000..400000, default 100000
#endif
  Wire.beginTransmission(_i2cAddress);

  if (Wire.endTransmission(true) != 0) return false; //safety check, make sure MCP4725 is connected
                                       return true;
}

/**************************************************************************/
/*
    setReferenceVoltage()

    Set reference voltage
*/
/**************************************************************************/
void MCP4725::setReferenceVoltage(float value)
{
   if   (value == 0) _refVoltage = MCP4725_REFERENCE_VOLTAGE; //sanity check, avoid division by zero
   else              _refVoltage = value;    

   _bitsPerVolt = (float)MCP4725_STEPS / _refVoltage;         //TODO: check accuracy with +0.5
}

/**************************************************************************/
/*
    getReferenceVoltage()

    Return reference voltage
*/
/**************************************************************************/
float MCP4725::getReferenceVoltage()
{
  return _refVoltage;
}

/**************************************************************************/
/*
    setValue()

    Set output voltage to a fraction of Vref

    NOTE:
    -  mode:
      - "MCP4725_FAST_MODE"...........writes 2-bytes, data & power type to
                                      DAC register & EEPROM is not affected
      - "MCP4725_REGISTER_MODE".......writes 3-bytes, data & power type to
                                      DAC register & EEPROM is not affected
      - "MCP4725_EEPROM_MODE".........writes 3-bytes, data & power type to
                                      DAC register & EEPROM
    - powerType:
      - "MCP4725_POWER_DOWN_OFF"......power down off, draws 0.40mA no load
                                      & 0.29mA maximum load
      - "MCP4725_POWER_DOWN_1KOHM"....power down on with 1 kOhm to ground,
                                      draws 60nA
      - "MCP4725_POWER_DOWN_100KOHM"..power down on with 100 kOhm to ground
      - "MCP4725_POWER_DOWN_500KOHM"..power down on with 500kOhm to ground
*/
/**************************************************************************/ 
bool MCP4725::setValue(uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  #ifndef MCP4725_DISABLE_SANITY_CHECK
  if (value > MCP4725_MAX_VALUE) value = MCP4725_MAX_VALUE; //make sure value never exceeds threshold
  #endif

  return writeComand(value, mode, powerType);
}

/**************************************************************************/
/*
    setVoltage()

    Set output voltage to a fraction of Vref
*/
/**************************************************************************/ 
bool MCP4725::setVoltage(float voltage, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  uint16_t value = 0;

  /* convert voltage to DAC bits */
  #ifndef MCP4725_DISABLE_SANITY_CHECK
  if      (voltage >= _refVoltage) value = MCP4725_MAX_VALUE;      //make sure value never exceeds threshold
  else if (voltage <= 0)           value = 0;
  else                             value = voltage * _bitsPerVolt; //xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
  #else
  value = voltage * _bitsPerVolt;                                  //xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
  #endif

  return writeComand(value, mode, powerType);
}

/**************************************************************************/
/*
    getValue()

    Read current DAC value from DAC register

    NOTE:
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725::getValue()
{
  uint16_t value = readRegister(MCP4725_READ_DAC_REG); //D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx        

  if (value != MCP4725_ERROR) return value >> 4;       //00,00,00,00,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0
                              return value;            //collision on i2c bus
}

/**************************************************************************/
/*
    getVoltage()

    Read current DAC value from DAC register & convert to voltage
*/
/**************************************************************************/ 
float MCP4725::getVoltage()
{
  float value = getValue();

  if (value != MCP4725_ERROR) return value / _bitsPerVolt;
                              return value;
}

/**************************************************************************/
/*
    getStoredValue()

    Read DAC value from EEPROM

    NOTE:
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725::getStoredValue()
{
  uint16_t value = readRegister(MCP4725_READ_EEPROM); //xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0

  if (value != MCP4725_ERROR) return value & 0x0FFF;  //00,00,00,00,D11,D10,D9,D8,   D7,D6,D5,D4,D3,D2,D1,D0
                              return value;           //collision on i2c bus
}

/**************************************************************************/
/*
    getStoredVoltage()

    Read stored DAC value from EEPROM & convert to voltage
*/
/**************************************************************************/ 
float MCP4725::getStoredVoltage()
{
  float value = getStoredValue();

  if (value != MCP4725_ERROR) return value / _bitsPerVolt;
                              return value;
}

/**************************************************************************/
/*
    getPowerType()

    Return current power type from DAC register

    NOTE:
    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0 
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
    - in the power-down modes Vout is off
    - see MCP4725 datasheet on p.15
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725::getPowerType()
{
  uint16_t value = readRegister(MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR)
  {
           value &= 0x0006;                             //00,00,00,00,00,PD1,PD0,00
    return value >> 1;                                  //00,00,00,00,00,00,PD1,PD0
  }

  return value;                                         //collision on i2c bus
}

/**************************************************************************/
/*
    getStoredPowerType()

    Return stored power type from EEPROM

    NOTE:
    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0 
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725::getStoredPowerType()
{
  uint16_t value = readRegister(MCP4725_READ_EEPROM); //xx,PD1,PD0,xx,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0

  if (value != MCP4725_ERROR)
  {
    value = value << 1;                               //PD1,PD0,xx,D11,D10,D9,D8,D7  D6,D5,D4,D3,D2,D1,D0,00
    return  value >> 14;                              //00,00,00,00,00,00,00,00      00,00,00,00,00,00,PD1,PD0
  }

  return value;                                       //collision on i2c bus
}

/**************************************************************************/
/*
    reset()

    Reset MCP4725 & upload data from EEPROM to DAC register

    NOTE:
    - use with caution, "general call" command may affect all slaves
      on i2c bus
    - if Vdd < 2v all circuits & output disabled, when the Vdd
      increases above Vpor device takes a reset state
*/
/**************************************************************************/ 
void MCP4725::reset()
{
  Wire.beginTransmission(MCP4725_GENERAL_CALL_ADDRESS);

  #if ARDUINO >= 100
  Wire.write(MCP4725_GENERAL_CALL_RESET);
  #else
  Wire.send(MCP4725_GENERAL_CALL_RESET);
  #endif

  Wire.endTransmission(true);
}

/**************************************************************************/
/*
    wakeUP()

    Wake up & upload value from DAC register

    NOTE:
    - use with caution, "general call" command may affect all slaves
      on i2c bus
    - resets current power-down bits, EEPROM power-down bit are
      not affected
*/
/**************************************************************************/ 
void MCP4725::wakeUP()
{
  Wire.beginTransmission(MCP4725_GENERAL_CALL_ADDRESS);

  #if ARDUINO >= 100
  Wire.write(MCP4725_GENERAL_WAKE_UP);
  #else
  Wire.send(MCP4725_GENERAL_WAKE_UP);
  #endif

  Wire.endTransmission(true);
}

/**************************************************************************/
/*
    getEepromBusyFlag()

    Return EEPROM writing status from DAC register 

    NOTE:
    - any new write command including repeat bytes during EEPROM write mode
      is ignored
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
bool MCP4725::getEepromBusyFlag()
{
  uint16_t value = readRegister(MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR) return bitRead(value, 7); //1 - completed, 0 - incompleted
                              return false;             //collision on i2c bus
}

/**************************************************************************/
/*
    writeComand()

    Writes value to DAC register or EEPROM

    NOTE:
    - "MCP4725_FAST_MODE" bit format:
      15    14    13   12   11   10   9   8   7   6   5   4   3   2   1   0-bit
      C2=0, C1=0, PD1, PD0, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0
    - "MCP4725_REGISTER_MODE" bit format:
      23    22    21    20   19   18   17   16  15   14   13  12  11  10  9   8   7   6    5   4  3   2   1   0-bit
      C2=0, C1=1, C0=0, xx,  xx,  PD1, PD0, xx, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, xx, xx, xx, xx
    - "MCP4725_EEPROM_MODE" bit format:
      23    22    21    20   19   18   17   16  15   14   13  12  11  10  9   8   7   6    5   4  3   2   1   0-bit
      C2=0, C1=1, C0=1, xx,  xx,  PD1, PD0, xx, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, xx, xx, xx, xx

    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0 
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
*/
/**************************************************************************/ 
bool MCP4725::writeComand(uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  Wire.beginTransmission(_i2cAddress);

  switch (mode)
  {
    case MCP4725_FAST_MODE:                                            //see MCP4725 datasheet on p.18
      #if ARDUINO >= 100
      Wire.write(mode | (powerType << 4) | highByte(value));           //C2,C1,PD1,PD0,D11,D10,D9,D8
      Wire.write(lowByte(value));                                      //D7,D6,D5,D4,D3,D2,D1,D0
      #else
      Wire.send(mode | (powerType << 4)  | highByte(value));
      Wire.send(lowByte(value));
      #endif
      break;

    case MCP4725_REGISTER_MODE: case MCP4725_EEPROM_MODE:              //see MCP4725 datasheet on p.19
      value = value << 4;                                              //D11,D10,D9,D8,D7,D6,D5,D4,  D3,D2,D1,D0,xx,xx,xx,xx
      #if ARDUINO >= 100
      Wire.write(mode | (powerType << 1));                             //C2,C1,C0,xx,xx,PD1,PD0,xx
      Wire.write(highByte(value));                                     //D11,D10,D9,D8,D7,D6,D5,D4
      Wire.write(lowByte(value));                                      //D3,D2,D1,D0,xx,xx,xx,xx
      #else
      Wire.send(mode  | (powerType << 1));
      Wire.send(highByte(value));
      Wire.send(lowByte(value));
      #endif
      break;
  }

  if (Wire.endTransmission(true) != 0) return false;                   //send data over i2c & check for collision on i2c bus

  if (mode == MCP4725_EEPROM_MODE)
  {
    if (getEepromBusyFlag() == true) return true;                      //write completed, success!!!
                                     delay(MCP4725_EEPROM_WRITE_TIME); //typical EEPROM write time 25 msec
    if (getEepromBusyFlag() == true) return true;                      //write completed, success!!!
                                     delay(MCP4725_EEPROM_WRITE_TIME); //maximum EEPROM write time 25 + 25 = 50 msec
  }

  return true;                                                         //success!!!
}

/**************************************************************************/
/*
    readRegister()

    Read DAC register via i2c bus

    NOTE:
    - read output bit format:
      39  38  37 36 35 34  33  32  31  30  29 28 27 26 25 24  23 22 21 20 19 18 17 16  15 14  13  12 11  10  9  8   7  6  5  4  3  2  1  0-bit
      BSY,POR,xx,xx,xx,PD1,PD0,xx, D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx, xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0
      ------ Settings data ------  ---------------- DAC register data ---------------  ------------------- EEPROM data --------------------
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint16_t MCP4725::readRegister(MCP4725_READ_TYPE dataType)
{
  uint16_t value = dataType;                             //convert enum to integer to avoid compiler warnings                                    

  /* read 1, 3 or 5 bytes via i2c */
  #if defined(_VARIANT_ARDUINO_STM32_)
  Wire.requestFrom(_i2cAddress, value);
  #else
  Wire.requestFrom(_i2cAddress, value, true);            //read data over i2c, true = stop message after transmission & releas i2c bus
  #endif

  if (Wire.available() != value) return MCP4725_ERROR;   //check "wire.h" rxBuffer & collision on i2c bus

  /* skip unwanted data */
  switch (dataType)
  {
    case MCP4725_READ_SETTINGS:
      break;

    case MCP4725_READ_DAC_REG:                           //skip 39-bit...32-bit in "wire.h" rxBuffer
      Wire.read();
      break;

    case MCP4725_READ_EEPROM:                            //skip 39-bit...16-bit in "wire.h" rxBuffer
      Wire.read();
      Wire.read();
      Wire.read();
      break;
  }

  /* read data from "wire.h" rxBuffer */
  switch (dataType)
  {
    case MCP4725_READ_SETTINGS:
      #if ARDUINO >= 100
      value = Wire.read();
      #else
      value = Wire.receive();
      #endif
      break;

    case MCP4725_READ_DAC_REG: case MCP4725_READ_EEPROM:
      #if ARDUINO >= 100
      value = Wire.read();
      value = (value << 8) | Wire.read();
      #else
      value = Wire.receive();
      value = (value << 8) | Wire.receive();
      #endif
      break;
  }

  return value;
}
