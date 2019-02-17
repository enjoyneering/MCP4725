/***************************************************************************************************/
/*
   This is an Arduino sketch for MCP4725 12-bit DAC library
 
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

   This DAC uses I2C bus to communicate, specials pins are required to interface
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
#include <Wire.h>
#include <MCP4725.h>

#define DAC_REF_VOLTAGE 5.140  //dac supply-reference voltage
#define I2C_BUS_SPEED   100000 //i2c bus speed, 100 000Hz or 400 000Hz

uint16_t value   = 0;
float    voltage = 0;

MCP4725 dac(MCP4725A0_ADDR_A00, DAC_REF_VOLTAGE);

/**************************************************************************/
/*
    setup()

    Main setup
*/
/**************************************************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println();

  while (dac.begin() != true)
  {
    Serial.println(F("MCP4725 is not connected")); //(F()) saves string to flash & keeps dynamic memory free
    delay(5000);
  }

  Serial.println(F("MCP4725 is OK"));

  Wire.setClock(I2C_BUS_SPEED);                    //experimental! i2c bus speed 100kHz..400kHz/100000..400000, default 100000
}


/**************************************************************************/
/*
    loop()

    Main loop
*/
/**************************************************************************/
void loop()
{
  /* set value 0...4096, writes 16-bits data to DAC register, power down off */
  Serial.println(F("Set max value & normal mode"));
  dac.setValue(4095, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
  delay(5000);

  /* set value 0...4096, writes 24-bits data to DAC register, power down on with 1 kOhm to ground (in the power-down Vout is off) */
  Serial.println(F("Set middle value & power down mode"));
  if (dac.setValue(2048, MCP4725_REGISTER_MODE, MCP4725_POWER_DOWN_1KOHM) != true) Serial.println(F("Collision on i2c bus"));
  delay(5000);

  /* set voltage 0...Vref, writes 16-bits data to DAC register, power down on with 100 kOhm to ground (in the power-down Vout is off) */
  Serial.println(F("Set 1.5v & power down mode"));
  dac.setVoltage(1.50, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_100KOHM);
  delay(5000);

  /* resets power-down type (EEPROM power-down bit are not affected) & upload value from data register */
  Serial.println(F("Wake Up"));
  dac.wakeUP();
  delay(5000);

  /* set voltage 0...Vref, writes 24-bits data to DAC register & 14-bits to EEPROM, power down off */
  Serial.println(F("Set 3.33v & normal mode"));
  if (dac.setVoltage(3.33, MCP4725_EEPROM_MODE, MCP4725_POWER_DOWN_OFF) != true) Serial.println(F("Collision on i2c bus"));
  delay(5000);

  /* get dac value via i2c bus, return 65535 if there is a communication error */
  value = dac.getValue();

  Serial.print(F("Current DAC value - "));

  if   (value != MCP4725_ERROR) Serial.println(value);
  else                          Serial.println(F("collision on i2c bus"));

  /* get dac value via i2c bus & convert it to voltage, return 65535 if there is a communication error */
  voltage = dac.getVoltage();

  Serial.print(F("Current DAC voltage - "));

  if   (voltage != MCP4725_ERROR) Serial.println(voltage);
  else                            Serial.println(F("collision on i2c bus"));

  /* get dac value stored in eeprom via i2c bus, return 65535 if there is a communication error */
  value = dac.getStoredValue();

  Serial.print(F("Stored EEPROM value - "));

  if   (value != MCP4725_ERROR) Serial.println(value);
  else                          Serial.println(F("collision on i2c bus"));

  /* get dac value stored in eeprom via i2c bus & convert it to voltage, return 65535 if there is a communication error */
  voltage = dac.getStoredVoltage();

  Serial.print(F("Stored EEPROM voltage - "));

  if   (voltage != MCP4725_ERROR) Serial.println(voltage);
  else                            Serial.println(F("collision on i2c bus"));

  /* get dac power type via i2c bus, return 65535 if there is a communication error */
  Serial.print(F("Current power type - "));

  switch (dac.getPowerType())
  {
    case MCP4725_POWER_DOWN_OFF:
     Serial.println(F("Normal"));    
      break;

    case MCP4725_POWER_DOWN_1KOHM:
      Serial.println(F("Power down with 1 kOhm resistor to ground"));
      break;

    case MCP4725_POWER_DOWN_100KOHM:
      Serial.println(F("Power down with 100 kOhm resistor to ground"));
      break;

    case MCP4725_POWER_DOWN_500KOHM:
      Serial.println(F("Power down with 500 kOhm resistor to ground"));
      break;

    case MCP4725_ERROR:
      Serial.println(F("Collision on i2c bus"));
      break;
  }

  /* get dac power type stored in eeprom via i2c bus */
  Serial.print(F("Eeprom power type - "));

  switch (dac.getStoredPowerType())
  {
    case MCP4725_POWER_DOWN_OFF:
     Serial.println(F("Normal"));    
      break;

    case MCP4725_POWER_DOWN_1KOHM:
      Serial.println(F("Power down with 1 kOhm resistor to ground"));
      break;

    case MCP4725_POWER_DOWN_100KOHM:
      Serial.println(F("Power down with 100 kOhm resistor to ground"));
      break;

    case MCP4725_POWER_DOWN_500KOHM:
      Serial.println(F("Power down with 500 kOhm resistor to ground"));
      break;

    case MCP4725_ERROR:
      Serial.println(F("Collision on i2c bus"));
      break;
  }
}
