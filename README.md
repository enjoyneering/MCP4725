[![license-badge][]][license] ![version] [![stars][]][stargazers] ![hit-count] [![github-issues][]][issues]

# Microchip MCP4725

This is an Arduino library for I²C MCP4725 12-bit Digital-to-Analog Converter with EEPROM

- operating/reference voltage 2.7v - 5.5v
- output voltage from 0 to operating voltage
- maximum output current 25mA
- output impedance 1 Ohm
- maximum output load 1000pF/0.001μF in parallel with 5 kOhm
- voltage settling time 6 μsec - 10 μsec 
- slew rate 0.55 V/μs
- add 100μF & 0.1 μF bypass capacitors within 4mm to Vdd
- device has 14-bit EEPROM with on-chip charge pump circuit for fail-safe writing
- estimated EEPROM endurance 1 million write cycles
- if Vdd < 2v all circuits & output disabled, when Vdd
  increases above Vpor device takes a reset state & upload data from EEPROM

Supports all MCP4725 features:

- Fast write
- Register read** & write
- EEPROM read** & write
- Power down
- General reset
- General wake-up

Tested on:

- Arduino AVR
- Arduino ESP8266
- Arduino ESP32
- Arduino STM32

** Library returns 65535 if there is a communication error on the I²C bus

[license-badge]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license]:       https://choosealicense.com/licenses/gpl-3.0/
[version]:       https://img.shields.io/badge/Version-1.0.0-green.svg
[stars]:         https://img.shields.io/github/stars/enjoyneering/MCP4725.svg
[stargazers]:    https://github.com/enjoyneering/MCP4725/stargazers
[hit-count]:     https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fenjoyneering%2FMCP4725&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false
[github-issues]: https://img.shields.io/github/issues/enjoyneering/MCP4725.svg
[issues]:        https://github.com/enjoyneering/MCP4725/issues/
