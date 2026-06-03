# WCT-526AT
Author: Adib Fridiansya
Company: PT Piranti Kecerdasan Buatan

The WCT-526AT is an AC energy monitoring device designed to measure metrological parameters, including voltage, current, active power, power factor, frequency, and active energy. The WCT-526AT-30A utilizes the ATM90E26 metrology chip, which interfaces with an ESP32 microcontroller via SPI. This device is capable of reading currents up to 30A using an External Current Transformer (CT) and features data loss protection in the event of a main power failure.

System status can be monitored through four physical LED indicators, which function as calibration and connectivity indicators. Furthermore, the device is integrated with a GPRS cellular modem for wireless data transmission and supports various data communication protocols.

Characteristics
1. AC Parameter Measurement: Voltage, Current, Active Power, Power Factor, Frequency, and Active Energy.
2. Non-Invasive Current Sensor: Utilizes an External Current Transformer (CT).
3. Galvanic Isolation: Galvanically isolated AC voltage.
4. Noise Reduction: High-frequency noise reduction.
5. Surge Protection: Transient surge voltage protection.
6. Energy Accumulation: 64-bit energy accumulation.
7. Data Protection: Data loss protection during main power failures.
8. Wireless Communication: Wireless data transmission via a GPRS cellular module.
9. Self-Recovery System: Hardware watchdog timer (WDT) for automated system recovery.
10. Programmable Communication Protocols: Supports MQTT, HTTP, Firebase, or TCP/UDP (Default: MQTT).
