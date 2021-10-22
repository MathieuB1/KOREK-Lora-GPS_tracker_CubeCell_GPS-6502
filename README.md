# KOREK-Lora-GPS_tracker_CubeCell_GPS-6502

# Description:
Send GPS data to a Lora Local Node using the Heltec Cubecell AB02S Board 

# Features:

## Scenario:
 * 1. Receive conf from Local Server see https://github.com/MathieuB1/KOREK-WifiLora-GPS_tracker (AES key + Name + Sleep Frequency)
 * 2. Get GPS and Send it to a Lora Local Node Server
 * 3. Sleep each 30 seconds for waiting a "po", then send a "pi"
 * 4. Trigger GPS until whistle timeout
 * 5. Clear conf on EEPROM

# How to install:

 * 1. Install arduino https://www.arduino.cc/en/software
 * 2. Install Cubecell package for Arduino https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/quick_start.html
 * 3. Add aes.h and aes.cpp to files to Arduino
 * 4. Compile and Upload the code by using the Arduino GUI