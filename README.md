# TTN-OLED-SENSORNODE-STM32
A sensor node for the things network using stm32/bluepill

This is my repo for my experments with an STM32 based bluepill board+AiThinker Ra01 lora node.

A few modifications in the IBM LMIC library is required...most important being diabling the disableIRQ and interrupts in hal.cpp



this sketch is based on otaa example and it also features an ssd1306 based i2c oled. Messages are display on both the serial monitor as well as the tiny oled.

v3-oled sends temp/pressure/altitude readings from a cheap bmp180 sensor to the TTN...many messages are displayed on the oled also

If you have any comments please record in the issues tab.

thank you.
