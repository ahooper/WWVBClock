# WWVBClock
WWVB radio clock parts list  

Arduino Uno  

Receiver module 
<https://universal-solder.ca/product/wwvb-msf-jjy60-atomic-clock-receiver-module-60khz/>

<http://canaduino.ca/downloads/60khz.pdf>

Receiver IC <http://canaduino.ca/downloads/MAS6180C.pdf>
```
#define RADIO_POWERDOWN_PIN 6 // P1  
#define RADIO_OUT_PIN       7 // T
```

Sparkfun DeadOn Real Ttime Clock 
<https://www.robotshop.com/ca/en/sfe-deadon-real-time-clock.html>

<https://learn.sparkfun.com/tutorials/deadon-rtc-breakout-hookup-guide>
```
#define RTC_SELECT_PIN     10  
#define RTC_INTERRUPT_PIN  2  
// GND - GND  
// VCC - 5V  
// SQW - D2  
// CLK - D13  ** conflicts with LED_BUILTIN!  
// MISO - D12  
// MOSI - D11  
// SS - D10
```

8 x 7 segment LED display module (DFR0090)
<https://www.robotshop.com/ca/en/dfrobot-8-character-7-segment-spi-led-module.html>

<https://www.dfrobot.com/wiki/index.php/3-Wire_LED_Module_(SKU:DFR0090)>
```
#define LED_LATCH_PIN     8  
#define LED_CLOCK_PIN     3  
#define LED_DATA_PIN      9  
```
