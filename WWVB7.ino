#include <SparkFunDS3234RTC.h>
    // Library from https://learn.sparkfun.com/tutorials/deadon-rtc-breakout-hookup-guide

#define DEBUG_PIN           5

#define CENTURY 2000

// WWVB reference https://www.nist.gov/sites/default/files/documents/2017/04/28/SP-432-NIST-Time-and-Frequency-Services-2012-02-13.pdf
// Indices for parts of WWVB frame
enum {
    FPRM, // Frame reference marker: .8L+.2H
    FPUU, // Unweighted: .2L+.8H
    // d1: .5L+.5H / 0 = .2L+.8H
    FPM1, // 10 minutes
    FPM2, //  1 minutes
    FPH1, // 10 hours
    FPH2, //  1 hours
    FPD1, //100 days
    FPD2, // 10 days
    FPD3, //  1 days
    FPUS, // UTC sign
    FPUC, // UTC correction
    FPY1, // 10 years
    FPY2, //  1 years
    FPLY, // Leap year
    FPLS, // Leap second
    FPDS, // Daylight saving time
    FPEF}; // End of frame same as FPRM
// Order of received frame, one per second
const byte FramePattern[] =
    // .0   .1   .2   .3   .4   .5   .6   .7   .8   .9
/*0.*/{FPRM,FPM1,FPM1,FPM1,FPUU,FPM2,FPM2,FPM2,FPM2,FPRM,
/*1.*/ FPUU,FPUU,FPH1,FPH1,FPUU,FPH2,FPH2,FPH2,FPH2,FPRM,
/*2.*/ FPUU,FPUU,FPD1,FPD1,FPUU,FPD2,FPD2,FPD2,FPD2,FPRM,
/*3.*/ FPD3,FPD3,FPD3,FPD3,FPUU,FPUU,FPUS,FPUS,FPUS,FPRM,
/*4.*/ FPUC,FPUC,FPUC,FPUC,FPUU,FPY1,FPY1,FPY1,FPY1,FPRM,
/*5.*/ FPY2,FPY2,FPY2,FPY2,FPUU,FPLY,FPLS,FPDS,FPDS,FPEF};
#define FRAME_SIZE 60

// Receiver module http://canaduino.ca/downloads/60khz.pdf
// Receiver IC http://canaduino.ca/downloads/MAS6180C.pdf
/*
P.2 Note.2 OUT = VSS(low) when carrier amplitude at maximum;
OUT = VDD(high) when carrier amplitude is reduced (modulated)
P.7 Table.5 Recommended pulse width recognition limits for WWVB
Symbol  Min Max Unit
T 200ms 100 300 ms
T 500ms 400 600 ms
T 800ms 700 900 ms
*/
#define RADIO_POWERDOWN_PIN 6 // P1
#define RADIO_OUT_PIN       7 // T
uint8_t radioPort, radioBit;
#define SAMPLE_HZ  100 // must be a factor of 62500: 2, 4, 5, 10, 20, 25, 50, 100, 125, and less than 128
byte samples, samplesHigh, samplesLow, prevMod;
#define CODE_N 0
#define CODE_U 1
#define CODE_W 2
#define CODE_P 3
#define CODE_X 4
byte code = CODE_N;
unsigned long cyclesSinceTimeSet = 0x80000000;

/* Timer 1 interrupt to measure signal
http://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
Timer0 8bit used for the timer functions, like delay(), millis() and micros()
Timer1 16bit the Servo library uses timer1 on Arduino Uno (timer5 on Arduino Mega)
Timer2 8bit the tone() function uses timer2
Timer 3,4,5 16bit only available on Arduino Mega boards
*/
ISR(TIMER1_COMPA_vect)  {
  // OUT = VSS(low) when carrier amplitude at maximum;
  // OUT = VDD(high) when carrier amplitude is reduced (modulated)
  // !digitalRead(RADIO_OUT_PIN)
  byte modulated = *portInputRegister(radioPort) & radioBit;
  if (modulated & prevMod) {
    samplesLow++;
  } else if (!(modulated | prevMod)) {
    samplesHigh++;
  }
  prevMod = modulated;
  ++cyclesSinceTimeSet;
  if (--samples == 0) {
  	// end of one sample interval
    if      (samplesLow > 63*SAMPLE_HZ/100 && samplesLow < 90*SAMPLE_HZ/100) code = CODE_P;
    else if (samplesLow > 33*SAMPLE_HZ/100 && samplesLow < 60*SAMPLE_HZ/100) code = CODE_W;
    else if (samplesLow >  5*SAMPLE_HZ/100 && samplesLow < 30*SAMPLE_HZ/100) code = CODE_U;
    else code = CODE_X;
    samples = SAMPLE_HZ; samplesLow = samplesHigh = 0; // clear for next sample
  }
}

// 8 x 7 segment LED display module (DFR0090) https://www.dfrobot.com/wiki/index.php/3-Wire_LED_Module_(SKU:DFR0090)
#define LED_LATCH_PIN     8
#define LED_CLOCK_PIN     3
#define LED_DATA_PIN      9
// Table of segments for digits 0-9
const byte LED_Digit_Segments[] = {
    // 0    1    2    3    4    5    6    7    8    9
    0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
#define LED_SEGMENTS_OFF     0xFF
// Table of segments for letters A-Z
byte LED_Letter_Segments[]={
    // A    B    C    D    E    F    G    H    I    J    K    L    M
    0xA0,0x83,0xa7,0xa1,0x86,0x8e,0xc2,0x8b,0xe6,0xe1,0x89,0xc7,0xaa,
    // N    O    P    Q    R    S    T    U    V    W    X    Y    Z
    0xc8,0xa3,0x8c,0x98,0xce,0x9b,0x87,0xc1,0xe3,0xd5,0xb6,0x91,0xb8};
byte display_segments[8];

void displayShift(byte segments) {
  digitalWrite(LED_LATCH_PIN, LOW);
  shiftOut(LED_DATA_PIN, LED_CLOCK_PIN, MSBFIRST, segments);
  digitalWrite(LED_LATCH_PIN, HIGH);
}

void displaySend(void) {
  for (int d = 7;  d >= 0;  d--) displayShift(display_segments[d]);
}

// Sparkfun DeadOn Real Ttime Clock https://learn.sparkfun.com/tutorials/deadon-rtc-breakout-hookup-guide
#define RTC_SELECT_PIN     10
#define RTC_INTERRUPT_PIN  2
// GND - GND
// VCC - 5V
// SQW - D2
// CLK - D13  ** conflicts with LED_BUILTIN!
// MISO - D12
// MOSI - D11
// SS - D10

int zoneHours = 0;

byte frame[FRAME_SIZE], frameIndex = 0;
short decode[FPEF];   // accumulated parts of frame
boolean timeSet = false;
byte daysInMonth[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
//                     JanFebMarAprMayJunJulAugSepOctNovDec  

void decodeAndSetTime(void) {
  // Decode frame parts
  memset(decode,0,sizeof decode);
  for (int x = 0; x < FRAME_SIZE; x++) {
    byte p = FramePattern[x], c = frame[x];
    switch (p) {
    case FPRM:  // Frame reference marker: .8L+.2H
      if (c != CODE_P) return;
      break;
    case FPEF:  // End of frame same as FPRM
      if (c != CODE_P) return;
      break;
    case FPUU:  // Unweighted: .2L+.8H
      if (c != CODE_U) return;
      break;
    default:  // bit 1: .5L+.5H / 0 = .2L+.8H
      // binary coding
      if (c == CODE_U) decode[p] = (decode[p] << 1);
      else if (c == CODE_W) decode[p] = (decode[p] << 1) | 1;
      else return;
      break;
    }
  }
  // Combine decimal digits
  int mn, hr, dy, us, uc, yr, ly, ls, ds;
  mn = decode[FPM1]*10 + decode[FPM2];
  hr = decode[FPH1]*10 + decode[FPH2];
  dy = decode[FPD1]*100 + decode[FPD2]*10 + decode[FPD3];
  us = decode[FPUS];
  uc = decode[FPUC];
  yr = decode[FPY1]*10 + decode[FPY2] + CENTURY;
  ly = decode[FPLY];
  ls = decode[FPLS];
  ds = decode[FPDS];
  if (Serial) {
    Serial.print("Y"); Serial.print(yr);
    Serial.print("D"); Serial.print(dy);
    Serial.print("H"); Serial.print(hr);
    Serial.print("M"); Serial.print(mn);
    Serial.print("US"); if (us==5) Serial.write('+'); else if (us==2) Serial.write('-'); else Serial.print(us);
    Serial.print("UC"); Serial.print(uc);
    Serial.print("LY"); Serial.print(ly);
    Serial.print("LS"); Serial.print(ls);
    Serial.print("DS"); Serial.print(ds);
    Serial.write('\r'); Serial.write('\n');
  }
  // Correct for 1 minute coding delay from on-time point
  mn += 1;
  if (mn >= 60) {
    hr += 1;  mn = 0;
    if (hr >= 24) {
      dy += 1;  hr = 0;
      if (dy >= 365+ly) {
        yr += 1;  dy = 1;
      }
    }
  }
  // Update crystal clock
  rtc.setSecond(1);  //TODO correct for delay from time of reception
  rtc.setMinute(mn);
  rtc.setHour(hr);
  rtc.setYear(yr);
  int mo=1, dim;
  while (1) {
    dim = daysInMonth[mo];
    if (mo == 2 && ly == 1) dim += 1;
    if (dy <= dim) break;
    dy -= dim;  mo += 1;
  }
  rtc.setMonth(mo);
  rtc.setDay(dy);
  timeSet = true;
  cyclesSinceTimeSet = 0;
}

void timeToDisplay(void) {
  rtc.update();
  unsigned long sinceTimeSet = cyclesSinceTimeSet / (SAMPLE_HZ * 86400L); // days
  if (sinceTimeSet == 0) {
    sinceTimeSet = cyclesSinceTimeSet / (SAMPLE_HZ * 8640L);  // tenth days
    if (sinceTimeSet > 9) display_segments[7] = 0xB6; // X
    else display_segments[7] = LED_Digit_Segments[sinceTimeSet];
    display_segments[6] = 0x47; // L.
  } else if (sinceTimeSet <= 9) {
    display_segments[7] = LED_Digit_Segments[sinceTimeSet];
    display_segments[6] = 0xB6; // X
  } else {
    display_segments[7] = display_segments[6] = 0xB6; // X X
  }
  int d = rtc.second();
  display_segments[5] = LED_Digit_Segments[d % 10];
  display_segments[4] = LED_Digit_Segments[d / 10];
  d = rtc.minute();
  display_segments[3] = LED_Digit_Segments[d % 10];
  display_segments[2] = LED_Digit_Segments[d / 10];
  d = rtc.hour();
  display_segments[1] = LED_Digit_Segments[d % 10];
  display_segments[0] = LED_Digit_Segments[d / 10];
  displaySend();
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  pinMode(RADIO_POWERDOWN_PIN, OUTPUT);
  pinMode(RADIO_OUT_PIN, INPUT);
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(LED_LATCH_PIN, OUTPUT);
  pinMode(LED_DATA_PIN, OUTPUT);  
  pinMode(LED_CLOCK_PIN, OUTPUT);
  // Clear display
  for (int d = 7;  d >= 0;  d--) displayShift(LED_SEGMENTS_OFF);
  // Initialize RT clock library
  rtc.begin(RTC_SELECT_PIN);  
  rtc.writeSQW(SQW_SQUARE_1);  // 1Hz signal on RTC_INTERRUPT_PIN
  if (rtc.readFromSRAM(0) == 'Z') {
    zoneHours = '0' - rtc.readFromSRAM(1);
    Serial.print("Zone hours ");
    Serial.println(zoneHours);
  }
  // Start radio
  digitalWrite(RADIO_POWERDOWN_PIN, LOW);  // turn on radio
  radioPort = digitalPinToPort(RADIO_OUT_PIN);  // for optimized digitalRead
  radioBit = digitalPinToBitMask(RADIO_OUT_PIN);
  samples = SAMPLE_HZ; samplesLow = samplesHigh = 0;
  // Start timer1 for periodic interrupt at SAMPLE_HZ per second
  noInterrupts(); {
  	TCCR1A = 0;
  	TCCR1B = 0;
  	TCNT1  = 0;
  	OCR1A = F_CPU / 256 / SAMPLE_HZ - 1;    // compare match register 16MHz / 256 prescaler / SAMPLE_HZ
                                    // https://github.com/ahooper/WWVBClock/issues/1
  	TCCR1B |= (1 << WGM12);   // CTC mode
  	TCCR1B |= (1 << CS12);    // 256 prescaler 
  	TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  } interrupts(); 
}

byte prevCode = CODE_N;
//                          _N  _U  _W  _P  _X
char printCode[CODE_X+1] = {' ','.','-','|','*'};
int lastCycle = HIGH, prevSerial = 0;
byte signalSegments[] = {
      /*0*/~(0x80),
      /*1*/~(0x08),
      /*2*/~(0x08),
      /*3*/~(0x08|0x04|0x10),
      /*4*/~(0x08|0x40),
      /*5*/~(0x08|0x40),
      /*6*/~(0x08|0x40|0x04|0x10),
      /*7*/~(0x08|0x40|0x01),
      /*8*/~(0x08|0x40|0x01),
      /*9*/~(0x08|0x40|0x01|0x04|0x10),
      /*10*/~(0x08|0x40|0x01|0x02|0x20),
};

void loop(void) {
  if (code > CODE_N) {
    // once per second
    if (code == CODE_P && prevCode == CODE_P) {
      // once per minute
      if (frameIndex == FRAME_SIZE) {
        decodeAndSetTime();
      }
      if (Serial) { Serial.write('\r'); Serial.write('\n'); }
      frameIndex = 0;
    } 
    if (frameIndex < FRAME_SIZE) frame[frameIndex++] = code;
      if (Serial) { Serial.write(printCode[code]); }
  	prevCode = code;  code = CODE_N;
  } else if (digitalRead(RTC_INTERRUPT_PIN) != lastCycle) {
    // 1 Hz signal from crystal clock
    lastCycle = digitalRead(RTC_INTERRUPT_PIN);
    if (lastCycle == LOW) {
      timeToDisplay();
      /*
      if (timeSet) timeToDisplay();
      else {
        // show carrier samples while waiting for lock
        displayShift(signalSegments[(samplesLow*10)/SAMPLE_HZ]);
      }
      */
    }
  } else if (Serial && Serial.available()) {
    int r = Serial.read();
    if (prevSerial == 'Z') {
      // Set time zone
      if (r >= '0' && r <= '9') {
        zoneHours = '0' - r;
        rtc.writeToSRAM(0,'Z'); rtc.writeToSRAM(1,r);
        Serial.print("Zone hours ");
        Serial.println(zoneHours);
      }
    }
    prevSerial = r;
  }
}
