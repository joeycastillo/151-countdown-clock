#include <RTClib.h>
#include <Bounce2.h>
#include <Wire.h>

// A countdown clock based around: 
//  * three chained 74HC595 shift registers connected to
//  * three 7-segment displays, plus a
//  * DS3231 RTC on the I2C interface, and
//  * two optional buttons for setting the time.

// Hook up pins as described below:
// Buttons. Should connect to GND when depressed.
const int PIN_BUTTON_R = 2;
const int PIN_BUTTON_L = 3;
// 74HC595 pins. I put them on A0-A2 but they're digital signals, you can use any pins.
const int PIN_DISPLAY_DATA = A0;
const int PIN_DISPLAY_CLOCK = A1;
const int PIN_DISPLAY_LATCH = A2;
// RTC uses the I2C interface. SDA is A4 and SCL is A5.
RTC_DS3231 rtc;

byte Digits[] =
{
  B1111110, // 0
  B0110000, // 1
  B1101101, // 2
  B1111001, // 3
  B0110011, // 4
  B1011011, // 5
  B1011111, // 6
  B1110000, // 7
  B1111111, // 8
  B1111011, // 9
  B0111011, // Y
  B1110110, // M
  B1111110, // D
  B0110111, // H
  B0010101, // N
  B1011011, // S
  B0011100, // y
  B0010101, // m
  B0111101, // d
};

// Oh how we're abusing the DS3231.
// Registers 0x07 through 0x09 are the seconds, minutes, hours and days for Alarm 1.
// We're using 0x08 to store the desired year for our countdown and 0x09 for the month
// (conveniently clamped to 1-12 like hours). The day register gets to be used for 
// its intended 1-31 range, as long as a flag elsewhere in the byte is set to 0.
// There are other flags in other registers, but we want them all set to 0 anyway.
// Also note this won't work for years after 2059. I suppose you could use register
// 0x07 to store a value you add to 0x08, but for now I want to use it to confirm the
// RTC is connected and working (plus we're gonna have bigger problems come 2060).
#define STATUS_CHECK_REGISTER 0x07
#define DEADLINE_YEAR_REGISTER 0x08
#define DEADLINE_MONTH_REGISTER 0x09
#define DEADLINE_DAY_REGISTER 0x0A

// Mode. Two buttons wired to D2 and D3 let you set the date / time and the target
// for the countdown. Mode is 0 for normal operation or 10-18 for setting things.
#define MODE_NORMAL 0
#define MODE_SET_YEAR 10
#define MODE_SET_MONTH 11
#define MODE_SET_DAY 12
#define MODE_SET_HOUR 13
#define MODE_SET_MINUTE 14
#define MODE_SET_SECOND 15
#define MODE_SET_COUNTDOWN_YEAR 16
#define MODE_SET_COUNTDOWN_MONTH 17
#define MODE_SET_COUNTDOWN_DAY 18

#define MODE_LAST MODE_SET_COUNTDOWN_DAY
uint8_t mode = MODE_NORMAL;

// Bounce2 library. Makes it easy to detect button presses without a lot of boilerplate.
Bounce buttonR = Bounce();
Bounce buttonL = Bounce();

// The julian date we're counting down to. Is set based on the values in the RTC
// registers in setup(), and reset when we change the date in settings.
long deadline = 0;

void setup()
{
  // set up shift register
  pinMode(PIN_DISPLAY_DATA, OUTPUT);
  pinMode(PIN_DISPLAY_LATCH, OUTPUT);
  pinMode(PIN_DISPLAY_CLOCK, OUTPUT);
  // buttons
  buttonR.attach(PIN_BUTTON_R, INPUT_PULLUP);
  buttonL.attach(PIN_BUTTON_L, INPUT_PULLUP);
  // and RTC.
  rtc.begin();

  // set the time if not already set. while we're at it, set a target date.
  if (rtc.lostPower())
  {
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    write_i2c_register(DS3231_ADDRESS, STATUS_CHECK_REGISTER, 7); // magic number, we just check for it again below.
    write_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER, packValue(19)); // years after 2000
    write_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER, packValue(7));
    write_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER, packValue(31));
  }

  // Try to read back the value we previously set in STATUS_CHECK_REGISTER. 
  // Not there? Maybe the RTC isn't connected!
  if (read_i2c_register(DS3231_ADDRESS, STATUS_CHECK_REGISTER) != 7)
  {
    writeToLED(0b01001111, 0b00000101, 0b00000101); // display "Err"
    while (1);
  }

  // Finally, update the deadline. Now we're all set!
  deadline = JulianDate(2000 + unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER)));
}

void loop()
{
  buttonL.update();
  buttonR.update();

  if (buttonR.fell())
  {
    // mode goes 0 -> 10 -> 11 ... 17 -> 18 -> 0
    if (mode == MODE_NORMAL) mode = MODE_SET_YEAR;
    else mode++;

    if (mode > MODE_LAST) mode = MODE_NORMAL;
  }

  DateTime now = rtc.now();
  int component = 0;

  // this switch is just for settings, and all those cases return before leaving the switch.
  switch (mode)
  {
    case MODE_NORMAL:
      // for normal operation, break and continue below.
      break;
    case MODE_SET_YEAR:
      component = now.year();
      if (buttonL.fell())
      {
        now = rtc.now();
        component = now.year() + 1;
        if (component > 2099) component = 2000;
        rtc.adjust(DateTime(component, now.month(), now.day(), now.hour(), now.minute(), now.second()));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      return;
    case MODE_SET_MONTH:
      component = now.month();
      if (buttonL.fell())
      {
        now = rtc.now();
        component = now.month() + 1;
        if (component > 12) component = 1;
        rtc.adjust(DateTime(now.year(), component, now.day(), now.hour(), now.minute(), now.second()));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      return;
    case MODE_SET_DAY:
      component = now.day();
      if (buttonL.fell())
      {
        now = rtc.now();
        component = now.day() + 1;
        // FIXME: determine max day base on which month is set.
        if (component > 28) component = 1;
        rtc.adjust(DateTime(now.year(), now.month(), component, now.hour(), now.minute(), now.second()));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      return;
    case MODE_SET_HOUR:
      component = now.hour();
      if (buttonL.fell())
      {
        now = rtc.now();
        component = now.hour() + 1;
        if (component > 23) component = 0;
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), component, now.minute(), now.second()));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      return;
    case MODE_SET_MINUTE:
      component = now.minute();
      if (buttonL.fell())
      {
        now = rtc.now();
        component = now.minute() + 1;
        if (component > 59) component = 0;
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), component, now.second()));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      return;
    case MODE_SET_SECOND:
      component = now.second();
      if (buttonL.fell())
      {
        // just set seconds to 0 whenever the button is pressed in this mode.
        now = rtc.now();
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), 0));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      return;
    case MODE_SET_COUNTDOWN_YEAR:
      component = unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER));
      if (buttonL.fell())
      {
        component++;
        if (component > 59) component = 0;
        write_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER, packValue(component));
        component = unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      deadline = JulianDate(2000 + unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER)));
      return;
    case MODE_SET_COUNTDOWN_MONTH:
      component = unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER));
      if (buttonL.fell())
      {
        component++;
        if (component > 12) component = 1;
        write_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER, packValue(component));
        component = unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      deadline = JulianDate(2000 + unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER)));
      return;
    case MODE_SET_COUNTDOWN_DAY:
      component = unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER));
      if (buttonL.fell())
      {
        component++;
        if (component > 31) component = 1;
        write_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER, packValue(component));
        component = unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER));
      }
      writeToLED(Digits[mode], Digits[(component / 10) % 10], Digits[component % 10]);
      deadline = JulianDate(2000 + unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_YEAR_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_MONTH_REGISTER)), unpackValue(read_i2c_register(DS3231_ADDRESS, DEADLINE_DAY_REGISTER)));
      return;
  }

  // default mode begins here
  long julianNow = JulianDate(now.year(), now.month(), now.day());
  long remaining = deadline - julianNow - 1;

  unsigned char ones = remaining % 10;
  unsigned char tens = (remaining / 10) % 10;
  unsigned char hundreds = (remaining / 100) % 10;

  if (remaining < 1)
  {
    // for 0 display "-0-"
    writeToLED(0b00000001, Digits[0], 0b00000001);
  } else if (hundreds != 0)
  {
    // for three digits display all
    writeToLED(Digits[hundreds], Digits[tens], Digits[ones]);
  } else if (tens != 0)
  {
    // for two digits omit the leading 0
    writeToLED(0, Digits[tens], Digits[ones]);
  } else
  {
    // for one digit, center it
    writeToLED(0, Digits[ones], 0);
  }

  delay(1000); // pause for 1 second before updating again.
}

/**********************
 ** HELPER FUNCTIONS **
 **********************/

// Write three bytes to the display in the order it wants them.
// I wired A0-A3 to the shift register connected to the leftmost display, so it
// shifts the first two bytes sent out to the displays to the right. this means we
// need to send the last byte first so it gets shifted out to the rightmost display.
void writeToLED(byte b1, byte b2, byte b3)
{
  digitalWrite(PIN_DISPLAY_LATCH, LOW);
  shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, LSBFIRST, b3);
  shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, LSBFIRST, b2);
  shiftOut(PIN_DISPLAY_DATA, PIN_DISPLAY_CLOCK, LSBFIRST, b1);
  digitalWrite(PIN_DISPLAY_LATCH, HIGH);
}

// These methods pack and unpack a value for the DS2321 alarm registers.
// It's odd. The tens place gets stuffed into the least significant bytes
// of the upper nibble, and the ones place gets stuffed into the lower nibble.
// There are sometimes some flags off to the left, but if you give this an
// appropriate value for the register you're setting they'll all be 0.
uint8_t packValue(uint8_t value)
{
  uint8_t ones = value % 10;
  uint8_t tens = (value / 10) % 10;
  return (tens << 4) | ones;
}

uint8_t unpackValue(uint8_t packed)
{
  uint8_t ones = packed & 0xF;
  uint8_t tens = (packed >> 4) & 0xF;
  return tens * 10 + ones;
}

// ganked these two methods from RTCLib.
// we need them so we can mess with the alarm registers directly.
uint8_t read_i2c_register(uint8_t addr, uint8_t reg)
{
  Wire.beginTransmission(addr);
  Wire.write((byte)reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);
  return Wire.read();
}

void write_i2c_register(uint8_t addr, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(addr);
  Wire.write((byte)reg);
  Wire.write((byte)val);
  Wire.endTransmission();
}

// from the Arduino forums. use Julian dates to ensure consistent date differentials.
long JulianDate(int year, int month, int day)
{
  long centuries = year/100;
  long leaps = centuries/4;
  long leapDays = 2 - centuries + leaps;  // note is negative!!
  long yearDays = 365.25 * (year + 4716); // days until 1 jan this year
  long monthDays = 30.6001* (month + 1);  // days until 1st month
  return leapDays + day + monthDays + yearDays -1524.5;
}
