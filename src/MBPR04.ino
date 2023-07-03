/*
 * Multiband Phasing Direct Conversion SSB Receiver
 *
 * Copyright 2023 Ian Mitchell VK7IAN
 * Version 1.0
 *
 * Uses
 * ====
 *  
 * https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *
 * Libraries
 * =========
 *
 * https://github.com/etherkit/Si5351Arduino
 * https://github.com/brianlow/Rotary
 * https://github.com/dxinteractive/ResponsiveAnalogRead
 * https://github.com/datacute/Tiny4kOLED
 *
 * Note: modify SI5351_PLL_VCO_MIN to 440000000 in si5351.h:
 *
 * #define SI5351_PLL_VCO_MIN 440000000
 *
 */


#include <si5351.h>
#include <Rotary.h>
#include <ResponsiveAnalogRead.h>
#include <Tiny4kOLED.h>
#include "agc.h"

#if SI5351_PLL_VCO_MIN != 440000000
#err set SI5351_PLL_VCO_MIN to 440000000 in si5351.h
#endif

//#define MDPR_DEBUG
//#define DEFAULT_FREQUENCY 7105000ul
//#define DEFAULT_FREQUENCY 7040000ul
//#define DEFAULT_MODE      MODE_LSB
//#define DEFAULT_BAND      BAND_40
#define DEFAULT_FREQUENCY 14060000ul
#define DEFAULT_MODE      MODE_USB
#define DEFAULT_BAND      BAND_20M
#define MIN_FREQUENCY     3500000UL
#define MAX_FREQUENCY     30000000UL

#define PIN_FINE   A0
#define PIN_AGCIN  A1
#define PIN_FREE   A2
#define PIN_AGCOUT D3 // PWM
#define PIN_SDA    D4
#define PIN_SCL    D5
#define PIN_ENCBUT D6
#define PIN_BAND1  D7
#define PIN_BAND0  D8
#define PIN_ENCA   D9
#define PIN_ENCB  D10

// this the value of noise to subtract from the peak to get a better resolution S meter
#define NOISE_FACTOR 20

// for built in LED, low is on
#define LED_BUILTIN_1 16u
#define LED_BUILTIN_2 17u
#define LED_BUILTIN_3 25u

#define TCXO_FREQ 27000000ul
//#define XTAL_FREQ  27000000ul
//#define SI5351_CAL 16099L

enum radio_mode_t
{
  MODE_NONE,
  MODE_LSB,
  MODE_USB
};

enum radio_band_t
{
  BAND_80M,
  BAND_40M,
  BAND_20M,
  BAND_15M,
  BAND_10M
};

enum button_state_t
{
  BUTTON_STATE_WAIT_PRESS,
  BUTTON_STATE_WAIT_RELEASE
};

static const float smeter_factor[] = 
{
  19.22f,
  17.41f,
  16.47f,
  16.47f
};

volatile static struct
{
  uint32_t frequency;
  uint32_t divisor;
  uint32_t filter;
  radio_mode_t mode;
  radio_band_t band;
  bool mute;
  const struct
  {
    const uint32_t lo;
    const uint32_t hi;
  } filters[];
}
radio =
{
  DEFAULT_FREQUENCY,
  0UL,
  0UL,
  DEFAULT_MODE,
  DEFAULT_BAND,
  false,
  {
    { 3500000UL, 4000000UL},
    { 4000000UL, 8000000UL},
    { 8000000UL,16000000UL},
    {16000000UL,30000000UL}
  }
};

Si5351 si5351;
Rotary r = Rotary(PIN_ENCB,PIN_ENCA);
ResponsiveAnalogRead fineTune(PIN_FINE,true);

volatile static bool setup_complete = false;
volatile static uint32_t set_frequency = 0;
volatile static uint32_t set_signal = 0;
volatile static radio_mode_t set_mode = MODE_NONE;
auto_init_mutex(radio_metadata);

static const uint32_t get_filter(const uint32_t f)
{
  for (uint32_t i=0;i<4;i++)
  {
    if (f>=radio.filters[i].lo && f<=radio.filters[i].hi)
    {
      return i;
    }
  }
  return 0;  
}

static void set_filter(const uint32_t filter)
{
  switch (filter)
  {
    case 0: digitalWrite(PIN_BAND1,HIGH); digitalWrite(PIN_BAND0,HIGH); break;
    case 1: digitalWrite(PIN_BAND1,HIGH); digitalWrite(PIN_BAND0,LOW);  break;
    case 2: digitalWrite(PIN_BAND1,LOW);  digitalWrite(PIN_BAND0,HIGH); break;
    case 3: digitalWrite(PIN_BAND1,LOW);  digitalWrite(PIN_BAND0,LOW);  break;
  }
}

static const radio_mode_t get_mode(const uint32_t f)
{
  if (f==7074000ul)
  {
    return MODE_USB;
  }
  if (f<10000000ul)
  {
    return MODE_LSB;
  }
  return MODE_USB;
}

static const uint32_t get_divisor(const uint32_t f)
{
  if (f < 6850000ul)
  {
    return 126;
  }
  if (f < 9500000ul)
  {
    return 88;
  }
  if (f < 13600000ul)
  {
    return 64;
  }
  if (f < 17500000ul)
  {
    return 44;
  }
  if (f < 25000000ul)
  {
    return 34;
  }
  if (f < 36000000ul)
  {
    return 24;
  }
  if (f < 45000000ul)
  {
    return 18;
  }
  if (f < 60000000ul)
  {
    return 14;
  }
  if (f < 80000000ul)
  {
    return 10;
  }
  if (f < 100000000ul)
  {
    return 8;
  }
  if (f < 146600000ul)
  {
    return 6;
  }
  return 4;
}

void setup()
{
  // make sure the built-in LED is off
  pinMode(LED_BUILTIN_1,OUTPUT);
  pinMode(LED_BUILTIN_2,OUTPUT);
  pinMode(LED_BUILTIN_3,OUTPUT);
  digitalWrite(LED_BUILTIN_1,HIGH);
  digitalWrite(LED_BUILTIN_2,HIGH);
  digitalWrite(LED_BUILTIN_3,HIGH);
#ifdef MDPR_DEBUG
  for (int i=0;i<5;i++)
  {
    delay(500);
    digitalWrite(LED_BUILTIN_1,LOW);
    digitalWrite(LED_BUILTIN_2,LOW);
    digitalWrite(LED_BUILTIN_3,LOW);
    delay(100);
    digitalWrite(LED_BUILTIN_1,HIGH);
    digitalWrite(LED_BUILTIN_2,HIGH);
    digitalWrite(LED_BUILTIN_3,HIGH);
  }
  delay(2000);
#endif
  analogWriteResolution(8);
  analogWriteFreq(500000ul);
  analogWrite(PIN_AGCOUT,0);
  pinMode(PIN_ENCBUT,INPUT_PULLUP);
  pinMode(PIN_ENCA,INPUT_PULLUP);
  pinMode(PIN_ENCB,INPUT_PULLUP);
  pinMode(PIN_BAND0,OUTPUT);
  pinMode(PIN_BAND1,OUTPUT);
  pinMode(PIN_FREE,OUTPUT);
  digitalWrite(PIN_FREE,LOW);
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.setClock(400000ul);
  analogReadResolution(12);
  analogRead(PIN_FINE);
  analogRead(PIN_AGCIN);
  fineTune.setAnalogResolution(4096);
  r.begin();
  radio.frequency = DEFAULT_FREQUENCY;
  radio.divisor = get_divisor(radio.frequency);
  radio.filter = get_filter(radio.frequency);
  set_filter(radio.filter);
#ifdef MDPR_DEBUG
  for (int i=0;i<5;i++)
  {
    delay(500);
    digitalWrite(LED_BUILTIN_1,LOW);
    digitalWrite(LED_BUILTIN_2,LOW);
    digitalWrite(LED_BUILTIN_3,LOW);
    delay(100);
    digitalWrite(LED_BUILTIN_1,HIGH);
    digitalWrite(LED_BUILTIN_2,HIGH);
    digitalWrite(LED_BUILTIN_3,HIGH);
  }
  delay(2000);
#endif
  si5351.init(SI5351_CRYSTAL_LOAD_0PF,TCXO_FREQ,0);
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_8MA);

  const uint64_t f = radio.frequency*SI5351_FREQ_MULT;
  const uint64_t p = radio.frequency*radio.divisor*SI5351_FREQ_MULT;
  si5351.set_freq_manual(f,p,SI5351_CLK0);
  si5351.set_freq_manual(f,p,SI5351_CLK1);
  si5351.set_phase(SI5351_CLK0,0);
  si5351.set_phase(SI5351_CLK1,radio.divisor);
  si5351.pll_reset(SI5351_PLLA);

  oled.begin(64,32,sizeof(tiny4koled_init_64x32br),tiny4koled_init_64x32br);
  oled.enableChargePump();
  oled.setRotation(0);
  oled.setInternalIref(true);
  oled.clear();
  oled.on();
  oled.switchRenderFrame();

  // intro screen
  oled.clear();
  oled.setFont(FONT6X8);
  oled.setCursor(0,0);
  oled.print("Phasing RX");
  oled.setCursor(0,1);
  oled.print("    by    ");
  oled.setFont(FONT8X16);
  oled.setCursor(0,2);
  oled.print(" VK7IAN");
  oled.switchFrame();

  // splash delay
  delay(2000);

  setup_complete = true;
}

void setup1()
{
  // wait for setup() to complete
  while (!setup_complete)
  {
    // setup() not done yet
  }
}

void loop()
{
  static uint32_t old_frequency = radio.frequency;
  static uint32_t old_divisor = radio.divisor;
  static int32_t fine_tune = 0;
  static button_state_t button_state = BUTTON_STATE_WAIT_PRESS;

  // process AGC
  uint32_t peak_signal = AGC::peak((int16_t)analogRead(PIN_AGCIN)-2048l);
  peak_signal = peak_signal>NOISE_FACTOR?peak_signal-NOISE_FACTOR:0;
  const int32_t agc_pwm = AGC::attenuation(peak_signal);
  analogWrite(PIN_AGCOUT,agc_pwm);

  // process tuning
  fineTune.update(analogRead(PIN_FINE)>>4);
  fine_tune = ((int32_t)fineTune.getValue()-128L);
  fine_tune = fine_tune<-10?fine_tune+10:(fine_tune>10?fine_tune-10:0);
  fine_tune = constrain(fine_tune,-100,100)*5;

  switch (r.process())
  {
    case DIR_CW:
    {
      radio.frequency += 1000;
      break;
    }
    case DIR_CCW:
    {
      radio.frequency -= 1000;
      break;
    }
  }

  // band change
  switch (button_state)
  {
    case BUTTON_STATE_WAIT_PRESS:
    {
      if (digitalRead(PIN_ENCBUT)==LOW)
      {
        delay(50);
        switch (radio.band)
        {
          case BAND_80M: radio.band = BAND_40M; radio.frequency =  7100000ul; break;
          case BAND_40M: radio.band = BAND_20M; radio.frequency = 14200000ul; break;
          case BAND_20M: radio.band = BAND_15M; radio.frequency = 21100000ul; break;
          case BAND_15M: radio.band = BAND_10M; radio.frequency = 28400000ul; break;
          case BAND_10M: radio.band = BAND_80M; radio.frequency =  3600000ul; break;
        }
        radio.mute = true;
        button_state = BUTTON_STATE_WAIT_RELEASE;
      }
      break;
    }
    case BUTTON_STATE_WAIT_RELEASE:
    {
      if (digitalRead(PIN_ENCBUT)==LOW)
      {
        break;
      }
      delay(50);
      button_state = BUTTON_STATE_WAIT_PRESS;
    }
    break;
  }

  // set frequency, mode and filter
  radio.frequency = constrain(radio.frequency,MIN_FREQUENCY,MAX_FREQUENCY);
  uint32_t tuned_frequency = radio.frequency+fine_tune;
  tuned_frequency = constrain(tuned_frequency,MIN_FREQUENCY,MAX_FREQUENCY);
  radio.mode = get_mode(tuned_frequency);
  radio.filter = get_filter(tuned_frequency);
  set_filter(radio.filter);

  // muting
  if (radio.mute)
  {
    analogWrite(PIN_AGCOUT,0);
    delay(50);
  }

  // 50 pixels is S9
  // @3.5MHz divisor = 50/log10(400) = 19.22
  // @7Mhz   divisor = 50/log10(744) = 17.41
  // @14Mhz  divisor = 50/log10(1085) = 16.47
  const float smeter_signal = (float)(peak_signal?smeter_factor[radio.filter]*log10f(peak_signal):peak_signal);

  // pass the data to core1
  mutex_enter_blocking(&radio_metadata);
  set_frequency = tuned_frequency;
  set_mode = radio.mode;
  set_signal = (uint32_t)smeter_signal;
  mutex_exit(&radio_metadata);

  // set mute delay
  if (radio.mute)
  {
    radio.mute = false;
    delay(100);
  }
}

void loop1()
{
  // get the data from core0
  mutex_enter_blocking(&radio_metadata);
  volatile const uint32_t new_frequency = set_frequency;
  volatile const uint32_t new_signal = set_signal;
  volatile const radio_mode_t new_mode = set_mode;
  mutex_exit(&radio_metadata);

  // if frequency changes, change local oscillator and update display
  volatile static uint32_t current_frequency = 0;
  volatile static uint32_t current_divisor = 0;
  volatile static uint32_t current_signal = 0;
  volatile static radio_mode_t current_mode = MODE_NONE;
  bool update_display = false;
  if ((current_frequency != new_frequency) || (current_mode != new_mode))
  {
    current_frequency = new_frequency;
    current_mode = new_mode;
    const uint32_t new_divisor = get_divisor(current_frequency);
    const uint64_t f = current_frequency * SI5351_FREQ_MULT;
    const uint64_t p = current_frequency * new_divisor * SI5351_FREQ_MULT;
    si5351.set_freq_manual(f,p,SI5351_CLK0);
    si5351.set_freq_manual(f,p,SI5351_CLK1);
    switch (current_mode)
    {
      case MODE_LSB:
      {
        si5351.set_phase(SI5351_CLK0,0);
        si5351.set_phase(SI5351_CLK1,new_divisor);
        break;
      }
      case MODE_USB:
      {
        si5351.set_phase(SI5351_CLK0,new_divisor);
        si5351.set_phase(SI5351_CLK1,0);
        break;
      }
    }
    if (current_divisor != new_divisor)
    {
      current_divisor = new_divisor;
      si5351.pll_reset(SI5351_PLLA);
    }
    update_display = true;
  }

  if (update_display || (current_signal != new_signal))
  {
    current_signal = new_signal;
    char sz_frequency[16] = "";
    memset(sz_frequency,0,sizeof(sz_frequency));
    ultoa(new_frequency,sz_frequency,10);
    const uint8_t fp = strlen(sz_frequency)==7?10u:7u;
    oled.clear();
    oled.setFont(FONT6X8);
    oled.setCursor(fp,0);
    oled.print(sz_frequency);
    oled.setCursor(0,1);
    oled.print("Mode:  ");
    oled.print(new_mode==MODE_LSB?"LSB":"USB");
    oled.setCursor(0,2);
    oled.print("-3-5-7-9-+");
    const uint8_t sig = min(new_signal,63);
    oled.bitmap(0, 3, sig, 4, AGC::meter);
    oled.switchFrame();
    delay(20);
  }
}