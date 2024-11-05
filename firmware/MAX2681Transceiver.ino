/*
MAX2681 Transceiver

Jason Milldrum, NT7S
4 November 2024


IF frequency is positive for sum product (IF = RF + LO) and negative for diff (IF = RF - LO)
VFO signal output on CLK0, BFO signal on CLK2

*/
#include <si5351.h>
#include <RotaryEncoder.h>
#include <U8g2lib.h>


// Pin defines
#define TX_PIN 0
#define LED_PIN 3
#define VSENSE_PIN 4
#define PADDLE_DIT_PIN 6
#define PADDLE_DAH_PIN 7
#define ENC_A_PIN 11
#define ENC_B_PIN 10
#define MUTE_PIN 13
#define SIDETONE_PIN 14
#define BUTTON_PIN 15

// Limits
#define KEYER_SPEED_LOWER 5
#define KEYER_SPEED_UPPER 40
#define BUTTON_ADC_MARGIN 40
#define BUTTON_PRESS_SHORT 10
#define BUTTON_PRESS_LONG 1000

#define MCP4725A1_BUS_BASE_ADDR 0x62
#define MCP4725A1_VREF 3300UL

// Other constants
#define BUTTON_1_ADC 192
#define BUTTON_1_ADC_LOW BUTTON_1_ADC - BUTTON_ADC_MARGIN
#define BUTTON_1_ADC_HIGH BUTTON_1_ADC + BUTTON_ADC_MARGIN
#define BUTTON_2_ADC 379
#define BUTTON_2_ADC_LOW BUTTON_2_ADC - BUTTON_ADC_MARGIN
#define BUTTON_2_ADC_HIGH BUTTON_2_ADC + BUTTON_ADC_MARGIN
#define BUTTON_3_ADC 751
#define BUTTON_3_ADC_LOW BUTTON_3_ADC - BUTTON_ADC_MARGIN
#define BUTTON_3_ADC_HIGH BUTTON_3_ADC + BUTTON_ADC_MARGIN
#define BUTTON_4_ADC 1018
#define BUTTON_4_ADC_LOW BUTTON_4_ADC - BUTTON_ADC_MARGIN
#define BUTTON_4_ADC_HIGH BUTTON_4_ADC + BUTTON_ADC_MARGIN
#define BUTTON_5_ADC 1315
#define BUTTON_5_ADC_LOW BUTTON_5_ADC - BUTTON_ADC_MARGIN
#define BUTTON_5_ADC_HIGH BUTTON_5_ADC + BUTTON_ADC_MARGIN
#define BUTTON_ENC_ADC 2065
#define BUTTON_ENC_ADC_LOW BUTTON_ENC_ADC - BUTTON_ADC_MARGIN
#define BUTTON_ENC_ADC_HIGH BUTTON_ENC_ADC + BUTTON_ADC_MARGIN

#define _TASK_TIMECRITICAL
#define _TASK_PRIORITY
#include <TaskScheduler.h>

// Enumerations
enum class KeyerState {IDLE, DIT, DAH, DITIDLE, DAHIDLE, CHARSPACE, PLAYBACK, ANNUNCIATE, UART, TUNE};
enum class Button {NONE, S1, S2, S3, S4, S5, S_ENC, HOLD};
enum class Iambic {A, B};

// Class instantiation
Si5351 si5351;
RotaryEncoder *encoder = nullptr;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Scheduler runner;
Scheduler hprunner;

// Task function prototypes
void draw_oled();
void process_inputs();
void pa_enable();
void pa_disable();

// Tasks
Task task_draw_oled(100, TASK_FOREVER, &draw_oled, &runner, true);
Task task_process_inputs(TASK_IMMEDIATE, TASK_FOREVER, &process_inputs, &hprunner, true);
Task task_pa_enable(TASK_IMMEDIATE, TASK_ONCE, &pa_enable, &hprunner, false);
Task task_pa_disable(TASK_IMMEDIATE, TASK_ONCE, &pa_disable, &hprunner, false);

// Interrupt service routine variables
volatile unsigned long frequency = 14060000UL; // This will be the frequency it always starts on.
volatile unsigned long cw_tone = 600UL;  // CW listening frequency
volatile bool tx = false;
volatile bool led = false;
volatile bool paddle_tip_active = false;
volatile bool paddle_ring_active = false;

// Global variables
unsigned long iffreq = 6143900UL; // set the IF frequency in Hz.

const unsigned long freqstep[] = {50, 100, 500, 1000, 5000, 10000}; // set this to your wanted tuning rate in Hz.
int corr = 0; // this is the correction factor for the Si5351, use calibration sketch to find value.
unsigned int lastReportedPos = 1;   // change management
int inData;
int freqsteps = 1;
#define arraylength       (sizeof(freqstep) / sizeof(freqstep[0]))
uint32_t supply_voltage = 0;
uint16_t button_adc = 0;
uint32_t button_press_time;
Button last_button = Button::NONE;

// This interrupt routine will be called on any change of one of the input signals
void checkPosition() {
  encoder->tick(); // just call tick() to check the state.
}

void sprintf_seperated(char *str, unsigned long num) {
  // We will print out the frequency as a fixed length string and pad if less than 100s of MHz
  char temp_str[6];
  int zero_pad = 0;
  
  // MHz
  if(num / 1000000UL > 0)
  {
    sprintf(str, "%3lu", num / 1000000UL);
    zero_pad = 1;
  }
  else
  {
    strcat(str, "   ");
  }
  num %= 1000000UL;
  yield();
  
  // kHz
  if(zero_pad == 1)
  {
    sprintf(temp_str, ",%03lu", num / 1000UL);
    strcat(str, temp_str);
  }
  else if(num / 1000UL > 0)
  {
    sprintf(temp_str, ",%3lu", num / 1000UL);
    strcat(str, temp_str);
    zero_pad = 1;
  }
  else
  {
    strcat(str, "   ");
  }
  num %= 1000UL;
  yield();
  
  // Hz
  if(zero_pad == 1)
  {
    sprintf(temp_str, ".%03lu", num);
    strcat(str, temp_str);
  }
  else
  {
    sprintf(temp_str, ".%3lu", num);
    strcat(str, temp_str);
  }
}

void draw_oled(void) {
  char temp_str[21];
  u8g2.clearBuffer();

  // Draw frequency
  u8g2.setFont(u8g2_font_logisoso20_tn);
  sprintf_seperated(temp_str, frequency);
  yield();
  u8g2.drawStr(0, 32, temp_str);
  yield();

  // Draw frequency step
  // 10s digit x-value: 118
  switch(freqsteps) {
    case 0:  // 50 Hz
      u8g2.drawBox(115, 33, 11, 2);
      break;
    case 1:  // 100 Hz
      u8g2.drawBox(104, 33, 5, 2);
      break;
    case 2:  // 500 Hz
      u8g2.drawBox(101, 33, 11, 2);
      break;
    case 3:  // 1000 Hz
      u8g2.drawBox(82, 33, 5, 2);
      break;
    case 4:  // 5000 Hz
      u8g2.drawBox(79, 33, 11, 2);
      break;
    case 5:  // 10000 Hz
      u8g2.drawBox(68, 33, 5, 2);
      break;
  }
  // u8g2.drawBox(68, 33, 5, 2);
  yield();

  // Draw supply voltage
  u8g2.setFont(u8g2_font_unifont_tr);
  uint8_t volts = supply_voltage / 10000;
  uint8_t millivolts = (supply_voltage % 10000) / 1000;
  sprintf(temp_str, "%2d.%1dV", volts, millivolts);
  yield();
  u8g2.drawStr(0, 63, temp_str);
  yield();

  // sprintf(temp_str, "%d", last_button);
  // u8g2.drawStr(63, 63, temp_str);
  // yield();

  // Send the buffer to the screen
  // u8g2.sendBuffer();
  
  u8g2.updateDisplayArea(0, 0, 16, 2);
  // yield();
  u8g2.updateDisplayArea(0, 2, 16, 2);
  // yield();
  u8g2.updateDisplayArea(0, 4, 16, 2);
  // yield();
  u8g2.updateDisplayArea(0, 6, 16, 2);
}

void dah_change() {
  task_draw_oled.disable();
  task_process_inputs.disable();
  if(digitalRead(PADDLE_DAH_PIN) == LOW) {
    // key_down();
    task_pa_enable.set(TASK_IMMEDIATE, TASK_ONCE, &pa_enable);
    task_pa_enable.enable();
    digitalWrite(MUTE_PIN, HIGH);
    tone(SIDETONE_PIN, cw_tone);
    digitalWrite(LED_PIN, HIGH);
    // setPABias(1800);
    // si5351.output_enable(SI5351_CLK1, 1);
    yield();
    digitalWrite(TX_PIN, HIGH);
  }
  else {
    // key_up();
    task_pa_disable.set(TASK_IMMEDIATE, TASK_ONCE, &pa_disable);
    task_pa_disable.enable();
    digitalWrite(TX_PIN, LOW);
    // setPABias(0);
    // si5351.output_enable(SI5351_CLK1, 0);
    yield();
    noTone(SIDETONE_PIN);
    digitalWrite(MUTE_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
  }
  // task_process_inputs.enableDelayed();
  // task_draw_oled.enableDelayed();
}

void key_down() {
  // task_draw_oled.disable();
  // task_process_inputs.disable();
  // digitalWrite(MUTE_PIN, HIGH);
  // tone(SIDETONE_PIN, cw_tone);
  // digitalWrite(LED_PIN, HIGH);
  // setPABias(1800);
  // si5351.output_enable(SI5351_CLK1, 1);
  // digitalWrite(TX_PIN, HIGH);
  // task_process_inputs.enableIfNot();
  // task_draw_oled.enableIfNot();
}

void key_up() {
  // task_draw_oled.disable();
  // task_process_inputs.disable();
  // digitalWrite(TX_PIN, LOW);
  // setPABias(0);
  // si5351.output_enable(SI5351_CLK1, 0);
  // digitalWrite(LED_PIN, LOW);
  // noTone(SIDETONE_PIN);
  // digitalWrite(MUTE_PIN, LOW);
}

void pa_enable() {
  // setPABias(1800);
  si5351.output_enable(SI5351_CLK1, 1);
  task_process_inputs.enableDelayed();
  task_draw_oled.enableDelayed();
}

void pa_disable() {
  // setPABias(0);
  si5351.output_enable(SI5351_CLK1, 0);
  task_process_inputs.enableDelayed();
  task_draw_oled.enableDelayed();
}

Button process_button() {
  button_adc = analogRead(BUTTON_PIN);
  if (button_adc > BUTTON_1_ADC_LOW && button_adc < BUTTON_1_ADC_HIGH)
  {
    return Button::S1;
  }
  else if (button_adc > BUTTON_2_ADC_LOW && button_adc < BUTTON_2_ADC_HIGH)
  {
    return Button::S2;
  }
  else if (button_adc > BUTTON_3_ADC_LOW && button_adc < BUTTON_3_ADC_HIGH)
  {
    return Button::S3;
  }
  else if (button_adc > BUTTON_4_ADC_LOW && button_adc < BUTTON_4_ADC_HIGH)
  {
    return Button::S4;
  }
  else if (button_adc > BUTTON_5_ADC_LOW && button_adc < BUTTON_5_ADC_HIGH)
  {
    return Button::S5;
  }
  else if (button_adc > BUTTON_ENC_ADC_LOW && button_adc < BUTTON_ENC_ADC_HIGH)
  {
    return Button::S_ENC;
  }
  else
  {
    return Button::NONE;
  }
}

// Voltage specified in millivolts
void setPABias(uint16_t voltage) {
  uint32_t reg;
  uint8_t reg1, reg2;

  // Bounds checking
  if (voltage > MCP4725A1_VREF)
  {
    voltage = MCP4725A1_VREF;
  }

  // Convert millivolts to the correct register value
  reg = ((uint32_t)voltage * 4096UL) / MCP4725A1_VREF;
  reg1 = (uint8_t)((reg >> 8) & 0xFF);
  reg2 = (uint8_t)(reg & 0xFF);

  // Write the register to the MCP4725A1
  Wire.beginTransmission(MCP4725A1_BUS_BASE_ADDR);
  Wire.write(reg1);
  Wire.write(reg2);
  Wire.endTransmission();
}

// void led_toggle() {
//   led = !led;
//   digitalWrite(LED_PIN, led);
// }

void process_inputs() {
    // Process buttons
  Button cur_button = process_button();
  // yield();
  if (cur_button != Button::NONE) // Handle a button press
  {
    if (last_button == Button::NONE) // Short press
    {
      last_button = cur_button;
      button_press_time = millis();
    }
    else if (millis() > (button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
    {
      switch(cur_button)
      {
        case Button::S1:
          break;
        case Button::S2:
          break;
        case Button::S3:
          break;
        case Button::S4:
          break;
        case Button::S5:
          break;
        case Button::S_ENC:
          break;
      }
      // last_button = Button::NONE;
    }
  }
  else
  {
    if (last_button != Button::NONE)  // Check if this is a release
    {
      // digitalWrite(LED_PIN, HIGH);
      if ((millis() >= (button_press_time + BUTTON_PRESS_SHORT)) && (millis() < (button_press_time + BUTTON_PRESS_LONG))) // Short press
      {
        // digitalWrite(LED_PIN, HIGH);
        switch(last_button) {
          case Button::S1:
            break;
          case Button::S2:
            break;
          case Button::S3:
            break;
          case Button::S4:
            break;
          case Button::S5:
            break;
          case Button::S_ENC:
            freqsteps++;
            if (freqsteps > arraylength - 1 ) {
              freqsteps = 0;
            }
            break;
        }
      }
      // button_press_time = UINT32_MAX;
      last_button = Button::NONE;
    }
  }

  // Read paddles
  // if (digitalRead(PADDLE_DAH_PIN) == LOW && tx == false) {
  //   tx = true;
  //   key_down();
  // }
  // else if (digitalRead(PADDLE_DAH_PIN) == HIGH && tx == true) {
  //   tx = false;
  //   key_up();
  // }

  // Read supply voltage
  int supply_temp = analogRead(VSENSE_PIN);
  // 12-bit ADC, 3.3 V supply, each bit is 805.5 uV
  // Voltage divider is 22k/100k, so scale factor is 5.55
  // Scaling factors multiplied together are 0.0045
  supply_voltage = 45UL * supply_temp;

  // Process encoder
  static int pos = 0;
  encoder->tick(); // just call tick() to check the state.

  // int newPos = encoder->getPosition();
  // if (pos != newPos) {
  RotaryEncoder::Direction dir = encoder->getDirection();
  switch(dir) {
    case RotaryEncoder::Direction::CLOCKWISE:
      {
        if (!tx) {
          frequency += freqstep[freqsteps]; // here is the amount to increase the freq
        }
        // digitalWrite(LED_PIN, HIGH);
        // pos = newPos;
      }
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      {
        if (!tx) {
          frequency -= freqstep[freqsteps]; // here is the amount to decrease the freq
        }
        // digitalWrite(LED_PIN, LOW);
        // pos = newPos;
      } 
      break;
  } // switch
  
  if (lastReportedPos != frequency) {
    lastReportedPos = frequency;
    si5351.set_freq((frequency + iffreq - cw_tone) * 100ULL, SI5351_CLK0);
    si5351.set_freq(frequency * 100ULL, SI5351_CLK1);
  }
}

void setup() {

  // Wire.setClock(400000UL);

  // Set GPIO
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(PADDLE_DIT_PIN, INPUT_PULLUP);
  pinMode(PADDLE_DAH_PIN, INPUT_PULLUP);
  pinMode(TX_PIN, OUTPUT);
  pinMode(MUTE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SIDETONE_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(PADDLE_DAH_PIN), dah_change, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PADDLE_DAH_PIN), key_up, RISING);

  // Set up encoder
  encoder = new RotaryEncoder(ENC_A_PIN, ENC_B_PIN, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), checkPosition, CHANGE);

  // Set ADC resolution to 12 bits
  analogReadResolution(12);

  // Initialize the display
  u8g2.setBusClock(400000);
  u8g2.begin();

  // Initialize the Si5351A  
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, 0, 0);
  si5351.set_correction(corr, SI5351_PLL_INPUT_XO);
  si5351.set_freq((frequency + iffreq - cw_tone) * 100ULL, SI5351_CLK0);
  si5351.set_freq(frequency * 100ULL, SI5351_CLK1);
  si5351.set_freq(iffreq * 100ULL, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK1, 0);

  // Set up task scheulder
  runner.setHighPriorityScheduler(&hprunner);
  // task_draw_oled.enable();
  // task_process_inputs.enable();
  // runner.enableAll(true);

  // Wire.setClock(400000);
  setPABias(1800);
}

void loop()
{
  runner.execute();

  // if (Serial.available() > 0)   // see if incoming serial data:
  // {
  //   inData = Serial.read();  // read oldest byte in serial buffer:
  // }
  
  // if (inData == 'F')
  // {
  //   frequency = Serial.parseInt();
  //   inData = 0;
  // }

}




