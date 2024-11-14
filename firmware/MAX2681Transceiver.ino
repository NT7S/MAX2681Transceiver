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
#include <Morse.h>

// Pin defines
#define TX_PIN 0
#define TX_PWR_FWD_PIN 1
#define TX_PWR_REV_PIN 2
#define LED_PIN 3
#define VSENSE_PIN 4
#define PADDLE_RING_PIN 6
#define PADDLE_TIP_PIN 7
#define ENC_A_PIN 11
#define ENC_B_PIN 10
#define MUTE_PIN 13
#define SIDETONE_PIN 14
#define BUTTON_PIN 15

// Other hardware defines
#define MCP4725A1_BUS_BASE_ADDR 0x62
#define MCP4725A1_VREF 3300UL

// Limits
#define KEYER_SPEED_LOWER 5
#define KEYER_SPEED_UPPER 40
#define BUTTON_ADC_MARGIN 40
#define BUTTON_PRESS_SHORT 10
#define BUTTON_PRESS_LONG 1000
#define SWR_BUFFER_SIZE 10

// Defaults
#define DEFAULT_KEYER_SPEED 16

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

// #define _TASK_MICRO_RES
#define _TASK_TIMECRITICAL
#define _TASK_PRIORITY
#include <TaskScheduler.h>

// Enumerations
enum class KeyerState {IDLE, DIT, DAH, DITIDLE, DAHIDLE, CHARSPACE, PLAYBACK, ANNUNCIATE, UART, TUNE};
enum class Button {NONE, S1, S2, S3, S4, S5, S_ENC, HOLD};
enum class KeyerMode {A, B, STRAIGHT};

// Class instantiation
Si5351 si5351;
RotaryEncoder *encoder = nullptr;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Scheduler runner;
Scheduler hprunner;
Morse morse(TX_PIN, DEFAULT_KEYER_SPEED);

// Task function prototypes
void draw_oled();
void process_inputs();
void process_keyer();
void key_down();
void key_up();
void keyer_ditdah_expire();
void keyer_charspace_expire();

// Tasks
Task task_draw_oled(100, TASK_FOREVER, &draw_oled, &runner, true);
Task task_process_inputs(TASK_IMMEDIATE, TASK_FOREVER, &process_inputs, &hprunner, true);
Task task_process_keyer(1, TASK_FOREVER, &process_keyer, &hprunner, true);
Task task_key_down(TASK_IMMEDIATE, TASK_ONCE, &key_down, &hprunner, false);
Task task_key_up(TASK_IMMEDIATE, TASK_ONCE, &key_up, &hprunner, false);
Task task_keyer_ditdah_expire(TASK_IMMEDIATE, TASK_ONCE, &keyer_ditdah_expire, &hprunner, false);
Task task_keyer_charspace_expire(TASK_IMMEDIATE, TASK_ONCE, &keyer_charspace_expire, &hprunner, false);

// Interrupt service routine variables
volatile unsigned long frequency = 14060000UL; // This will be the frequency it always starts on.
volatile unsigned long cw_tone = 600UL;  // CW listening frequency
volatile bool tx = false;
volatile bool break_in = true;
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
uint32_t tx_pwr_fwd = 0;
uint32_t tx_pwr_rev = 0;
uint16_t tx_pwr_fwd_buf[SWR_BUFFER_SIZE] = {};
uint16_t tx_pwr_rev_buf[SWR_BUFFER_SIZE] = {};
uint8_t tx_pwr_fwd_buf_head = 0;
uint8_t tx_pwr_rev_buf_head = 0;
KeyerState prev_keyer_state = KeyerState::IDLE;
KeyerState curr_keyer_state = KeyerState::IDLE;
KeyerState next_keyer_state = KeyerState::IDLE;
bool paddle_reverse = false;
KeyerMode keyer_mode = KeyerMode::A;
uint16_t dit_length;
uint32_t keyer_speed = DEFAULT_KEYER_SPEED;

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

  u8g2.setFont(u8g2_font_unifont_tr);
  sprintf(temp_str, "%u", next_keyer_state);
  u8g2.drawStr(0, 50, temp_str);

  if(tx && break_in) {
      // Draw SWR
    const uint16_t swr_calibration_corr = 250;
    u8g2.setFont(u8g2_font_unifont_tr);

    tx_pwr_fwd = 0;
    tx_pwr_rev = 0;
    for (uint8_t i; i < SWR_BUFFER_SIZE; i++) {
      tx_pwr_fwd += tx_pwr_fwd_buf[i];
      tx_pwr_rev += tx_pwr_rev_buf[i];
    }
    tx_pwr_fwd /= SWR_BUFFER_SIZE;
    tx_pwr_rev /= SWR_BUFFER_SIZE;

    if(tx_pwr_rev >= swr_calibration_corr) {
      tx_pwr_rev -= swr_calibration_corr; // Fudge factor for calibration
    }
    else {
      tx_pwr_rev = 0;
    }
    if(tx_pwr_fwd >= swr_calibration_corr) {
      tx_pwr_fwd -= swr_calibration_corr; // Fudge factor for calibration
    }
    else {
      tx_pwr_fwd = 0;
    }
    if (tx_pwr_rev > tx_pwr_fwd) {
      tx_pwr_rev = tx_pwr_fwd;
    }

    float gamma = ((float)tx_pwr_rev) / ((float)tx_pwr_fwd);
    float swr = (1.0 + gamma) / (1.0 - gamma);
    if(swr >= 10.0) {
      swr = 10.0;
      sprintf(temp_str, ">10");
    }
    else {
      uint8_t temp_swr = (uint8_t)(swr * 10) % 10;
      sprintf(temp_str, "%1d.%d", (uint8_t) swr, temp_swr);
    }
    u8g2.drawStr(0, 63, temp_str);
    
    // sprintf(temp_str, "%lu", dit_length);
    // u8g2.drawStr(0, 50, temp_str);

    // Draw SWR graph
    const uint8_t swr_graph_x_pos = 27;
    const uint8_t swr_graph_y_pos = 52;
    const uint8_t swr_graph_width = 100;
    const uint8_t swr_graph_height = 12;
    
    // TODO: should probably change to LUT for space savings
    float swr_graph = log10(swr);
    u8g2.setDrawColor(1);
    u8g2.drawFrame(swr_graph_x_pos, swr_graph_y_pos, swr_graph_width, swr_graph_height);
    uint8_t swr_graph_bar = (swr_graph_width - 4) * swr_graph;
    u8g2.drawBox(swr_graph_x_pos + 2, swr_graph_y_pos + 2, swr_graph_bar, swr_graph_height - 4);
  }
  else {
  // Draw supply voltage
    u8g2.setFont(u8g2_font_unifont_tr);
    uint8_t volts = supply_voltage / 10000;
    uint8_t millivolts = (supply_voltage % 10000) / 1000;
    sprintf(temp_str, "%2d.%1dV", volts, millivolts);
    yield();
    u8g2.drawStr(0, 63, temp_str);
    yield();
  }

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

void paddle_ring_change() {
  // task_draw_oled.disable();
  // task_process_inputs.disable();
  if(digitalRead(PADDLE_RING_PIN) == LOW) {
    paddle_ring_active = true;
    if(keyer_mode == KeyerMode::STRAIGHT) {
      task_key_down.set(TASK_IMMEDIATE, TASK_ONCE, &key_down);
      task_key_down.enable();
    }
  }
  else {
    paddle_ring_active = false;
    if(keyer_mode == KeyerMode::STRAIGHT) {
      task_key_up.set(TASK_IMMEDIATE, TASK_ONCE, &key_up);
      task_key_up.enable();
    }
  }
}

void paddle_tip_change() {
  if(digitalRead(PADDLE_TIP_PIN) == LOW) {
    paddle_tip_active = true;
    // if(keyer_mode == KeyerMode::STRAIGHT) {
    //   // task_key_down.set(TASK_IMMEDIATE, TASK_ONCE, &key_down);
    //   // task_key_down.enable();
    // }
  }
  else {
    // key_up();
    paddle_tip_active = false;
  //   if(keyer_mode == KeyerMode::STRAIGHT) {
  //     // task_key_up.set(TASK_IMMEDIATE, TASK_ONCE, &key_up);
  //     // task_key_up.enable();
  //   }
  }
}

void key_down() {
  task_draw_oled.disable();
  task_process_inputs.disable();
  tx = true;
  // setPABias(1800);
  digitalWrite(MUTE_PIN, HIGH);
  tone(SIDETONE_PIN, cw_tone);
  // digitalWrite(LED_PIN, HIGH);
  if(break_in) {
    digitalWrite(LED_PIN, HIGH);
    si5351.output_enable(SI5351_CLK1, 1);
    digitalWrite(TX_PIN, HIGH);
  }
  task_process_inputs.enableDelayed();
  task_draw_oled.enableDelayed();
}

void key_up() {
  task_draw_oled.disable();
  task_process_inputs.disable();
  digitalWrite(TX_PIN, LOW);
  // setPABias(0);
  si5351.output_enable(SI5351_CLK1, 0);
  noTone(SIDETONE_PIN);
  digitalWrite(MUTE_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  tx = false;
  memset(tx_pwr_fwd_buf, 0, SWR_BUFFER_SIZE);
  memset(tx_pwr_rev_buf, 0, SWR_BUFFER_SIZE);
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

void process_keyer() {
  // Process the keyer state machine
  switch (curr_keyer_state)
  {
    case KeyerState::IDLE:
      // Check paddle inputs
      if (paddle_tip_active && (keyer_mode != KeyerMode::STRAIGHT))
      {
        if (paddle_reverse)
        {
          // prev_keyer_state = KeyerState::DAH;
          curr_keyer_state = KeyerState::DAH;
          send_dah();
        }
        else
        {
          // prev_keyer_state = KeyerState::DIT;
          curr_keyer_state = KeyerState::DIT;
          send_dit();
        }
        prev_keyer_state = KeyerState::IDLE;
        next_keyer_state = KeyerState::IDLE;
      }
      else if (paddle_ring_active && (keyer_mode != KeyerMode::STRAIGHT))
      {
        if (paddle_reverse)
        {
          // prev_keyer_state = KeyerState::DIT;
          curr_keyer_state = KeyerState::DIT;
          send_dit();
        }
        else
        {
          // prev_keyer_state = KeyerState::DAH;
          curr_keyer_state = KeyerState::DAH;
          send_dah();
        }
        prev_keyer_state = KeyerState::IDLE;
        next_keyer_state = KeyerState::IDLE;
      }
      else
      {
        // keyline_off();
      }
      break;
    case KeyerState::DIT:  // Where the squeeze keying happens
      if (keyer_mode == KeyerMode::A)
      {
        if (paddle_reverse ? paddle_tip_active : paddle_ring_active && next_keyer_state == KeyerState::IDLE)
        {
          prev_keyer_state = KeyerState::DIT;
          next_keyer_state = KeyerState::DAH;
        }
        else if (!paddle_ring_active && !paddle_tip_active)
        {
          // next_keyer_state = KeyerState::IDLE;
        }
      }
      else if (keyer_mode == KeyerMode::B)
      {
        if (paddle_reverse ? paddle_tip_active : paddle_ring_active)
        {
          prev_keyer_state = KeyerState::DIT;
          next_keyer_state = KeyerState::DAH;
        }
        else if (!paddle_ring_active && !paddle_tip_active && next_keyer_state == KeyerState::DAH)
        {
          prev_keyer_state = KeyerState::IDLE;
          next_keyer_state = KeyerState::DAH;
        }
      }
        
      // }
      break;
    case KeyerState::DAH:  // Where the squeeze keying happens
      if (keyer_mode == KeyerMode::A)
      {
        if (paddle_reverse ? paddle_ring_active : paddle_tip_active && next_keyer_state == KeyerState::IDLE)
        {
          // prev_keyer_state = KeyerState::DAH;
          next_keyer_state = KeyerState::DIT;
        }
        else if (!paddle_ring_active && !paddle_tip_active)
        {
          // next_keyer_state = KeyerState::IDLE;
        }
      }
      else if (keyer_mode == KeyerMode::B)
      {
        if (paddle_reverse ? paddle_ring_active : paddle_tip_active)
        {
          prev_keyer_state = KeyerState::DAH;
          next_keyer_state = KeyerState::DIT;
        }
        else if (!paddle_ring_active && !paddle_tip_active && next_keyer_state == KeyerState::DIT)
        {
          prev_keyer_state = KeyerState::IDLE;
          next_keyer_state = KeyerState::DIT;
        }
      }
      // }
      break;
    case KeyerState::CHARSPACE:
      if (prev_keyer_state == KeyerState::DAH && next_keyer_state == KeyerState::IDLE) {
        if (paddle_reverse ? paddle_ring_active : paddle_tip_active) {
          next_keyer_state = KeyerState::DIT;
        }
        else if (paddle_reverse ? paddle_tip_active : paddle_ring_active) {
          next_keyer_state = KeyerState::DAH;
        }
        else if (paddle_tip_active && paddle_ring_active) {
          next_keyer_state = KeyerState::DIT;
        }
      }
      else if (prev_keyer_state == KeyerState::DIT && next_keyer_state == KeyerState::IDLE) {
        if (paddle_reverse ? paddle_tip_active : paddle_ring_active) {
          next_keyer_state = KeyerState::DAH;
        }
        else if (paddle_reverse ? paddle_ring_active : paddle_tip_active) {
          next_keyer_state = KeyerState::DIT;
        }
        else if (paddle_tip_active && paddle_ring_active) {
          next_keyer_state = KeyerState::DAH;
        }
      }
      // if (paddle_reverse ? paddle_ring_active : paddle_tip_active) {
      //   next_keyer_state = KeyerState::DIT;
      // }
      // else if (paddle_reverse ? paddle_tip_active : paddle_ring_active) {
      //   next_keyer_state = KeyerState::DAH;
      // }
      // break;
      // if (prev_keyer_state == KeyerState::DAH) {
      //   if (paddle_reverse ? paddle_ring_active : paddle_tip_active) {
      //     next_keyer_state = KeyerState::DIT;
      //   }
      // }
      // else if (prev_keyer_state == KeyerState::DIT) {
      //   if (paddle_reverse ? paddle_tip_active : paddle_ring_active) {
      //     next_keyer_state = KeyerState::DAH;
      //   }
      // }
    case KeyerState::PLAYBACK:
      // if (morse.busy == false)
      // {
      //   noTone();
      //   curr_keyer_state = KeyerState::IDLE;
      // }
      // if (sidetone_active)
      // {
      //   if (morse.tx)
      //   {
      //     tone(SIDETONE_OUTPUT, DEFAULT_SIDETONE_FREQ);
      //   }
      //   else
      //   {
      //     noTone();
      //   }
      // }
      break;
    case KeyerState::ANNUNCIATE:
      break;
    case KeyerState::TUNE:
      break;
    case KeyerState::UART:
      // TODO: UART idle timer to go back to sleep
      // if (Serial.available())
      // {
      //   char buf[35];
      //   char out[41];
      //   int size = Serial.readBytesUntil('\n', buf, 34);
      //   buf[size] = '\0'; // Null terminate so it's a string

      //   if (buf[1] == '?')  // Get parameter
      //   {
      //     switch (toupper(buf[0]))
      //     {
      //       case '1':
      //       case '2':
      //       case '3':
      //         EEPROM.get((buf[0] - 49) * EEP_M2_ADDR, out);
      //         Serial.println(out);
      //         break;
      //       case 'W':  // Get WPM
      //         Serial.println(keyer_speed);
      //         break;
      //     }
      //   }
      //   else if (buf[1] == ':')  // Set parameter
      //   {
      //     switch (toupper(buf[0]))
      //     {
      //       case 'X':  // Exit UART mode
      //         exit_uart();
      //         break;
      //       case 'R':
      //         paddle_reverse = 1;
      //         EEPROM.put(EEP_PADDLE_REV, 1);
      //         break;
      //       case 'N':
      //         paddle_reverse = 0;
      //         EEPROM.put(EEP_PADDLE_REV, 0);
      //         break;
      //       case 'A':
      //         keyer_mode = Iambic::A;
      //         EEPROM.put(EEP_KEYER_MODE, keyer_mode);
      //         break;
      //       case 'B':
      //         keyer_mode = Iambic::B;
      //         EEPROM.put(EEP_KEYER_MODE, keyer_mode);
      //         break;
      //       case 'S':
      //         // sidetone_active = true;
      //         // EEPROM.put(EEP_SIDETONE_ON, 1);
      //         set_sidetone(true);
      //         break;
      //       case 'O':
      //         // sidetone_active = false;
      //         // EEPROM.put(EEP_SIDETONE_ON, 0);
      //         set_sidetone(false);
      //         break;
      //       case '1':
      //       case '2':
      //       case '3':
      //         uint8_t addr = buf[0] - 49;
      //         // set_message((buf[0] - 49) * EEP_M2_ADDR, buf);
      //         memmove(buf, buf + 2, 35);
      //         strupr(buf);
      //         EEPROM.put(addr * EEP_M2_ADDR, buf);
      //         break;
      //     }
      //   }

      //   // Clear serial buffer
      //   memset(buf,0,strlen(buf));
      // }
      break;
  }
}

void keyer_ditdah_expire() {
  // Turn off display updates until keying is done
  // task_draw_oled.disable();
  // task_process_inputs.disable();

  // Key up
  task_key_up.set(TASK_IMMEDIATE, TASK_ONCE, &key_up);
  task_key_up.enable();
  // digitalWrite(LED_PIN, LOW);

  prev_keyer_state = curr_keyer_state;
  curr_keyer_state = KeyerState::CHARSPACE;
  task_keyer_charspace_expire.set(dit_length, TASK_ONCE, &keyer_charspace_expire);
  task_keyer_charspace_expire.enableDelayed();
}

void keyer_charspace_expire() {
  if (next_keyer_state == KeyerState::DIT)
  {
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DIT;
    next_keyer_state = KeyerState::IDLE;
    
    send_dit();
  }
  else if (next_keyer_state == KeyerState::DAH)
  {
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DAH;
    next_keyer_state = KeyerState::IDLE;

    send_dah();
  }
  else
  {
    curr_keyer_state = KeyerState::IDLE;
    next_keyer_state = KeyerState::IDLE;

    // Turn off display updates until keying is done
    // task_draw_oled.disable();
    // task_process_inputs.disable();

    // Key up
    task_key_up.set(TASK_IMMEDIATE, TASK_ONCE, &key_up);
    task_key_up.enable();
  }
}

void send_dit() {
  // Turn off display updates until keying is done
    // task_draw_oled.disable();
    // task_process_inputs.disable();

    // Key down
    task_key_down.set(TASK_IMMEDIATE, TASK_ONCE, &key_down);
    task_key_down.enable();
    // digitalWrite(LED_PIN, HIGH);

    // Delay for a dit length
    task_keyer_ditdah_expire.set(dit_length, TASK_ONCE, &keyer_ditdah_expire);
    task_keyer_ditdah_expire.enableDelayed();
}

void send_dah() {
  // Turn off display updates until keying is done
    // task_draw_oled.disable();
    // task_process_inputs.disable();

    // Key down
    task_key_down.set(TASK_IMMEDIATE, TASK_ONCE, &key_down);
    task_key_down.enable();
    // digitalWrite(LED_PIN, HIGH);

    // Delay for a dah length
    task_keyer_ditdah_expire.set(dit_length * 3, TASK_ONCE, &keyer_ditdah_expire);
    task_keyer_ditdah_expire.enableDelayed();
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
  // if (digitalRead(PADDLE_RING_PIN) == LOW && tx == false) {
  //   tx = true;
  //   key_down();
  // }
  // else if (digitalRead(PADDLE_RING_PIN) == HIGH && tx == true) {
  //   tx = false;
  //   key_up();
  // }

  // Read supply voltage
  int supply_temp = analogRead(VSENSE_PIN);
  // 12-bit ADC, 3.3 V supply, each bit is 805.5 uV
  // Voltage divider is 22k/100k, so scale factor is 5.55
  // Scaling factors multiplied together are 0.0045
  supply_voltage = 45UL * supply_temp;

  // Read SWR into ring buffer
  if(tx) {
    // tx_pwr_fwd = analogRead(TX_PWR_FWD_PIN);
    // tx_pwr_rev = analogRead(TX_PWR_REV_PIN);
    tx_pwr_fwd_buf[tx_pwr_fwd_buf_head] = analogRead(TX_PWR_FWD_PIN);
    tx_pwr_rev_buf[tx_pwr_rev_buf_head] = analogRead(TX_PWR_REV_PIN);
    if(++tx_pwr_fwd_buf_head > SWR_BUFFER_SIZE) {
      tx_pwr_fwd_buf_head = 0;
    }
    if(++tx_pwr_rev_buf_head > SWR_BUFFER_SIZE) {
      tx_pwr_rev_buf_head = 0;
    }
  }

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
  pinMode(PADDLE_TIP_PIN, INPUT_PULLUP);
  pinMode(PADDLE_RING_PIN, INPUT_PULLUP);
  pinMode(TX_PIN, OUTPUT);
  pinMode(MUTE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SIDETONE_PIN, OUTPUT);
  pinMode(TX_PWR_FWD_PIN, INPUT);
  pinMode(TX_PWR_REV_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(PADDLE_RING_PIN), paddle_ring_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PADDLE_TIP_PIN), paddle_tip_change, CHANGE);

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

  dit_length = (1200 / keyer_speed);
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




