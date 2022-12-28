/*
 * ProMicroMidiController.ino
 *
 * Author: Jason Mackay
 */
// MIDI over USB 
#include <MIDIUSB.h>  // https://github.com/arduino-libraries/MIDIUSB

// Display
#include <Wire.h>
#include <Adafruit_SH110X.h>

// Non-volatile storage
#include <EEPROM.h>
#include <avr/pgmspace.h>

// Display setuup 
#define i2c_Address 0x3c  // also try 0x3d, or use 
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// UI pages
#define NUM_UI_PAGES 5 
enum UIPage{
  VOLUME = 0,
  MIDI_VALUE = 1,
  MIDI_CHANNEL = 2,
  MIDI_CC_NUM = 3,
  SCREEN_SAVER = 4
};
UIPage current_page = UIPage::VOLUME; // UI page to show

// Non-volatile state
#define MAGIC_VALUE 0xA5
#define MAGIC_OFFSET 0
#define MIDI_CH_EEPROM_OFFSET MAGIC_OFFSET + sizeof(int)
#define CC_NUM_EEPROM_OFFSET MIDI_CH_EEPROM_OFFSET + sizeof(int)

// Busy LED
#define BUSY_LED_PIN A5

// Rotary Setup 
#define ROTARY_CLK A6
#define ROTARY_DT A7
#define ROTARY_SW A9
int rot_position = 0;
int rotation;
int value;
boolean left_right;
unsigned long last_rotary_event_ts = millis();
unsigned int beta = 1; // acceleration level

// Next thing to add is a motor driver. We want the device to
// respond to reciept of a cc on its channel
// https://www.engineersgarage.com/bidirectional-motor-with-arduino/

// Board Pinout Definitions
#define FADER_IN_PIN 8  // https://i.stack.imgur.com/SEiwb.png
#define MOTOR_R A10     // A PWM pin that will be used to control the motor

// Code constants
#define DEFAULT_MIDI_OUT_CH 1
#define VOLUME_CC_NUM 7

// Screen saver constants
unsigned long last_sc_render_time = 0;
#define SC_RENDER_PERIOD_MS 250
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000
};

/// HISTORY ////////////////////////////////////////////////////////////

// History buffer used for de-noising analog measurements
#define HISTORY_BUFF_SIZE 10
class HistoryBuffer {
public:
  HistoryBuffer();
  void insertValue(float value);
  bool ready();
  float mean();
  void display();
private:
  float _sum = 0;
  bool _ready = false;
  float _buffer[HISTORY_BUFF_SIZE];
  unsigned int _current_idx = HISTORY_BUFF_SIZE - 1;
};

HistoryBuffer::HistoryBuffer() {}

void HistoryBuffer::insertValue(float value) {
  _current_idx = (_current_idx + 1) % HISTORY_BUFF_SIZE;
  _sum -= _buffer[_current_idx];
  _buffer[_current_idx] = value;
  _sum += _buffer[_current_idx];

  // this flag tells us when the mean is valid
  if (!_ready && _current_idx == HISTORY_BUFF_SIZE - 1) {
    _ready = true;
  }
}

bool HistoryBuffer::ready() {
  return _ready;
}

float HistoryBuffer::mean() {
  return _sum / (float)HISTORY_BUFF_SIZE;
}

HistoryBuffer history;

byte LAST_VALUE_SENT = 0;
float target_value = 0;
bool updateDisplay = true;
int midi_ch = 1;
int cc_number = VOLUME_CC_NUM;
/// /HISTORY ////////////////////////////////////////////////////////////

//// LOGO    //////////////////////////////////////////////////////////// 
static const byte POLYGON[][2] PROGMEM = {{53, 33},
                {53, 35},
                {58, 42},
                {66, 41},
                {69, 40},
                {70, 35},
                {82, 42},
                {79, 42},
                {77, 34},
                {77, 5},
                {73, 3},
                {75, 3},
                {72, 4},
                {72, 33},
                {67, 40},
                {71, 37},
                {63, 41},
                {55, 41},
                {56, 40},
                {53, 36},
                {57, 40},
                {65, 42},
                {52, 40},
                {52, 59},
                {51, 59},
                {51, 60},
                {48, 60},
                {48, 59},
                {47, 59},
                {47, 3},
                {50, 2},
                {48, 2},
                {49, 2},
                {51, 4},
                {51, 3},
                {52, 5},
                {52, 33}};
                  
void drawLogo() {
  byte length = 37; //sizeof(POLYGON) / (2 * sizeof(byte));
  for (int i = 0; i < length; i++) {

    byte x1 = pgm_read_byte(&POLYGON[i][0]);
    byte y1 = pgm_read_byte_near(&POLYGON[i][1]);
    byte x2 = pgm_read_byte_near(&POLYGON[(i + 1) % length][0]);
    byte y2 = pgm_read_byte_near(&POLYGON[(i + 1) % length][1]);
    display.drawLine(
      x1,
      y1,
      x2,
      y2,
      SH110X_WHITE
    );
  }

  floodfill(48, 5);
  floodfill(76, 5);
}

void floodfill(byte x, byte y) {
  byte* point_stack = malloc(2 * 128);
  int stack_top = 0;

  // push
  point_stack[stack_top * 2] = x;
  point_stack[stack_top * 2 + 1] = y;

  while (stack_top >= 0) {
    // pop  
    x = point_stack[stack_top * 2];
    y = point_stack[stack_top * 2 + 1];
    stack_top--;

    display.drawPixel(x, y, SH110X_WHITE);
    display.display();

    // push siblings
    byte siblings[][2] = {
      {x-1, y},
      {x+1, y},
      {x, y + 1},
      {x, y - 1}      
    };

    for (int i = 0; i < 4; i++) {
      byte x2 = siblings[i][0];
      byte y2 = siblings[i][1];
      if (x2 >= 0 && x2 < 128 && y2 >= 0 && y2 < 64) {
        if (display.getPixel(x2, y2) == 0) {
          stack_top++;
          point_stack[stack_top * 2] = x2;
          point_stack[stack_top * 2 + 1] = y2;
        }
      }
    }

    delay(1);
  }

  free(point_stack);
}

//// LOGO    //////////////////////////////////////////////////////////// 

// Fetch avaiable RAM
int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval);  
}

// incoming cc handler
void controlChange(byte channel, byte control, byte value) {
  Serial.print(F("Control change: control="));
  
  Serial.print(control);
  Serial.print(F(", value="));
  Serial.print(value);
  Serial.print(F(", channel="));
  Serial.println(channel);
}

// Global flakes locations
uint8_t icons[NUMFLAKES][3];

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(freeRam());
  
  int magic = EEPROM.read(MAGIC_OFFSET);
  midi_ch = EEPROM.read(MIDI_CH_EEPROM_OFFSET);
  cc_number = EEPROM.read(CC_NUM_EEPROM_OFFSET);

  // In case EEPROM read is corrupt, default to Ch 1, CC 7
  if ((midi_ch < 1) || (midi_ch > 16) || (magic != MAGIC_VALUE)) 
    midi_ch = DEFAULT_MIDI_OUT_CH;  
  if ((cc_number < 0) || (cc_number > 127) || (magic != MAGIC_VALUE)) 
    cc_number = VOLUME_CC_NUM;  
  if (magic != MAGIC_VALUE) {
    magic = MAGIC_VALUE;
    EEPROM.write(MAGIC_OFFSET, magic);
    EEPROM.write(MIDI_CH_EEPROM_OFFSET, midi_ch);
    EEPROM.write(CC_NUM_EEPROM_OFFSET, cc_number);
  }

  // Setup rotary encoder
  pinMode(ROTARY_CLK, INPUT);
  pinMode(ROTARY_DT, INPUT);
  pinMode(ROTARY_SW, INPUT_PULLUP);
  rotation = digitalRead(ROTARY_CLK);

  // Setup display
  delay(250);  // wait for display to init
  display.begin(i2c_Address, true);

  display.clearDisplay();
  // before bytes = 26068
  // after bytes = 26756
  drawLogo();
  display.display();

  delay(3000);

  // Rx/Tx LED
  pinMode(BUSY_LED_PIN, OUTPUT);

  // Initialize screen saver
  for (uint8_t f = 0; f < NUMFLAKES; f++) {
    icons[f][XPOS] = random(display.width());
    icons[f][YPOS] = 0;
    icons[f][DELTAY] = random(5) + 1;
  }

  // // Display screen saver for a sec so serial bus can connect
  // for (int i = 0; i < 8; i++) {
  //   display.clearDisplay();
  //   render_screen_saver_page();
  //   display.display();
  //   delay(SC_RENDER_PERIOD_MS);
  // }

  // display.clearDisplay();

  Serial.println(F("Starting up"));
  Serial.flush();
}

unsigned long last_display_update_time = millis();
#define SLEEP_TIMEOUT_MS 5000
UIPage previous_page = current_page;
bool ss_triggered = false;

void loop() {
  loop_rx_cc();
  loop_tx_cc();
  loop_rotary();
  loop_move_fader();
 
  unsigned long now_ms = millis();
  if (updateDisplay) {
    updateDisplay = false;    
    if (ss_triggered) {
      current_page = previous_page;
      ss_triggered = false;
    }
    loop_render();
    last_display_update_time = now_ms;
  } else {
    if (current_page == UIPage::SCREEN_SAVER) {
      // Render screen saver periodically if its active
      unsigned long delta = now_ms - last_sc_render_time;
      if (delta > SC_RENDER_PERIOD_MS) {
        loop_render();

        last_sc_render_time = now_ms;
      }
    } else {
      if ((now_ms - last_display_update_time) > SLEEP_TIMEOUT_MS) {
        //Serial.println("Entering screen saver");
        //Serial.flush();
        ss_triggered = true;
        previous_page = current_page;
        current_page = UIPage::SCREEN_SAVER;
      }
    }     
  }

  Serial.flush();
  MidiUSB.flush();

  delayMicroseconds(500);
}

void loop_rx_cc() {
  midiEventPacket_t rx = MidiUSB.read();

  bool activity = false;
  switch (rx.header) {
    case 0:
      break;  //No pending events

    case 0xB:
      controlChange(
        rx.byte1 & 0xF,  //channel
        rx.byte2,        //control
        rx.byte3         //value
      );

      // TODO: We want to control the motor to seek this value
      // This code does not do that. Yet.
      target_value = midi_level_to_voltage(rx.byte3);
      activity = true;
      break;

    default:
      Serial.print(F("Unhandled MIDI message: "));
      Serial.print(rx.header, HEX);
      Serial.print(F("-"));
      Serial.print(rx.byte1, HEX);
      Serial.print(F("-"));
      Serial.print(rx.byte2, HEX);
      Serial.print(F("-"));
      Serial.println(rx.byte3, HEX);
      activity = true;
  }

  if (activity) flash();
}

void loop_tx_cc() {
    // Read fader value and emit cc
  float fader_voltage = analogRead(FADER_IN_PIN);
  float midi_fractional = voltage_to_midi_level(fader_voltage);
  // We stashed full res floats into the history buffer.
  // For all outputs now we will convert to integral
  byte midi_value = (byte)round(midi_fractional);
  history.insertValue(midi_fractional);

  if (history.ready()) {
    float value_delta = history.mean() - (float)LAST_VALUE_SENT;
    if (abs(value_delta) >= 1.0 && LAST_VALUE_SENT != midi_value) {
      // send out a cc
      midiEventPacket_t event = { 0x0B, 0xB0 | (midi_ch - 1), cc_number, midi_value };
      MidiUSB.sendMIDI(event);

      Serial.print(F("CC: "));  // Not seeing 127 easily enough, wondering if we can fix analog ref...
      Serial.println(midi_value);

      LAST_VALUE_SENT = midi_value;
      target_value = fader_voltage;  // manual send should override any prior received pos values

      updateDisplay = true;
      flash();
    } else {
      // // Do nothing
      // Serial.print("mean: ");
      // Serial.print(history.mean());
      // Serial.print(", reading: ");
      // Serial.println(midi_fractional);
    }
  }
}

void loop_move_fader() {
  // if (DRIVE_THAT_FADER) {
  //   // Driving the fader
  //   float fader_pos_delta = target_value - analogRead(FADER_IN_PIN);
  //   while (abs(fader_pos_delta) > (0.01 * 1023)) {
  //     Serial.print("delta: ");
  //     Serial.println(fader_pos_delta);

  //     if (fader_pos_delta > 0) {
  //       Serial.println("Higher"); // Just a prompt for human intervention :)
  //     } else {
  //       Serial.println("Lower"); // for now!
  //       // Make Robots do work here!
  //     }

  //     fader_pos_delta = target_value - analogRead(FADER_IN_PIN);
  //   }
  // }
}

void loop_rotary() {
  // Main loop - HANDLE ROTARY
  value = digitalRead(ROTARY_CLK);
  if (value != rotation) {
    if (value == 1) {
      if (digitalRead(ROTARY_DT) != value) {
        // Clockwise
        rot_position++;
        left_right = true;
        Serial.println(F("Rotary clockwise"));
      } else {
        // Counterclockwise
        rot_position--;
        left_right = false;
        Serial.println(F("Rotary counter-clockwise"));
      }

      int ACCELERATION_WINDOW_MS = 100;
      unsigned long now_ms = millis();
      unsigned long ms_since_last_rot_event = now_ms - last_rotary_event_ts;
      if (ms_since_last_rot_event < ACCELERATION_WINDOW_MS) {
        beta++;
        if (beta > 4) {
          beta = 4;
        }
      } else {
        beta = 1;
      }

      last_rotary_event_ts = now_ms;
      int delta = pow(2, beta) / 2;

      if (!ss_triggered) { // disable controls in ss
        switch (current_page) {
          case VOLUME:
            break;
          case MIDI_VALUE:
            break;
          case MIDI_CHANNEL:
            if (left_right) {
              midi_ch++;            
            } else {
              midi_ch--;
            }

            if (midi_ch < 1) midi_ch = 16;
            if (midi_ch > 16) midi_ch = 1;          

            EEPROM.write(MIDI_CH_EEPROM_OFFSET, midi_ch);
            updateDisplay = true; 
            break;
          case MIDI_CC_NUM:
            if (left_right) {
              cc_number += delta;  
            } else {
              cc_number -= delta;
            }

            if (cc_number < 0) cc_number += 128;
            cc_number = cc_number % 128; // wrap around

            EEPROM.write(CC_NUM_EEPROM_OFFSET, cc_number);
            updateDisplay = true;
            break;
          case SCREEN_SAVER:
            break;
          default:
            // This should not happen
            Serial.println(F("Invalid UI page"));
            break;
        }
      }

      updateDisplay = true;
    }
  }  

  rotation = value;

  if (!digitalRead(ROTARY_SW)) {
    Serial.println(F("Rotary push"));
    while (!digitalRead(ROTARY_SW)) {} // Wait for button release

    if (!ss_triggered) { // disable ui during sc
      current_page = (current_page + 1) % NUM_UI_PAGES;
    }

    updateDisplay = true;
  }
}

void loop_render() {
  display.clearDisplay();
  display.setCursor(0,0);

  switch (current_page) {
    case VOLUME:
      render_volume_page();
      break;
    case MIDI_VALUE:
      render_midi_value_page();
      break;
    case MIDI_CHANNEL:
      render_midi_channel_page();
      break;
    case MIDI_CC_NUM:
      render_midi_ch_num_page();
      break;
    case SCREEN_SAVER:
      render_screen_saver_page();
      break;
    default:
      // This should not happen
      Serial.println(F("Invalid UI page"));
      break;
  }

  display.display();
}

void render_volume_page() {
  char outputText[20];
  char valueStringBuffer[6];
  dtostrf(midiVolumeToDb(LAST_VALUE_SENT), 1, 1, valueStringBuffer);
  sprintf(outputText, "%s dB", valueStringBuffer);

  renderTitleText("Volume");
  renderCenterText(outputText);
}

void render_midi_value_page() {
  char outputText[20];
  sprintf(outputText, "%d", LAST_VALUE_SENT);

  renderTitleText("MIDI Value");
  renderCenterText(outputText);  
}

void render_midi_channel_page() {
  char outputText[20];
  sprintf(outputText, "%d", midi_ch);

  renderTitleText("MIDI Channel");
  renderCenterText(outputText);  
}

void render_midi_ch_num_page() {
  char outputText[20];
  sprintf(outputText, "%d", cc_number);

  renderTitleText("CC Number");
  renderCenterText(outputText);  
}

void render_screen_saver_page() {  
  // draw a bitmap icon and 'animate' movement
  testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
}

// TODO: fold this function into render_screen_saver_page
void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  // draw each icon
  for (uint8_t f = 0; f < NUMFLAKES; f++) {
    display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SH110X_WHITE);
  }
  display.display();

  // then erase it + move it
  for (uint8_t f = 0; f < NUMFLAKES; f++) {
    // jmackay - disabled to see if we can just skip it
    //display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SH110X_BLACK);
    
    // move it
    icons[f][YPOS] += icons[f][DELTAY];
    // if its gone, reinit
    if (icons[f][YPOS] /* +h */ > display.height()) {
      icons[f][XPOS] = random(display.width());
      icons[f][YPOS] = 0;
      icons[f][DELTAY] = random(5) + 1;
    }
  }
}

void renderCenterText(char* outputText) {
  int16_t x = 0;
  int16_t y = 0;
  uint16_t tw = 0;
  uint16_t th = 0;
  display.setTextSize(2);
  display.getTextBounds(outputText, 0, 0, &x, &y, &tw, &th);

  display.setCursor((128 - tw) / 2, 64 - (64 - th) / 2);
  display.setTextColor(SH110X_WHITE, SH110X_WHITE);  // fg == bg => clear background
  display.println(outputText);
}

void renderTitleText(char* titleText) {
  int16_t x = 0;
  int16_t y = 0;
  uint16_t tw = 0;
  uint16_t th = 0;
  display.setTextSize(1);
  display.getTextBounds(titleText, 0, 0, &x, &y, &tw, &th);

  display.setCursor((128 - tw) / 2, th);
  display.setTextColor(SH110X_WHITE, SH110X_WHITE);  // fg == bg => clear background
  display.println(titleText);
}

void flash() {
  digitalWrite(BUSY_LED_PIN, HIGH);
  digitalWrite(BUSY_LED_PIN, LOW);  
}

float voltage_to_midi_level(float voltage) {
  return (voltage / 1023.0) * 127.0;
}

float midi_level_to_voltage(byte midi_level) {
  return (midi_level / 127.0) * 1023.0;
}

float midiVolumeToDb(byte volume) {
  return 40.0 * log10((volume / 127.0)) + 6.0;
}