/*
 * MIDIUSB_buzzer.ino
 *
 * Author: Paulo Costa
 */ 

#include <MIDIUSB.h> // https://github.com/arduino-libraries/MIDIUSB
//#include "pitchToFrequency.h"

// Motor driver
// https://www.engineersgarage.com/bidirectional-motor-with-arduino/

#define FADER_IN_PIN 8 // https://i.stack.imgur.com/SEiwb.png
#define MIDI_OUT_CH 1
#define VOLUME_CC_NUM 7
#define MOTOR_R A10

// History buffer used for de-noising analog measurements
#define HISTORY_BUFF_SIZE 5
float HISTORY_BUFF [HISTORY_BUFF_SIZE];

const char* pitch_name(byte pitch) {
  static const char* names[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  return names[pitch % 12];
}

int pitch_octave(byte pitch) {
  return (pitch / 12) - 1;
}

void noteOn(byte channel, byte pitch, byte velocity) {
  Serial.print("Note On: ");
  Serial.print(pitch_name(pitch));
  Serial.print(pitch_octave(pitch));
  Serial.print(", channel=");
  Serial.print(channel);
  Serial.print(", velocity=");
  Serial.println(velocity);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  Serial.print("Note Off: ");
  Serial.print(pitch_name(pitch));
  Serial.print(pitch_octave(pitch));
  Serial.print(", channel=");
  Serial.print(channel);
  Serial.print(", velocity=");
  Serial.println(velocity);
}

void controlChange(byte channel, byte control, byte value) {
  Serial.print("Control change: control=");
  Serial.print(control);
  Serial.print(", value=");
  Serial.print(value);
  Serial.print(", channel=");
  Serial.println(channel);
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_R, OUTPUT);
}

void loop() {
  midiEventPacket_t rx = MidiUSB.read();
  switch (rx.header) {
    case 0:
      break; //No pending events
      
    case 0x9:
      noteOn(
        rx.byte1 & 0xF,  //channel
        rx.byte2,        //pitch
        rx.byte3         //velocity
      );
      break;
      
    case 0x8:
      noteOff(
        rx.byte1 & 0xF,  //channel
        rx.byte2,        //pitch
        rx.byte3         //velocity
      );
      break;
      
    case 0xB:
      controlChange(
        rx.byte1 & 0xF,  //channel
        rx.byte2,        //control
        rx.byte3         //value
      );
      int pwm_value = 255 * midi_level_to_voltage(rx.byte3)/1024.0;
      Serial.print("PWN SET: ");
      Serial.println(pwm_value);
 //     digitalWrite(MOTOR_R, true);
      analogWrite(MOTOR_R, 255);
      break;
      
    default:
      Serial.print("Unhandled MIDI message: ");
      Serial.print(rx.header, HEX);
      Serial.print("-");
      Serial.print(rx.byte1, HEX);
      Serial.print("-");
      Serial.print(rx.byte2, HEX);
      Serial.print("-");
      Serial.println(rx.byte3, HEX);
  }

  float fader_voltage = analogRead(FADER_IN_PIN);
  float midi_volume = voltage_to_midi_level(fader_voltage);
  
  float sum = 0;
  for (int i = 1; i < HISTORY_BUFF_SIZE; i++) {
    float value = HISTORY_BUFF[i];    
    HISTORY_BUFF[i-1] = value;
    sum += value;
  }

  HISTORY_BUFF[HISTORY_BUFF_SIZE - 1] = midi_volume;

  sum += midi_volume;
  float moving_avg = sum / HISTORY_BUFF_SIZE;

  if (abs(midi_volume - moving_avg) >= 1) {
    Serial.print("Sending cc: ");
    Serial.print(VOLUME_CC_NUM);
    Serial.print(" - ");
    Serial.println((byte)midi_volume);

    // Serial.print("last:");
    // Serial.println(LAST_FADER_READING_SENT);

    midiEventPacket_t event = {0x0B, 0xB0 | MIDI_OUT_CH, VOLUME_CC_NUM, (byte)midi_volume};
    MidiUSB.sendMIDI(event);

    MidiUSB.flush();
  } else {
    // Serial.println("Holding");
  }
}

float voltage_to_midi_level(float voltage) {
  return (voltage / 1024.0) * 127.0;
}

float midi_level_to_voltage(byte midi_level) {
  return (midi_level/127.0) * 1024;
}
