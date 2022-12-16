/*
 * ProMicroMidiController.ino
 *
 * Author: Jason Mackay (well, cobblled together by...)
 */ 
// Main USB Midi functionality is coming frm this library
#include <MIDIUSB.h> // https://github.com/arduino-libraries/MIDIUSB

// Next thing to add is a motor driver. We want the device to
// respond to reciept of a cc on its channel 
// https://www.engineersgarage.com/bidirectional-motor-with-arduino/

// Board Pinout Definitions
#define FADER_IN_PIN 8 // https://i.stack.imgur.com/SEiwb.png
#define MOTOR_R A10 // A PWM pin that will be used to control the motor

// Code constants
#define MIDI_OUT_CH 1
#define VOLUME_CC_NUM 7
#define DRIVE_THAT_FADER false // enables motorized fader control

/// HISTORY ////////////////////////////////////////////////////////////

// History buffer used for de-noising analog measurements
#define HISTORY_BUFF_SIZE 10
class HistoryBuffer {
  public:
    HistoryBuffer();
    void insertValue(float value);
    float mean();
    void display();
  private:
    float _sum = 0;
    float _buffer[HISTORY_BUFF_SIZE];
};

HistoryBuffer::HistoryBuffer() {}

void HistoryBuffer::insertValue(float value) {
  float new_sum = _sum;
  for (int i = 1; i < HISTORY_BUFF_SIZE; i++) {

    if (i==1) {
      // special case for the first item only
      // in order to maintain our sum (and therefore our average) 
      // inexpensively, we can ust drop the record leaving the horizon 
      // and add the new record.
      new_sum -= _buffer[i-1];
    }

    _buffer[i-1] = _buffer[i];
  }  

  _buffer[HISTORY_BUFF_SIZE-1] = value;
  new_sum += value;

   _sum = new_sum;
}

float HistoryBuffer::mean() {
  return _sum / (float) HISTORY_BUFF_SIZE;
}

void HistoryBuffer::display() {
  Serial.print("History Buffer: ");

  for (int i = 0; i < HISTORY_BUFF_SIZE; i++) {
    Serial.print("[");
    Serial.print(_buffer[i]);
    Serial.print("]");
    if (i != HISTORY_BUFF_SIZE - 1) {
      Serial.print(", ");
    }
  }

  Serial.println("");
}

HistoryBuffer history;

byte LAST_VALUE_SENT = 0;
float target_value = 0;

/// /HISTORY ////////////////////////////////////////////////////////////

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

      // TODO: We want to control the motor to seek this value
      // This code does not do that. Yet.
      target_value = midi_level_to_voltage(rx.byte3);
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
  float midi_fractional = voltage_to_midi_level(fader_voltage);
  history.insertValue(midi_fractional);

  // We stashed full res floats into the history buffer.
  // For all outputs now we will convert to integral
  byte midi_value = (byte)round(midi_fractional);

  float value_delta = history.mean() - (float) LAST_VALUE_SENT;  
  // Serial.print("delta:");
  // Serial.println(delta);
  if (abs(value_delta) >= 1.0) {
    // send out a cc
    midiEventPacket_t event = {0x0B, 0xB0 | MIDI_OUT_CH, VOLUME_CC_NUM, midi_value};
    MidiUSB.sendMIDI(event);
    MidiUSB.flush();
    
    Serial.print("raw_voltage: ");
    Serial.println(fader_voltage);
    Serial.print("CC sent : "); // Not seeing 127 easily enough, wondering if we can fix analog ref...
    Serial.println(midi_value);

    LAST_VALUE_SENT = midi_value;
    target_value = fader_voltage; // manual send should override any prior received pos values
  } else {
    // // Do nothing
    // Serial.print("mean: ");
    // Serial.print(history.mean());
    // Serial.print(", reading: ");
    // Serial.println(midi_fractional);
  } 

  Serial.flush();

  if (DRIVE_THAT_FADER) {
    // Driving the fader
    float fader_pos_delta = target_value - analogRead(FADER_IN_PIN);
    while (abs(fader_pos_delta) > (0.01 * 1023)) {
      Serial.print("delta: ");
      Serial.println(fader_pos_delta);

      if (fader_pos_delta > 0) {
        Serial.println("Higher"); // Just a prompt for human intervention :)
      } else {
        Serial.println("Lower"); // for now!
        // Make Robots do work here!
      }

      fader_pos_delta = target_value - analogRead(FADER_IN_PIN);
    }
  }
}

float voltage_to_midi_level(float voltage) {
  return (voltage / 1023.0) * 127.0;
}

float midi_level_to_voltage(byte midi_level) {
  return (midi_level/127.0) * 1023.0;
}
