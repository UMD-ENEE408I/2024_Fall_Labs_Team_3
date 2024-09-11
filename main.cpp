#include <Arduino.h>

const unsigned int BUZZ = 26; // Check schematic and see pin connection to buzzer
const unsigned int BUZZ_CHANNEL = 0; //Selecting PWM channel 0

const unsigned int octave = 5;

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  ledcAttachPin(BUZZ, BUZZ_CHANNEL);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_C, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_D, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_F, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_G, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_A, octave);
  delay(500);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_B, octave);
  delay(500);
  noTone(BUZZ_CHANNEL);
}

void loop() {
  //
}

