#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPR121.h>
#include <Shifty.h>
#include <MIDI.h>
#include "midi_notes.h"
#include "ablogo_big.h"

#define DEBUG 1

#define INPUTS 12

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declaration for MPE121 touch board
Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

// Declaration for MIDI instance
MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);

//Declaration for HC595 shift register
Shifty shift; 

int song[INPUTS] = {};
int midiChannel[INPUTS] = {};

void drawbitmap(const unsigned char img[]);
void drawGrid();

void setup() {
  MIDI.begin();
  if (DEBUG) {
    Serial.begin(9600);
  }
  delay(2000);

  // Shifty library
  shift.setBitCount(16);
  shift.setPins(14, 10, 16); // data, clk, latch

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)  && DEBUG) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  } else if(DEBUG) {
    Serial.println(F("SSD1306 connected 0x3C"));
  }

  if (!cap.begin(0x5A)  && DEBUG) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  } else if(DEBUG) {
    Serial.println(F("MPR121 connected 0x5A"));
  } 

  drawbitmap(abLogo); 
  delay(2000);
  display.clearDisplay();
  display.display();

  drawGrid();

}

void loop() {

  // Read MPR121 output
  currtouched = cap.touched();
  for (uint8_t i = 0; i < INPUTS; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      shift.writeBit(i, HIGH);
      if (!DEBUG) {
        MIDI.sendNoteOn(song[i], 100, midiChannel[i]);
      } else {
        Serial.print("Pressed ");
        Serial.println (i);
      }
    }
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      shift.writeBit(i, LOW);
      if (!DEBUG) {
        MIDI.sendNoteOff(song[i], 100, midiChannel[i]);
      } else {
        Serial.print("Released ");
        Serial.println (i);
      }
    }
  }
  lasttouched = currtouched;
  delay(1);
}


void drawbitmap(const unsigned char img[]) {  // Display Logo centered on screen
  display.clearDisplay();
  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    img, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
}

void drawGrid() {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  uint8_t count = 1;  // calculate current cell
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      count = (j+1)+(i*4);
      display.drawRect(j*(display.width()/4), i*(display.height()/3), (display.width())/4, (display.height())/3, WHITE);
      display.setCursor(j*(display.width()/4) + 11 - ((count > 9) ? 6 : 0) , i*(display.height()/3) + 3);
      display.print(count);
    }
  }
  display.display();
}
