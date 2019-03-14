#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPR121.h>
#include <Shifty.h>
#include <MIDI.h>
#include <EEPROM.h>
#include "DualFunctionButton.h"
#include <avr/pgmspace.h>
#include "midi_notes.h"
#include "ablogo_big.h"

#define DEBUG 0

#define INPUTS 12

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declaration for MPE121 touch board
Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

// Declaration for MIDI instance
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

//Declaration for HC595 shift register
Shifty shift; 
const uint8_t ledClockPin = 10;  // HC595 pin 11
const uint8_t ledLatchPin = 11;  // HC595 pin 12
const uint8_t ledDataPin = 12;   // HC595 pin 14

// Rotary Encoder variables
const uint8_t encClk = 2; // Needs Interupt pin
const uint8_t encDt = 4;
const uint8_t encSw = 5;

volatile boolean up = false;
volatile boolean down = false;

// Declartaion for dualfunction button
DualFunctionButton button(encSw, 1000, INPUT_PULLUP);

// Menu
uint8_t page = 1;
uint8_t menuitem = 0;

// Note names matched to MIDI value
uint8_t midiAddress = INPUTS; //use address space after inputs
uint8_t volumeAddress = INPUTS*2; //use address space after inputs and channels

// buffer to read strings from PROGMEM
char buffer[12]; 

// temp value for EEPROM reading
uint8_t value;

uint8_t notes[INPUTS] = {60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71};
uint8_t midiChannels[INPUTS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t volumes[INPUTS] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

// Function declarations
void isr();
void drawbitmap(const unsigned char img[]);
void updateDisplay();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  MIDI.begin();
  Serial.begin(115200);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)  && DEBUG) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
     for(;;); // Don't proceed, loop forever
  } else if(DEBUG) {
    Serial.println(F("SSD1306 connected 0x3C"));
  }

  // Check MPR121 connected
  if (!cap.begin(0x5A)  && DEBUG) {
    Serial.println(F("MPR121 not found, check wiring?"));
    while (1);
  } else if(DEBUG) {
    Serial.println(F("MPR121 connected 0x5A"));
  } 

  // Shifty library
  shift.setBitCount(16);
  shift.setPins(ledDataPin, ledClockPin, ledLatchPin); // data, clk, latch
  if (DEBUG) {
    Serial.println("Testing LEDs");
    shift.writeBit(0, HIGH);
    delay(500);
    shift.writeBit(8, HIGH);
    delay(500);
    shift.writeBit(0,LOW);
    delay(500);
    shift.writeBit(8,LOW);
  }

  //read EEPROM values
  for (uint8_t i = 0; i < INPUTS; i++) {
    value = EEPROM.read(i);
    if (value > 11 && value < 128) {
      notes[i] = value;
    }
  }
  for (uint8_t i = 0; i < INPUTS; i++) {
    value = EEPROM.read(midiAddress + i);
    if (value >= 1 && value <=16) { //valid midiChannel
      midiChannels[i] = value;
    }
  }
  for (uint8_t i = 0; i < INPUTS; i++) {
    value = EEPROM.read(volumeAddress + i);
    if (value >= 0 && value <=127) { //valid volume
      volumes[i] = value;
    }
  }

  // Initiate Rotary encoder
  pinMode(encClk, INPUT);
  pinMode(encDt, INPUT);
  pinMode(encSw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encClk), isr, CHANGE);
  if(DEBUG) {
    Serial.println(F("Interrupt attached"));
  }

  drawbitmap(abLogo);
  if(DEBUG) {
    Serial.println(F("Bitmap Drawn"));
  } 
  delay(2000);
  if(DEBUG) {
    Serial.println(F("Updating display"));
  }
  updateDisplay();
  if(DEBUG) {
    Serial.println(F("Display updated"));
  }
}

void loop() {

  // Rotary encoder up
  if (up && page == 1) {
    up = false;
    if (menuitem < INPUTS-1) {
      menuitem++;
      updateDisplay();
    }
  } else if (up && page == 2) {
    up = false;
    if (notes[menuitem] < 127) {
      notes[menuitem]++;
      updateDisplay();
    }
  } else if (up && page == 3) {
    up = false;
    if (midiChannels[menuitem] < 16) {
      midiChannels[menuitem]++;
      updateDisplay();
    }
  } else if (up && page == 4) {
    up = false;
    if (volumes[menuitem] < 127) {
      volumes[menuitem]++;
      updateDisplay();
    }
  }

  // Rotary encoder down
  if (down && page == 1 && menuitem >= 0) {
    down = false;
    if (menuitem > 0) {
      menuitem = menuitem - 1;
      updateDisplay();
    }
  } else if (down && page == 2) {
    down = false;
    if (notes[menuitem] > 0) {
      notes[menuitem]--;
      updateDisplay();
    }
  } else if (down && page == 3) {
    down = false;
    if (midiChannels[menuitem] > 1) {
      midiChannels[menuitem]--;
      updateDisplay();
    }
  } else if (down && page == 4) {
    down = false;
    if (volumes[menuitem] > 1) {
      volumes[menuitem]--;
      updateDisplay();
    }
  }

//Rotary endcoder press
  if (button.shortPress()) {
    if (page == 1 ) {
      page=2;
    } else if (page ==2) {
      page = 1;
      //write value to EEPROM
      if (DEBUG) {
        Serial.print("Writing value ");
        Serial.print(notes[menuitem]);
        Serial.print(" to ");
        Serial.println(menuitem);
      }
      EEPROM.update(menuitem, notes[menuitem]);
    } else if (page ==3) {
      page = 2; //Return to Switch setting;
      //write value to EEPROM
      if (DEBUG) {
        Serial.print("Writing MIDI channel value ");
        Serial.print(midiChannels[menuitem]);
        Serial.print(" to ");
        Serial.println(menuitem);
      }
      EEPROM.update(midiAddress + menuitem, midiChannels[menuitem]);
    } else if (page ==4) {
      page = 2; //Return to Switch setting;
      //write value to EEPROM
      if (DEBUG) {
        Serial.print("Writing volume value ");
        Serial.print(volumes[menuitem]);
        Serial.print(" to ");
        Serial.println(menuitem);
      }
      EEPROM.update(volumeAddress + menuitem, volumes[menuitem]);
    } else if (page == 9) {
      page = 1;
    }
    updateDisplay();
  }
  if (button.longPress()) {
    if (DEBUG) {
      Serial.println("Long Press");
    }
    if (page == 1) {
      page = 9;
    } else if (page == 2) {
      page = 3;
    } else if (page == 3) {
      page = 4;
    }
    updateDisplay();
  }

  // Read MPR121 output
  currtouched = cap.touched();
  for (uint8_t i = 0; i < INPUTS; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      shift.writeBit(i, HIGH);
      if (!DEBUG) {
        MIDI.sendNoteOn(notes[i], 100, midiChannels[i]);
      } else {
        Serial.print("Pressed ");
        Serial.println (i);
      }
    }
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      shift.writeBit(i, LOW);
      if (!DEBUG) {
        MIDI.sendNoteOff(notes[i], 100, midiChannels[i]);
      } else {
        Serial.print("Released ");
        Serial.println (i);
      }
    }
  }
  lasttouched = currtouched;


  // DEBUG
  if (DEBUG) {
    if (up) {
      Serial.println("Rotary encoder: up");
      Serial.print("Menuitem: ");
      Serial.println(menuitem);
    }
    if (down) {
      Serial.println("Rotary encoder: down");
       Serial.print("Menuitem: ");
      Serial.println(menuitem);
    }
  }
  
  // up = false;
  // down = false;
  delay(1);

}

void isr() {
  static unsigned long lastInterupTime = 0;
  unsigned long interuptTime = millis();

  // Debounce signals to 5ms
  if (interuptTime - lastInterupTime > 5) {
    if (digitalRead(encDt) == digitalRead(encClk)) {
      down = true;
    } else {
      up = true;
    }
    lastInterupTime = interuptTime;
  }
}

void drawbitmap(const unsigned char img[]) {  // Display Logo centered on screen
  display.clearDisplay();
  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    img, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
}

void updateDisplay() {
  display.clearDisplay();
  if (page==1)  {
    display.setTextSize(2);
    uint8_t count = 1;  // calculate current cell
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        count = (j+1)+(i*4);
        if (count == menuitem+1) {
          display.fillRect(j*(display.width()/4), i*(display.height()/3), (display.width())/4, (display.height())/3, WHITE);
          display.setCursor(j*(display.width()/4) + 11 - ((count > 9) ? 6 : 0) , i*(display.height()/3) + 3);
          display.setTextColor(BLACK);
          display.print(count);
        } else {
          display.drawRect(j*(display.width()/4), i*(display.height()/3), (display.width())/4, (display.height())/3, WHITE);
          display.setCursor(j*(display.width()/4) + 11 - ((count > 9) ? 6 : 0) , i*(display.height()/3) + 3);
          display.setTextColor(WHITE);
          display.print(count);
        }
      }
    }
  }
  else if (page==2) {
    display.setCursor(19,3);
    display.setTextColor(WHITE);
    display.print("Switch:");
    display.print(menuitem+1);

    display.drawRect(0, 13, display.width(), display.height()-13, WHITE);
    uint8_t nameLength = strlen(strcpy_P(buffer, (char*)pgm_read_word(&(note_table[notes[menuitem]]))));
    display.setCursor((display.width()-(nameLength*12))/2,23);  // (display.width() - (size*12)) /2  , ((48-14) -16) /2) + 14
    display.setTextSize(2);
    display.print(strcpy_P(buffer, (char*)pgm_read_word(&(note_table[notes[menuitem]]))));
    display.setTextSize(1);
    display.setCursor(3,30);
    display.print("Ch");
    display.setCursor(3,38);
    if (midiChannels[menuitem] < 10) {
      display.print("0");
    }
    display.print(midiChannels[menuitem]);
    display.setCursor(64,30);
    display.print("Vol");
    display.setCursor(64,38);

    if (volumes[menuitem] < 100) {
      display.print(" ");
    }
    if (volumes[menuitem] < 10) {
      display.print(" ");
    }
    display.print(volumes[menuitem]);
  }
  else if (page==3) {
    display.setCursor(3,3);
    display.setTextColor(WHITE);
    // display.print("MIDI channel");
    display.print("MIDI ch Sw: ");
    display.print(menuitem+1);
    display.drawRect(0, 13, display.width(), display.height()-13, WHITE);
    display.setCursor(30,23);
    display.setTextSize(2);
    if (midiChannels[menuitem] < 10) {
      display.print("0");
    }
    display.print(midiChannels[menuitem]);
  }
  else if (page==4) {
    display.setCursor(6,3);
    display.setTextColor(WHITE);
    display.print("Volume Sw: ");
    display.print(menuitem+1);
    display.drawRect(0, 13, display.width(), display.height()-13, WHITE);
    display.setCursor(24,23);
    display.setTextSize(2);
    if (volumes[menuitem] < 100) {
      display.print("0");
    }
    if (volumes[menuitem] < 10) {
      display.print("0");
    }
    display.print(volumes[menuitem]);
  }
  else {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    for (uint8_t i = 0; i < INPUTS; i++) {
      display.setCursor(0,(i*8));
      display.print(i);
      display.setCursor(12,(i*8));

      display.print(strcpy_P(buffer, (char*)pgm_read_word(&(note_table[notes[i]]))));
      display.setCursor(42,(i*8));
      if (midiChannels[i] < 10) {
        display.print("0");
      }
      display.print(midiChannels[i]);
      display.setCursor(60,(i*8));
      if (volumes[i] < 100) {
        display.print(" ");
      }
      if (volumes[i] < 10) {
        display.print(" ");
      }
      display.print(volumes[i]);
    }
  }
  display.display();
}