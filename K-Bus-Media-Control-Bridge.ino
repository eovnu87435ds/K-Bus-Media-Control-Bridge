#include "HID-Project.h"  //HID Project Library from NicoHood. Add via Library Manager
const int runbutton = 6;

void setup() {
  pinMode(runbutton, INPUT_PULLUP);
  while (!digitalRead(runbutton)) {
    //This keeps the rest of the program running. useful for programming the atmega32u4
  }
  Keyboard.begin();
}

void loop() {

}



// HID functions for media control
void launchGoogle(){
  Keyboard.write(KeyboardKeycode(KEY_LEFT_GUI));
}

void sendPlayPause(){
  Keyboard.write(KeyboardKeycode(MEDIA_PLAY_PAUSE));
}

void sendFastForward(){
  Keyboard.write(KeyboardKeycode(MEDIA_FAST_FORWARD));
}

void sendRewind(){
  Keyboard.write(KeyboardKeycode(MEDIA_REWIND));
}
