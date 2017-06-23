#include <avr/interrupt.h>
#include "IbusGlobals.h"
#include "IbusSerial.h"
#include "RingBuffer.h"


IbusSerial ibus; // Create an instance of the Ibus library called ibus.
#define debugSerial Serial
//#include <AltSoftSerial.h>
//AltSoftSerial debugSerial;
//#include <SoftwareSerial.h>
//SoftwareSerial debugSerial(7, 8); // 7 is Rx, 8 is Tx

// Setting up our variables
boolean goodPacket; //boolean value set true if good IBUS message detected
byte source;
byte length;
byte *pDebugByte;
byte databytes[36]; // Byte array to store message data bytes. ibusByte array of 40 - 4 for (Source, length, destination, checksumByte) = 36
byte outgoingMsg[32];
int outgoingSize;
unsigned long currentTime;

void setup()
{
  ibus.setIbusSerial(Serial2); // Tell library which serial port is used for IBUS. Must be HardwareSerial port.
  ibus.setScrollSpeed(1500, 1000, 200, 2); // Start delay, End delay, scroll speed & number of repeats. Defaults to 2000, 1000, 200, 2 if no alternatives specified
  ibus.setIbusPacketHandler(packetHandler); // Define callback function. This is the function that is called by the library when it has a complete IBUS packet/message.
  ibus.sleepEnable(60); // Enable sleep timer, and set sleep time in seconds. Comment out/delete line to disable timer.
  debugSerial.begin(9600); // Set debug Baud rate
  ibus.setIbusDebug(debugSerial); // Tell library which serial port is used for DEBUG. Comment out line if debug disabled in IbusGlobals.h
  debugSerial.println(F("--IBUS reader V2--"));
  attachInterrupt(3, startTimer, CHANGE);
  pinMode(ledPin, OUTPUT);
}
void loop()
{
  ibus.run(); // This keeps the IbusSerial library running.
}

// HID functions for media control
void launchGoogle() {
 Keyboard.press(MODIFIERKEY_GUI);
 Keyboard.release(MODIFIERKEY_GUI);
}

void sendPlayPause() {
  Keyboard.press(KEY_MEDIA_PLAY_PAUSE);
    Keyboard.release(KEY_MEDIA_PLAY_PAUSE);

}

void sendFastForward() {
 Keyboard.press(KEY_MEDIA_FAST_FORWARD);
  Keyboard.release(KEY_MEDIA_FAST_FORWARD);

}

void sendRewind() {
  Keyboard.press(KEY_MEDIA_REWIND);
    Keyboard.release(KEY_MEDIA_REWIND);

}

// This is where we compare the received message with all the message bytes
// in IbusGlobals.h, or to known source/destination, and respond accordingly.
// 2. Implement timer to remove written text from radio display after X seconds

void packetHandler(byte *packet) // callback function
{

  source = packet[0];
  length = packet[1];
  byte destination = packet[2];
  pDebugByte = packet;

  // Identify message databytes - populate our databytes array
  for (int i = 0, s = 3; i <= length - 3; i++, s++) {
    databytes[i] = packet[s];
  }
  // Message from MFL_RT
  if (memcmp_P(packet, MFL_RT_PRESS, 6) == 0 ) {
    sendPlayPause();
  }
  if (memcmp_P(packet, MFL_NEXT, 6) == 0 ) {
    sendFastForward();
  }
  if (memcmp_P(packet, MFL_PREV, 6) == 0 ) {
    sendRewind();
  }
  if (memcmp_P(packet, MFL_SEND_END_PRESS, 6) == 0 ) {
    launchGoogle();
  }
  if (memcmp_P(packet, MFL_SES_PRESS, 6) == 0 ) {
    ibus.radioWrite("SEW FAUNSEH");
  }
}

void startTimer()
{
  ibus.startTimer();
}

