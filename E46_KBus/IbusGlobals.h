/*
  Copyright (c) 2015 Ian R. Haynes.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef IbusGlobals_h
#define IbusGlobals_h
#include <Arduino.h>

// Comment line to disable debug messages
#define IBUS_DEBUG 

// These tell the library which interface you're using. Only one can be selected.
#define MELEXIS
//#define MCP

extern const byte senSta; // sen/Sta output from Melexix TH3122.
extern const byte enable; // HIGH enables IBUS trasceiver chip.
extern volatile boolean clearToSend;
extern const byte ledPin;
extern unsigned long ibusLoopTime;
extern unsigned long packetTimer;

// IBUS message definitions. Add/Remove as needed.
const byte KEY_IN [7] PROGMEM = { 
  0x44 , 0x05 , 0xBF , 0x74 , 0x04 , 0x00 , 0x8E }; // Ignition key in
const byte KEY_OUT [7] PROGMEM = { 
  0x44 , 0x05 , 0xBF , 0x74 , 0x00 , 0xFF , 0x75 }; // Ignition key out
const byte MFL_VOL_UP [6] PROGMEM = { 
  0x50 , 0x04 , 0x68 , 0x32, 0x11 , 0x1F }; // Steering wheel Volume Up
const byte MFL_VOL_DOWN [6] PROGMEM = { 
  0x50 , 0x04 , 0x68 , 0x32, 0x10 , 0x1E }; // Steering wheel Volume Down
  
 const byte MFL_NEXT [6] PROGMEM = { 
  0x50 , 0x04 , 0x68 , 0x3B, 0x01 , 0x06 }; // Steering wheel Next Track
const byte MFL_PREV [6] PROGMEM = { 
  0x50 , 0x04 , 0x68 , 0x3B, 0x08 , 0x0F }; // Steering wheel Previous Track
  
  
  
  
const byte MFL_TEL_VOL_UP [6] PROGMEM = { 
  0x50 , 0x04 , 0xC8 , 0x32, 0x11 , 0xBF }; // Steering wheel Volume Up - Telephone
const byte MFL_TEL_VOL_DOWN [6] PROGMEM = { 
  0x50 , 0x04 , 0xC8 , 0x32, 0x10 , 0xBE }; // Steering wheel Volume Down - Telephone
const byte MFL_SES_PRESS [6] PROGMEM = { 
  0x50 , 0x04 , 0xB0 , 0x3B, 0x80 , 0x5F }; // Steering wheel press and hold phone button
const byte MFL_SEND_END_PRESS [6] PROGMEM = { 
  0x50 , 0x04 , 0xC8 , 0x3B, 0x80 , 0x27 }; // Steering wheel send/end press
const byte MFL_RT_PRESS [6] PROGMEM = { 
  0x50 , 0x04 , 0x68 , 0x3B , 0x02, 0x05 }; // MFL R/T press
const byte CD_STOP [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x01 , 0x00 , 0x4C }; // CD Stop command
const byte CD_PLAY [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x03 , 0x00 , 0x4E }; // CD Play command
const byte CD_PAUSE [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x02 , 0x00 , 0x4F }; // CD Pause command
const byte CD_STOP_STATUS [12] PROGMEM = { 
  0x18 , 0x0A , 0x68 , 0x39 , 0x00 , 0x02 , 0x00 ,  0x3F , 0x00 , 0x07 , 0x01 , 0x78 }; // CD stop request
const byte CD_PLAY_STATUS [12] PROGMEM = { 
  0x18 , 0x0A , 0x68 , 0x39 , 0x02 , 0x09 , 0x00 ,  0x3F , 0x00 , 0x07 , 0x01 , 0x71 }; // CD play request
const byte INCOMING_CALL [6] PROGMEM = { 
  0xC8 , 0x04 , 0xE7 , 0x2C , 0x05 , 0x02 }; // Incoming phone call
const byte PHONE_ON [6] PROGMEM = { 
  0xC8 , 0x04 , 0xE7 , 0x2C , 0x10 , 0x17 }; // Phone On
const byte HANDSFREE_PHONE_ON [6] PROGMEM = { 
  0xC8 , 0x04 , 0xE7 , 0x2C , 0x11 , 0x16 }; // Hands Free Phone On
const byte ACTIVE_CALL [6] PROGMEM = { 
  0xC8 , 0x04 , 0xE7 , 0x2C , 0x33 , 0x34 }; // Active phone call
const byte IGNITION_OFF [6] PROGMEM = { 
  0x80 , 0x04 , 0xBF , 0x11 , 0x00 , 0x2A }; // Ignition Off
const byte IGNITION_POS1 [6] PROGMEM = { 
  0x80 , 0x04 , 0xBF , 0x11 , 0x01 , 0x2B }; // Ignition Acc position - POS1
const byte IGNITION_POS2 [6] PROGMEM = { 
  0x80 , 0x04 , 0xBF , 0x11 , 0x03 , 0x29 }; // Ignition On position - POS2
const byte REMOTE_UNLOCK [6] PROGMEM = { 
  0x00 , 0x04 , 0xBF , 0x72 , 0x22 , 0xEB }; // Remote control unlock
const byte DSP_STATUS_REQUEST [5] PROGMEM = { 
  0x68 , 0x03 , 0x6A , 0x01 , 0x00 }; // DSP status request
const byte DSP_STATUS_REPLY [6] PROGMEM = { 
  0x6A , 0x04 , 0xFF , 0x02 , 0x00 , 0x93 }; // DSP status reply
const byte DSP_STATUS_REPLY_RST [6] PROGMEM = { 
  0x6A , 0x04 , 0xFF , 0x02 , 0x01 , 0x92 }; // DSP status ready after reset to LOC
const byte DSP_VOL_UP_1 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x32, 0x11 , 0x25 }; // Rotary Volume Up 1 step
const byte DSP_VOL_UP_2 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x32, 0x21 , 0x15 }; // Rotary Volume Up 2 step
const byte DSP_VOL_UP_3 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x32, 0x31 , 0x05 }; // Rotary Volume Up 3 step
const byte DSP_VOL_DOWN_1 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x32, 0x10 , 0x24 }; // Rotary Volume Down 1 step
const byte DSP_VOL_DOWN_2 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x32, 0x20 , 0x14 }; // Rotary Volume Down 2 step
const byte DSP_VOL_DOWN_3 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x32, 0x30 , 0x04 }; // Rotary Volume Down 3 step
const byte DSP_FUNC_0 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x36, 0x30 , 0x00 }; // DSP_Function 0
const byte DSP_FUNC_1 [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x36, 0xE1 , 0xD1 }; // DSP_Function 1
const byte DSP_SRCE_OFF [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x36, 0xAF , 0x9F }; // DSP Source = OFF
const byte DSP_SRCE_CD [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x36, 0xA0 , 0x90 }; // DSP Source = CD
const byte DSP_SRCE_TUNER [6] PROGMEM = { 
  0x68 , 0x04 , 0x6A , 0x36, 0xA1 , 0x91 }; // DSP Source = Tuner
const byte GO_TO_RADIO [6] PROGMEM = { 
  0x68 , 0x04 , 0xFF , 0x3B, 0x00 , 0xA8 }; // Go  to radio - I think
const byte REQUEST_TIME [7] PROGMEM = { 
  0x68 , 0x05 , 0x80 , 0x41, 0x01 , 0x01 , 0xAC}; // Request current time from IKE
const byte CLOWN_FLASH [7] PROGMEM = { 
  0x3F , 0x05 , 0x00 , 0x0C , 0x4E , 0x01 , 0x79 }; // Turn on clown nose for 3 seconds
const byte BACK_ONE [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x08 , 0x00 , 0x45 }; // Back
const byte BACK_TWO [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x08 , 0x01 , 0x44 }; // Back
const byte LEFT [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x0A , 0x01 , 0x46 }; // Left
const byte RIGHT [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x0A , 0x00 , 0x47 }; // Right
const byte SELECT [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x07 , 0x01 , 0x4B }; // Select
const byte BUTTON_ONE [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x06 , 0x01 , 0x4A }; // Button 1
const byte BUTTON_TWO [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x06 , 0x02 , 0x49 }; // Button 2
const byte BUTTON_THREE [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x06 , 0x03 , 0x48 }; // Button 3
const byte BUTTON_FOUR [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x06 , 0x04 , 0x4F }; // Button 4
const byte BUTTON_FIVE [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x06 , 0x05 , 0x4E }; // Button 5
const byte BUTTON_SIX [7] PROGMEM = { 
  0x68 , 0x05 , 0x18 , 0x38 , 0x06 , 0x06 , 0x4D }; // Button 6
const byte CDC_STATUS_REPLY_RST [6] PROGMEM = { 
  0x18 , 0x04 , 0xFF , 0x02 , 0x01 , 0xE0 }; // CDC status ready after reset to LOC
const byte CDC_STATUS_REQUEST [5] PROGMEM = { 
  0x68 , 0x03 , 0x18 , 0x01 , 0x72 }; // CDC status request
const byte CDC_STATUS_REPLY [6] PROGMEM = { 
  0x18 , 0x04 , 0xFF , 0x02 , 0x00 , 0xE1 }; // CDC status reply
const byte CD_STATUS [16] PROGMEM = { 
  0x18 , 0x0E , 0x68 , 0x39 , 0x00 , 0x82 , 0x00 , 0x3F , 0x00 , 0x07 , 0x00 , 0x00 , 0x01 , 0x01 , 0x01 , 0xFC }; // CD status
const byte BUTTON_PRESSED [6] PROGMEM = {
  0x68 , 0x04 , 0xFF , 0x3B ,  0x00 , 0xA8 }; // Radio/Telephone control, No_buttons_pressed 
const byte VOL_INCREMENT [64] PROGMEM = { 
  0,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,
  128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,168,170,172,174,176,
  178,180,182,184,186,188,190,192}; // Volume increments
  
#endif
