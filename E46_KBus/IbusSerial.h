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

#ifndef IbusSerial_h
#define IbusSerial_h

#include "Arduino.h"
#include "IbusGlobals.h"
#include "RingBuffer.h"

#define TIE 0x2
#define TEN 0x1
#if defined(__AVR_ATmega2560__)
#define Rx 15
#define Tx 14
#elif defined(__AVR_ATmega328P__) || defined(__MK20DX256__)
#define Rx 0
#define Tx 1
#endif

class IbusSerial
{

public:

	IbusSerial();
		~IbusSerial();
	void setIbusSerial(HardwareSerial &newIbusSerial);
#ifdef IBUS_DEBUG	
	void setIbusDebug(Stream &newIbusDebug);
#endif
	void run();
	void write(const byte message[], byte size);
	void printDebugMessage(const char * debugPrefix);
	typedef void IbusPacketHandler_t(byte *packet);
	void setIbusPacketHandler(IbusPacketHandler_t newHandler);
	void contentionTimer();
	void startTimer();
	void sleepEnable(unsigned long sleepTime);
	void sleep();
	byte transceiverType;
	boolean sleepEnabled;
	boolean debugEnabled;
	HardwareSerial *pSerial;
	void setScrollSpeed(int, int, int, byte);
	void radioWrite(const char *);
	void displayCurrentMessage();

	
private:

	enum ScrollStates 
	{
	SCROLL_IDLE,
	SCROLL_START,
	SCROLL_START_DELAY,
	SCROLL_SCROLLING,
	SCROLL_SCROLLING_DELAY,
	SCROLL_END,
	SCROLL_END_DELAY,
	SCROLL_END_DISPLAY
	} 
	scrollState;

	enum IbusFsmStates 
	{
	FIND_SOURCE,              // Read source byte
	FIND_LENGTH,              // Read length byte
	FIND_MESSAGE,             // Read in main body of message
	GOOD_CHECKSUM,            // Process message if checksum good
	BAD_CHECKSUM,             // Debug print bad message if checksum bad
	} 
	ibusState;

	void readIbus();
	void compareIbusPacket();
	void sendIbusPacket();
	void sendIbusMessageIfAvailable();
		
#ifdef IBUS_DEBUG
	Stream *pIbusDebug;
#endif
	byte source;
	byte length;
	byte destination;
	byte ibusByte[40];
	boolean busInSync;
	static const byte packetGap = 10;
	byte *pData;
	IbusPacketHandler_t *pIbusPacketHandler;
	
	void scroll();
	void buildStaticMessage(const char *);
	void buildScrollMessage(byte);
	void sendMessageIfAvailable();
	void writeMessage();
	unsigned int startDelay; // Delay before scrolling starts
	unsigned int endDelay; // Delay before scrolling re-starts
	unsigned int scrollSpeed; // Time in milliseconds between scroll steps
	unsigned int loops; // Number of scroll loops to display complete message
	unsigned long scrollTimer;
	byte maxChars; // Max number of characters to display on headunit before scrolling is required
	byte maxScrollRepeats; // No of scroll repeats
	byte scrollRepeatCounter; // Used to remember how many scroll repates we have done
	byte loopCounter; // Keep track of how many loops we have done
	byte startPos; // Index position into scrollStrng
	char scrollStrng[64]; // Used to hold messages to be scrolled on radio display
	static byte IBUS_RADIO_MESSAGE[32];
	

};
#endif