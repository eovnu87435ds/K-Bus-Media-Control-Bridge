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

#include "IbusSerial.h"

#define len 1
const byte senSta = 3; // Pin 3 on Teensy 3.1 - sen/Sta output from Melexix TH3122.
const byte enable = 4; // HIGH enables Melexix TH3122 chip
volatile boolean clearToSend = false;
unsigned long ibusTimeToSleep;
unsigned long ibusSleepTime = millis();
unsigned long packetTimer = millis();
const byte ledPin = 13;
RingBuffer ibusReceiveBuffer(128);
RingBuffer ibusSendBuffer(64);
RingBuffer messageBuffer(64);
byte IbusSerial::IBUS_RADIO_MESSAGE[32] = { 0xC8 , 0x00 , 0x80 , 0x23 , 0x42 , 0x32 };


IbusSerial::IbusSerial() // constructor
{
	busInSync = false;
	ibusSleepTime = millis();
	packetTimer = millis();
	source = 0;
	length = 0;
	destination = 0;
	ibusState = FIND_SOURCE;
	scrollState = SCROLL_IDLE;
#if defined(MCP)
	pinMode(Rx, INPUT_PULLUP);
#endif
	pinMode (enable, OUTPUT);
    digitalWrite (enable, HIGH); // enable MCP/Melexis transceiver
	contentionTimer();
	sleepEnabled = false;
	debugEnabled = false;
	startDelay = 2000;		// set default value
	endDelay = 1000;		// set default value
	scrollSpeed = 200;		// set default value
	maxScrollRepeats = 2;	// set default value
	loops = 0;
	maxChars = 11;
	scrollRepeatCounter = 0;
	loopCounter = 0;
	startPos = 0;
	scrollTimer	= millis();
}

//deconstructor - not really used, as we never destroy this instance
IbusSerial::~IbusSerial() 
{

}

void IbusSerial::setIbusSerial(HardwareSerial &newIbusSerial) 
{
    pSerial = &newIbusSerial;
	pSerial->begin(9600, SERIAL_8E1); // ibus always 9600 8E1
}

#ifdef IBUS_DEBUG
void IbusSerial::setIbusDebug(Stream &newIbusDebug)
{
    pIbusDebug = &newIbusDebug;
}
#endif

void IbusSerial::setIbusPacketHandler(IbusPacketHandler_t newHandler)
{
    pIbusPacketHandler = newHandler;
}

void IbusSerial::readIbus()
{
  if (pSerial->available() ) 
  {
    ibusReceiveBuffer.write(pSerial->read() );
  }
  switch(ibusState) 
  {
  case FIND_SOURCE:
    if (ibusReceiveBuffer.available() >= 1) 
    {
      source = ibusReceiveBuffer.peek(0);
      ibusState = FIND_LENGTH;
    }
    break;

  case FIND_LENGTH: 
    if (ibusReceiveBuffer.available() >= 2) 
    {
      length = ibusReceiveBuffer.peek(1);
      if (length >= 0x03 && length <= 0x24) { // Check if length byte between decimal 3 & 36
        ibusState = FIND_MESSAGE;
      }
      else {
        // remove source byte and start over
        ibusReceiveBuffer.remove(1); 
      }
    }
    break;

  case FIND_MESSAGE: 
    if (ibusReceiveBuffer.available() >= length + 2) // Check if enough bytes in buffer to complete message (based on length byte)
    { 
      byte checksumByte = 0;  
      for (int i = 0; i <= length; i++)
      {
        checksumByte ^= ibusReceiveBuffer.peek(i);
      }
      if (ibusReceiveBuffer.peek(length + 1) == checksumByte)
      {
        ibusSleepTime = millis();  // Restart sleep timer
        ibusState = GOOD_CHECKSUM;
      }
      else 
      {
        ibusState = BAD_CHECKSUM;
      }
    }
    break;

  case GOOD_CHECKSUM:
    // only process messages we're interested in
    if(source != 0x50    //MFL
    && source != 0x68    //RAD
    && source != 0x18    //CDC
    && source != 0x6A    //DSP
    && source != 0xC8    //TEL
    && source != 0xB0    //SES
	&& source != 0xE8	 //RLS
    ) 
    {
      // remove unwanted message from buffer and start over
#ifdef IBUS_DEBUG
      pIbusDebug->print(F("DISCARDED: "));
	  for (int i = 0; i <= length + 1; i++)
	  {
		if( ibusReceiveBuffer.peek(i) < 0x10)
	  {
		pIbusDebug->print(F("0"));
	  }
		pIbusDebug->print(ibusReceiveBuffer.peek(i), HEX);
		pIbusDebug->print(F(" "));
	  }
		pIbusDebug->println();
#endif
      ibusReceiveBuffer.remove(length + 2);
      ibusState = FIND_SOURCE;
      return;
    }
    // read message from buffer
    for (int i = 0; i <= length + 1; i++)
    {
      ibusByte[i] = ibusReceiveBuffer.read();
    }
#ifdef IBUS_DEBUG
    // debug print good message
    printDebugMessage("Good Message -> ");
#endif
    busInSync = true;
    compareIbusPacket();
    ibusState = FIND_SOURCE;
    break;

  case BAD_CHECKSUM:
#ifdef IBUS_DEBUG
    // debug print bad message
	printDebugMessage("Message Bad -> ");
#endif
	// remove first byte and start over
    ibusReceiveBuffer.remove(1);
    ibusState = FIND_SOURCE;
    break;

  } // end of switch
} // end of readIbus

#ifdef IBUS_DEBUG
void IbusSerial::printDebugMessage(const char * debugPrefix)
{
	pIbusDebug->print(debugPrefix);
	for(int i = 0; i <= length + 1; i++)
    {
      if( ibusByte[i] < 0x10)
	  {
	  pIbusDebug->print(F("0"));
	  }
	  pIbusDebug->print(ibusByte[i], HEX);
      pIbusDebug->print(F(" "));
    }
    pIbusDebug->println();
}
#endif

void IbusSerial::compareIbusPacket()
{
	//pIbusDebug->print(F("IbusSerial::compareIbusPacket"));
	byte *pData = &ibusByte[0];
	pIbusPacketHandler((byte *) pData);
}

void IbusSerial::write(const byte message[], byte size)
{
	ibusSendBuffer.write(size);
	for(unsigned int i = 0; i < size; i++)
	{
    ibusSendBuffer.write(pgm_read_byte(&message[i]));
    }
}

void IbusSerial::sendIbusMessageIfAvailable()
{
  if(clearToSend && ibusSendBuffer.available() > 0 )
  {
    if (millis() - packetTimer >= packetGap)
    {
      sendIbusPacket();
      packetTimer = millis(); 
    }
  }
}

void IbusSerial::sendIbusPacket() {
  int Length = ibusSendBuffer.read();
#if defined(__MK20DX256__) && defined(MELEXIS)
  if(digitalReadFast(senSta) == LOW && Length <= 32)
#elif defined(__MK20DX256__) && defined(MCP)
  if(digitalReadFast(Rx) == HIGH && Length <= 32)
#elif (defined(__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)) && defined(MELEXIS)
  if(digitalRead(senSta) == LOW && Length <= 32)
#elif (defined(__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)) && defined(MCP)
  if(digitalRead(Rx) == HIGH && Length <= 32)
#endif
  {
    pIbusDebug->print(F("TRANSMIT: "));
	for ( int i = 0; i < Length; i++)
    {
      if( ibusSendBuffer.peek(0) < 0x10)
	  {
	  pIbusDebug->print(F("0"));
	  }
	  pIbusDebug->print(ibusSendBuffer.peek(0), HEX);
	  pIbusDebug->print(F(" "));
	  pSerial->write(ibusSendBuffer.read()); // write byte to IBUS.
    }
	pIbusDebug->println();
  }
  else 
  {
    ibusSendBuffer.remove(Length);
    return; 
  }
}

void IbusSerial::sleepEnable(unsigned long sleepTime)
{
	ibusTimeToSleep = sleepTime * 1000;
	sleepEnabled = true;
}

void IbusSerial::sleep()
{
	if(sleepEnabled == true)
	{
		if (millis() - ibusSleepTime >= ibusTimeToSleep) 
		{ 
	#if defined(MELEXIS)
		digitalWrite (enable, LOW); // Shutdown TH3122
	#elif defined(MCP)
		digitalWrite (enable, LOW); // Shutdown MCP
		pSerial->write (0x00); // write Tx pin low - only required on MCP2025.
	#endif
		}
	}
}

void IbusSerial::run()
{
	scroll();
	sendMessageIfAvailable();
	readIbus();
	sendIbusMessageIfAvailable();
	sleep();
}

// Tennsy 3.1 with Melexis TH3122
#if defined(__MK20DX256__)&& defined(MELEXIS)
void IbusSerial::contentionTimer()
{
  pinMode(senSta, INPUT); 
  SIM_SCGC6 |= SIM_SCGC6_PIT;    // Enables/disables clock used by PIT timers
  PIT_MCR = 0x00;                // Enables and disables the PIT timers. Writing zero enables the timers and writing 1 disables them.
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);  //
  PIT_LDVAL0 = 0x1193F;          // Set timer period for 1.5ms
  //pIbusDebug->println(F("Teensy/Melexis"));
}

void IbusSerial::startTimer()
{

  if (digitalReadFast(senSta) == LOW )
  {
	PIT_TCTRL0 = TIE;             // enable Timer 0 interrupts 
    PIT_TCTRL0 |= TEN;            // enable Timer 0
    PIT_TFLG0 |= 1;               // clear Timer Interrupt flag
  }

  else if (digitalReadFast(senSta) == HIGH )
  {
	clearToSend = false;
    PIT_TCTRL0 &= ~TEN;           // disable Timer 0
    PIT_TCTRL0 &= ~TIE;           // disable Timer 0 interrupts
    PIT_TFLG0 = 1;                // clear Timer Interrupt flag
  }  
}

void pit0_isr(void)
{
  clearToSend = true;
  PIT_TFLG0 = 1;                  // clear Timer Interrupt flag
  PIT_TCTRL0 &= ~TEN;             // disable Timer 0
  PIT_TCTRL0 &= ~TIE;             // disable Timer 0 interrupts 
}
#endif

// Tennsy 3.1 with Microchip MCP2003/2004/2025
#if defined(__MK20DX256__)&& defined(MCP)
void IbusSerial::contentionTimer()
{
  SIM_SCGC6 |= SIM_SCGC6_PIT;    // Enables/disables clock used by PIT timers
  PIT_MCR = 0x00;                // Enables and disables the PIT timers. Writing zero enables the timers and writing 1 disables them.
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);  //
  PIT_LDVAL0 = 0x1193F;          // Set timer period for 1.5ms
  //pIbusDebug->println(F("Teensy/MCP"));
}

void IbusSerial::startTimer()
{
   
  if (digitalReadFast(Rx) == HIGH && digitalReadFast(Tx) == HIGH) // check Rx high and Tx high
  { // start/enable timer
	PIT_TCTRL0 = TIE;             // enable Timer 0 interrupts 
    PIT_TCTRL0 |= TEN;            // enable Timer 0
    PIT_TFLG0 |= 1;               // clear Timer Interrupt flag
  }

  else if (digitalReadFast(Rx) == LOW && digitalReadFast(Tx) == HIGH) // check Rx high and Tx low
  { // stop/disable timer
	clearToSend = false;
    PIT_TCTRL0 &= ~TEN;           // disable Timer 0
    PIT_TCTRL0 &= ~TIE;           // disable Timer 0 interrupts
    PIT_TFLG0 = 1;                // clear Timer Interrupt flag
  }  
}

void pit0_isr(void)
{
  clearToSend = true;
  PIT_TFLG0 = 1;                  // clear Timer Interrupt flag
  PIT_TCTRL0 &= ~TEN;             // disable Timer 0
  PIT_TCTRL0 &= ~TIE;             // disable Timer 0 interrupts 
}
#endif


// Mega or Atmega328P based Arduino with Melexis TH3122.
#if (defined (__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)) && defined(MELEXIS)

void IbusSerial::contentionTimer() {
#define START_TIMER2 TCCR2B |= (1 << CS22)|(1 << CS21) // //Set CS22 and CS21 bits for 256 prescaler
#define STOP_TIMER2  TCCR2B &= 0B11111000

  // initialize Timer1
  TCCR2A = 0;     // set entire TCCR2A register to 0
  TCCR2B = 0;     // same for TCCR2B

  // set compare match register to desired timer count:
  OCR2A = 94; // Set timer to fire CTC interrupt after approx 1.5ms

  // turn on CTC mode:
  TCCR2B |= (1 << WGM21);

  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE2A);
  pinMode(senSta, INPUT); 
  //pIbusDebug->println(F("Arduino / Melexis"));

}

//========================

void IbusSerial::startTimer() {
  
  if (digitalRead(senSta) == LOW)
  {
    START_TIMER2;
  }
  else if (digitalRead(senSta) == HIGH )
  {
    clearToSend = false;
    digitalWrite(ledPin, HIGH);
	STOP_TIMER2;
  }
}

//========================


ISR(TIMER2_COMPA_vect) { // this function runs when the Timer2A compare expires

  clearToSend = true;
  digitalWrite(ledPin, LOW);
  STOP_TIMER2;
  
}
#endif

// Arduino Mega with Microchip MCP2003/2004/2025
#if defined(__AVR_ATmega2560__) && defined(MCP)
void IbusSerial::contentionTimer() {

#define START_TIMER2 TCCR2B |= (1 << CS22)|(1 << CS21) | (1 << CS20) // //Set CS22, CS21 & CS20 bits for 1024 prescaler
#define STOP_TIMER2  TCCR2B &= 0B11111000

  // initialize Timer1
  TCCR2A = 0;     // set entire TCCR2A register to 0
  TCCR2B = 0;     // same for TCCR2B

  // set compare match register to desired timer count:
  OCR2A = 157; // Set timer to fire CTC interrupt after approx 10ms

  // turn on CTC mode:
  TCCR2B |= (1 << WGM21);

  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE2A);
  
  // enable pin change interrupt on pin 15 (Rx3)
  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9); // Mega digital pin 15
  
  //pIbusDebug->println(F("Mega / MCP"));
}

//========================

ISR (PCINT1_vect)
//void IbusSerial::startTimer()
{
  if (digitalRead(Rx) == HIGH && digitalRead(Tx) == HIGH) // check Rx high and Tx high
  {
	START_TIMER2;
  }
  else if (digitalRead(Rx) == LOW && digitalRead(Tx) == HIGH) // check Rx high and Tx low
  {
	clearToSend = false;
    digitalWrite(ledPin, HIGH);
    STOP_TIMER2;
  }
}

//========================

ISR(TIMER2_COMPA_vect)
{
  clearToSend = true;
  digitalWrite(ledPin, LOW);
  STOP_TIMER2;
}
#endif

// Atmega328P based Arduino with Microchip MCP2003/2004/2025
#if defined(__AVR_ATmega328P__) && defined(MCP)
void IbusSerial::contentionTimer() {
#define START_TIMER2 TCCR2B |= (1 << CS22)|(1 << CS21) | (1 << CS20) // //Set CS22, CS21 & CS20 bits for 1024 prescaler
#define STOP_TIMER2  TCCR2B &= 0B11111000

  // initialize Timer1
  TCCR2A = 0;     // set entire TCCR2A register to 0
  TCCR2B = 0;     // same for TCCR2B

  // set compare match register to desired timer count:
  OCR2A = 157; // Set timer to fire CTC interrupt after approx 10ms

  // turn on CTC mode:
  TCCR2B |= (1 << WGM21);

  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE2A);
  
  // enable pin change interrupt on pin 0 (Rx)
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT16);
  
  //pIbusDebug->println(F("Uno, Nano or Pro Mini / MCP"));
}

//========================

ISR (PCINT2_vect)
{
  if (digitalRead(Rx) == HIGH && digitalRead(Tx) == HIGH) // check Rx high and Tx high
  {
    START_TIMER2;
  }
  else if (digitalRead(Rx) == LOW && digitalRead(Tx) == HIGH) // check Rx high and Tx low
  {
    clearToSend = false;
    digitalWrite(ledPin, HIGH);
    STOP_TIMER2;
  }
}

//========================

ISR(TIMER2_COMPA_vect)
{
  clearToSend = true;
  digitalWrite(ledPin, LOW);
  STOP_TIMER2;
}
#endif

void IbusSerial::setScrollSpeed(int start, int end, int speed, byte repeats)
{
	startDelay = start;
	endDelay = end;
	scrollSpeed = speed;
	maxScrollRepeats = repeats;
}

//scroll state machine
void IbusSerial::scroll() 
{
  switch(scrollState) 
  {
  case SCROLL_IDLE: //0
    scrollRepeatCounter = 0; 
    break;

  case SCROLL_START:  //1  
    startPos = 0; 
    loopCounter = 0; 
    buildScrollMessage(startPos); // Build message and send it
    scrollState = SCROLL_START_DELAY; 
    scrollTimer = millis();
    break;

  case SCROLL_START_DELAY: //2
    if(millis() - scrollTimer >= startDelay)
	{ 
      scrollState = SCROLL_SCROLLING; 
    }
    break;

  case SCROLL_SCROLLING: //3
    if(loopCounter < loops)
	{
      loopCounter++; 
      buildScrollMessage(startPos++); // Build next part of message and send it
      scrollState = SCROLL_SCROLLING_DELAY;
      scrollTimer = millis();
	}
    else
	{
      scrollState = SCROLL_END;
      startPos = 0;
      loopCounter = 0;
      scrollRepeatCounter++;
    }
    break;

  case SCROLL_SCROLLING_DELAY: //4
    if(millis() - scrollTimer >= scrollSpeed)
	{
      scrollState = SCROLL_SCROLLING;
    }
    break;

  case SCROLL_END: //5
    if(scrollRepeatCounter < maxScrollRepeats)
	{ // Repeat for maxScrollRepeats
      scrollState = SCROLL_END_DELAY;
      scrollTimer = millis();
    }
    else
	{
      scrollState = SCROLL_END_DISPLAY;
	  scrollTimer = millis();
    }
    break;

  case SCROLL_END_DELAY: //6 
    if(millis() - scrollTimer >= endDelay)
	{
      scrollState = SCROLL_START;
	}
    break;
	
  case SCROLL_END_DISPLAY: //7
    if(millis() - scrollTimer >= endDelay)
	{
	startPos = 0;
	buildScrollMessage(startPos);
	scrollState = SCROLL_IDLE;
    }
    break;
  }
 }

//receives message and decides if it needs scrolling or not
void IbusSerial::radioWrite(const char * instrng) 
{
  scrollState = SCROLL_IDLE;
  byte messageLen = strlen(instrng);
  if(messageLen <= maxChars) { //static message
    buildStaticMessage(instrng);
  }
  else { //scroll message
    sprintf(scrollStrng, instrng);
    scrollRepeatCounter = 0;
    scrollState = SCROLL_START;
  }
}

//builds static (non scrolling) messages
void IbusSerial::buildStaticMessage(const char * instrng) 
{
  byte messageLen = strlen(instrng);
  memcpy(IBUS_RADIO_MESSAGE + 6, instrng, messageLen); //ibusSendBuffer.write(pgm_read_byte(&message[i]));
  IBUS_RADIO_MESSAGE[len] = messageLen + 5;
  int checksumPos = IBUS_RADIO_MESSAGE[1] + 1;
  int checksumByte = 0;  
  for (unsigned int i = 0; i <= IBUS_RADIO_MESSAGE[1]; i++){
    checksumByte ^= IBUS_RADIO_MESSAGE[i];
  }
  IBUS_RADIO_MESSAGE[checksumPos] = checksumByte;
  displayCurrentMessage();
}

//builds scrolling messages - driven by scroll state machine
void IbusSerial::buildScrollMessage(byte startPosn) 
{
  byte messageLen = strlen(scrollStrng);
  loops = messageLen - (maxChars - 1);
  memcpy(IBUS_RADIO_MESSAGE + 6, scrollStrng + startPosn, maxChars);
  if(IBUS_RADIO_MESSAGE[6] == 0x20) IBUS_RADIO_MESSAGE[6] = 0x86; 
  IBUS_RADIO_MESSAGE[len] = 5 + maxChars;
  int checksumPos = IBUS_RADIO_MESSAGE[1] + 1;
  int checksumByte = 0;  
  for (unsigned int cs = 0; cs <= IBUS_RADIO_MESSAGE[1]; cs++){
    checksumByte ^= IBUS_RADIO_MESSAGE[cs];
  }
  IBUS_RADIO_MESSAGE[checksumPos] = checksumByte;
  displayCurrentMessage();
}

void IbusSerial::displayCurrentMessage()
{
  messageBuffer.write(IBUS_RADIO_MESSAGE[1] + 2);
  for(unsigned int cs = 0; cs < IBUS_RADIO_MESSAGE[1] + 2; cs++)
  {
    messageBuffer.write(IBUS_RADIO_MESSAGE[cs]);
  }
}

void IbusSerial::writeMessage()
{
  unsigned int Length = messageBuffer.read();
#if defined(__MK20DX256__) && defined(MELEXIS)
  if(digitalReadFast(senSta) == LOW && Length <= 18)
#elif defined(__MK20DX256__) && defined(MCP)
  if(digitalReadFast(Rx) == HIGH && Length <= 18)
#elif (defined(__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)) && defined(MELEXIS)
  if(digitalRead(senSta) == LOW && Length <= 18)
#elif (defined(__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)) && defined(MCP)
  if(digitalRead(Rx) == HIGH && Length <= 18)
#endif  
  {
    for (unsigned int i = 0; i < Length; i++)
	{
      pSerial->write(messageBuffer.read()); // write byte to IBUS.
    }
  }
  else
  {
    messageBuffer.remove(Length);
    return; 
  }
}

//send static or scroll messages if available in buffers
void IbusSerial::sendMessageIfAvailable()
{
  if(clearToSend && messageBuffer.available() > 0 )
  {
    writeMessage();
  }
}