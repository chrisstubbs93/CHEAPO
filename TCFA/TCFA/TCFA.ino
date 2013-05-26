/*
 Interrupt Driven RTTY TX Demo
   
 Transmits data via RTTY with an interupt driven subroutine.
   
 By Anthony Stirk M0UPU  
 
 Edited by Chris Stubbs M6EDF (chris-stubbs.co.uk) for experimental purposes (temperature compensated frequency adjustment code)
 
 Temperature reference from http://www.airspayce.com/mikem/arduino/RF22/
   
 October 2012 Version 5
   
 Thanks and credits :
 Evolved from Rob Harrison's RTTY Code.
 Compare match register calculation by Phil Heron.
 Thanks to : http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b  
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
   
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
   
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
   
 See <http://www.gnu.org/licenses/>.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <RFM22.h>
 
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 100    // Baud rate for use with RFM22B Max = 600
//#define centerfreq 434.65025
 float target = 434.650;
 float calibratedial = 434.650;
 float centerfreq = (target + (target - calibratedial));
 
#define RFM22B_SDN 9
#define RFM22B_PIN 10
 
char datastring[80];
char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
unsigned int count=0;
int mid = 0x80;
 
 
rfm22 radio1(RFM22B_PIN);
 
void setup()
{
  initialise_interrupt();
  setupRadio();
  Serial.begin(9600);
}
 
void loop()
{
  radio1.write(0x0F, 0x00); // RF22_REG_0F_ADC_CONFIGURATION 0x0f  :  RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR 0x00 Temprtature sensor, oo, yes plz
  radio1.write(0x12, 0x00); // set temp range (-64 - +64 degC)
  radio1.write(0x12, 0x20); // set ENTSOFF (wtf is that?)
  radio1.write(0x0F, 0x80); // RF22_REG_0F_ADC_CONFIGURATION 0x0f  :  RF22_ADCSTART 0x80 Fuck knows what this does, maybe you call it start the ADC? After reading the manual you must set this self clearing bit to get the ADC to take a reading
  delayMicroseconds(360); //wait > 350 us for ADC converstion
  int xtemp = radio1.read(0x11); //Register 11h. ADC Value. What units this returns in I have no idea. Degrees bannana? - Oh its an ADC value, so probably 0-255
  //int xtal = radio1.read(0x09);
  int caltemp = xtemp * 0.5 - 64 - 3;
  
  int hzshift = (xtemp - 172) * 5;
  float mhzshift = hzshift * 0.00001;
  
  sprintf(datastring,"$$c,%i,%i", xtemp,hzshift); // Puts the text in the datastring
 // unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
 // char checksum_str[6];
  //sprintf(checksum_str, "*%04X", CHECKSUM);
 // strcat(datastring,checksum_str);
  strcat(datastring,"\n");
  //radio1.write(0x09, mid); //Chaning the crystal loading cap settings was a shitty idea
  //mid += 1;
  Serial.println(datastring);
  
  
  
  //centerfreq += 0.0001; // 150hz
  radio1.setFrequency(centerfreq + mhzshift);
  
  
  delay(1000);
  count++;
}
 
ISR(TIMER1_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) {  
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission.  
    strcpy(txstring,datastring);
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it.  
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else
    {
      txstatus=0; // Should be finished
      txj=0;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1);  
      else rtty_txbit(0);    
      txc = txc >> 1;
      break;
    }
    else
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    }  
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
 
  }
}
 
void rtty_txbit (int bit)
{
  if (bit)
  {
    radio1.write(0x73,mid + 3); // High 0x03
  }
  else
  {
    radio1.write(0x73,mid); // Low
  }
}
 
void setupRadio(){
  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO A3
  digitalWrite(RFM22B_SDN, LOW);
  delay(1000);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  radio1.setFrequency(centerfreq);
  radio1.write(0x6D, 0x04);// turn tx low power 11db
  radio1.write(0x07, 0x08);
  
  delay(500);
}
 
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}    
void initialise_interrupt()  
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
