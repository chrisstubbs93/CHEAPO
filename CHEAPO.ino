#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <RFM22.h>
#include <TinyGPS.h>

TinyGPS gps;

#define ASCII 7          // 7 bit ascii
#define STOPBITS 2       // 2 stop bits
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50    // Baud rate for use with RFM22B Max = 600
#define RADIO_FREQUENCY 434.300 // Low side frequency for transmission
#define RADIO_POWER  0x04 //Radio power (12mW)
#define RESTART_INTERVAL  50 //Restart rfm22b every x lines
#define RFM22B_SDN 9 //RFM Power
#define RFM22B_PIN 10 //RFM SPI pin

/* Radio output power settings:
 0x02 5db (3mW)
 0x03 8db (6mW)
 0x04 11db (12mW)
 0x05 14db (25mW)
 0x06 17db (50mW)
 0x07 20db (100mW)
 */

char datastring[80]; //where the telementry string is stored
char txstring[80]; //copy of telementry string for transmission
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
unsigned int count=0; //message counter
boolean setgpsmode; //has GPS been set to flight mode
boolean gps_set_sucess; //is flight mode set (bool)
boolean gps_powersave; //is powersave mode set (bool)
unsigned int alt; 
int sats = 0;
int psave;
char latstr[10] = "0"; //Lat converted to string
char lonstr[10] = "0"; //Lng converted to string
unsigned long time;
boolean radioready; //is the radio ready or down for reboot
int gpsmode; //is flight mode set? (int 1/0)
float vbatt = 0.00; //battery voltage as float
char vbatts[10] = "0"; //battery voltage as string
int xtemp = 0;
int caltemp = 0;

/*
double hlat = 51.710119;
double hlon = 0.577744;
float dist;
char diststr[5] = "0"; //dist converted to string
int R = 6371;
*/

rfm22 radio1(RFM22B_PIN);

void setup()
{
  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO D9
  delay(5000);// let the GPS settle down (probably not needed)
  Serial.begin(9600);
  initialise_interrupt();
  setupRadio();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  //sats = gps.satellites(); //reporting crazy values before fix, tinygps does not report sats anyway without 2d/3d fix

  // For 2 seconds we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 2000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if(setgpsmode == true)
  {
    navmode();
  }

  if (newData)
  {

    // if we have a GPS fix. then set the ublox into flight mode. Flight mode code by Upu
    delay(250);
    setgpsmode = true;

    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);

    dtostrf(flat,9,6,latstr); // convert lat from float to string
    dtostrf(flon,9,6,lonstr); // convert lon from float to string
    alt = gps.f_altitude(); // +/- altitude in meters 
    sats = gps.satellites();

    /*
    //Haversine distance calculation. Isnt very good on the arduino due to the float precision
     float dLat = ((flat - hlat) * 71) / 4068;
     float dLon = ((flon - hlon) * 71) / 4068;
     float lat1 = ((hlat) * 71) / 4068;
     float lat2 = ((hlon) * 71) / 4068;
     float a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
     float c = 2 * atan2(sqrt(a),sqrt(1-a));
     dist = R * c;
     dtostrf(dist,4,2,diststr); // convert dist from float to string
     */

    unsigned long fix_age;

    //If we have a good fix (>5 sats), enter uBlox Power save mode
    if(sats > 5)
    {
      if(gps_powersave == false)
      {
        uint8_t setPSM[] = { 
          0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92         }; // Setup for Power Save Mode (Default Cyclic 1s)
        sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t)); // send command to ublox
        gps_powersave = true;
      }
    }


    // time in hhmmsscc, date in ddmmyy
    gps.get_datetime(0, &time, &fix_age);

    //drop cc (millis) from time to leave us with HHMMSS
    time = time / 100;

    //correct no of 0's in long, (output: 00.575950/-0.575950)
    if (lonstr[0] == ' ') {
      lonstr[0] = '0';
    }
  }
  else {
    // No new GPS data, probably no satelites or comms failure. Set the sats flag to 0 to tell habitat its an old fix
    sats = 0;
  }


  //I didnt figure out how to sprintf a boolean, so use an integer instead for the IsFlightModeSet flag.
  if(setgpsmode){
    gpsmode = 1;
  } 
  else{
    gpsmode = 0;
  }

  //is gps in power save mode?
  if(gps_powersave){
    psave = 1;
  } 
  else{
    psave = 0;
  }

  //Im not sure if this delay is needed. But leaving it in for safekeeping.
  delay(1000);
}

//Timer that fires to send each RTTY bit. From Upu
ISR(TIMER1_COMPA_vect)
{
  if (radioready == true) //is the radio up and running? if not its down for reboot
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
      vbatt = ((3.2 / 1024)* analogRead(A0) * 11.2); // ((vcc / maxADC)* adcreading * voltageDividerRatio)
      dtostrf(vbatt,3,2,vbatts); // convert lat from float to string

      radio1.write(0x0F, 0x00); // RF22_REG_0F_ADC_CONFIGURATION 0x0f  :  RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR 0x00 Temprtature sensor, oo, yes plz
      radio1.write(0x12, 0x00); // set temp range (-64 - +64 degC)
      radio1.write(0x12, 0x20); // set ENTSOFF (wtf is that?)
      radio1.write(0x0F, 0x80); // RF22_REG_0F_ADC_CONFIGURATION 0x0f  :  RF22_ADCSTART 0x80  After reading the manual you must set this self clearing bit to get the ADC to take a reading
      delayMicroseconds(400); //wait > 350 us for ADC converstion
      xtemp = radio1.read(0x11); //Register 11h. ADC Value. What units this returns in I have no idea. Degrees bannana? - Oh its an ADC value, so probably 0-255
      caltemp = xtemp * 0.5 - 64 - 7;


      sprintf(datastring,"$$$$CHEAPO,%i,%06lu,%s,%s,%i,%i,%i%i,%s,%i",count,time,latstr,lonstr,alt,sats,gpsmode,psave,vbatts,caltemp); //put together all var into one string //now runs at end of loop()
      crccat(datastring + 4); //add checksum (lunars code)
      count = count + 1;
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

        //power cycle
        if(count % RESTART_INTERVAL == 0)
        {
          powercycle();
        }


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
  else
  {
    // do nothing, radio is resetting.
  }
}

//Switch the radios frequency Low/High depending on the current bit.
void rtty_txbit (int bit)
{
  if (bit)
  {
    radio1.write(0x73,0x03); // High
  }
  else
  {
    radio1.write(0x73,0x00); // Low
  }
}

//Turn on and set up the RFM22B radio transmitter.
void setupRadio(){
  digitalWrite(RFM22B_SDN, LOW);
  delay(1000);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, RADIO_POWER);
  radio1.write(0x07, 0x08);
  delay(500);
  radioready = true;
}


//Start the timer interrupt to send RTTY bits.
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


// Send a command (byte array of UBX protocol) to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
    // Serial.print(MSG[i], HEX);
  }
  Serial.println();
}



// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      // Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds, if this keeps happeneing check GPS TX/RX and data voltage levels.
    if (millis() - startTime > 3000) { 
      //Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        // Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

// CRC16 Checksum from Lunar_Lander
uint16_t crccat(char *msg)
{
  uint16_t x;
  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);
  snprintf(msg, 8, "*%04X\n", x);
  return(x);
}


//Reboot radio
void powercycle()
{
  radioready = false; //radio down for reboot

  //The delay dosent seem to do anything, so run it loads of times. One day I will look into this.
  for (int i = 0; i < 50; i++)  {
    digitalWrite(RFM22B_SDN, HIGH); //power down radio
    delay(500); //pause 500ms   
  }

  delay(500); //Attempt to pause 500ms. Not that delay() does anything here.
  setupRadio(); //turn on radio and set up
}

void navmode()
{
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
    delay(1000);
  }
  gps_set_sucess=0;
  setgpsmode = true;
}




