#include <TinyGPS.h>
#include <SD.h>

TinyGPS gps;
char latstr[10] = "0"; //Lat converted to string
char lonstr[10] = "0"; //Lng converted to string
unsigned int alt; 
int sats;
char datastring[80]; //where the telementry string is stored

void setup()
{
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(A2, OUTPUT);
  SD.begin(A2);
  pinMode(9, OUTPUT);    // RFM22B SDN is on ARDUINO D9
  digitalWrite(9, HIGH); //power down radio
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  unsigned long time;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    
    dtostrf(flat,9,6,latstr); // convert lat from float to string
    dtostrf(flon,9,6,lonstr); // convert lon from float to string
    alt = gps.f_altitude(); // +/- altitude in meters 
    sats = gps.satellites();

    unsigned long fix_age;


    // time in hhmmsscc, date in ddmmyy
    gps.get_datetime(0, &time, &fix_age);

    //drop cc (millis) from time to leave us with HHMMSS
    time = time / 100;


    //correct no of 0's in long, (output: 00.575950/-0.575950)
    if (lonstr[0] == ' ') {
      lonstr[0] = '0';
    }
    
    sprintf(datastring,"%06lu,%s,%s,%i,%i",time,latstr,lonstr,alt,sats); //put together all var into one string
    
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
    if (dataFile) {
    dataFile.println(datastring);
    dataFile.close();
    }  
    
  }
}
