#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



byte START_CMD[] = {
                    0xA0,
                    0xA2,
                    0x00,
                    0x18};
byte NMEA_SET[] = {
                  0x81,
                  0x02,
                  0x01, //GGA
                  0x01, //check sum
                  0x00, //GLL  
                  0x00, //check sum
                  0x00, //GSA
                  0x00, //check sum
                  0x00, //GSV
                  0x00, //check sum
                  0x01, //RMC
                  0x01, //check sum
                  0x00, //VTG
                  0x00, //check sum
                  0x00, //MSS
                  0x00, //check sum
                  0x00, //EPE
                  0x00, //check sum
                  0x00, //ZDA
                  0x00, //check sum
                  0x00, //unused
                  0x00, //unused
                  0x12, //baud in hex high byte
                  0xC0}; //baud in hex low byte


boolean sentenceBegins = false;
char buffer[90];
int index = 0;
const byte wakePin = 16;
const byte Control = 3;
const int OnOffDelay = 200;
char messageID[6];
char time[11];
char fixindicator[2];
char satsUsed[3];
char HDOP[4];
char MSLalt[10];
char Units[2];
char Geoid[6];
char GeoUnits[2];
char GPSstatus[2];
char GPSspeed[8];
char GPScourse[7];
char date[9];
char Dummy[12];
boolean lastbuttonPress = false;
boolean buttonPress = false;
byte debounce = 20;
byte selection = 1;
int longpress = 1000;
unsigned long timer1;
boolean menu = true;
boolean toggle = false;
boolean GPSOn = false;
boolean recording = false;
boolean newReading = false;
signed long segdist = 0;
signed long distance = 0;
signed long last_latitude = 0;
signed long last_longitude = 0;
signed long latDegrees, longDegrees, latFract, longFract;
char lat_fract[7];
char long_fract[7];
char segdist_fract[5];
char dist_fract[5];
char n_s[2];
char e_w[2];
const int milesPerLat = 6900; //length per degree of latitude is 69 miles
const int milesPerLong = 5253; //length per degree of longitude at
                                  //40.68 degrees of latitude is 52.53 miles.  It would
                                  //be better if I could put this conversion
                                  //in code so that it could be dynamically
                                  //calculated based on the location data from
                                  //the GPS.

//adafruit OLED stuff
#define OLED_DC 11
#define OLED_CS 9
#define OLED_CLK 12
#define OLED_MOSI 13
#define OLED_RESET 10
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);


#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif


void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(4800);
//  while (!Serial) 
//  {
//    ; // wait for serial port to connect. Needed for Leonardo only
//  }
  
  
  pinMode(wakePin,INPUT);  //GPS Wakeup Pin
  pinMode(Control,INPUT);  //Control Button
  pinMode(A3,OUTPUT); //GPS ON/OFF Pin
  pinMode(2, OUTPUT);
  digitalWrite(A3,LOW);
  digitalWrite(2, LOW);
  digitalWrite(Control,HIGH);
  //initializeGPS();
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
  display.clearDisplay();
  display.display();
//  if (!SD.begin()) 
//  {
//    display.clearDisplay();
//    display.setCursor(0,0);
//    display.setTextColor(WHITE, BLACK);
//    display.setTextSize(1);
//    display.print("Error Opening Card!!!");
//    display.display();
//    delay(2000);
//    return;
//  }
  attachInterrupt(0, btnPress, LOW);
  updateDisplay();
}






void initializeGPS()
{
  //Serial.println("Initializing GPS");
  //turn on GPS
  digitalWrite(A3, HIGH);
  delay(OnOffDelay);
  digitalWrite(A3, LOW);

  //Serial.println("Switching to Binary");
  //set to binary mode
  Serial1.println("$PSRF100,0,4800,8,1,0*0F");
  delay(100);
  
  //set back to NMEA mode with GGA and RMC messages active
  //Serial.println("Switching to NMEA");
  for(int x=0; x<4; x++)
  {
    Serial1.write(START_CMD[x]);
  }
  
  for(int x = 0; x<24; x++)
  {
    Serial1.write(NMEA_SET[x]);
  }
  
  Serial1.write(highByte(calc_check()));
  Serial1.write(lowByte(calc_check()));  
  Serial1.write(0xB0);
  Serial1.write(0xB3);
  Serial1.flush();
  //turn GPS off
  digitalWrite(A3, HIGH);
  delay(OnOffDelay);
  digitalWrite(A3, LOW);
}





void loop()
{
  if(buttonPress != lastbuttonPress)
  {
    timer1 = millis() + longpress; //to determine what kind of press
    lastbuttonPress = buttonPress; //set up for next press
    delay(debounce); //ignore noise
    while(digitalRead(Control) == LOW){}
    if(timer(timer1)) //a long press was received
    {
      
      menu = !menu;
      if(menu) 
      {
        selection = 1;
        updateDisplay();
      }
      //display.print("updating");
      else
      {
        switch(selection)
        {
          case 2:
            GPSOn = !GPSOn;
            digitalWrite(A3, HIGH);
            digitalWrite(2, HIGH);
            delay(OnOffDelay);
            
            digitalWrite(A3, LOW);
            digitalWrite(2, LOW);
            //if(!GPSOn) Serial1.println("$PSRF117,16*0B");
            break;
          case 3:
            distance = 0;
            segdist = 0;
            initializeGPS();
            break;
            
          case 4:
            recording = !recording;
            break;
        }
        update_GPSData_Displayed();
        
      }
      

      while(digitalRead(Control) == LOW){}
      delay(debounce);
      attachInterrupt(0, btnPress, LOW);
    }
    else //a short press was received
    {
      toggle = true;
    }
    
  }
  if(toggle && menu) //ycle through the options
  {
    selection++;
    if(selection>4) selection = 1;
    toggle = false;
    updateDisplay();
    delay(debounce);
    attachInterrupt(0, btnPress, LOW);
  }
  
  if(toggle && !menu) //don't do anything
  {
    toggle = false;
    delay(debounce);
    attachInterrupt(0,btnPress, LOW);
  }
  
  
  if(checkforSentence()) //if we have a complete sentence
  {                      //update the display
    Serial.println(buffer);
    updateDisplay();
//    if(recording && newReading)
//    {
//    if(strcmp(messageID, "GPGGA") == 0)
//      {
//        char filename[13];
//        sprintf(filename, "%s.txt", date);
//        if(!SD.exists(filename)) Serial.println("Making Directory");
//        File dataFile = SD.open(filename, FILE_WRITE);
//        
//        // if the file is available, write to it:
//        if (dataFile) 
//        {
//          dataFile.print(latDegrees);
//          dataFile.print(",");
//          dataFile.print(longDegrees);
//          dataFile.print(",");
//          dataFile.println(satsUsed);
//          dataFile.close();
//        }  
//      }
//    }
  }
}








void updateDisplay()
{
  //display.print("updating");
  //display.display();
  if(menu)
  {
    display.clearDisplay();
    switch(selection)   //each button toggle should highlight
    {                  //the next option in the menu
      case 1:
        display.setTextSize(1);
        display.setCursor(0,0);
        display.setTextColor(BLACK, WHITE);
        display.println("Exit");
        display.setTextColor(WHITE, BLACK);
        if(!GPSOn)
        {
        display.println("Turn On GPS");
        }
        else
        {
          display.println("Turn Off GPS");
        }
        display.println("Reset Distance");
        if(!recording)
        {
        display.println("Start Recording");
        }
        else
        {
        display.println("Stop Recording");
        }
        break;
      case 2:
        display.setTextSize(1);
        display.setCursor(0,0);
        display.setTextColor(WHITE, BLACK);
        display.println("Exit");
        display.setTextColor(BLACK, WHITE);
        if(!GPSOn)
        {
        display.println("Turn On GPS");
        }
        else
        {
        display.println("Turn Off GPS");
        }
        display.setTextColor(WHITE, BLACK);
        display.println("Reset Distance");
        if(!recording)
        {
        display.println("Start Recording");
        }
        else
        {
        display.println("Stop Recording");
        }
        break;
      case 3:
        display.setTextSize(1);
        display.setCursor(0,0);
        display.setTextColor(WHITE, BLACK);
        display.println("Exit");
        if(!GPSOn)
        {
        display.println("Turn On GPS");
        }
        else
        {
        display.println("Turn Off GPS");
        }
        display.setTextColor(BLACK, WHITE);
        display.println("Reset Distance");
        display.setTextColor(WHITE, BLACK);
        if(!recording)
        {
        display.println("Start Recording");
        }
        else
        {
        display.println("Stop Recording");
        }
        break;
      case 4:
        display.setTextSize(1);
        display.setCursor(0,0);
        display.setTextColor(WHITE, BLACK);
        display.println("Exit");
        if(!GPSOn)
        {
        display.println("Turn On GPS");
        }
        else
        {
        display.println("Turn Off GPS");
        }
        display.println("Reset Distance");
        display.setTextColor(BLACK, WHITE);
        if(!recording)
        {
        display.println("Start Recording");
        }
        else
        {
        display.println("Stop Recording");
        }
        break;
    }
    
  }
  else if(Process_message() && strcmp(messageID, "GPGGA") == 0)
  {
      buffer[0] = '\0';
      update_GPSData_Displayed();
  }
  
  display.display();

  
}

boolean checkforSentence()
{
  char c;
  while(Serial1.available())
  {
    c = Serial1.read();
    //Serial.print(c);
    
    if(sentenceBegins && c == '\r') //we have a full sentence
    {
      sentenceBegins = false;
      return true;
    }
    
    if(sentenceBegins) //store characters to buffer
    {
      buffer[index] = c;
      index++;
      buffer[index] = '\0';
    }
    
    if(c == '$') //beginning of sentence...start saving to buffer
    {
      sentenceBegins = true;
      index = 0;
    }
   
  }
  return false;
}

//checksum for GPS initialization commands
int calc_check()
{
  byte msglen = sizeof(NMEA_SET);
  byte index = 0;
  int checksum = 0;
  while(index<msglen)
  {
    checksum+=NMEA_SET[index++];
    checksum&=(0x7FFF);
  }
  return checksum;
}

//function to break apart the NMEA strings into pieces that I can
//handle
const char* mytok(char* pDst, const char* pSrc, char sep = ',')
{
    while ( *pSrc )
    {
        if ( sep == *pSrc )
        {
            *pDst = '\0';

            return ++pSrc;
        }

        *pDst++ = *pSrc++;
    }

    *pDst = '\0';

    return NULL;
}


//function that does all of the work
boolean Process_message()
{
  char latit1[5];
  char latit2[6];
  char longit1[6];
  char longit2[6];
  char NS[2];
  char EW[2];
  
  const char*     ptr;
  
  //Serial.println(buffer);
  //check message ID to see what kind of message we got
  ptr = mytok(messageID, buffer);
  //Serial.println(messageID);
  
  //if it is GGA, read in the data and write to SDCard if
  //the data is valid and an SD file has been created
  if(strcmp(messageID, "GPGGA") == 0)
  {
    ptr = mytok(time, ptr); if(ptr == NULL) return false;
    ptr = mytok(latit1, ptr, '.'); if(ptr == NULL) return false; //get the first half of latitude
    ptr = mytok(latit2, ptr); if(ptr == NULL) return false;//get the second half of latitude
    ptr = mytok(NS, ptr); if(ptr == NULL) return false;
    ptr = mytok(longit1, ptr, '.'); if(ptr == NULL) return false;
    ptr = mytok(longit2, ptr); if(ptr == NULL) return false;
    ptr = mytok(EW, ptr); if(ptr == NULL) return false;
    ptr = mytok(fixindicator, ptr); if(ptr == NULL) return false;
    ptr = mytok(satsUsed, ptr); if(ptr == NULL) return false;
    ptr = mytok(HDOP, ptr); if(ptr == NULL) return false;
    ptr = mytok(MSLalt, ptr); if(ptr == NULL) return false;
    ptr = mytok(Geoid, ptr); if(ptr == NULL) return false;
    //ptr = mytok(GeoUnits, ptr); if(ptr == NULL) {Serial.println("Out13"); return;}
    


    if(atoi(satsUsed) < 4) return false;
    
    
    unsigned long multiplier = 1000000UL;
    const int mins = 60;
    unsigned long temp;

    
    latDegrees = (atoi(latit1))/100;  //isolate degrees
    latFract = (atoi(latit1)) - (latDegrees * 100);  //isolate whole minutes
    latDegrees *= multiplier;    //scale for fixed point math
    latFract *= multiplier;      //scale for fixed point math
    latFract += ((atoi(latit2))*100UL);  //add the fractions of a minute
    latFract = latFract/60;  //convert minutes to fractions of a degree
    latDegrees += latFract;
    
    longDegrees = (atoi(longit1))/100;  //isolate degrees
    longFract = (atoi(longit1)) - (longDegrees * 100);  //isolate whole minutes
    longDegrees *= multiplier;    //scale for fixed point math
    longFract *= multiplier;      //scale for fixed point math
    longFract += ((atoi(longit2))*100UL);  //add the fractions of a minute
    longFract = longFract/60;  //convert minutes to fractions of a degree
    longDegrees += longFract;
    
    //figure out how many zeros we need after the decimal
    if(latFract<10) {strcpy(lat_fract, ".00000");}
    else if (latFract<100) {strcpy(lat_fract, ".0000");}
    else if (latFract<1000) {strcpy(lat_fract, ".000");}
    else if (latFract<10000) {strcpy(lat_fract, ".00");}
    else if (latFract<100000) {strcpy(lat_fract, ".0");}
    else {strcpy(lat_fract, ".");}

    if(longFract<10) {strcpy(long_fract, ".00000");}
    else if (longFract<100) {strcpy(long_fract, ".0000");}
    else if (longFract<1000) {strcpy(long_fract, ".000");}
    else if (longFract<10000) {strcpy(long_fract, ".00");}
    else if (longFract<100000) {strcpy(long_fract, ".0");}
    else {strcpy(long_fract, ".");}
    
    if(NS[0] == 'S') strcpy(n_s, "-");

    if(EW[0] == 'W') strcpy(e_w, "-");
    
    //calculate the distance
    if(last_latitude == 0)
    {
      last_latitude = latDegrees;
      last_longitude = longDegrees;
    }

    if(!(last_latitude == 0)) //just check to make sure that it isn't the first reading
    //calculate distance of segment
    {
      long temp = 0;
      segdist = sqrt(pow(((latDegrees - last_latitude)*milesPerLat)/10000L,2) + pow(((longDegrees - last_longitude)*milesPerLong)/10000L,2));
      if(segdist<100)
      {
        segdist = 0;
      }
      temp = segdist - ((segdist/10000) * 10000); //isolate the fractional amount
      if(temp<10) {strcpy(segdist_fract, ".000");}
      else if (temp<100) {strcpy(segdist_fract, ".00");}
      else if (temp<1000) {strcpy(segdist_fract, ".0");}
      else {strcpy(segdist_fract, ".");}
      
      distance+=segdist;
      temp = 0;
      temp = distance - ((distance/10000)*10000);
      if(temp<10) {strcpy(dist_fract, ".000");}
      else if (temp<100) {strcpy(dist_fract, ".00");}
      else if (temp<1000) {strcpy(dist_fract, ".0");}
      else {strcpy(dist_fract, ".");}
      
      if(!(segdist<100))
      {
        last_latitude = latDegrees;
        last_longitude = longDegrees;
      }
      return true;


    }
    
    
    //add the segment distance to the total distance



  }
  
  if(strcmp(messageID, "GPRMC") == 0)
  {
    char tempdate[7];
    
    ptr = mytok(time, ptr); if(ptr == NULL) return false;
    ptr = mytok(GPSstatus, ptr); if(ptr == NULL) return false;
    ptr = mytok(latit1, ptr, '.'); if(ptr == NULL) return false;//get the first half of latitude
    ptr = mytok(latit2, ptr); if(ptr == NULL) return false;//get the second half of latitude
    ptr = mytok(NS, ptr); if(ptr == NULL) return false;
    ptr = mytok(longit1, ptr, '.'); if(ptr == NULL) return false;
    ptr = mytok(longit2, ptr); if(ptr == NULL) return false;
    ptr = mytok(EW, ptr); if(ptr == NULL) return false;
    ptr = mytok(GPSspeed, ptr); if(ptr == NULL) return false;
    ptr = mytok(GPScourse, ptr); if(ptr == NULL) return false;
    ptr = mytok(tempdate, ptr); if(ptr == NULL) return false;
    
  
    //GPSstatus tells us if data is valid
    if(GPSstatus[0] == 'V') return false;
    
    //re-order the date so that it will be in a nicer format for
    //sorting on the sd card
    date[0] = tempdate[4];
    date[1] = tempdate[5];
    date[2] = '_';
    date[3] = tempdate[2];
    date[4] = tempdate[3];
    date[5] = '_';
    date[6] = tempdate[0];
    date[7] = tempdate[1];
    date[8] = '\0';
    
    return true;
  }
  return false;
}


void btnPress()
{
      detachInterrupt(0);
      buttonPress = !buttonPress;
}

void update_GPSData_Displayed()
{
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(0,0);
      display.print("Lat = ");
      display.print(n_s);
      display.print(latDegrees/1000000UL);
      display.print(lat_fract);
      display.print(latFract);
      display.println("  ");
      display.print("Long = ");
      display.print(e_w);
      display.print(longDegrees/1000000UL);
      display.print(long_fract);
      display.print(longFract);
      display.println("  ");
      display.setCursor(110,0);
      display.print(satsUsed);
      display.setCursor(0,16);
      display.print("Seg = ");
      display.print(segdist/10000);
      display.print(segdist_fract);
      display.print(segdist - ((segdist/10000) * 10000));
      display.println("  ");
      display.print("Dist = ");
      display.print(distance/10000);
      display.print(dist_fract);
      display.print(distance - ((distance/10000)*10000));
      display.println("  ");
      display.display();
}

//keep track of time and handle millis() rollover
boolean timer(unsigned long timeout)
  {
    return (long)(millis() - timeout) >= 0;
  }
  
