#include <unwind-cxx.h>
#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <utility.h>

#if defined(ESP8266)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
#include <Wire.h>  // must be incuded here so that Arduino library object file references work

#include <RtcDS1307.h>
#include "SevSeg.h"
#include <IRremote.h>
#include <SendOnlySoftwareSerial.h>

#include <EEPROM.h>

#include <SPI.h>
#include "RF24.h"

#include <vector>

#define DEFAULT_MODES 5
#define DISPLAY_HR_MIN 0
#define DISPLAY_SEC 1
#define ALTERNATE_HR_MIN_SEC 2
#define ALARM_SET 3
#define ALARM_SET_HR 31
#define ALARM_SET_MIN 32
#define ALARM_SET_FIRE 33
#define ALARMS_SHOW_ENABLED 37
#define ALARMS_SHOW_DISABLED 38
#define SYNC_SET 4
#define SYNC_OFFSET 41
#define SYNC_ACCEPT 42
#define DELAY_ALTERNATE 2500UL
#define DELAY_DASHES 700UL
#define RECV_PIN A0

int mode = DISPLAY_HR_MIN;

IRrecv irrecv(RECV_PIN);

decode_results results;

SevSeg minutes; //Instantiate a seven segment controller object
SevSeg hours;

SendOnlySoftwareSerial mySerial(1);

int alarmMinutes = 0;
int alarmHours = 0;
bool alarmFired = false;
bool alarmTimeout = false;

RtcDS1307<TwoWire> Rtc(Wire);

bool radioNumber = 1;
RF24 radio(9,10);
byte addresses[][6] = {"1Node","2Node"};

class Alarm {
  int HH;
  int MM;
  int DT;
  int EN;
  public:
  Alarm(int hh, int mm, int dt, int en) {
    HH=hh; MM=mm; DT=dt;EN=en;
  }
  bool isEnabled() {
    return EN;
  }
  int gHH(){return HH;}
  int gMM(){return MM;}
  int gDT(){return DT;}
  int gEN(){return EN;}
};

std::vector <Alarm> alarmy;

void setAlarm(Alarm A) {
  byte alarmCount = EEPROM.read(0);
  mySerial.print("current AlarmCount: ");
  mySerial.println(alarmCount);
  EEPROM.update(0, alarmCount+1);
  EEPROM.update(2*alarmCount+1, ((A.gDT()<<5)+A.gHH()));
  EEPROM.update(2*alarmCount+2, ((A.gMM()<<2)+A.gEN()));
  mySerial.println(EEPROM.read(2*alarmCount+1));
  mySerial.println(EEPROM.read(2*alarmCount+2));
  alarmy.push_back(A);
}

void updateAlarm(int n, Alarm A) {
  byte alarmCount = EEPROM.read(0);
  if (n>alarmCount) setAlarm(A);
  else {
    EEPROM.update(2*n+1, ((A.gDT()<<5)+A.gHH()));
    EEPROM.update(2*n+2, ((A.gMM()<<2)+A.gEN()));
  }
  alarmy[n] = A;
}

void obtainAlarms() {
  byte alarmCount = EEPROM.read(0);
  mySerial.print(alarmCount);
  mySerial.println(" alarms in EEPROM");
  for (int i=0; i<alarmCount; ++i) {
    byte b0 = EEPROM.read(2*i+1);
    byte b1 = EEPROM.read(2*i+2);
    alarmy.push_back(Alarm(b0%32,(b1>>2),(b0>>5), b1%2));
    mySerial.print(alarmy[i].gHH());
    mySerial.print(":");
    mySerial.print(alarmy[i].gMM());
    mySerial.print(",every:");
    mySerial.print(alarmy[i].gDT());
    mySerial.print(",");
    mySerial.println(alarmy[i].gEN());
  }
}

void setup () 
{
    mySerial.begin(57600);

    mySerial.print("compiled: ");
    mySerial.print(__DATE__);
    mySerial.println(__TIME__);

    //--------RTC SETUP ------------
    Rtc.Begin();

    // if you are using ESP-01 then uncomment the line below to reset the pins to
    // the available pins for SDA, SCL
    // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    //printDateTime(compiled);
    mySerial.println(__DATE__);
    mySerial.println(__TIME__);

    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Cuases:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        mySerial.println("RTC lost confidence in the DateTime!");

        // following line sets the RTC to the date & time this sketch was compiled
        // it will also reset the valid flag internally unless the Rtc device is
        // having an issue

        Rtc.SetDateTime(compiled);
    }

    if (!Rtc.GetIsRunning())
    {
        mySerial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        mySerial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        mySerial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        mySerial.println("RTC is the same as compile time! (not expected but all is fine)");
    }

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    Rtc.SetSquareWavePin(DS1307SquareWaveOut_Low); 

    byte numDigits = 2;   
    byte digitPinsHours[] = {8, 6}; //Digits: 1,2,3,4 <--put one resistor (ex: 220 Ohms, or 330 Ohms, etc, on each digit pin)
    byte digitPinsMinutes[] = {2, 4};
    byte segmentPins[] = {A2, A1, 5, 3, 7, 0, A3, 77}; //Segments: A,B,C,D,E,F,G,Period

    hours.begin(COMMON_CATHODE, numDigits, digitPinsHours, segmentPins);
    minutes.begin(COMMON_CATHODE, numDigits, digitPinsMinutes, segmentPins);
    hours.setBrightness(1);
    minutes.setBrightness(1);

    irrecv.enableIRIn(); // Start the receiver

    //obtainAlarms();
    
    RtcDateTime now1 = Rtc.GetDateTime();
    int hrs = now1.Hour();
    int mins = now1.Minute();
    int secs = now1.Second();
    mySerial.print("current time: ");
    mySerial.print(hrs);
    mySerial.print(":");
    mySerial.print(mins);
    mySerial.print(":");
    mySerial.println(secs);

    radio.begin();
    radio.setPALevel(RF24_PA_LOW);

  // Open a writing and reading pipe on each radio, with opposite addresses
    if(radioNumber){
      radio.openWritingPipe(addresses[1]);
      radio.openReadingPipe(1,addresses[0]);
    }else{
      radio.openWritingPipe(addresses[0]);
      radio.openReadingPipe(1,addresses[1]);
    }
    radio.startListening();
    mySerial.println(F("end of setup"));
}

struct query {
  char co;
  char date[12];
  char hour[9];
};

bool syncTime() {
  mySerial.println("synctime1");
  radio.stopListening();    
  mySerial.println(F("Now sending"));

  unsigned long start_time = micros();                           // Take the time, and send it.  This will block until complete
  query request; request.co='t';
  if (!radio.write( &request, sizeof(query) )){
     mySerial.println(F("failed"));
  } else mySerial.print(F("Sent "));

  radio.startListening();                                    // Now, continue listening
    
  unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not

  while ( ! radio.available() ){                             // While nothing is received
    if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }
  }

  if ( timeout ){                                             // Describe the results
      mySerial.println(F("Failed, response timed out."));
      return false;
  }else{
      query response; query result;                                 // Grab the response, compare, and send to debugging spew
      radio.read( &response, sizeof(query) );
      mySerial.print(F(", Got response "));
      mySerial.println(response.co);
      delay(3995);
      if (response.co == 'o') {
      // Spew it
        while (radio.available()) {
          radio.read(&result, sizeof(query) );
        }
        mySerial.println(result.co);
        mySerial.println(result.date);
        mySerial.println(result.hour);
      }
  }
  mySerial.println("synctime2");
  return true;
}
bool synced=false;
void loop () 
{
  //mySerial.println("loop");
  //mySerial.println(mode);
    //if (!Rtc.IsDateTimeValid()) 
    //{
        // Common Cuases:
        //    1) the battery on the device is low or even missing and the power line was disconnected
    //    mySerial.println("RTC lost confidence in the DateTime!");
    //}

    RtcDateTime now = Rtc.GetDateTime();
    switch (mode) {
      case DISPLAY_HR_MIN: {
        synced=false;
        //mySerial.print(".");
        int hrs = now.Hour();
        int mins = now.Minute();
        hours.setNumber(hrs, 0);
        minutes.setNumber(mins, 1);
        hours.refreshDisplay(); // Must run repeatedly
        minutes.refreshDisplay();
        break;
      }
      case DISPLAY_SEC: {
        int mins = now.Second();
        minutes.setNumber(mins, 1);
        minutes.refreshDisplay();
        break;
      }
      case ALTERNATE_HR_MIN_SEC: {
        unsigned long currUpTime = millis();
        if (currUpTime % DELAY_ALTERNATE == currUpTime % (2*DELAY_ALTERNATE)) {
          int hrs = now.Hour();
          int mins = now.Minute();
          hours.setNumber(hrs, 0);
          minutes.setNumber(mins, 1);
          hours.refreshDisplay(); // Must run repeatedly
          minutes.refreshDisplay();
        } else {
          int mins = now.Second();
          minutes.setNumber(mins, 1);
          minutes.refreshDisplay();
        }
        break;
      }
      case ALARM_SET: {
        int hrs = 100;
        int mins = 100;
        hours.setNumber(hrs, 0);
        minutes.setNumber(mins, 1);
        hours.refreshDisplay(); // Must run repeatedly
        minutes.refreshDisplay();
        break;
      }
      case ALARM_SET_HR: {
        int hrs = alarmHours;
        int mins = 100;
        hours.setNumber(hrs, 0);
        minutes.setNumber(mins, 1);
        hours.refreshDisplay(); // Must run repeatedly
        unsigned long currUpTime = millis();
        if (currUpTime % DELAY_DASHES == currUpTime % (2* DELAY_DASHES))
          minutes.refreshDisplay();
        break;
      }
      case ALARM_SET_MIN: {
        int hrs = 100;
        int mins = alarmMinutes;
        hours.setNumber(hrs, 0);
        minutes.setNumber(mins, 1);
        minutes.refreshDisplay(); // Must run repeatedly
        unsigned long currUpTime = millis();
        if (currUpTime % DELAY_DASHES == currUpTime % (2* DELAY_DASHES))
          hours.refreshDisplay();
        break;
      }
      case ALARM_SET_FIRE: {
        unsigned long currUpTime = millis();
        if (currUpTime % DELAY_DASHES == currUpTime % (2* DELAY_DASHES)) {
          int hrs = alarmHours;
          int mins = alarmMinutes;
          hours.setNumber(hrs, 0);
          minutes.setNumber(mins, 1);
          hours.refreshDisplay();
          minutes.refreshDisplay(); // Must run repeatedly
        } else {
          int hrs = 100;
          int mins = 100;
          hours.setNumber(hrs, 0);
          minutes.setNumber(mins, 1);
          hours.refreshDisplay();
          minutes.refreshDisplay();
        }
        break;
      }
      case SYNC_SET: {
        synced=false;
        int hrs = 54;
        int mins=100;
        hours.setNumber(hrs, 0);
        minutes.setNumber(mins, 1);
        hours.refreshDisplay();
        minutes.refreshDisplay();
        break;
      }
      case SYNC_OFFSET: {
        //mySerial.print("!");
        if (!synced)  synced = syncTime();
        else {
          unsigned long currUpTime = millis();
          if (currUpTime % DELAY_DASHES == currUpTime % (2* DELAY_DASHES)) {
            int hrs=54;
            hours.setNumber(hrs, 0);
            hours.refreshDisplay();
          } else {
            int hrs = 54;
            int mins=100;
            hours.setNumber(hrs, 0);
            minutes.setNumber(mins, 1);
            hours.refreshDisplay();
            minutes.refreshDisplay();
          }
        }
        break;
      }
      
      default: {
        mode=DISPLAY_HR_MIN;
      }
    }
    
    if (irrecv.decode(&results)) {
      //mySerial.println(results.value, HEX);
      if (results.value == 0x7121) {
        //mySerial.println(mode);
        if (mode > DEFAULT_MODES) mode = DEFAULT_MODES-1;
        mode++;
        mode%=DEFAULT_MODES;
        mySerial.println(mode);
      }
      if (mode == ALARM_SET_FIRE) {
        switch (results.value) {
            case 0x1421: {
              mode = ALARM_SET_HR;
              break;
            }
            case 0x4421: {
              setAlarm(Alarm(alarmHours, alarmMinutes, 0, 1));
              mySerial.print("Alarm is set to");
              mySerial.print(alarmHours);
              mySerial.print(":");
              mySerial.println(alarmMinutes);
              mode = DISPLAY_HR_MIN;
              break;
            }
          }
      } else
      if (mode == ALARM_SET_MIN) {
        switch (results.value) {
            case 0x6621: {
              ++alarmMinutes;
              break;
            }
            case 0x2621: {
              --alarmMinutes;
              break;
            }
            case 0x1621: {
              alarmMinutes+=5;
              break;
            }
            case 0x5621: {
              alarmMinutes-=5;
              break;
            }
            case 0x1421: {
              mode = ALARM_SET_FIRE;
              break;
            }
          }
        if (alarmMinutes>=60) alarmMinutes-=60;
        if (alarmMinutes<0) alarmMinutes+=60;
      } else 
      if (mode == ALARM_SET_HR) {
        switch (results.value) {
            case 0x6621: {
              ++alarmHours;
              break;
            }
            case 0x2621: {
              --alarmHours;
              break;
            }
            case 0x1621: {
              alarmHours+=4;
              break;
            }
            case 0x5621: {
              alarmHours-=4;
              break;
            }
            case 0x1421: {
              mode = ALARM_SET_MIN;
              break;
            }
          }
        if (alarmHours<0) alarmHours+=24;
        if (alarmHours>=24) alarmHours-=24;
      } else
      if (mode == ALARM_SET) {
        switch (results.value) {
            case 0x1421: {
              mode = ALARM_SET_HR;
              break;
            }
          }
      } else
      if (mode == SYNC_SET) {
        if (results.value==0x1421) {
          mode = SYNC_OFFSET;
          }
      }
      irrecv.resume(); // Receive the next value
    }
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

