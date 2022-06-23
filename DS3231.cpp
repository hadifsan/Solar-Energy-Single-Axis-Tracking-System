int output1 = 5;
int output2 = 6;
const int onHour = 18;
const int onMin = 17;
const int onSec = 00;
const int onHour1 = 18;
const int onMin1 = 17;
const int onSec1 = 10;

#include <Wire.h> //I2C library
#include <RtcDS3231.h> //RTC library

RtcDS3231<TwoWire> rtcObject(Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //Starts serial connection
  rtcObject.Begin();     //Starts I2C
  pinMode(output1, OUTPUT); // relay 1
  pinMode(output2, OUTPUT); // relay 2
  //RtcDateTime currentTime = RtcDateTime(22, 04, 13, 16, 40, 00); //define date and time object
  //rtcObject.SetDateTime(currentTime);                           //configure the RTC with object
  digitalWrite(output1, HIGH);
  digitalWrite(output2, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  RtcDateTime currentTime = rtcObject.GetDateTime();    //get the time from the RTC
  
  char str[20];   //declare a string as an array of chars
  
  sprintf(str, "%d/%d/%d %d:%d:%d",     //%d allows to print an integer to the string
          currentTime.Year(),   //get year method
          currentTime.Month(),  //get month method
          currentTime.Day(),    //get day method
          currentTime.Hour(),   //get hour method
          currentTime.Minute(), //get minute method
          currentTime.Second()  //get second method
         );
  
  Serial.println(str); //print the string to the serial port
  
  delay(1000); //20 seconds delay

  if ( currentTime.Hour() == onHour && currentTime.Minute() == onMin && currentTime.Second()== onSec){ 
      digitalWrite(output1, LOW);
      //digitalWrite(output2, HIGH); //RETRACT
  }
  else if ( currentTime.Hour() == onHour1 &&currentTime.Minute() == onMin1 && currentTime.Second()== onSec1){ 
     //digitalWrite(output1, HIGH);
     digitalWrite(output2, LOW);
  } 
}
