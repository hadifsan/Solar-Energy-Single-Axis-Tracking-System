#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h> //I2C library
#include <RtcDS3231.h> //RTC library
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

RtcDS3231<TwoWire> rtcObject(Wire);
MPU6050 mpu(0x69);

const int sensorpin = A0;
int sensorValue;
int waterlevel = 512;
int output1 = 5; // relay output
int output2 = 6; // relay

const int onHour = 18, onMin = 00, onHour1 = 10, onMin1 = 00, onDeg1 = 54, onHour2 = 12, onMin2 = 00, onDeg2 = 70, onHour3 = 13, onMin3 = 00, onDeg3 = 83, onHour4 = 14, onMin4 = 00, onDeg4 = 101;
const int onHour5 = 16, onMin5 = 00, onDeg5 = 130;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);
    rtcObject.Begin();     //Starts I2C
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   // while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(40);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(-25);
    mpu.setZAccelOffset(1606); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
       // mpu.CalibrateAccel(6);
       // mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(sensorpin, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(output1, OUTPUT); // relay 1
    pinMode(output2, OUTPUT); // relay 2
    //RtcDateTime currentTime = RtcDateTime(22, 06, 15, 15, 00, 00); //define date and time object
    //rtcObject.SetDateTime(currentTime);                           //configure the RTC with object
    digitalWrite(output1, HIGH);
    digitalWrite(output2, HIGH);
}



void loop() {
  // put your main code here, to run repeatedly:
    // read the input on analog pin 0:
   sensorValue = analogRead(sensorpin);
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
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        
    }
    if ( currentTime.Hour() == onHour2 && currentTime.Minute() == onMin2 ) { //extend
     digitalWrite(output1, HIGH);
     digitalWrite(output2, LOW); 
    }
    if ( ypr[1] * 180/M_PI >= onDeg2 ){ 
     digitalWrite(output1, HIGH);
     digitalWrite(output2, HIGH);
  }
    if ( currentTime.Hour() == onHour3 && currentTime.Minute() == onMin3 ) { //extend
     digitalWrite(output1, HIGH);
     digitalWrite(output2, LOW); 
    }
    if ( ypr[1] * 180/M_PI >= onDeg3 ){ 
     digitalWrite(output1, HIGH);
     digitalWrite(output2, HIGH);
  }
    if ( currentTime.Hour() == onHour4 && currentTime.Minute() == onMin4 ) { //extend
     digitalWrite(output1, HIGH);
     digitalWrite(output2, LOW); 
    }
    if ( ypr[1] * 180/M_PI >= onDeg4 ){ 
     digitalWrite(output1, HIGH);
     digitalWrite(output2, HIGH);
  }
    if ( currentTime.Hour() == onHour5 && currentTime.Minute() == onMin5 ) { //extend
     digitalWrite(output1, HIGH);
     digitalWrite(output2, LOW); 
    }
    if ( ypr[1] * 180/M_PI >= onDeg5 ){ 
     digitalWrite(output1, HIGH);
     digitalWrite(output2, HIGH);
  }
    if ( currentTime.Hour() == onHour && currentTime.Minute() == onMin || sensorValue >= waterlevel){ //extend
     digitalWrite(output1, LOW);
     digitalWrite(output2, HIGH); 
    }
    if ( ypr[1] * 180/M_PI <= onDeg1 ){ 
     digitalWrite(output1, HIGH);
     digitalWrite(output2, HIGH);
  }
   
}
