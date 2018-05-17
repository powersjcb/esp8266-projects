// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
#include <ESP8266WiFi.h>
#include <FastLED.h>
#include "MemoryFree.h"
#include <movingAvg.h>

#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 29
#define DATA_PIN 12
#define CLOCK_PIN 14
#define COLOR_ORDER BGR

CRGB leds[NUM_LEDS_PER_STRIP];

//AFS_SEL=0 // +/-2g, 16,384 LSB/g
#define MPU6050_ACONFIG_AFS_SEL_BIT 0

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;

// setup mpu
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


unsigned short period = 2;
Quaternion qu;           // [w, x, y, z]         quaternion container
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t fifoCount;     // count of all bytes currently in FIFO
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


int16_t ax, ay, az;
int last_window = 0;
int window_size = 5; // milliseconds
int brightness;

int bufferSize = 100;
float serialBuffer[300];
int bufferIdx = 0;



//#define INTERRUPT_PIN 0 // must not use pin D0 (GPI16) on d1 mini
#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
movingAvg ave_x(50), ave_y(50), ave_z(50);


void setup() {
  // turn off the wifi to remove extra interrupts
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(74880);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  Serial.println(devStatus);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS_PER_STRIP);
  ave_x.begin();
  ave_y.begin();
  ave_z.begin();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void bufferedLog(int message) {
  if (bufferIdx > bufferSize) {
        for (int i=0; i < bufferSize; i++) {
          Serial.println(serialBuffer[i], 5);
        }
    bufferIdx = 0;
  }
  serialBuffer[bufferIdx] = message;
  bufferIdx += 1;
}

void loop() {
  unsigned long current_time = millis();
  while (!mpuInterrupt && fifoCount < packetSize) {
    // maybe do some interpolation
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
   // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


    mpu.getAcceleration(&ax, &ay, &az);
    mpu.dmpGetQuaternion(&qu, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &qu);
    mpu.dmpGetYawPitchRoll(ypr, &qu, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
    
    ax = ave_x.reading(ax);
    // ax -> (long axis of assembly, pointing down is negative)
    // ay -> (right positive from front face)
    // az -> (positive is the direction the LEDs are facing)
    double Ay = ypr[0];
    brightness = constrain(pow(abs(3.1415 / 2.0 - abs(Ay)) + 1, 4.0), 0, 150);
    bufferedLog(Ay);
    last_window = current_time / window_size;
    
    if (current_time > 10000) {
      for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
        if ((current_time) % period == 0) {
          leds[i] = CRGB(0, 0, brightness);
        }
      }
      FastLED.show();
    }
  }
}
