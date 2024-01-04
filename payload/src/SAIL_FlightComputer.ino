#include <SD.h>           //includes SD card library for datalogging
#include <SdFat.h>
#include <Wire.h>         //includes Wire.h library for I2C interface
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP3XX.h>

// For the BNO055 IMU, Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)

//This is for the BMP390 barometer
Adafruit_BMP3XX bmp;
double inHg = 29.92; // enter altimiter value
double hPa = inHg * 33.8639;
#define SEALEVELPRESSURE_HPA (hPa) // default: 1013.25

float previousAltitude = 0.0;  // Store the previous altitude reading
float minAltitudeChange = 1.0;  // Minimum altitude change threshold (adjust as needed)

// Define constants
const float LIFTOFF_ACCELERATION_THRESHOLD = 10.0;  // Adjust as needed
const unsigned long LIFTOFF_CONFIRMATION_DURATION = 2000;  // 1 second (adjust as needed)
const float ALTITUDE_CHANGE_THRESHOLD = 10.0;  // Adjust as needed
const float JERK_THRESHOLD = 5.0;  // Adjust as needed
const int DROGUE_DESCENT_ALTITUDE = 500;  // Adjust as needed (in meters)

unsigned long liftoffStartTime = 0;  // Declare 'liftoffStartTime' at the global scope
float initialAltitude = 0;  // Declare 'initialAltitude' at the global scope
unsigned long apogeeTime = 0;  // Declare 'apogeeTime' at the global scope

enum RocketState {
  IDLE,
  LAUNCH_DETECT,
  IN_FLIGHT,
  DROGUE_DESCENT,
  DESCENT,
  RECOVERY,
  LANDING
};

RocketState rocketState = IDLE;  // Initialize to IDLE state


//create objects
File file; 

void setup() {
  // put your setup code here, to run once:

  // Set initial state to IDLE
  rocketState = IDLE;

}

void loop() {
  // put your main code here, to run repeatedly:
  float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // Read current altitude

// IMU detects acceleration
sensors_event_t linearAccelData;
bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  sensors_event_t event;  // Declare 'event' here

  // Read sensor data
  bno.getEvent(&event);
  
  switch (rocketState) {
    case IDLE:
      // Check for acceleration above a threshold to detect liftoff
      if (event.acceleration.z >= LIFTOFF_ACCELERATION_THRESHOLD) {
        // Record the start time of acceleration
        liftoffStartTime = millis();
        rocketState = LAUNCH_DETECT;
        Serial.println(rocketState = LAUNCH_DETECT);
      }
      break;

    case LAUNCH_DETECT:
      // Continue reading acceleration data
      // Check for sustained acceleration for a certain duration
      if ((millis() - liftoffStartTime) >= LIFTOFF_CONFIRMATION_DURATION) {
        // Rocket has liftoff, perform necessary actions
        // For example, trigger parachute deployment or log liftoff data
        rocketState = IN_FLIGHT;
        Serial.println(rocketState = IN_FLIGHT);
      }
      break;

    case IN_FLIGHT:
      // Check for altitude change to detect apogee
      if (abs(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude) >= ALTITUDE_CHANGE_THRESHOLD) {
        // Apogee detected, record the time
        apogeeTime = millis();
        rocketState = DESCENT;
        Serial.println(rocketState = DESCENT);
      }
      break;

    case DESCENT:
      // Check for jerk to detect the drogue chute
      if (event.acceleration.z >= JERK_THRESHOLD) {
        // Jerk detected, switch to drogue descent
        rocketState = DROGUE_DESCENT;
      }
      // Check for altitude to switch to drogue descent
      else if (bmp.readAltitude(SEALEVELPRESSURE_HPA) <= DROGUE_DESCENT_ALTITUDE) {
        rocketState = DROGUE_DESCENT;
        Serial.println(rocketState = DROGUE_DESCENT);
      }
      break;

    case DROGUE_DESCENT:
      // Perform actions for drogue descent
      break;

    case LANDING:
      // Your code for handling landing operations goes here
      break;

    // Add more states and transitions as needed

    default:
      // Handle unexpected state
      break;
  }

  }
