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
  PAD_IDLE,
  LAUNCH_DETECT,
  BOOST,
  BURNOUT,
  COAST,
  APOGEE,
  DROGUE_DETECT,
  DROGUE_DESCENT,
  MAIN_DETECT,
  MAIN_DESCENT,
  FAIRING_RELEASE,
  SAIL_START,
  SAIL_CONTROLLED_DESCENT,
  LAND_DETECT,
  LANDED
};

RocketState rocketState = PAD_IDLE;  // Initialize to IDLE state


//create objects
File file; 

void setup() {
  // put your setup code here, to run once:

  // Set initial state to IDLE
  rocketState = PAD_IDLE;

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
    case PAD_IDLE:
      // Check for acceleration above a threshold to detect liftoff
      if (event.acceleration.z >= LIFTOFF_ACCELERATION_THRESHOLD) {
        // Record the start time of acceleration
        liftoffStartTime = millis();
        // If Acceleration detected, Go to LAUNCH_DETECT
        rocketState = LAUNCH_DETECT;
        Serial.println(rocketState = LAUNCH_DETECT);
      }
      break;

    case LAUNCH_DETECT:
      // Continue reading acceleration data
      // If Acceleration was brief, Back to PAD_IDLE
      // Check for sustained acceleration for a certain duration
      if ((millis() - liftoffStartTime) >= LIFTOFF_CONFIRMATION_DURATION) {
        // Rocket has liftoff, perform necessary actions
        // For example, trigger parachute deployment or log liftoff data
        rocketState = BOOST;
        Serial.println(rocketState = BOOST);
      }
      break;

    case BOOST:
     // Your code for boost operations goes here
     // No Acceleration -> Go to BURNOUT
    break;

    case BURNOUT:
    // No Accel and Time Delay -> Go to COAST
    // Acceleration Detected -> Back to BOOST
    break;
    
    case COAST:
    // Check for altitude change to detect apogee
      if (abs(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude) >= ALTITUDE_CHANGE_THRESHOLD) {
        // Apogee detected, record the time
        apogeeTime = millis();
        rocketState = APOGEE;
        Serial.println(rocketState = APOGEE);
      }
      break;

    case APOGEE:
     // If Jerk Detected from IMU -> Go to DROGUE DETECT
      if (event.acceleration.z >= JERK_THRESHOLD) {
        // Jerk detected, switch to drogue descent
        rocketState = DROGUE_DETECT;
      }
      break;

    case DROGUE_DETECT:
    // Check for altitude to switch to drogue descent
      if (bmp.readAltitude(SEALEVELPRESSURE_HPA) <= DROGUE_DESCENT_ALTITUDE) {
        rocketState = DROGUE_DESCENT;
        Serial.println(rocketState = DROGUE_DESCENT);
      }
      break;

    case DROGUE_DESCENT:
    // Perform actions for drogue descent
    // If Jerk Detected by IMU, Go to MAIN_DETECT
      break;

    case MAIN_DETECT:
      // Main expected to deploy at 600ft
      // If Jerk Detected by Barometer, Go to MAIN_DESCENT
          rocketState = MAIN_DESCENT;
      // If Jerk NOT Detected by Barometer, Back to DROGUE_DESCENT
          rocketState = DROGUE_DESCENT;
      break;

    case MAIN_DESCENT:
    // If GO message received, and <=400 ft -> Go to FAIRING_RELEASE
      break;
      
    case FAIRING_RELEASE:
    // If GO message received, and 400+-15 ft -> Cut Wire
    // If accelerating and time delay, go to SAIL_OPERATION
      break;

    case SAIL_START:
    // Landing leg activates for ~5 seconds
    // Prop motor starts at _ Hz
    // RWheel motor starts at _ Hz
    // If velocity between 15-20ft/s, go to SAIL_CONTROLLED_DESCENT
      break;

     case SAIL_CONTROLLED_DESCENT:
     // control system
     // If Minimal Altitude change in 2 seconds, Go to Land Detect
      break;

     case LAND_DETECT:
     // If Minimal Altitude change in 2 seconds, Go to LANDED
     // If too much altitude change, Back to SAIL_CONTROLLED_DESCENT
       
     case LANDED:
     // Your code for handling landing operations goes here
     // Dump flash memory to SD Card
      break;  

    // Add more states and transitions as needed

    default:
      // Handle unexpected state
      break;
  }

  }
