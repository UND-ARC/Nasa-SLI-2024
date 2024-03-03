#include <SD.h>           //includes SD card library for datalogging
#include <SdFat.h>
#include <Wire.h>         //includes Wire.h library for I2C interface
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP3XX.h>
#include <RH_RF95.h>
#include <SPI.h>

RH_RF95 rf95(8,3);

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
const float BURNOUT_ACCELERATION_THRESHOLD = 5.0;
unsigned long startTime = 0;
bool belowThresholdFlag = false;
const int BURNOUT_CHECK_DURATION = 1000; // 1 second in milliseconds

float previousAccelZ = 0;
unsigned long previousTime = 0;

const unsigned long BMP_CHECK_INTERVAL = 5000;
unsigned long lastBMPCheckTime = 0;

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

/************************
 Jerk from BMP390
************************/
float checkBarometerJerk() {
  // Read current altitude from the BMP390 sensor
  float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // Calculate time elapsed since the last measurement
  unsigned long currentTime = millis();
  float dt = (currentTime - lastBMPCheckTime) / 1000.0; // Convert to seconds

  // Calculate jerk as the rate of change of altitude
  float jerk = (currentAltitude - previousAltitude) / dt;

  // Update previous values
  previousAltitude = currentAltitude;
  lastBMPCheckTime = currentTime;

 return jerk;
}

/**********************************
 Wire Cutters (Move this code to Fairing Release Controller)
 **********************************/

void fireWireCutters() {
  // Implement your pyro charge activation logic here
  // This function is called when a "go" signal is received below 400 feet AGL
  // Replace this with your actual implementation
  Serial.println("Enabling pyro charges.");
}



//create objects
File file;

void setup() {
  // put your setup code here, to run once:

  // Set initial state to IDLE
  rocketState = PAD_IDLE;

}

void loop() {
  // put your main code here, to run repeatedly:
  

  // IMU detects acceleration
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  sensors_event_t event;  // Declare 'event' here

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000; // Convert to seconds
  float imujerkZ = (event.acceleration.z - previousAccelZ) / dt;

  previousAccelZ = event.acceleration.z;
  previousTime = currentTime;

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
      else if (event.acceleration.z < LIFTOFF_ACCELERATION_THRESHOLD) {
        rocketState = PAD_IDLE;
        Serial.println(rocketState = PAD_IDLE);
      }
      break;

    case BOOST:
      // Your code for boost operations goes here
      // No Acceleration -> Go to BURNOUT
      if (event.acceleration.z < BURNOUT_ACCELERATION_THRESHOLD) {
        rocketState = BURNOUT;
        Serial.println(rocketState = BURNOUT);
      }
      break;

    case BURNOUT:
      // No Accel and Time Delay -> Go to COAST
      startTime = millis();
      if (event.acceleration.z < BURNOUT_ACCELERATION_THRESHOLD) {
        if (!belowThresholdFlag) {
          // Start the timer when the acceleration falls below the threshold
          startTime = millis();
          belowThresholdFlag = true;
        }
        // Check if it has been below the threshold for 1 second
        if (millis() - startTime >= BURNOUT_CHECK_DURATION) {
          // The acceleration has been below the threshold for 1 second
          rocketState = COAST;
          Serial.println(rocketState = COAST);
        }
      } else {
        belowThresholdFlag = false; // Reset the flag when acceleration goes above the threshold
        // Acceleration Detected -> Back to BOOST
        rocketState = BOOST;
        Serial.println(rocketState = BOOST);
      }
      break;

    case COAST:
      // Check for altitude change to detect apogee
      //float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // Read current altitude -----------------------------------------
      if (abs(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude) >= ALTITUDE_CHANGE_THRESHOLD) {
        // Apogee detected, record the time
        apogeeTime = millis();
        rocketState = APOGEE;
        Serial.println(rocketState = APOGEE);
      }
      break;

    case APOGEE:
      // If Jerk Detected from IMU -> Go to DROGUE DETECT
      currentTime = millis();
      //float dt = (currentTime - previousTime) / 1000; // Convert to seconds

      if (imujerkZ > JERK_THRESHOLD) {
        rocketState = DROGUE_DETECT; // Jerk detected, Go to DROGUE_DETECT
      }
      break;

    case DROGUE_DETECT:
      // Check if it's time to check the BMP sensor for jerk
      if (currentTime - lastBMPCheckTime >= BMP_CHECK_INTERVAL) {
        float barometerJerk = checkBarometerJerk(); // Function to check BMP390 for jerk

        if (barometerJerk <= 0.0) {
          rocketState = APOGEE; // Change the state back to APOGEE if jerk is not detected by BMP390
        } else {
          rocketState = DROGUE_DESCENT; // Change to DROGUE_DESCENT if jerk is detected
        }
          lastBMPCheckTime = currentTime;
        }
        break;

      case DROGUE_DESCENT:
        // Perform actions for drogue descent
        // If Jerk Detected by IMU, Go to MAIN_DETECT

      if (imujerkZ > JERK_THRESHOLD) {
        rocketState = MAIN_DETECT; // Jerk detected, Go to DROGUE_MAIN
      }
        break;

      case MAIN_DETECT:
        // Main expected to deploy at 600ft
        // If Jerk Detected by Barometer, Go to MAIN_DESCENT
        // If Jerk NOT Detected by Barometer, Back to DROGUE_DESCENT

        if (currentTime - lastBMPCheckTime >= BMP_CHECK_INTERVAL) {
        float barometerJerk = checkBarometerJerk(); // Function to check BMP390 for jerk

        if (barometerJerk <= 0.0) {
          rocketState = DROGUE_DESCENT; // Change the state back to DROGUE_DESCENT if jerk is not detected by BMP390
        } else {
          rocketState = MAIN_DESCENT; // Change to MAIN_DESCENT if jerk is detected
        }
          lastBMPCheckTime = currentTime;
        }
        break;

      case MAIN_DESCENT:
        // If GO message received, and <=400 ft -> Go to FAIRING_RELEASE
        break;

      case FAIRING_RELEASE:
        // For Fairing Release Controller!!!!!!!
        // If GO message received, and 400+-15 ft -> Cut Wire
        // If accelerating and time delay, go to SAIL_OPERATION
        break;
          if (rf95.available()) {
            // Message received
            uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
            uint8_t len = sizeof(buf);
    
            if (rf95.recv(buf, &len)) {
              // Message received successfully
              buf[len] = '\0'; // Null-terminate the message for string manipulation
              String receivedMessage = (char*)buf;
      
              // Check if the received message is a "go" signal
              if (receivedMessage.equals("go")) {
                // Go signal received
                Serial.println("Go signal received.");

              // Check the rocket's altitude (replace with your altitude measurement method)
              float currentAltitudeAGL = bmp.readAltitude(SEALEVELPRESSURE_HPA);
              
                // If altitude is below 400 feet AGL, enable pyro charges
                if (currentAltitudeAGL < 400.0) {
                  fireWireCutters();
                }
              }
            }
          }

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
