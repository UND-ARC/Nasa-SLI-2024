#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <utility/imumaths.h>
#include <SD.h>

Adafruit_BNO055 bno = Adafruit_BNO055();
Servo ESC;

double Kp = 0.8;
double Ki = 0.2;
double Kd = 0.001;
const double max_angular_velocity = 45.0; // Maximum allowable angular velocity in deg/s

double error_prior = 0;
double integral = 0;
double setpoint = 0; // Initial setpoint

const double map_outP_min = -100;
const double map_outP_max = 100;
const int map_ESC_pwm_min = 1000;
const int map_ESC_pwm_max = 1800;


File dataFile;

double readAngularVelocity() {
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  return angVelocityData.gyro.z; // Corrected return statement
}

inline double calculatePID(double error) {
  double derivative = error - error_prior;
  integral += error;
  double outP = Kp * error + Ki * integral + Kd * derivative;
  error_prior = error;
  return outP;
}

void setup() {
  Serial.begin(115200);
  ESC.attach(23); // Attach the ESC signal cable to pin 1

  // Complete the arming procedure by setting the throttle to its lowest position
  Serial.println("Sending lowest throttle to complete arming...");
  ESC.writeMicroseconds(1000); // Adjust this value if needed; 1000us usually indicates minimum throttle
  delay(2000); // Wait a bit longer for the ESC to recognize the arming sequence
  Serial.println("ESC should be armed now.");
  
  // Your ESC should now be armed and ready

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Initialize the SD card
  if (!SD.begin(5)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
  
  // Create a new file on the SD card
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening data.txt");
    return;
  }
  Serial.println("Logging data to data.txt...");
  
  // Write header to the file
  dataFile.println("AngularVelocity,PWM,PIDOutput");
  dataFile.close();
}

void loop() {
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);  
  double angular_velocity = readAngularVelocity();
  double error = setpoint - angular_velocity;

  double outP = calculatePID(error);

  int ESC_pwm = constrain(map(outP, map_outP_min, map_outP_max, map_ESC_pwm_min, map_ESC_pwm_max), map_ESC_pwm_min, map_ESC_pwm_max);
  ESC.writeMicroseconds(ESC_pwm);

  // Open the file in append mode
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    // Write data to the file
    dataFile.print(angular_velocity);
    dataFile.print(",");
    dataFile.print(ESC_pwm);
    dataFile.print(",");
    dataFile.println(outP);
    dataFile.close();
  } else {
    Serial.println("Error opening data.txt for writing.");
  }

  Serial.print("Angular rad/s: ");
  Serial.println(angular_velocity); // Corrected printing angular velocity
  Serial.print("ESC PWM: ");
  Serial.println(ESC_pwm);
  Serial.print("PID Output");
  Serial.println(outP);
 
  delay(100);

}