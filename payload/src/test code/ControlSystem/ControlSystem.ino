// First Draft Code for control PID Control of Propeller and reaction wheel
// Chat GPT is very useful!


#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Define the PID constants for descent rate control
double Kp_descent = 0.1;  // Proportional gain for descent rate
double Ki_descent = 0.0;  // Integral gain for descent rate
double Kd_descent = 0.0;  // Derivative gain for descent rate

// Define the PID constants for roll rate control
double Kp_roll = 0.2;  // Proportional gain for roll rate
double Ki_roll = 0.0;  // Integral gain for roll rate
double Kd_roll = 0.0;  // Derivative gain for roll rate

// Define variables for PID control
double setpoint_descent = 15.0; // Target descent rate in ft/s
double input_descent;           // Current descent rate
double output_descent;          // PID output for descent rate
double integral_descent = 0;
double last_error_descent = 0;

double setpoint_roll = 0.0; // Target roll rate in deg/s
double input_roll;          // Current roll rate
double output_roll;         // PID output for roll rate
double integral_roll = 0;
double last_error_roll = 0;

// Define PWM control pins for the ESCs
int propellerMotorPin = 9;   // Pin for the propeller motor
int reactionWheelMotorPin = 10; // Pin for the reaction wheel motor

// Define objects for sensors
Adafruit_BNO055 imu = Adafruit_BNO055();
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

void setup() {
  // Initialize sensors
  imu.begin();
  bmp.begin();
  
  // Initialize PWM control for the ESCs
  pinMode(propellerMotorPin, OUTPUT);
  pinMode(reactionWheelMotorPin, OUTPUT);
  // Set up PWM frequency and other parameters if required
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the IMU to get current descent rate and roll rate
  sensors_event_t event;
  imu.getEvent(&event);
  double current_descent_rate = -event.acceleration.z * 32.174; // Convert m/s^2 to ft/s^2
  double current_roll_rate = event.gyro.x * 57.2958; // Convert rad/s to deg/s

  // Read BMP390 for altitude and pressure data
  double altitude = bmp.readAltitude(1013.25); // Adjust for sea level pressure
  // Calculate current descent rate based on altitude change over time
  double delta_altitude = altitude - last_altitude;
  double delta_time = (millis() - last_time) / 1000.0; // Convert to seconds
  input_descent = delta_altitude / delta_time;
  last_altitude = altitude;
  last_time = millis();
  
  // Calculate the error for descent rate control
  double error_descent = setpoint_descent - input_descent;
  
  // Calculate the PID terms for descent rate control
  integral_descent += error_descent * delta_time;
  double derivative_descent = (error_descent - last_error_descent) / delta_time;
  
  // Calculate the PID output for descent rate control
  output_descent = Kp_descent * error_descent + Ki_descent * integral_descent + Kd_descent * derivative_descent;
  
  // Map the PID output for descent rate control to the PWM range (assuming the ESC uses 1000-2000 microseconds)
  int propellerMotorSpeed = map(output_descent, -15.0, 15.0, 1000, 2000);

  // Calculate the error for roll rate control
  double error_roll = setpoint_roll - current_roll_rate;
  
  // Calculate the PID terms for roll rate control
  integral_roll += error_roll * delta_time;
  double derivative_roll = (error_roll - last_error_roll) / delta_time;
  
  // Calculate the PID output for roll rate control
  output_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll;
  
  // Map the PID output for roll rate control to the PWM range (assuming the ESC uses 1000-2000 microseconds)
  int reactionWheelMotorSpeed = map(output_roll, -180.0, 180.0, 1000, 2000);

  // Send PWM signals to control the propeller and reaction wheel
  analogWrite(propellerMotorPin, propellerMotorSpeed);
  analogWrite(reactionWheelMotorPin, reactionWheelMotorSpeed);

  // Update last errors for the next iteration
  last_error_descent = error_descent;
  last_error_roll = error_roll;

  // Print debugging information
  Serial.print("Descent Rate: ");
  Serial.println(input_descent);
  Serial.print("Roll Rate: ");
  Serial.println(current_roll_rate);
  Serial.print("Propeller Output: ");
  Serial.println(propellerMotorSpeed);
  Serial.print("Reaction Wheel Output: ");
  Serial.println(reactionWheelMotorSpeed);

  // Delay for a short period before the next iteration
  delay(100); // Adjust as needed
}
