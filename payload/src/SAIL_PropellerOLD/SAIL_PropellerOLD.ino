#include <Wire.h>
#include <Adafruit_BMP3XX.h> // BMP390 Library
#include <PulsePosition.h>  // Generates precise PWM signals for BLDC motors

// Pin configuration for ESC
const int PROP_ESC = 22;  // Connect the signal wire of ESC to digital pin 22

// BMP390 Sensor
Adafruit_BMP3XX bmp; // Create a BMP390 sensor object

// Descent rate setpoint (in feet per second)
const float DESCENT_RATE_SETPOINT = 20.0; // Adjust as needed

// Variables for PID control
float descent_rate_error, previous_descent_rate_error, descent_rate_integral;
float kp = 0.1; // Proportional gain
float ki = 0.01; // Integral gain
float kd = 0.01; // Derivative gain
float throttle;

void setup() {
  Serial.begin(9600);
  
  // Initialize the ESC control
  pinMode(PROP_ESC, OUTPUT);
  digitalWrite(PROP_ESC, LOW); // Start with the motor off

  // Initialize the BMP390 sensor
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read pressure from BMP390 sensor
  float currentPressure = bmp.readPressure();

  // Calculate descent rate (in feet per second)
  float currentDescentRate = (currentPressure - bmp.readPressure()) / 1000.0; // Adjust for units as needed

  // Calculate descent rate error
  descent_rate_error = DESCENT_RATE_SETPOINT - currentDescentRate;

  // Calculate control output (throttle) using PID control
  float p_term = kp * descent_rate_error;
  descent_rate_integral += descent_rate_error;
  float i_term = ki * descent_rate_integral;
  float d_term = kd * (descent_rate_error - previous_descent_rate_error);
  throttle = p_term + i_term + d_term;

  // Limit the throttle to prevent over-control
  throttle = constrain(throttle, 0, 100); // Adjust the range as needed (0-100%)

  // Send throttle signal to ESC
  int throttle_pwm = map(throttle, 0, 100, 1000, 2000); // Map throttle to ESC pulse width
  analogWrite(PROP_ESC, throttle_pwm);

  // Print debug information to the serial monitor
  Serial.print("Desired Descent Rate: ");
  Serial.print(DESCENT_RATE_SETPOINT);
  Serial.print(" ft/s, Current Descent Rate: ");
  Serial.print(currentDescentRate);
  Serial.print(" ft/s, Throttle: ");
  Serial.print(throttle);
  Serial.print("%, P-Term: ");
  Serial.print(p_term);
  Serial.print(", I-Term: ");
  Serial.print(i_term);
  Serial.print(", D-Term: ");
  Serial.print(d_term);
  Serial.println();

  // Store the current descent rate as previous for the next iteration
  previous_descent_rate_error = descent_rate_error;
}
