// POWER ESC OFF BEFORE UPLOADING CODE AND POWERING ON TEENSY

#include <Servo.h>

Servo esc; // Create a servo object to control the ESC

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  
  esc.attach(22); // Attach the ESC signal cable to pin 1

  // Start with throttle at maximum to signal the beginning of the arming procedure
  Serial.println("Sending lowest throttle for arming...");
  esc.writeMicroseconds(0); // Adjust this value if needed; 2000us usually indicates maximum throttle
  delay(2000); // Short delay to ensure the signal is stable and power on ESC
  Serial.println("Power on the ESC now, then wait for the next instruction.");
 
  // IMPORTANT: At this point, you should manually power on the ESC
  
  delay(5000); // Wait 5 seconds for the user to power on the ESC and for it to initialize
  
  // Complete the arming procedure by setting the throttle to its lowest position
  Serial.println("Sending lowest throttle to complete arming...");
  esc.writeMicroseconds(1000); // Adjust this value if needed; 1000us usually indicates minimum throttle
  delay(2000); // Wait a bit longer for the ESC to recognize the arming sequence
  Serial.println("ESC should be armed now.");
  
  // Your ESC should now be armed and ready
}

void loop() {
  // Example: Run motor at a low throttle setting for 1 second
  Serial.println("Running motor at low throttle...");
  esc.writeMicroseconds(1100); // Adjust this value based on your ESC's requirements
  delay(1000); // Motor runs at low throttle for 1 second
  
  // Stop the motor (or set to minimum throttle)
  Serial.println("Stopping motor...");
  esc.writeMicroseconds(1000); // Send minimum throttle to stop the motor
  delay(4000); // Wait for 4 seconds before the next loop iteration (to provide clear separation in debug messages)
}
