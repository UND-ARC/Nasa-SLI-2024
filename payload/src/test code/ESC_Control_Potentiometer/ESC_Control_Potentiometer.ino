#include <PulsePosition.h>

/*
 * 
 * #include <Servo.h>

byte servoPin = 9; // signal pin for the ESC.
byte potentiometerPin = A0; // analog input pin for the potentiometer.
Servo servo;

void setup() {
servo.attach(servoPin);
servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.

delay(7000); // delay to allow the ESC to recognize the stopped signal.
}
*/

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(A0, INPUT); // Assuming the potentiometer is connected to analog pin A0
  analogWriteFrequency(1, 250);
  analogWriteResolution(12);
  delay(250);

  // Arm the ESC by sending a neutral PWM signal (1500µs)
  analogWrite(1, 1500); // Send a 1500µs PWM signal
  delay(1000); // Wait for 1 second to allow the ESC to recognize the signal
}

/*
void loop() {
  read_receiver();
  InputThrottle=ReceiverValue[2];
  analogWrite(1,1.024*InputThrottle);
}
*/

void loop() {
  int potValue = analogRead(A0); // Read the potentiometer value (0-1023)
  float normalizedValue = map(potValue, 0, 1023, 0, 1.0); // Map the value to a range between 0 and 1.0
  analogWrite(1, normalizedValue * 4095); // Control motor speed based on the potentiometer value
}
