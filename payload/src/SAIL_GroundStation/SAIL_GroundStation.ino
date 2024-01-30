#include <SPI.h>
#include <RH_RF95.h>

// Define the RFM95 module pins
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Define the arming switch pin
#define ARMING_SWITCH_PIN 3 // Change to your arming switch's pin
// Define the push-button switch pin
#define BUTTON_PIN 4 // Change to your button's pin
// Define the LED pins
#define ARM_LED_PIN 5 // LED for arming status (Solid when armed)
#define GO_LED_PIN 6  // LED for "go" signal (Flashing when "go" signal sent)
#define FEEDBACK_LED_PIN 7 // LED for feedback (Solid when feedback is received)

// Create an RF95 object
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Set the radio frequency (in MHz) and transmit power (in dBm)
float frequencyMHz = 915.0; // Change this to 915MHz
int transmitPowerdBm = 20;  // Change this to your desired power level

bool isArmed = false;
bool goSignalSent = false;

void setup() {
  Serial.begin(9600);

  // Initialize the LEDs
  pinMode(ARM_LED_PIN, OUTPUT);
  pinMode(GO_LED_PIN, OUTPUT);
  pinMode(FEEDBACK_LED_PIN, OUTPUT);

  // Initialize the LEDs to their initial state
  digitalWrite(ARM_LED_PIN, LOW);
  digitalWrite(GO_LED_PIN, LOW);
  digitalWrite(FEEDBACK_LED_PIN, LOW);

  // Initialize the RF95 module
  if (!rf95.init()) {
    Serial.println("RFM95 module initialization failed.");
    while (1);
  }

  // Set the frequency and power
  rf95.setFrequency(frequencyMHz);
  rf95.setTxPower(transmitPowerdBm);

  // Initialize the arming switch pin
  pinMode(ARMING_SWITCH_PIN, INPUT_PULLUP);

  // Initialize the push-button switch pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // Check the arming switch state
  bool armingSwitchState = digitalRead(ARMING_SWITCH_PIN);

  if (armingSwitchState) {
    // Arming switch is active
    isArmed = true;
    digitalWrite(ARM_LED_PIN, HIGH); // Turn on the arming LED when armed
  } else {
    // Arming switch is not active
    isArmed = false;
    digitalWrite(ARM_LED_PIN, LOW); // Turn off the arming LED when not armed
  }

  // Check if the system is armed before sending a message
  if (isArmed) {
    // Enter your logic for determining when to send a "go" signal here.
    // For this example, we'll use the push-button switch to trigger a "go" signal.

    // Read the button state (you can use a physical button or a digital input)
    int buttonState = digitalRead(BUTTON_PIN);
    
    if (buttonState == LOW && !goSignalSent) {
      // Button is pressed and "go" signal not sent yet
      sendMessage("GO");
      goSignalSent = true;
      // Turn on the "go" LED (flashing)
      flashLED(GO_LED_PIN);
    }
  }

  // Check for feedback from the payload
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      buf[len] = '\0'; // Null-terminate the received message
      Serial.print("Received feedback: ");
      Serial.println((char*)buf);
      // Turn on the feedback LED (solid)
      digitalWrite(FEEDBACK_LED_PIN, HIGH);
    }
  }
}

void sendMessage(const char* message) {
  Serial.print("Sending message: ");
  Serial.println(message);

  // Send the message if the system is armed
  if (isArmed) {
    rf95.send((uint8_t*)message, strlen(message));
    rf95.waitPacketSent();
    Serial.println("Message sent successfully!");
  } else {
    Serial.println("System is not armed. Message not sent.");
  }
}

void flashLED(int pin) {
  // Flash an LED connected to the specified pin
  for (int i = 0; i < 5; i++) { // Flash 5 times
    digitalWrite(pin, HIGH);
    delay(500); // On for 500ms
    digitalWrite(pin, LOW);
    delay(500); // Off for 500ms
  }
}
