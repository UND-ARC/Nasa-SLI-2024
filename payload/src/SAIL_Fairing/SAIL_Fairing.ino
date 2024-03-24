#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_BMP3XX.h"

#define RFM95_CS                8
#define RFM95_INT               3
#define RFM95_RST               4

#define RF95_FREQ 915.0

//This is for the pyro channel
int PYRO = 13;

//Set up the LEDs
int R_LED = 19;
int G_LED = 20;
int B_LED = 21;

//This is for the BMP390 barometer
Adafruit_BMP3XX bmp;
double inHg = 29.92; // enter altimiter value
double hPa = inHg * 33.8639;
#define SEALEVELPRESSURE_HPA (hPa) // default: 1013.25

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void flashWhiteLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED (white)
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED
    delay(duration);
  }
}

void flashRedLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED (red)
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED
    delay(duration);
  }
}

void FailLED() {
    digitalWrite(R_LED, LOW);
    delay(1000);
    digitalWrite(R_LED, HIGH);
}

void PassLED() {
    digitalWrite(G_LED, LOW);
    delay(1000);
    digitalWrite(G_LED, HIGH);
}

void sendMessage(String message) {
  if (rf95.send((uint8_t *)message.c_str(), message.length())) {
    rf95.waitPacketSent();
    Serial.println("Message sent: " + message);
  } else {
    Serial.println("Message sending failed: " + message);
  }
}

void awaitSignal() {
  
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  Serial.println("Awaiting signal from Huntsville...");
  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(buf, &len)) {
      Serial.print("Received signal: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Check the received signal
      if (strcmp((char*)buf, "Go") == 0) {
        Serial.println("Go signal received.");
        // Handle Go signal
        sendMessage("Go signal received, firing at 400ft AGL");
        FireBelow400; // Check altitude for pyro trigger
        
      } else if (strcmp((char*)buf, "No-Go") == 0) {        
        Serial.println("No-Go signal received.");
        // Handle No-Go signal
      
      } else if (strcmp((char*)buf, "Force Open") == 0) {
        Serial.println("Force Open signal received.");
        // Handle Force Open signal
        sendMessage("Force Open received, releasing Fairing");
        digitalWrite(13, HIGH);
        Serial.println("Force Open, pyro firing.");
        delay(5000);
        digitalWrite(13, LOW);
        Serial.println("End of Force Open, pyro off.");          
      }
    } else {
      Serial.println("Receive failed");
    }
  }
}

void FireBelow400() {
  Serial.println("Waiting for altitude to drop below 400ft...");

  // Loop until altitude drops below 400ft
    float altitudeMeters = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Read altitude from BMP390 sensor in meters
    float altitudeFeet = altitudeMeters * 3.281; // Convert altitude to feet

    Serial.print("Altitude: ");
    Serial.print(altitudeFeet);
    Serial.println(" ft");

    if (altitudeFeet < 400.0) {
      // Altitude is below 400ft
      // Activate pyro charge
      digitalWrite(13, HIGH);
      Serial.println("Below 400ft, Pyro charge activated.");
      delay(5000);
      digitalWrite(13, LOW);
      Serial.println("Pyro charge deactivated.");      
    }
    
    delay(200); // Wait before checking altitude again
}


void setup() {

  Serial.println("Fairing Controller Startup!.");

  // Barometer Check
  Serial.println("Let's see if the BMP390 Barometer is connected. Standby...");
  delay(2000);
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find the BMP390 sensor :( Check your soldering and inspect for bad connections");
    FailLED();
    delay(3000);
    Serial.println();
    Serial.println("Proceeding with startup");
    Serial.println();
    delay(1000);
  }
  else{
    // Set up oversampling and filter initialization for BMP390
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
     
    Serial.println("Found the BMP390 sensor! Here's some data...");
    PassLED();
    
    for (int i = 0; i <= 10; i++) 
    {
    Serial.println();
    Serial.print(F("Temperature = "));
//    Serial.print(bmp.readTemperature());
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
//    Serial.print(bmp.readPressure());
 //   Serial.println(" Pa");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m"); Serial.println();
    delay(200);
  }
}

  //Test pyro channels
  Serial.println();
  Serial.println("To finish up here, let's test the pyro channel");
  Serial.println();
  pinMode(PYRO, OUTPUT);
  digitalWrite(PYRO, LOW);
  delay(1000);
  
          //////////////////////////////////////////////////////////////////////////////////////////
          //The following code MUST BE REMOVED before you connect anything to the pyro channels
          Serial.println("We'll now cycle through each channel, turning each one on for 2 seconds");
          delay(1000);
          digitalWrite(PYRO, HIGH);
          Serial.println("PYRO is on!");
          flashRedLED(5,200);
          delay(2000);
          digitalWrite(PYRO, LOW);
          Serial.println("Pyro is off");
          digitalWrite(R_LED, HIGH);
          delay(2000);
          //////////////////////////////////////////////////////////////////////////////////////////
          
  Serial.println();
  Serial.println("Done with the pyro channel testing");
  Serial.println();
  Serial.println();
  delay(1000);
  Serial.println("Continuing with Fairing Startup!");
  delay(1000);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  /*
  Serial.begin(9600);
  while (!Serial) delay(1);
  */
  //set system led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  // We're using a common anode LED, so we write LOW to turn it on, and HIGH to turn it off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.println("Radio Check!");
  delay(1000);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    FailLED();
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  PassLED();
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    FailLED();
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
}

void loop() {
  
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(buf, &len)) {
      Serial.print("Fairing has received message: ");
      Serial.println((char*)buf);
      
      // Check if the received message is "Hello Fairing"
      if (strcmp((char*)buf, "Hello Fairing") == 0) {
        // Reply back "And hello to you, Huntsville"
        sendMessage("And hello to you, Huntsville. Awaiting signal.");
        awaitSignal();
      }
      
    } else {
      Serial.println("Receive failed");
    }
      if (awaitSignal) {
        // Flash white LED while waiting for signal
        flashWhiteLED(1, 100); // Flash once every 100 milliseconds
      } else {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
      }
  }
}
