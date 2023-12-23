
//Thanks for downloading the Blip test code, and being a patron of BPS.space!
//
//You can run this code on the regular Blip R1 computer to see if everything
//is hooked up correctly. You're welcome to modify or change this for your own
//non-commercial designs, but please do not distribute or publish it. - Joe Barnard


#include <Wire.h>
#include <SD.h>
#include <SPI.h>
//#include <Adafruit_BMP280.h> 
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <SPIFlash.h>
#include <Servo.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "Adafruit_BMP3XX.h"
#include <RH_RF95.h>


//First, we'll set up the LEDs and buzzer
//define system led pins
int R_LED = 9;
int G_LED = 3;
int B_LED = 6;
int Buzzer = 4;


//This is for the BMP280 barometer
//Adafruit_BMP280 bmp;


//This is for the MPU6050 IMU
//MPU6050 accelgyro;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
//#define OUTPUT_READABLE_ACCELGYRO

// For the BNO055 IMU, Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)

//This is for the SD card
Sd2Card card;
SdVolume volume;
SdFile root;
const int SDchipSelect = 5;

//This is for the SPI Flash chip
#define CHIPSIZE MB64 // 128?
SPIFlash flash(1);
uint8_t pageBuffer[256];
String serialCommand;
char printBuffer[128];
uint16_t page;
uint8_t offset, dataByte;
uint16_t dataInt;
String inputString, outputString;

//This is for the pyro channel
int Pyro1 = 21;

//This is for the TVC servos
//int TVCXpin = 3;
//int TVCYpin = 4;
//Servo TVCXservo;
//Servo TVCYservo;

// This is for the RFM95 Radio
#define RFM95_CS 10
#define RFM95_RST 2
//#define RFM95_INT 2
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission

/**************************************************************************/
/*
    Displays some basic information on the BNO055 sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the BNO055 sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display BNO055 sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */


/* From Datasheet
3.10.1 Accelerometer Calibration
- Place the device in 6 different stable positions for a period of few seconds to allow the
accelerometer to calibrate.
- Make sure that there is slow movement between 2 stable positions
- The 6 stable positions could be in any direction, but make sure that the device is lying at
least once perpendicular to the x, y and z axis.
- The register CALIB_STAT can be read to see the calibration status of the accelerometer.

3.10.2 Gyroscope Calibration
- Place the device in a single stable position for a period of few seconds to allow the
gyroscope to calibrate
- The register CALIB_STAT can be read to see the calibration status of the gyroscope.

3.10.3 Magnetometer Calibration
Magnetometer in general are susceptible to both hard-iron and soft-iron distortions, but
majority of the cases are rather due to the former. And the steps mentioned below are to
calibrate the magnetometer for hard-iron distortions.
Nevertheless certain precautions need to be taken into account during the positioning of
the sensor in the PCB which is described in our HSMI (Handling, Soldering and Mounting
Instructions) application note to avoid unnecessary magnetic influences. */
  
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void setup() {
  while(!Serial){
    //wait for the serial port to be available, comment this out before running on battery power!!!!
  }
  Serial.println();Serial.println();
  Serial.println("Hey! I'm the Blip flight computer! Let's get started.");
  Serial.println();Serial.println();
  
  //set system led pins as outputs
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  // We're using a common anode LED, so we write LOW to turn it on, and HIGH to turn it off
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);
  digitalWrite(B_LED, HIGH);

  //Now Barometer
  //Serial.println("First, let's see if the BMP280 Barometer is connected. Standby...");
  Serial.println("First, let's see if the BMP390 Barometer is connected. Standby...");
  delay(1000);
  if (!bmp.begin()) {
    Serial.println("Could not find the BMP280 sensor :( Check your soldering and inspect for bad connections");
    delay(5000);
    Serial.println("Proceeding with the rest of the startup process");
  }
  else{
    //Configure the barometer
 //   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
 //                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
   //               Adafruit_BMP280::FILTER_X16,      /* Filtering. */
   //               Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

// Set up oversampling and filter initialization for BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
   
    Serial.println("Found the BMP280 sensor! Here's some data...");
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
//  Serial.print(bmp.readAltitude(1013.25));
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m"); Serial.println();
    delay(1000);
  }



 /********* 
  //Now the IMU
  //Serial.println("Now we'll look for the the MPU6050 IMU. Standby...");
 
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "Found it! MPU6050 connection successful." : "MPU6050 connection failed :(");
  delay(1000);
  Serial.println();
  Serial.println("Here's a little bit of data!");
  for (int i = 0; i <= 20; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
    delay(20);
  }
  Serial.println("Great! Onward to the SD card");
  delay(1000);
*******/

  Serial.println("Now we'll look for the the BNO055 IMU. Standby...");
  bno.begin();
  Serial.println(bno.begin() ? "Found it! BNO055 connection successful." : "BNO055 connection failed :(");
  delay(1000);
  Serial.println();

  if(bno.begin())
  {
    Serial.println("Here's a little bit of IMU data!");
    /* Display some basic information on this sensor */
    displaySensorDetails();
     /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

     /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
    
    for (int i = 0; i <= 50; i++) 
    {
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      
      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");

      /*
      // Quaternion data
      imu::Quaternion quat = bno.getQuat();
      Serial.print("qW: ");
      Serial.print(quat.w(), 4);
      Serial.print(" qX: ");
      Serial.print(quat.x(), 4);
      Serial.print(" qY: ");
      Serial.print(quat.y(), 4);
      Serial.print(" qZ: ");
      Serial.print(quat.z(), 4);
      Serial.print("\t\t");
      */

      /* Display calibration status for each sensor. */
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      Serial.print("CALIBRATION: Sys=");
      Serial.print(system, DEC);
      Serial.print(" Gyro=");
      Serial.print(gyro, DEC);
      Serial.print(" Accel=");
      Serial.print(accel, DEC);
      Serial.print(" Mag=");
      Serial.println(mag, DEC);

      /* Wait the specified delay before requesting nex data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
         
      /* Optional: Display calibration status */
      displayCalStatus();
    
      /* Optional: Display sensor status (debug only) */
      //displaySensorStatus();

      /* New line for the next sample */
    //  Serial.println("");
    
    }
  }

    /********** END OF IMU *************/

  //Now the SD card
  Serial.print("Looking for an SD card. Standby...");
  if (!card.init(SPI_HALF_SPEED, SDchipSelect)) {
    Serial.println("Looks like there's no card here! Trying checking the following:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your soldering correct?");
    Serial.println("* is the chipselect pin correct?");
    Serial.println();
    delay(4000);
    Serial.println("We'll continue with the startup without the SD card now :(");
  } else {
    Serial.println("Found an SD card! Here's some information about it.");
    delay(1000);
    Serial.println();
    Serial.print("Card type:         ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
      default:
        Serial.println("Unknown");
    }
    if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());
  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);
  Serial.println();
  Serial.println();
  delay(1000);
  }

  /********** END OF SD CARD *********/
  
  /********** Now the FLASH **********/
  Serial.println("Looking for the SPI flash chip. Standby...");
  if (flash.begin(CHIPSIZE)){
    delay(500);
    Serial.println("Great, looks like there's one here!");
    Serial.println("Here's some info about it...");
    Serial.println();
    delay(1000);
    uint8_t b1, b2, b3;
    uint32_t JEDEC = flash.getJEDECID();
    b1 = (JEDEC >> 16);
    b2 = (JEDEC >> 8);
    b3 = (JEDEC >> 0);
    sprintf(printBuffer, "Manufacturer ID: %02xh\nMemory Type: %02xh\nCapacity: %02xh", b1, b2, b3);
    Serial.println(printBuffer);
    clearprintBuffer();
    sprintf(printBuffer, "JEDEC ID: %04lxh", JEDEC);
    Serial.println(printBuffer);
    Serial.println();
    Serial.println();
    delay(1000);
  }
  else{
    delay(500);
    Serial.println();
    Serial.println("Hmmm, looks like there's no flash chip here. Try checking the following:");
    Serial.println(" - Is the chip soldered on in the correct orientation?");
    Serial.println(" - Is the correct chip select pin defined in the SPIFlash constructor?");
    Serial.println(" - Is the correct CHIPSIZE defined?");
    delay(5000);
    Serial.println();
    Serial.println("Proceding with the rest of the startup process");
    delay(1000);
    Serial.println();
  }

  /***** END OF FLASH TEST *******/
  
  /******* Now we'll test the radio ********/
  Serial.println("Arduino LoRa TX Test!");
   
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
      Serial.println("LoRa radio init failed");
      while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  delay(1000);


  /******* END OF RADIO TEST *******/
  
 
  
  //Now we'll test the buzzer and the LEDs
  Serial.println("Cool beans! Let's see if the buzzer works.");
  delay(500);
  tone(Buzzer, 2000); delay(50); noTone(Buzzer); delay(75);
  tone(Buzzer, 2000); delay(50); noTone(Buzzer); delay(200);
  tone(Buzzer, 1000); delay(50); noTone(Buzzer); delay(75);
  tone(Buzzer, 1000); delay(50); noTone(Buzzer); delay(400);

  tone(Buzzer, 1319); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 1760); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 2217); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 2637); delay(100); noTone(Buzzer);
  delay(100);
  tone(Buzzer, 2217); delay(50); noTone(Buzzer);
  delay(50);
  tone(Buzzer, 2637); delay(200); noTone(Buzzer);
  delay(400);
  Serial.println("Charge!");
  int slide = 5000;
  for (int i = 0; i <= 120; i++) {
    tone(Buzzer, slide); delay(10); noTone(Buzzer);
    slide = slide - 40;
  }
  noTone(Buzzer);

  Serial.println("Now let's test the LEDs");
  digitalWrite(R_LED, LOW);
  delay(500);
  digitalWrite(R_LED, HIGH);
  delay(500);

  digitalWrite(G_LED, LOW);
  delay(500);
  digitalWrite(G_LED, HIGH);
  delay(500);

  digitalWrite(B_LED, LOW);
  delay(500);
  digitalWrite(B_LED, HIGH);
  delay(1000);
  Serial.println("Great, done!");
  delay(1000);


  //Test pyro channels
  Serial.println();
  Serial.println("To finish up here, let's test all the pyro channels");
  Serial.println();
  pinMode(Pyro1, OUTPUT);
  pinMode(Pyro2, OUTPUT);
  pinMode(Pyro3, OUTPUT);
  pinMode(Pyro4, OUTPUT);
  digitalWrite(Pyro1, LOW);
  digitalWrite(Pyro2, LOW);
  digitalWrite(Pyro3, LOW);
  digitalWrite(Pyro4, LOW);
  delay(1000);

          //The following code MUST BE REMOVED before you connect anything to the pyro channels
          Serial.println("We'll now cycle through each channel, turning each one on for 2 seconds");
          delay(1000);
          digitalWrite(Pyro1, HIGH);
          Serial.println("Pyro 1 is on!");
          delay(2000);
          digitalWrite(Pyro1, LOW);
          Serial.println("Pyro 1 is off");
          delay(1000);
          digitalWrite(Pyro2, HIGH);
          Serial.println("Pyro 2 is on!");
          delay(2000);
          digitalWrite(Pyro2, LOW);
          Serial.println("Pyro 2 is off");
          delay(1000);
          digitalWrite(Pyro3, HIGH);
          Serial.println("Pyro 3 is on!");
          delay(2000);
          digitalWrite(Pyro3, LOW);
          Serial.println("Pyro 3 is off");
          delay(1000);
          digitalWrite(Pyro4, HIGH);
          Serial.println("Pyro 4 is on!");
          delay(2000);
          digitalWrite(Pyro4, LOW);
          Serial.println("Pyro 4 is off");
          delay(1000);
  
  Serial.println();
  Serial.println("Done with the pyro channel testing");
  Serial.println();
  Serial.println();
  delay(500);
  
  //Now we'll wiggle the TVC servos
  Serial.println("Now time to check the TVC servos");
  delay(500);
  Serial.println("We'll set each servo to 90 degrees and wiggle ±30 degrees");
  delay(500);
  Serial.println("First, the X servo");
  delay(500);
  TVCXservo.attach(TVCXpin);
  TVCXservo.write(90); delay(500);
  TVCXservo.write(120); delay(500);
  TVCXservo.write(90); delay(500);
  TVCXservo.write(60); delay(500);
  TVCXservo.write(90); delay(500);
  Serial.println("Now, the Y servo");
  delay(500);
  TVCYservo.attach(TVCYpin);
  TVCYservo.write(90); delay(500);
  TVCYservo.write(120); delay(500);
  TVCYservo.write(90); delay(500);
  TVCYservo.write(60); delay(500);
  TVCYservo.write(90); delay(500);
  Serial.println("All done! That concludes the startup process, now on to the loop function.");
  delay(1000);
}

void loop() {

  digitalWrite(R_LED, LOW);
  delay(1000);
  digitalWrite(R_LED, HIGH);
  delay(1000);

  digitalWrite(G_LED, LOW);
  delay(1000);
  digitalWrite(G_LED, HIGH);
  delay(1000);

  digitalWrite(B_LED, LOW);
  delay(1000);
  digitalWrite(B_LED, HIGH);
  delay(1000);

}


//The MPU6050 setup code is from the I2Cdev lib under the MIT licence
//Copyright (c) 2011 Jeff Rowberg
/*
Permission is hereby granted, free of charge, to any person obtaining a copy
of the MPU6050 portion and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

This does not invalidate the header comments regarding the entire program,
but refers only to the IMU portion of the code.
*/

void clearprintBuffer()
{
  for (uint8_t i = 0; i < 128; i++) {
    printBuffer[i] = 0;
  }
}
