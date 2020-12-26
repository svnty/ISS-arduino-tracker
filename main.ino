#include <CheapStepper.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>

// REAL TIME CLOCK
tmElements_t tm;

// GPS MODULE 
SoftwareSerial serial_connection(12, 13); // RX=pin 12, TX=pin 13
TinyGPSPlus gps; //This is the GPS object with the NMEA data

// COMPASS
QMC5883LCompass compass;

// STEPPER MOTOR
//CheapStepper stepperOne (7,8,9,10);
//CheapStepper stepperTwo (2,3,4,5);
//bool stepperOneClockwise = true;
//bool stepperTwoClockwise = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) ; // wait for serial
  serial_connection.begin(9600);//This opens up communications to the GPS
  delay(200);

  compass.init();

  getRTC();
  getGPS();
  getXYZ();

  /*
  stepperOne.setRpm(16);
  stepperTwo.setRpm(16);
  Serial.begin(9600);
  stepperOne.newMoveToDegree(stepperOneClockwise, 360);
  */
}

void getXYZ() {
  int x, y, z, a, b;
  char myArray[3];
  
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  
  a = compass.getAzimuth();
  
  b = compass.getBearing(a);

  compass.getDirection(myArray, a);
  
  
  Serial.print("X: ");
  Serial.print(x);

  Serial.print(" Y: ");
  Serial.print(y);

  Serial.print(" Z: ");
  Serial.print(z);

  Serial.print(" Azimuth: ");
  Serial.print(a);

  Serial.print(" Bearing: ");
  Serial.print(b);

  Serial.print(" Direction: ");
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);
}

void getRTC() {
    if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (DD/MM/YYYY) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
  }
}

void getGPS() {
  Serial.println("\nGPS initializing\n");
  while (true) {
    while(serial_connection.available()) {
      // This feeds the serial NMEA data into the library one char at a time
      gps.encode(serial_connection.read());
    }
    // Get the latest info from the gps object which it derived from the data sent by the GPS unit
    if(gps.location.isUpdated()) {
      // Confirm accuracy
      if (gps.satellites.value() > 1){
        Serial.println("Satellite Count:");
        Serial.println(gps.satellites.value());
        Serial.println("Latitude:");
        Serial.println(gps.location.lat(), 6);
        Serial.println("Longitude:");
        Serial.println(gps.location.lng(), 6);
        Serial.println("Speed MPH:");
        Serial.println(gps.speed.mph());
        Serial.println("Altitude Feet:");
        Serial.println(gps.altitude.feet());
        Serial.println("");
        break; 
      }
    }
  }
}

void loop() {
  /*
  stepperOne.run();
  stepperTwo.run();
  int stepperOneStepsLeft = stepperOne.getStepsLeft();
  int stepperTwoStepsLeft = stepperTwo.getStepsLeft();

  if (stepperOneStepsLeft == 0){
    // stepperOneClockwise = !stepperOneClockwise;
    stepperOne.newMoveDegrees(stepperOneClockwise, 360);
  }

  if (stepperTwoStepsLeft == 0) {
    // stepperTwoClockwise = !stepperTwoClockwise;
    stepperTwo.newMoveDegrees(stepperTwoClockwise, 180);
  }
  */
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
