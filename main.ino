#include <Wire.h>
#include <TimeLib.h>
#include <Stepper.h>
#include <DS1307RTC.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>


// REAL TIME CLOCK
tmElements_t tm;

// GPS MODULE
SoftwareSerial serial_connection(12, 13); // (RX, TX)
TinyGPSPlus gps;

// COMPASS
QMC5883LCompass compass;
int x, y, z, a, b;
char direction[3];

// STEPPER MOTOR
Stepper stepperXYAxis(2048, 7, 9, 8, 10);
Stepper stepperPointer(2048, 3, 5, 4, 6);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  serial_connection.begin(9600); 
  delay(200);
  compass.init();

  spinToAlignXYAxis();
  spinToAlignPointer();
  getRTC();
  getGPS();
  getXYZ();
}

void spinToAlignXYAxis() {
  int photoresistorSensorPin = A0;
  int laserDiodePin = 11;
  int threshold = 100;
  int resistance = 0;

  stepperXYAxis.setSpeed(10);
  pinMode(laserDiodePin, OUTPUT);
  digitalWrite(laserDiodePin, HIGH);

  while (resistance > threshold) {
    resistance = analogRead(photoresistorSensorPin);
    stepperXYAxis.step(1);
  }

  if (resistance < threshold) {
    delay(200);
    resistance = analogRead(photoresistorSensorPin);
    if (resistance < threshold) {
     digitalWrite(laserDiodePin, LOW);
    } else {
      spinToAlignXYAxis();
    }
  }
}

void spinToAlignPointer() {
  int hallEffectSensorPin = 2;
  boolean state = 1;

  pinMode(hallEffectSensorPin, INPUT);
  stepperPointer.setSpeed(10);

  while (state) {
    state = digitalRead(hallEffectSensorPin);
    stepperPointer.step(1);
  }

  if (!state) {
    delay(200);
    state = digitalRead(hallEffectSensorPin);
    if (state) {
      spinToAlignPointer();
    }
  }
}

void getXYZ() {
  compass.read();
  
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  a = compass.getAzimuth();
  b = compass.getBearing(a);

  compass.getDirection(direction, a);

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
  Serial.print(direction[0]);
  Serial.print(direction[1]);
  Serial.print(direction[2]);
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

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void getGPS() {
  Serial.println("\nGPS initializing\n");
  while (true) {
    while (serial_connection.available()) {
      gps.encode(serial_connection.read());
    }
    if (gps.location.isUpdated()) {
      if (gps.satellites.value() > 1) {
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
}
