#include <Wire.h>
#include <TimeLib.h>
#include <Stepper.h>
#include <DS1307RTC.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>
#include "sgp4ext.h"
#include "sgp4unit.h"
#include "sgp4io.h"
#include "sgp4coord.h"

// REAL TIME CLOCK
tmElements_t tm;

// GPS MODULE
SoftwareSerial serial_connection(12, 13); // (RX, TX)
TinyGPSPlus gps;

// COMPASS
QMC5883LCompass compass;

// STEPPER MOTOR
Stepper stepperXYAxis(2048, 7, 9, 8, 10);
Stepper stepperPointer(2048, 3, 5, 4, 6);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  serial_connection.begin(9600);
  delay(200);

  compass.init();
  delay(200);

  Serial.println("\nInitalizing... \n");
  //spinToAlignXYAxis();
  //spinToAlignPointer();
  getRTC();
  delay(200);
  getGPS();
  delay(200);
  getXYZ();
  delay(200);

  Serial.println(tm.Hour);
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
  int x, y, z, a, b;
  char direction[3];
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  a = compass.getAzimuth();
  b = compass.getBearing(a);
  Serial.println("xyz");

  compass.getDirection(direction, a);
  Serial.println("direction");

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
      Serial.println("DS1307 read error! Please check the circuitry.");
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
        Serial.println("Altitude KiloMeters:");
        Serial.println(gps.altitude.kilometers());
        Serial.println("");
        break;
      }
    }
  }
}

void loop() {
  //SET UP SOME VARIABLES
  double ro[3];
  double vo[3];
  double recef[3];
  double vecef[3];
  char typerun, typeinput, opsmode;
  gravconsttype  whichconst;

  double sec, secC, jd, jdC, tsince, startmfe, stopmfe, deltamin;
  double tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2;
  double latlongh[3]; //lat, long in rad, h in km above ellipsoid
  double siteLat, siteLon, siteAlt, siteLatRad, siteLonRad;
  double razel[3];
  double razelrates[3];
  int  year; int mon; int day; int hr; int min;
  int yearC; int monC; int dayC; int hrC; int minC;
  typedef char str3[4];
  str3 monstr[13];
  elsetrec satrec;
  double steps_per_degree = 2048/360; //Stepper motor steps per degree azimuth
  float elevation;
  
  //VARIABLES FOR STEPPER CALCULATIONS
  float azimuth; //-180 to 0 to 180
  float prevAzimuth;
  float cAzimuth; //From 0 to 359.99
  float prevcAzimuth;
  bool stepperRelative = 0; //Has the stepper direction been initialized?
  float azimuthDatum = 0;
  int stepsFromDatum = 0;
  int stepsNext = 0;
  int dirNext = 1;
  int totalSteps = 0;
  int prevDir = 3; //Initialize at 3 to indicate that there is no previous direction yet (you can't have a "previous direction" until the third step)
  double azError = 0;

  //SET VARIABLES
  opsmode = 'i';
  typerun = 'c';
  typeinput = 'e';
  whichconst = wgs72;
  getgravconst(whichconst, tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2);
  strcpy(monstr[1], "Jan");
  strcpy(monstr[2], "Feb");
  strcpy(monstr[3], "Mar");
  strcpy(monstr[4], "Apr");
  strcpy(monstr[5], "May");
  strcpy(monstr[6], "Jun");
  strcpy(monstr[7], "Jul");
  strcpy(monstr[8], "Aug");
  strcpy(monstr[9], "Sep");
  strcpy(monstr[10], "Oct");
  strcpy(monstr[11], "Nov");
  strcpy(monstr[12], "Dec");

  //ENTER TWO-LINE ELEMENT HERE
  char longstr1[] = "1 25544U 98067A   15239.40934558  .00012538  00000-0  18683-3 0  9996";
  char longstr2[] = "2 25544  51.6452  88.4122 0001595  95.9665 324.8493 15.55497522959124";

  siteLat = gps.location.lat();
  siteLon = gps.location.lng();
  siteAlt = gps.altitude.kilometers();
  siteLatRad = siteLat * pi / 180.0;
  siteLonRad = siteLon * pi / 180.0;

  // INITIALIZE SATELLITE TRACKING    
  twoline2rv(longstr1, longstr2, typerun, typeinput, opsmode, whichconst, startmfe, stopmfe, deltamin, satrec);
  // Call propogator to get initial state vector value
  sgp4(whichconst, satrec, 0.0, ro, vo);
  jd = satrec.jdsatepoch;    
  invjday(jd, year, mon, day, hr, min, sec);
  //jdC = getJulianFromUnix();
  invjday( jdC, yearC, monC, dayC, hrC, minC, secC);
}
