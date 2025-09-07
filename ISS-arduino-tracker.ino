#include <Arduino.h>
#include <AS5600.h>
#include <LiquidCrystal_I2C.h>
#include <QMC5883LCompass.h>
#include "sgp4unit.h"
#include "sgp4coord.h"
#include "sgp4ext.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <TMCStepper.h>
#include <WiFiEspAT.h>
#include <Wire.h>

// -------- SYSTEM TOGGLES --------
static const bool ENABLE_WIFI = false;
static const bool ENABLE_LOG = false;

// -------- CONSTANTS --------
static const int AZIMUTH_DIR_PIN = 22;
static const int AZIMUTH_STEP_PIN = 23;
static const int ELEVATION_DIR_PIN = 24;
static const int ELEVATION_STEP_PIN = 25;
static const int GPS_RX_PIN = 11;
static const float TMC2209_RESISTANCE = 0.11;
static const char *WIFI_SSID = "Jakes iPhone", *WIFI_PASSWORD = "izeh5zemxa05r";
static const double deg2rad = pi / 180.0;
static const double rad2deg = 180.0 / pi;

// -------- VARIABLES --------
static unsigned long lastRecordedTime = 0;
static unsigned long currentTime = 0;
static uint32_t numberOfSatellites = 0;
static bool foundISSbyWifi = false;
static double observerLatitude = 0.0;
static double observerLongitude = 0.0;
static double observerAltitude = 0.0;
static char tle_line1[70];    // TLE storage
static char tle_line2[70];    // TLE storage
static int year, month, day, hour, minute, second;
static bool ISSIsVisible = true;

// -------- SERIALS --------
#define AZIMUTH_SERIAL Serial1
#define ELEVATION_SERIAL Serial2
#define WIFI_SERIAL Serial3
static SoftwareSerial GPS_SERIAL(GPS_RX_PIN, -1);

// -------- OBJECTS --------
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);
TMC2209Stepper azimuthDriver(&AZIMUTH_SERIAL, TMC2209_RESISTANCE, 0);
TMC2209Stepper elevationDriver(&ELEVATION_SERIAL, TMC2209_RESISTANCE, 1);
AS5600 as5600;
QMC5883LCompass compass;
elsetrec satrec;

// -------- FUNCTIONS --------
void lcdClear() {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void lcdSetFirstLine(String firstLineText) {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(firstLineText);
}

void lcdSetSecondLine(String secondLineText) {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(secondLineText);
}

void parseISSTLE(const char* line1, const char* line2, elsetrec& satrec) {
  // Parse TLE manually without the problematic preprocessing
  char temp[20];
  
  // Line 1 parsing
  // Epoch year (cols 19-20)
  strncpy(temp, line1 + 18, 2);
  temp[2] = '\0';
  satrec.epochyr = atoi(temp);
  
  // Epoch days (cols 21-32)
  strncpy(temp, line1 + 20, 12);
  temp[12] = '\0';
  satrec.epochdays = atof(temp);
  
  // Mean motion first derivative (cols 34-43)
  strncpy(temp, line1 + 33, 10);
  temp[10] = '\0';
  satrec.ndot = atof(temp);
  
  // BSTAR (cols 54-61) - need to handle scientific notation
  strncpy(temp, line1 + 53, 8);
  temp[8] = '\0';
  // Convert BSTAR from packed format (e.g., "23296-3" means 0.23296e-3)
  if (strlen(temp) >= 6) {
    char mantissa[6], exponent[3];
    strncpy(mantissa, temp, 5);
    mantissa[5] = '\0';
    strncpy(exponent, temp + 6, 2);
    exponent[2] = '\0';
    double mant = atof(mantissa) / 100000.0; // Add decimal point
    int exp = atoi(exponent);
    if (temp[5] == '-') exp = -exp;
    satrec.bstar = mant * pow(10.0, exp);
  } else {
    satrec.bstar = 0.0;
  }
  
  // Line 2 parsing
  // Inclination (cols 9-16)
  strncpy(temp, line2 + 8, 8);
  temp[8] = '\0';
  satrec.inclo = atof(temp) * (3.14159265359 / 180.0); // Convert to radians
  
  // RAAN (cols 18-25)
  strncpy(temp, line2 + 17, 8);
  temp[8] = '\0';
  satrec.nodeo = atof(temp) * (3.14159265359 / 180.0); // Convert to radians
  
  // Eccentricity (cols 27-33) - no decimal point in TLE
  strncpy(temp, line2 + 26, 7);
  temp[7] = '\0';
  satrec.ecco = atof(temp) / 10000000.0; // Add decimal point (0.0004212)
  
  // Argument of perigee (cols 35-42)
  strncpy(temp, line2 + 34, 8);
  temp[8] = '\0';
  satrec.argpo = atof(temp) * (3.14159265359 / 180.0); // Convert to radians
  
  // Mean anomaly (cols 44-51)
  strncpy(temp, line2 + 43, 8);
  temp[8] = '\0';
  satrec.mo = atof(temp) * (3.14159265359 / 180.0); // Convert to radians
  
  // Mean motion (cols 53-63)
  strncpy(temp, line2 + 52, 11);
  temp[11] = '\0';
  double no_revs_per_day = atof(temp);
  
  // Convert from revolutions per day to radians per minute
  satrec.no = no_revs_per_day * 2.0 * pi / 1440.0; // rad/min
  
  // Set other required values
  satrec.nddot = 0.0; // Second derivative of mean motion (usually 0)
  satrec.satnum = 25544; // ISS satellite number
  
  // Calculate Julian date for epoch
  int year;
  if (satrec.epochyr < 57)
    year = satrec.epochyr + 2000;
  else
    year = satrec.epochyr + 1900;
    
  int mon, day, hr, minute;
  double sec;
  days2mdhms(year, satrec.epochdays, mon, day, hr, minute, sec);
  jday(year, mon, day, hr, minute, sec, satrec.jdsatepoch);
  
  // Debug output
  if (ENABLE_LOG) {
    Serial.print("epochyr: "); Serial.println(satrec.epochyr);
    Serial.print("epochdays: "); Serial.println(satrec.epochdays, 8);
    Serial.print("inclo (rad): "); Serial.println(satrec.inclo, 6);
    Serial.print("nodeo (rad): "); Serial.println(satrec.nodeo, 6);
    Serial.print("ecco: "); Serial.println(satrec.ecco, 8);
    Serial.print("argpo (rad): "); Serial.println(satrec.argpo, 6);
    Serial.print("mo (rad): "); Serial.println(satrec.mo, 6);
    Serial.print("no (rad/min): "); Serial.println(satrec.no, 8);
    Serial.print("bstar: "); Serial.println(satrec.bstar, 8);
    Serial.print("jdsatepoch: "); Serial.println(satrec.jdsatepoch, 10);
  
    if (isnan(satrec.no) || satrec.no <= 0.0) {
      Serial.println("ERROR: Mean motion is invalid!");
    }
  }
}

// -------- ARDUINO --------
void setup() {
  currentTime = millis();
  lastRecordedTime = currentTime;

  // monitor
  Serial.begin(115200);

  // i2c
  Wire.begin();

  // lcd
  lcd.init();
  lcd.backlight();
  lcdSetFirstLine("MEGA2560 INIT");
  delay(1000);

  // gps
  GPS_SERIAL.begin(9600);
  lcdSetFirstLine("GPS INIT");

  while (!gps.location.isValid()) {
    while (GPS_SERIAL.available()) {
      char c = GPS_SERIAL.read();
      gps.encode(c);
    }
  }
  lcdSetSecondLine("LOCATIION OK");
  delay(1000);
  while (!gps.date.isValid()) {
    while (GPS_SERIAL.available()) {
      char c = GPS_SERIAL.read();
      gps.encode(c);
    }
  }
  lcdSetSecondLine("DATE OK");
  delay(1000);
  lcdClear();

  // wifi
  if (ENABLE_WIFI) {
    WIFI_SERIAL.begin(115200);
    WiFi.init(&WIFI_SERIAL);
    lcdSetFirstLine("WIFI INIT");
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      if (WiFi.status() != WL_CONNECTED) {
        delay(5000);
      }
      if (millis() - lastRecordedTime >= 30000) {  // 30s timeout
        lcdSetSecondLine("ERROR");
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      lcdSetSecondLine("OK");
    }
    delay(1000);
    lcdClear();
  }

  // stepper motors
  pinMode(AZIMUTH_STEP_PIN, OUTPUT);
  pinMode(AZIMUTH_DIR_PIN, OUTPUT);
  digitalWrite(AZIMUTH_DIR_PIN, HIGH);
  AZIMUTH_SERIAL.begin(115200);
  azimuthDriver.begin();
  azimuthDriver.rms_current(900);
  azimuthDriver.microsteps(16);
  azimuthDriver.pwm_autoscale(true);
  pinMode(ELEVATION_STEP_PIN, OUTPUT);
  pinMode(ELEVATION_DIR_PIN, OUTPUT);
  digitalWrite(AZIMUTH_DIR_PIN, HIGH);
  ELEVATION_SERIAL.begin(115200);
  elevationDriver.begin();
  elevationDriver.rms_current(1200);
  elevationDriver.microsteps(1);
  elevationDriver.pwm_autoscale(true);

  // absolute position encoder for elevation motor
  lcdSetFirstLine("AS5600 INIT");
  if (!as5600.begin()) {
    lcdSetSecondLine("ERROR");
  } else {
    lcdSetSecondLine("OK");
  }
  delay(1000);
  lcdClear();

  // azimuth position magnet
  compass.init();
  lcdSetFirstLine("QMC5883L INIT");
  delay(1000);
  lcdClear();

  // get ISS data
  if (WiFi.status() == WL_CONNECTED) {
    lcdSetFirstLine("FETCH HTTP DATA");

    WiFiClient client;
    if (!client.connect("celestrak.com", 80)) {
      Serial.println("Connection to CelesTrak failed!");
      return false;
    }

    client.println("GET /NORAD/elements/stations.txt HTTP/1.1");
    client.println("Host: celestrak.com");
    client.println("Connection: close");
    client.println();

    unsigned long start = millis();

    while (client.connected() && millis() - start < 5000) {
      if (client.available()) {
        String line = client.readStringUntil('\n');

        if (line.startsWith("ISS (ZARYA)")) {
          String l1 = client.readStringUntil('\n');
          String l2 = client.readStringUntil('\n');

          strncpy(tle_line1, l1.c_str(), sizeof(tle_line1) - 1);
          strncpy(tle_line2, l2.c_str(), sizeof(tle_line2) - 1);

          foundISSbyWifi = true;
          break;
        }
      }
    }

    client.stop();
  }
  lcdClear();

  if (!foundISSbyWifi) {
    // TLE dated 6 September 2025 (celestrak.com/NORAD/elements/stations.txt)
    strcpy(tle_line1, "1 25544U 98067A   25249.87397102  .00012937  00000+0  23296-3 0  9995");
    strcpy(tle_line2, "2 25544  51.6325 262.1963 0004212 309.4705  50.5911 15.50156361527839");
  }

  // home
  // while (compass.getAzimuth() != 0) {
  //   digitalWrite(AZIMUTH_STEP_PIN, HIGH);
  //   delayMicroseconds(1000);
  //   digitalWrite(AZIMUTH_STEP_PIN, LOW);
  //   delayMicroseconds(1000);
  // }
}

void loop() {
  currentTime = millis();
  delay(1000);

  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // Only calculate if we have valid GPS data
  if (!gps.location.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
    Serial.println("Waiting for valid GPS data...");
    delay(1000);
    return;
  }

  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();
  hour = gps.time.hour();
  minute = gps.time.minute();
  second = gps.time.second();
  observerLatitude = gps.location.lat();
  observerLongitude = gps.location.lng();
  observerAltitude = gps.altitude.kilometers();

  double jdnow;
  jday(year, month, day, hour, minute, second, jdnow);
  parseISSTLE(tle_line1, tle_line2, satrec);
  double tsince = (jdnow - satrec.jdsatepoch) * 24.0 * 60.0;
  double ro[3], vo[3];
  sgp4(wgs72, satrec, tsince, ro, vo);

  // Check if SGP4 calculation was successful
  if (satrec.error != 0) {
    Serial.print("SGP4 Error: ");
    Serial.println(satrec.error);
    delay(1000);
    return;
  }

  // double recef[3], vecef[3];
  // teme2ecef(ro, vo, jdnow, recef, vecef);
  // double latlongh[3];
  // ijk2ll(recef, latlongh);
  double razel[3], razelrates[3];
  rv2azel(ro, vo, observerLatitude * deg2rad, observerLongitude * deg2rad, observerAltitude, jdnow, razel, razelrates);

  // Convert to degrees for display
  double azimuth_deg = razel[1] * rad2deg;
  double elevation_deg = razel[2] * rad2deg;
  double range_km = razel[0];
  if (azimuth_deg < 0) azimuth_deg += 360.0;

  if (ENABLE_LOG) {
    Serial.print("Parsed epochyr: "); Serial.println(satrec.epochyr);
    Serial.print("Parsed epochdays: "); Serial.println(satrec.epochdays, 8);
    Serial.print("Obs lat: "); Serial.println(observerLatitude, 6);
    Serial.print("Obs lon: "); Serial.println(observerLongitude, 6);
    Serial.print("Obs alt: "); Serial.println(observerAltitude, 6);
    Serial.print("r: "); Serial.print(ro[0]); Serial.print(", "); Serial.print(ro[1]); Serial.print(", "); Serial.println(ro[2]);
    Serial.print("v: "); Serial.print(vo[0]); Serial.print(", "); Serial.print(vo[1]); Serial.print(", "); Serial.println(vo[2]);
    Serial.print("Range (km): "); Serial.println(range_km);
    Serial.print("Azimuth (deg): "); Serial.println(azimuth_deg);
    Serial.print("Elevation (deg): "); Serial.println(elevation_deg);
    Serial.print("jdnow: "); Serial.println(jdnow, 10);
    Serial.print("jdsatepoch: "); Serial.println(satrec.jdsatepoch, 10);
    Serial.print("tsince: "); Serial.println(tsince, 6);
  }

  if (elevation_deg > 0) {
    ISSIsVisible = true;
    lcdSetFirstLine("ISS:");
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "Az:%03.0f El:%02.0f", azimuth_deg, elevation_deg);
    lcdSetSecondLine(buffer);
  } else {
    if (ISSIsVisible == true) {
      ISSIsVisible = false;
      lcdSetFirstLine("ISS");
      lcdSetSecondLine("BELOW HORIZON");
    }
  }
}
