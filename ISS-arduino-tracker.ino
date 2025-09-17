/**
 * International Space Station (ISS) orbit tracker
 * Tracks the ISS using TLE data from Celestrak, observer location from GPS,
 * magnetic declination from NOAA, and points a motorized antenna mount
 * towards the ISS.
 *
 * @author svnty (Jake Walklate)
 * @date September 2025
 * @copyright © 2025 Jake Walklate
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <QMC5883LCompass.h>
#include <Servo.h>
#include <TinyGPSPlus.h>
#include <WiFiS3.h>
#include <Wire.h>

#include "SGP4_vallado/sgp4coord.cpp"
#include "SGP4_vallado/sgp4coord.h"
#include "SGP4_vallado/sgp4ext.cpp"
#include "SGP4_vallado/sgp4ext.h"
#include "SGP4_vallado/sgp4unit.cpp"
#include "SGP4_vallado/sgp4unit.h"

// -------- SYSTEM TOGGLES --------
static const bool ENABLE_WIFI = true;
static const bool ENABLE_LOG = true;
static const bool ENABLE_GPS = true;

// -------- CONSTANTS --------
// pins
static const uint8_t ELEVATION_PIN = 9;
static const uint8_t AZIMUTH_DIR_PIN = 10;
static const uint8_t AZIMUTH_STEP_PIN = 11;
static const uint8_t GPS_RX_PIN = 12;
// stepper motor
static const float TMC2209_RESISTANCE = 0.11;  // ohms
// wifi
static const char *WIFI_SSID = "Jakes iPhone", *WIFI_PASSWORD = "izeh5zemxa05r";
// calculations
static const double DEG_2_RAD = pi / 180.0;
static const double RAD_2_DEG = 180.0 / pi;
// http
static const char *DECLINATION_HOST = "www.ngdc.noaa.gov";
static const char *DECLINATION_API_KEY = "zNEw7";
static const char *TLE_API_HOST = "celestrak.org";
static const uint16_t PORT = 443;
// i2c
static const uint8_t MAX_i2c_DEVICES = 16;
// compass
static const float COMPASS_AZIMUTH_OFFSET = 180 + 17;          // degrees
static const float COMPASS_JUMP_THRESHOLD = 5.0;               // degrees
static const unsigned long COMPASS_CHECK_INTERVAL = 5 * 1000;  // 5 seconds
// lcd
static const unsigned long LCD_UPDATE_INTERVAL = 5 * 1000;  // 5 seconds
// wifi
static const unsigned long WIFI_UPDATE_INTERVAL = 60 * 60 * 1000;  // 1 hour
// gps
static const unsigned long GPS_UPDATE_INTERVAL = 10 * 60 * 1000;  // 10 minutes
static const unsigned long GPS_TIMEOUT_INTERVAL = 60 * 1000;      // 1 minute

// -------- VARIABLES --------
// timers
static unsigned long lastLcdUpdateTime = 0;
static unsigned long lastGpsUpdateTime = 0;
static unsigned long lastWiFiUpdateTime = 0;
static unsigned long lastWiFiReconnectAttemptTime = 0;
static unsigned long gpsTimeoutTimer = 0;
static unsigned long lastTimeUpdateMillis = 0;
// http data
static bool foundISSbyWifi = false;
static uint8_t findISSbyWifiAttempts = 0;
static bool foundDeclinationByWifi = false;
static uint8_t findDeclinationByWifiAttempts = 0;
static float magneticDeclination = 12.7;  // degrees
static char tle_line1[70];
static char tle_line2[70];
// gps data
static double observerLatitude = 0.0, observerLongitude = 0.0, observerAltitude = 0.0;
static int year, month, day, hour, minute, second;
static bool isGpsFixed = false;
static uint32_t gpsDateAge = 0;
static uint32_t gpsTimeAge = 0;
static uint32_t gpsLocationAge = 0;
// motor control
static float currentAzimuth = 0.0;
static float currentElevation = 95.0;
// compass
static float previousCompassHeading = 0.0;
static unsigned long lastCompassCheckTime = 0;
static unsigned long compassLastCalibrate = 0;
// i2c
static uint8_t i2cDeviceAddresses[MAX_i2c_DEVICES];
static uint8_t i2cDeviceCount = 0;
// lcd
static unsigned long lcdDisplayMode = 0;

// -------- SERIALS --------
HardwareSerial &GPS_SERIAL = Serial1;

// -------- OBJECTS --------
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);
QMC5883LCompass compass;
elsetrec satrec;
Servo elevation;

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

void updateLCD(double azimuth_deg, double elevation_deg, double range_km) {
  lcdClear();
  // If ISS is overhead (elevation > 10°), always show ISS position
  bool issOverhead = elevation_deg > 10.0;

  if (issOverhead) {
    lcdSetFirstLine("ISS OVERHEAD!");
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "Az:%03.0f El:%02.0f", azimuth_deg, elevation_deg);
    lcdSetSecondLine(buffer);
    return;
  }

  // If ISS is visible but not overhead, show ISS info
  if (elevation_deg > 0) {
    lcdSetFirstLine("ISS VISIBLE");
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "Az:%03.0f El:%02.0f", azimuth_deg, elevation_deg);
    lcdSetSecondLine(buffer);
    return;
  }

  unsigned long modeTime = (millis() / LCD_UPDATE_INTERVAL) % 7;

  switch (modeTime) {
    case 0:  // ISS status and range
      lcdSetFirstLine("ISS DISTANCE");
      {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%03.0f km", range_km);
        lcdSetSecondLine(buffer);
      }
      break;
    case 1:  // Observer location
      lcdSetFirstLine("OBSERVER LOCATION");
      {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%.2f,%.2f", observerLatitude, observerLongitude);
        lcdSetSecondLine(buffer);
      }
      break;
    case 2:  // Current time
      lcdSetFirstLine("TIME (UTC)");
      {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%02d:%02d %d/%d/%02d", hour, minute, day, month, (year % 100));
        lcdSetSecondLine(buffer);
      }
      break;
    case 3:  // Compass heading and motor position
      lcdSetFirstLine("COMPASS/MOTOR");
      {
        char buffer[16];
        float compassHeading = getCompassHeading();
        snprintf(buffer, sizeof(buffer), "C:%03.0f M:%03.0f", compassHeading, currentAzimuth);
        lcdSetSecondLine(buffer);
      }
      break;
    case 4:  // ISS current azimuth (even when below horizon)
      lcdSetFirstLine("ISS DIRECTION");
      {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "Az:%03.0f E:%0.0fkm", azimuth_deg, elevation_deg);
        lcdSetSecondLine(buffer);
      }
      break;
    case 5:
      lcdSetFirstLine("ISS");
      lcdSetSecondLine("BELOW HORIZON");
      break;
    case 6:
      if (isGpsFixed) {
        lcdSetFirstLine("GPS ACCURATE");
      } else {
        lcdSetFirstLine("GPS ERROR");
      }
      if (foundISSbyWifi) {
        lcdSetSecondLine("ISS ACCURATE");
      } else {
        lcdSetSecondLine("ISS DATA ERROR");
      }
      break;
  }
}

void makeTleApiRequest() {
  Serial.println("Making HTTPS API request...");

  WiFiSSLClient client;

  Serial.print("Connecting to ");
  Serial.print(TLE_API_HOST);
  Serial.print(":");
  Serial.println(PORT);

  if (!client.connect(TLE_API_HOST, PORT)) {
    Serial.println("Connection to server failed!");
    lcdSetSecondLine("TLE ERROR");
    delay(2000);
    return;
  }

  Serial.println("Connected to server");

  String request = "GET " + String("/NORAD/elements/gp.php?GROUP=stations") + " HTTP/1.1\r\n";
  request += "Host: " + String(TLE_API_HOST) + "\r\n";
  request += "User-Agent: ArduinoUnoR4Wifi/1.0\r\n";
  request += "Cache-Control: no-cache\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";

  client.print(request);
  Serial.println("Request sent");
  Serial.println("Request details:");
  Serial.println(request);

  String httpResponse = "";
  bool headersEnded = false;
  unsigned long timeout = millis();
  const unsigned long timeoutLimit = 10000;  // 10 second timeout

  while ((client.connected() || client.available()) && (millis() - timeout < timeoutLimit)) {
    if (client.available()) {
      String line = client.readStringUntil('\n');

      if (!headersEnded) {
        Serial.println("Header: " + line);

        // Check for end of headers (empty line)
        if (line.length() <= 1) {
          headersEnded = true;
          Serial.println("--- End of Headers ---");
          Serial.println("Reading response body...");
        }
      } else {
        // We're in the body now
        httpResponse += line;
      }

      // Reset timeout when data is received
      timeout = millis();
    }
    delay(1);
  }

  client.stop();
  Serial.println("Connection closed");

  int issIndex = httpResponse.indexOf("ISS (ZARYA)");
  if (httpResponse.isEmpty() == false && issIndex >= 0) {
    foundISSbyWifi = true;
    Serial.println(httpResponse);
    parseTLEData(httpResponse, issIndex);
  } else {
    foundISSbyWifi = false;
  }

  if (!foundISSbyWifi) {
    Serial.println("No valid TLE data received!");
    return;
  }
}

void parseTLEData(String data, int issIndex) {
  Serial.println("Parsing TLE data...");

  Serial.println("Found ISS (ZARYA) data!");

  // Find the first TLE line (starts with "1 25544")
  int line1Index = data.indexOf("1 25544", issIndex);
  if (line1Index == -1) {
    Serial.println("TLE Line 1 not found!");
    return;
  }

  // Find the second TLE line (starts with "2 25544")
  int line2Index = data.indexOf("2 25544", line1Index);
  if (line2Index == -1) {
    Serial.println("TLE Line 2 not found!");
    return;
  }

  // Extract Line 1 (69 characters for standard TLE format)
  String line1 = data.substring(line1Index, line1Index + 69);
  line1.trim();

  // Extract Line 2 (69 characters for standard TLE format)
  String line2 = data.substring(line2Index, line2Index + 69);
  line2.trim();

  // Copy to global variables
  strcpy(tle_line1, line1.c_str());
  strcpy(tle_line2, line2.c_str());

  Serial.println("TLE Line 1: " + line1);
  Serial.println("TLE Line 2: " + line2);
  Serial.println("TLE data parsed successfully!");
  lcdSetSecondLine("TLE OK");
}

void makeDeclinationApiRequest() {
  Serial.println("Making HTTPS API request...");

  WiFiSSLClient client;

  Serial.print("Connecting to ");
  Serial.print(DECLINATION_HOST);
  Serial.print(":");
  Serial.println(PORT);

  if (!client.connect(DECLINATION_HOST, PORT)) {
    Serial.println("Connection to server failed!");
    lcdSetSecondLine("DECLINATION ERROR");
    delay(2000);
    return;
  }

  Serial.println("Connected to server");

  char apiPath[256];
  snprintf(apiPath, sizeof(apiPath), "/geomag-web/calculators/calculateDeclination?lat1=%.6f&lon1=%.6f&key=%s&resultFormat=json", observerLatitude, observerLongitude, DECLINATION_API_KEY);

  String request = "GET " + String(apiPath) + " HTTP/1.1\r\n";
  request += "Host: " + String(DECLINATION_HOST) + "\r\n";
  request += "User-Agent: ArduinoUnoR4Wifi/1.0\r\n";
  request += "Accept: application/json\r\n";
  request += "Cache-Control: no-cache\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";

  client.print(request);
  Serial.println("Request sent");
  Serial.println("Request details:");
  Serial.println(request);

  String jsonResponse = "";
  bool headersEnded = false;
  unsigned long timeout = millis();
  const unsigned long timeoutLimit = 10000;

  while ((client.connected() || client.available()) && (millis() - timeout < timeoutLimit)) {
    if (client.available()) {
      String line = client.readStringUntil('\n');

      if (!headersEnded) {
        Serial.println("Header: " + line);

        // Check for end of headers (empty line)
        if (line.length() <= 1) {
          headersEnded = true;
          Serial.println("--- End of Headers ---");
          Serial.println("Reading response body...");
        }
      } else {
        jsonResponse += line;
      }

      // Reset timeout when data is received
      timeout = millis();
    }
    delay(1);
  }

  client.stop();
  Serial.println("Connection closed");

  if (jsonResponse.length() == 0) {
    Serial.println("No response body received!");
    return;
  } else {
    // assuming the data is valid if we got something
    // todo: improve this check
    foundDeclinationByWifi = true;
  }

  parseDeclinationJsonResponse(jsonResponse);
}

String cleanDeclinationChunkedJSON(String rawJson) {
  int jsonStart = rawJson.indexOf('{');

  int jsonEnd = rawJson.lastIndexOf('}');

  if (jsonStart >= 0 && jsonEnd > jsonStart) {
    String cleanJson = rawJson.substring(jsonStart, jsonEnd + 1);
    Serial.println("Cleaned JSON extracted successfully");
    return cleanJson;
  } else {
    Serial.println("Could not find JSON boundaries!");
    return rawJson;
  }
}

void parseDeclinationJsonResponse(String jsonString) {
  Serial.println("Parsing JSON response...");
  Serial.println("Raw JSON:");
  Serial.println(jsonString);
  Serial.println("JSON length: " + String(jsonString.length()));

  String cleanedJson = cleanDeclinationChunkedJSON(jsonString);
  Serial.println("Using cleaned JSON:");
  Serial.println(cleanedJson);

  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, cleanedJson);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    Serial.println("Attempting to debug JSON structure...");
    Serial.println("First 50 chars: " + cleanedJson.substring(0, 50));
    return;
  }

  Serial.println("JSON parsed successfully!");

  // Parse the NOAA geomagnetic data
  if (doc["result"].is<JsonArray>()) {
    JsonArray results = doc["result"];
    if (results.size() > 0) {
      JsonObject result = results[0];

      if (!result["declination"].isNull()) {
        magneticDeclination = result["declination"];
        foundDeclinationByWifi = true;
        lcdSetSecondLine("DECLINATION OK");
        Serial.print("Magnetic Declination: ");
        Serial.print(magneticDeclination, 5);
        Serial.println("°");
      }
    }
  } else {
    Serial.println("Unexpected JSON structure: 'result' is not an array");
  }
}

void parseISSTLE(const char *line1, const char *line2, elsetrec &satrec) {
  char temp[20];

  // Line 1 parsing
  strncpy(temp, line1 + 18, 2);
  temp[2] = '\0';
  satrec.epochyr = atoi(temp);

  strncpy(temp, line1 + 20, 12);
  temp[12] = '\0';
  satrec.epochdays = atof(temp);

  strncpy(temp, line1 + 33, 10);
  temp[10] = '\0';
  satrec.ndot = atof(temp);

  strncpy(temp, line1 + 53, 8);
  temp[8] = '\0';
  if (strlen(temp) >= 6) {
    char mantissa[6], exponent[3];
    strncpy(mantissa, temp, 5);
    mantissa[5] = '\0';
    strncpy(exponent, temp + 6, 2);
    exponent[2] = '\0';
    double mant = atof(mantissa) / 100000.0;
    int exp = atoi(exponent);
    if (temp[5] == '-') exp = -exp;
    satrec.bstar = mant * pow(10.0, exp);
  } else {
    satrec.bstar = 0.0;
  }

  // Line 2 parsing
  strncpy(temp, line2 + 8, 8);
  temp[8] = '\0';
  satrec.inclo = atof(temp) * (3.14159265359 / 180.0);

  strncpy(temp, line2 + 17, 8);
  temp[8] = '\0';
  satrec.nodeo = atof(temp) * (3.14159265359 / 180.0);

  strncpy(temp, line2 + 26, 7);
  temp[7] = '\0';
  satrec.ecco = atof(temp) / 10000000.0;

  strncpy(temp, line2 + 34, 8);
  temp[8] = '\0';
  satrec.argpo = atof(temp) * (3.14159265359 / 180.0);

  strncpy(temp, line2 + 43, 8);
  temp[8] = '\0';
  satrec.mo = atof(temp) * (3.14159265359 / 180.0);

  strncpy(temp, line2 + 52, 11);
  temp[11] = '\0';
  double no_revs_per_day = atof(temp);
  satrec.no = no_revs_per_day * 2.0 * pi / 1440.0;

  satrec.nddot = 0.0;
  satrec.satnum = 25544;

  int year;
  if (satrec.epochyr < 57)
    year = satrec.epochyr + 2000;
  else
    year = satrec.epochyr + 1900;

  int mon, day, hr, minute;
  double sec;
  days2mdhms(year, satrec.epochdays, mon, day, hr, minute, sec);
  jday(year, mon, day, hr, minute, sec, satrec.jdsatepoch);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);
  WiFi.disconnect();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());  // TODO: check if this works with R4
    Serial.print("Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    lcdSetSecondLine("OK");

    // Dummy HTTP request to wake up ESP32
    Serial.println("Sending dummy data to wake up ESP32");
    WiFiSSLClient client;
    client.connect("google.com", 443);
    client.println("GET / HTTP/1.1");
    client.println("Host: google.com");
    client.println("Connection: close");
    client.println();
    client.stop();
  }
}

float getCompassHeading() {
  compass.read();
  float reading = compass.getAzimuth();

  // Apply compass offset so azimuth is corrected
  float trueHeading = reading + magneticDeclination + COMPASS_AZIMUTH_OFFSET;
  if (trueHeading < 0) trueHeading += 360;
  if (trueHeading >= 360) trueHeading -= 360;
  return trueHeading;
}

float calculateAngleDifference(float angle1, float angle2) {
  float diff = angle2 - angle1;
  // Handle wraparound cases
  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }
  return abs(diff);
}

void recalibrateAzimuth() {
  lcdSetFirstLine("CALIBRATING");
  lcdSetSecondLine("AZIMUTH");
  float currentHeading = 0.0;

  // Rotate until we're pointing north again
  while (true) {
    i2cBusCheck();
    currentHeading = getCompassHeading();
    previousCompassHeading = currentHeading;

    Serial.print("Calibrating - Current heading: ");
    Serial.println(currentHeading);

    // Check if we're close to north (0 degrees)
    if (abs(currentHeading) <= 0.5 || abs(currentHeading) >= 359.5) {
      break;
    }

    // Determine direction to turn
    if (currentHeading > 180) {
      digitalWrite(AZIMUTH_DIR_PIN, HIGH);
    } else {
      digitalWrite(AZIMUTH_DIR_PIN, LOW);
    }

    // Step the motor towards north
    digitalWrite(AZIMUTH_STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(AZIMUTH_STEP_PIN, LOW);
    delayMicroseconds(1000);
  }

  // Reset our azimuth position tracking
  currentAzimuth = currentHeading;
  compassLastCalibrate = millis();

  lcdSetSecondLine("OK");
  delay(2000);

  Serial.println("Compass recalibration complete");
}

bool checkForCompassJump() {
  float currentHeading = getCompassHeading();
  Serial.print("Current compass heading: ");
  Serial.println(currentHeading);
  Serial.print("Previous compass heading: ");
  Serial.println(previousCompassHeading);

  if (millis() - compassLastCalibrate < 10000) {
    Serial.println("Skipping jump check - within 10s of calibration");
    previousCompassHeading = currentHeading;
    return false;
  }

  if (millis() - lastCompassCheckTime >= COMPASS_CHECK_INTERVAL) {
    lastCompassCheckTime = millis();
    if (abs(calculateAngleDifference(currentHeading, previousCompassHeading)) >= COMPASS_JUMP_THRESHOLD) {
      previousCompassHeading = currentHeading;
      return true;
    }
  }

  return false;
}

void moveAzimuthTo(float targetAzimuth) {
  // Apply offset to target azimuth so movement is corrected
  float angleDifference = targetAzimuth - currentAzimuth;
  float currentHeading = 0.0;

  // Handle wraparound (e.g., from 350° to 10°)
  if (angleDifference > 180) {
    angleDifference -= 360;
  } else if (angleDifference < -180) {
    angleDifference += 360;
  }

  if (abs(angleDifference) >= 10) {
    lcdSetFirstLine("FINDING ISS");
  }

  if (abs(angleDifference) >= 0.5) {
    // Calculate direction ONCE at the beginning based on initial heading
    float initialHeading = getCompassHeading();
    float initialDifference = targetAzimuth - initialHeading;

    // Handle wraparound for initial heading difference
    if (initialDifference > 180) {
      initialDifference -= 360;
    } else if (initialDifference < -180) {
      initialDifference += 360;
    }

    // Determine direction and stick with it
    bool moveClockwise = (initialDifference > 0);

    Serial.print("moveAzimuthTo - Initial heading: ");
    Serial.println(initialHeading);
    Serial.print("moveAzimuthTo - Target azimuth: ");
    Serial.println(targetAzimuth);
    Serial.print("moveAzimuthTo - Initial difference: ");
    Serial.println(initialDifference);
    Serial.print("moveAzimuthTo - Direction chosen: ");
    Serial.println(moveClockwise ? "CLOCKWISE" : "COUNTER-CLOCKWISE");

    digitalWrite(AZIMUTH_DIR_PIN, moveClockwise ? HIGH : LOW);

    while (true) {
      i2cBusCheck();
      currentHeading = getCompassHeading();

      Serial.print("moveAzimuthTo - Current heading:");
      Serial.println(currentHeading);

      // Calculate current difference to target
      float headingDifference = targetAzimuth - currentHeading;
      // Handle wraparound for heading difference
      if (headingDifference > 180) {
        headingDifference -= 360;
      } else if (headingDifference < -180) {
        headingDifference += 360;
      }

      Serial.println("moveAzimuthTo - Heading difference: " + String(headingDifference));

      // Check if close enough to target
      if (abs(headingDifference) <= 0.5) {
        break;
      }

      // Take step in the predetermined direction (NO direction recalculation)
      digitalWrite(AZIMUTH_STEP_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(AZIMUTH_STEP_PIN, LOW);
      delayMicroseconds(1000);
    }

    float finalHeading = getCompassHeading();
    currentAzimuth = finalHeading;
    compassLastCalibrate = millis();

    Serial.println("Finish moveAzimuthTo");
    Serial.print("Previous compass heading: ");
    Serial.println(previousCompassHeading);
    Serial.print("Current azimuth: ");
    Serial.println(currentAzimuth);

    previousCompassHeading = currentHeading;

    // Normalize to 0-360 range
    if (currentAzimuth < 0) currentAzimuth += 360;
    if (currentAzimuth >= 360) currentAzimuth -= 360;
  }
}

void moveElevationTo(float targetElevation) {
  // Convert elevation angle to servo position, Horizon = 90°, up = 0°, down = 180°
  // ISS elevation: 0° = horizon, 90° = directly up, negative = below horizon

  float servoPosition;
  if (targetElevation < 0) {
    // ISS is below horizon, point down at it
    // Convert negative elevation to servo position below horizon
    servoPosition = 90 + abs(targetElevation);

    // Constrain to maximum downward angle (180°)
    if (servoPosition > 180) servoPosition = 180;
  } else {
    // Convert ISS elevation (0° = horizon, 90° = up) to servo position (90° = horizon, 0° = up)
    servoPosition = 90 - targetElevation;

    // Constrain servo position to valid range
    if (servoPosition < 0) servoPosition = 0;
  }

  // Only move if the difference is significant (>= 1 degree)
  if (abs(servoPosition - currentElevation) >= 1.0) {
    elevation.write((int)servoPosition);
    currentElevation = servoPosition;

    // Small delay to allow servo to move
    delay(50);
  }
}

void i2cBusCheck() {
  bool busOk = true;
  for (uint8_t i = 0; i < i2cDeviceCount; i++) {
    Wire.beginTransmission(i2cDeviceAddresses[i]);
    byte err = Wire.endTransmission();
    if (err != 0) {
      busOk = false;
      break;
    }
  }

  if (!busOk) {
    Serial.println("Bus frozen! Recovering...");
    i2cBusRecover();
    return;
  }
}

void i2cBusRecover() {
  const uint8_t SDA_PIN = SDA;
  const uint8_t SCL_PIN = SCL;

  // Clock out 9 pulses to release any stuck slave
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  // Send STOP condition
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH);
  digitalWrite(SDA_PIN, HIGH);

  // Restore pins for Wire
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  // Reinitialize I2C master
  Wire.begin();
  Serial.println("I2C bus recovered!");
}

// -------- ARDUINO --------
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Serial...");
  Wire.begin();
  i2cBusRecover();

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      if (i2cDeviceCount < MAX_i2c_DEVICES) {
        i2cDeviceAddresses[i2cDeviceCount++] = address;
        Serial.print("Found device at 0x");
        Serial.println(address, HEX);
      }
    }
  }

  // Elevation Motor Initialization
  elevation.attach(ELEVATION_PIN);
  elevation.write(currentElevation);

  // Azimuth Motor Initialization
  pinMode(AZIMUTH_STEP_PIN, OUTPUT);
  pinMode(AZIMUTH_DIR_PIN, OUTPUT);
  digitalWrite(AZIMUTH_DIR_PIN, HIGH);

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  lcdSetFirstLine("ARDUINO INIT");
  delay(1000);

  // GPS Initialization
  if (ENABLE_GPS) {
    lcdSetFirstLine("GPS INIT");
    Serial.println("Initializing GPS...");
    GPS_SERIAL.begin(9600);
    updateGPSData();

    if (gps.location.isValid()) {
      lcdSetSecondLine("LOCATION OK");
      delay(2000);
    } else {
      lcdSetSecondLine("NO LOCATION");
      delay(2000);
    }

    if (gps.date.isValid()) {
      lcdSetSecondLine("DATE OK");
      delay(2000);
    } else {
      lcdSetSecondLine("NO DATE");
      delay(2000);
    }

    if (gps.time.isUpdated()) {
      lcdSetSecondLine("TIME OK");
      delay(2000);
    } else {
      lcdSetSecondLine("NO TIME");
      delay(2000);
    }
    
    lcdClear();
  }

  if (!gps.location.isValid()) {
    Serial.println("Gps location: INVALID");
    observerLatitude = -33.88336;
    observerLongitude = 152.20148;
    observerAltitude = 0.035;
  }
  if (!gps.date.isValid()) {
    Serial.println("Gps date: INVALID");
    year = 2025;
    month = 9;
    day = 17;
    hour = 12;
    minute = 0;
    second = 0;
  }

  // WiFi Initialization
  if (ENABLE_WIFI) {
    lcdSetFirstLine("WIFI INIT");
    connectToWiFi();
    lastWiFiReconnectAttemptTime = millis();
    while (millis() - lastWiFiReconnectAttemptTime < 10000) {
      delay(1);
      if (WiFi.status() == WL_CONNECTED) {
        break;
      }
    }
    int wifiReconnectAttempts = 1;

    while (WiFi.status() != WL_CONNECTED) {
      wifiReconnectAttempts++;
      lcdSetSecondLine("ATTEMPT " + String(wifiReconnectAttempts));
      connectToWiFi();
      lastWiFiReconnectAttemptTime = millis();
      while (millis() - lastWiFiReconnectAttemptTime < 15000) {
        delay(1);
        if (WiFi.status() == WL_CONNECTED) {
          break;
        }
      }
      if (wifiReconnectAttempts >= 3) {
        lcdSetSecondLine("ERROR");
        break;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      lcdSetSecondLine("FETCH ISS TLE");
      while (!foundISSbyWifi && findISSbyWifiAttempts < 3) {
        findISSbyWifiAttempts++;
        makeTleApiRequest();
      }

      lcdSetSecondLine("FETCH DECLINATION");
      while (!foundDeclinationByWifi && findDeclinationByWifiAttempts < 3) {
        findDeclinationByWifiAttempts++;
        makeDeclinationApiRequest();
      }
    } else {
      lcdSetSecondLine("ERROR");
    }
  }
  if (!foundISSbyWifi) {
    strcpy(tle_line1, "1 25544U 98067A   25259.15672217  .00010925  00000+0  19686-3 0  9993");
    strcpy(tle_line2, "2 25544  51.6329 216.1838 0004346 344.8645  15.2213 15.50326031529273");
    lcdSetFirstLine("USING DEFAULT TLE");
  }
  parseISSTLE(tle_line1, tle_line2, satrec);
  lcdSetSecondLine("DATA PARSED");
  delay(2000);
  lcdClear();

  // Magnometer Initialization
  lcdSetFirstLine("QMC5883L INIT");
  compass.init();
  delay(2000);

  // Initial compass calibration
  recalibrateAzimuth();
  previousCompassHeading = getCompassHeading();
  lastCompassCheckTime = millis();

  lcdClear();
}

void updateLocalTime() {
  unsigned long currentMillis = millis();

  // If this is the first time or if millis() has rolled over, initialize
  if (lastTimeUpdateMillis == 0 || currentMillis < lastTimeUpdateMillis) {
    lastTimeUpdateMillis = currentMillis;
    return;
  }

  // Calculate elapsed seconds since last update
  unsigned long elapsedMillis = currentMillis - lastTimeUpdateMillis;
  unsigned long elapsedSeconds = elapsedMillis / 1000;

  if (elapsedSeconds > 0) {
    // Update the time variables
    second += elapsedSeconds;

    // Handle seconds overflow
    if (second >= 60) {
      minute += second / 60;
      second = second % 60;
    }

    // Handle minutes overflow
    if (minute >= 60) {
      hour += minute / 60;
      minute = minute % 60;
    }

    // Handle hours overflow (24-hour format)
    if (hour >= 24) {
      hour = hour % 24;
      // Note: We're not updating day/month/year here since that would
      // require more complex calendar logic. GPS updates every 10 minutes
      // should handle date changes adequately.
    }

    // Update the timestamp
    lastTimeUpdateMillis = currentMillis;
  }
}

void updateGPSData() {
  gpsTimeoutTimer = millis();

  if (!isGpsFixed) {
    while (!gps.location.isValid() && !gps.date.isValid()) {
      if (millis() - gpsTimeoutTimer >= GPS_TIMEOUT_INTERVAL) {
        if (ENABLE_LOG) {
          Serial.println("GPS timeout waiting for initial fix");
        }
        break;
      }
      while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read();
        if (ENABLE_LOG) {
          Serial.write(c);
        }
        gps.encode(c);
      }
    }
  } else {
    // Wait for fresh GPS data (data that's newer than what we started with)
    while (!(gps.date.age() >= gpsDateAge) && !(gps.time.age() >= gpsTimeAge) && !(gps.location.age() >= gpsLocationAge)) {
      if (millis() - gpsTimeoutTimer >= GPS_TIMEOUT_INTERVAL) {
        if (ENABLE_LOG) {
          Serial.println("GPS timeout waiting for fresh data");
        }
        isGpsFixed = false;
        break;
      }
      while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read();
        if (ENABLE_LOG) {
          Serial.write(c);
        }
        gps.encode(c);
      }
    }
  }

  // Update our variables with the current GPS data (fresh or existing)
  if (gps.date.isValid()) {
    year = gps.date.year();
    month = gps.date.month();
    day = gps.date.day();
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
    // Reset local time tracking since we have GPS time
    lastTimeUpdateMillis = millis();
    gpsDateAge = gps.date.age();
    gpsTimeAge = gps.time.age();
    gpsLocationAge = gps.location.age();

    if (ENABLE_LOG) {
      Serial.print("GPS time data age: ");
      Serial.print(gps.time.age());
      Serial.println(" ms");
    }
  }

  if (gps.location.isValid()) {
    observerAltitude = gps.altitude.kilometers();
    observerLatitude = gps.location.lat();
    observerLongitude = gps.location.lng();

    if (ENABLE_LOG) {
      Serial.print("GPS location data age: ");
      Serial.print(gps.location.age());
      Serial.println(" ms");
    }
  }

  if (gps.location.isValid() && gps.date.isValid()) {
    isGpsFixed = true;
  }

  if (ENABLE_LOG) {
    Serial.println();
    Serial.print("GPS Date: ");
    Serial.print(gps.date.year());
    Serial.print("-");
    Serial.print(gps.date.month());
    Serial.print("-");
    Serial.println(gps.date.day());
    Serial.print("GPS Time (UTC): ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
    Serial.print("GPS Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("GPS Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("GPS Altitude (km): ");
    Serial.println(gps.altitude.kilometers(), 3);
  }
  lastGpsUpdateTime = millis();
}

void loop() {
  i2cBusCheck();
  updateLocalTime();

  // update GPS data every hour
  if (ENABLE_GPS) {
    if (millis() - lastGpsUpdateTime >= GPS_UPDATE_INTERVAL) {
      lcdClear();
      lcdSetFirstLine("UPDATING GPS");
      updateGPSData();
    }
  }

  // update the TLE and declination every hour
  if (ENABLE_WIFI) {
    if (millis() - lastWiFiUpdateTime >= WIFI_UPDATE_INTERVAL) {
      lcdClear();
      lastWiFiUpdateTime = millis();
      lcdSetFirstLine("FETCHING DATA");
      if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
      }
      if (WiFi.status() == WL_CONNECTED) {
        makeTleApiRequest();
        makeDeclinationApiRequest();
      }
    }
  }

  // Check for compass jumps indicating the tracker was moved
  if (checkForCompassJump()) {
    recalibrateAzimuth();
  }

  double jdnow;
  jday(year, month, day, hour, minute, second, jdnow);

  if (ENABLE_LOG) {
    Serial.println("==================");
    Serial.print("Julian day now: ");
    Serial.println(jdnow, 8);
    Serial.print("Satellite epoch JD: ");
    Serial.println(satrec.jdsatepoch, 8);
  }

  double tsince = (jdnow - satrec.jdsatepoch) * 24.0 * 60.0;

  if (ENABLE_LOG) {
    Serial.print("Time since epoch (minutes): ");
    Serial.println(tsince, 2);
  }

  double ro[3], vo[3];
  sgp4(wgs72, satrec, tsince, ro, vo);

  if (satrec.error != 0) {
    lcdSetFirstLine("SGP4 ERROR");
    lcdSetSecondLine(String(satrec.error));
    Serial.print("SGP4 Error: ");
    Serial.println(satrec.error);
    delay(1000);
    return;
  }

  if (ENABLE_LOG) {
    Serial.print("SGP4 position (km): ");
    Serial.print(ro[0], 3);
    Serial.print(", ");
    Serial.print(ro[1], 3);
    Serial.print(", ");
    Serial.println(ro[2], 3);
  }

  double razel[3], razelrates[3];
  rv2azel(ro, vo, observerLatitude * DEG_2_RAD, observerLongitude * DEG_2_RAD, observerAltitude, jdnow, razel, razelrates);

  if (ENABLE_LOG) {
    Serial.print("Raw razel: ");
    Serial.print(razel[0], 3);
    Serial.print(" km, ");
    Serial.print(razel[1] * RAD_2_DEG, 3);
    Serial.print("° az, ");
    Serial.print(razel[2] * RAD_2_DEG, 3);
    Serial.println("° el");
  }

  double azimuth_deg = razel[1] * RAD_2_DEG;
  double elevation_deg = razel[2] * RAD_2_DEG;
  double range_km = razel[0];
  if (azimuth_deg < 0) {
    azimuth_deg += 360.0;
  }

  if (ENABLE_LOG) {
    Serial.print("FINAL: Azimuth: ");
    Serial.print(azimuth_deg, 2);
    Serial.print("°, Elevation: ");
    Serial.print(elevation_deg, 2);
    Serial.print("°, Range: ");
    Serial.print(range_km, 2);
    Serial.println(" km");
    Serial.println("==================");
  }

  moveElevationTo((float)elevation_deg);
  moveAzimuthTo((float)azimuth_deg);

  // Update LCD less frequently to reduce I2C interference
  if (millis() - lastLcdUpdateTime >= LCD_UPDATE_INTERVAL) {
    updateLCD(azimuth_deg, elevation_deg, range_km);
    lastLcdUpdateTime = millis();
  }
}