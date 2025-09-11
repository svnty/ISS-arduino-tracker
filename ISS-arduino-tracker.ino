#include <Arduino.h>
#include <ArduinoJson.h>
// #include <AS5600.h>
#include <LiquidCrystal_I2C.h>
#include <PWMServo.h>
#include <QMC5883LCompass.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <TinyGPSPlus.h>
#include <WiFiEspAT.h>

#include "SGP4_vallado/sgp4coord.cpp"
#include "SGP4_vallado/sgp4coord.h"
#include "SGP4_vallado/sgp4ext.cpp"
#include "SGP4_vallado/sgp4ext.h"
#include "SGP4_vallado/sgp4unit.cpp"
#include "SGP4_vallado/sgp4unit.h"

// -------- SYSTEM TOGGLES --------
static const bool ENABLE_WIFI = true;
static const bool ENABLE_LOG = true;
static const bool ENABLE_GPS = false;

// -------- CONSTANTS --------
static const uint8_t AZIMUTH_DIR_PIN = 22;
static const uint8_t AZIMUTH_STEP_PIN = 23;
static const uint8_t GPS_RX_PIN = 12;
static const float TMC2209_RESISTANCE = 0.11;
static const char *WIFI_SSID = "Jakes iPhone", *WIFI_PASSWORD = "izeh5zemxa05r";
static const double DEG_2_RAD = pi / 180.0;
static const double RAD_2_DEG = 180.0 / pi;
static const int ELEVATION_PIN = SERVO_PIN_C;
static const float DEFAULT_DECLINATION = 0.0;  // Update this with your local magnetic declination as fallback
static const char* DECLINATION_HOST = "www.ngdc.noaa.gov";
static const char* DECLINATION_API_KEY = "zNEw7";
static const char* TLE_API_HOST = "celestrak.com";

// -------- VARIABLES --------
// timers
static unsigned long currentTime = 0;
static unsigned long lastLcdUpdateTime = 1000;
static unsigned long lastDeclinationUpdateTime = 0;
static unsigned long wifiTimeout = 10000;
static unsigned long lastWiFiReconnectAttempt = 0;
static unsigned long wifiReconnectInterval = 5000;
// http data
static bool foundISSbyWifi = false;
static float magneticDeclination = 0.0;
static char tle_line1[70];
static char tle_line2[70];
static bool tleParsed = false;
// gps data
static double observerLatitude = 0.0, observerLongitude = 0.0, observerAltitude = 0.0;
static int year, month, day, hour, minute, second;
static bool ISSIsVisible = true;
static int8_t direction = 1;
// motor control
static float currentAzimuth = 0.0;
static float currentElevation = 90.0;                                  // Start at horizon
static const float AZIMUTH_STEPS_PER_DEGREE = (200.0 * 16.0) / 360.0;  // 200 steps * 16 microsteps / 360 degrees

// -------- SERIALS --------
#define AZIMUTH_SERIAL Serial1
#define WIFI_SERIAL Serial3
static SoftwareSerial GPS_SERIAL(GPS_RX_PIN, -1);

// -------- OBJECTS --------
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);
TMC2209Stepper azimuthDriver(&AZIMUTH_SERIAL, TMC2209_RESISTANCE, 0);
// AS5600 as5600;
QMC5883LCompass compass;
elsetrec satrec;
PWMServo elevation;
WiFiSSLClient sslClient;

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

void makeTleApiRequest() {
  Serial.println("Making HTTPS API request...");

  WiFiSSLClient client;

  Serial.print("Connecting to ");
  Serial.print(TLE_API_HOST);
  Serial.print(":");
  Serial.println(443);

  if (!client.connect(TLE_API_HOST, 443)) {
    Serial.println("Connection to server failed!");
    lcdSetSecondLine("TLE ERROR");
    delay(2000);
    return;
  }

  Serial.println("Connected to server");

  String request = "GET " + String("/NORAD/elements/stations.txt") + " HTTP/1.1\r\n";
  request += "Host: " + String(DECLINATION_HOST) + "\r\n";
  request += "User-Agent: Arduino/1.0\r\n";
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

  if (!foundISSbyWifi) {
    lcdSetSecondLine("TLE ERROR");
    return;
  }

  parseTLEData(httpResponse);
}

void parseTLEData(String data) {
  Serial.println("Parsing TLE data...");

  int lineCount = 0;
  int startIndex = 0;
  String lines[100];

  while (startIndex < data.length() && lineCount < 100) {
    int endIndex = data.indexOf('\n', startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
    }

    String line = data.substring(startIndex, endIndex);
    line.trim();

    if (line.length() > 0) {
      lines[lineCount] = line;
      lineCount++;
    }

    startIndex = endIndex + 1;
  }

  // Look for ISS (ZARYA) and parse its TLE data
  for (int i = 0; i < lineCount - 2; i++) {
    if (lines[i].indexOf("ISS (ZARYA)") >= 0) {
      Serial.println("Found ISS (ZARYA) data!");

      strcpy(tle_line1, lines[i + 1].c_str());
      strcpy(tle_line2, lines[i + 2].c_str());

      Serial.println("TLE data parsed successfully!");
      lcdSetSecondLine("TLE OK");
      foundISSbyWifi = true;
      break;
    }
  }
}

void makeDeclinationApiRequest() {
  Serial.println("Making HTTPS API request...");

  WiFiSSLClient client;

  Serial.print("Connecting to ");
  Serial.print(DECLINATION_HOST);
  Serial.print(":");
  Serial.println(443);

  if (!client.connect(DECLINATION_HOST, 443)) {
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
  request += "User-Agent: Arduino/1.0\r\n";
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
        float declination = result["declination"];
        Serial.print("Magnetic Declination: ");
        Serial.print(declination, 5);
        Serial.println("°");
      }
    }
  }

  storeDeclinationDataForUse(doc);
}

void storeDeclinationDataForUse(JsonDocument& doc) {
  if (doc["result"].is<JsonArray>()) {
    JsonArray results = doc["result"];
    if (results.size() > 0) {
      JsonObject result = results[0];

      if (!result["declination"].isNull()) {
        magneticDeclination = result["declination"];
        lcdSetSecondLine("DECLINATION OK");
        delay(2000);
      }
    }
  }

  lastDeclinationUpdateTime = millis();
}

void parseISSTLE(const char* line1, const char* line2, elsetrec& satrec) {
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

void homeAzimuthToTrueNorth() {
  lcdSetFirstLine("CALIBRATING");
  lcdSetSecondLine("AZIMUTH");

  compass.read();
  float magneticHeading = compass.getAzimuth();

  // Calculate true heading by applying declination correction
  float trueHeading = magneticHeading + magneticDeclination;

  // Normalize to 0-360 degrees
  if (trueHeading < 0) trueHeading += 360;
  if (trueHeading >= 360) trueHeading -= 360;

  // Now rotate until we're pointing at true north (0 degrees)
  // The stepper motor should move the difference between current true heading and 0
  float angleToRotate = -trueHeading;  // Negative because we want to go to 0

  // Convert angle to steps (assuming 200 steps per revolution * 16 microsteps)
  long stepsToMove = (angleToRotate * (200 * 16)) / 360;

  // Move the stepper motor
  for (long i = 0; i < abs(stepsToMove); i++) {
    digitalWrite(AZIMUTH_DIR_PIN, stepsToMove > 0 ? HIGH : LOW);
    digitalWrite(AZIMUTH_STEP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(AZIMUTH_STEP_PIN, LOW);
    delayMicroseconds(100);
  }

  lcdSetFirstLine("OK");
  delay(2000);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void moveAzimuthTo(float targetAzimuth) {
  // Calculate the shortest path to target azimuth
  float angleDifference = targetAzimuth - currentAzimuth;

  // Handle wraparound (e.g., from 350° to 10°)
  if (angleDifference > 180) {
    angleDifference -= 360;
  } else if (angleDifference < -180) {
    angleDifference += 360;
  }

  // Only move if the difference is significant (>= 0.1 degrees)
  if (abs(angleDifference) >= 0.1) {
    long stepsToMove = (long)(angleDifference * AZIMUTH_STEPS_PER_DEGREE);

    // Set direction
    digitalWrite(AZIMUTH_DIR_PIN, stepsToMove > 0 ? HIGH : LOW);

    // Move the motor
    for (long i = 0; i < abs(stepsToMove); i++) {
      digitalWrite(AZIMUTH_STEP_PIN, HIGH);
      delayMicroseconds(500);  // Adjust speed as needed
      digitalWrite(AZIMUTH_STEP_PIN, LOW);
      delayMicroseconds(500);
    }

    // Update current position
    currentAzimuth = targetAzimuth;

    // Normalize to 0-360 range
    if (currentAzimuth < 0) currentAzimuth += 360;
    if (currentAzimuth >= 360) currentAzimuth -= 360;
  }
}

void moveElevationTo(float targetElevation) {
  // Convert elevation angle to servo position
  // Your system: horizon = 90°, up = 0°, down = 180°
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

// -------- ARDUINO --------
void setup() {
  Serial.begin(115200);

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  lcdSetFirstLine("MEGA2560 INIT");
  delay(1000);

  // GPS Initialization
  lcdSetFirstLine("GPS INIT");
  if (ENABLE_GPS) {
    GPS_SERIAL.begin(9600);
    unsigned long gpsTimeoutTimer = millis();
    while (!gps.location.isValid()) {
      if (currentTime - gpsTimeoutTimer > 30000) {
        lcdSetSecondLine("ERROR");
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
    lcdSetSecondLine("LOCATION OK");
    delay(1000);

    while (!gps.date.isValid()) {
      if (currentTime - gpsTimeoutTimer > 30000) {
        lcdSetSecondLine("ERROR");
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
    lcdSetSecondLine("DATE OK");
    delay(1000);

    year = gps.date.year();
    month = gps.date.month();
    day = gps.date.day();
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
    observerLatitude = gps.location.lat();
    observerLongitude = gps.location.lng();
    observerAltitude = gps.altitude.kilometers();
    lcdClear();
  } else {
    lcdSetSecondLine("DISABLED");
    delay(2000);

    year = 2025;
    month = 9;
    day = 13;
    hour = 12;
    minute = 0;
    second = 0;
    observerLatitude = -33.88336;
    observerLongitude = 152.20148;
    observerAltitude = 0.035;
    lcdClear();
  }

  // WiFi Initialization
  if (ENABLE_WIFI) {
    WIFI_SERIAL.begin(115200);
    WiFi.init(&WIFI_SERIAL);
    lcdSetFirstLine("WIFI INIT");
    connectToWiFi();

    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWiFiReconnectAttempt > wifiReconnectInterval) {
        connectToWiFi();
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      lcdSetFirstLine("FETCH HTTP DATA");
      lcdSetSecondLine("ISS TLE");
      makeTleApiRequest();

      lcdSetSecondLine("DECLINATION");
      makeDeclinationApiRequest();
    } else {
      lcdSetSecondLine("ERROR");
    }
  }
  delay(5000);
  lcdClear();

  // Azimuth Motor Initialization
  pinMode(AZIMUTH_STEP_PIN, OUTPUT);
  pinMode(AZIMUTH_DIR_PIN, OUTPUT);
  digitalWrite(AZIMUTH_DIR_PIN, HIGH);
  AZIMUTH_SERIAL.begin(115200);
  azimuthDriver.begin();
  azimuthDriver.rms_current(900);
  azimuthDriver.microsteps(16);
  azimuthDriver.pwm_autoscale(true);

  // Elevation Motor Initialization
  elevation.attach(ELEVATION_PIN);
  elevation.write(90);  // Start at horizon position
  currentElevation = 90.0;

  // Magnometer Initialization
  lcdSetFirstLine("QMC5883L INIT");
  compass.init();
  delay(2000);

  homeAzimuthToTrueNorth();
  lcdClear();

  if (!foundISSbyWifi) {
    strcpy(tle_line1, "1 25544U 98067A   25249.87397102  .00012937  00000+0  23296-3 0  9995");
    strcpy(tle_line2, "2 25544  51.6325 262.1963 0004212 309.4705  50.5911 15.50156361527839");
  }
}

void loop() {
  currentTime = millis();

  if (!tleParsed) {
    parseISSTLE(tle_line1, tle_line2, satrec);
    tleParsed = true;
  }

  double jdnow;
  jday(year, month, day, hour, minute, second, jdnow);
  double tsince = (jdnow - satrec.jdsatepoch) * 24.0 * 60.0;
  double ro[3], vo[3];
  sgp4(wgs72, satrec, tsince, ro, vo);

  if (satrec.error != 0) {
    Serial.print("SGP4 Error: ");
    Serial.println(satrec.error);
    delay(1000);
    return;
  }

  double razel[3], razelrates[3];
  rv2azel(ro, vo, observerLatitude * DEG_2_RAD, observerLongitude * DEG_2_RAD, observerAltitude, jdnow, razel, razelrates);

  double azimuth_deg = razel[1] * RAD_2_DEG;
  double elevation_deg = razel[2] * RAD_2_DEG;
  double range_km = razel[0];
  if (azimuth_deg < 0) {
    azimuth_deg += 360.0;
  }

  // Update LCD less frequently to reduce I2C interference
  if (currentTime - lastLcdUpdateTime >= 2000) {  // Every 2 seconds instead of 1
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
    lastLcdUpdateTime = currentTime;
  }

  moveAzimuthTo((float)azimuth_deg);
  moveElevationTo((float)elevation_deg);
}
