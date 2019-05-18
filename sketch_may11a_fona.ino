// Libraries
#include <TinyGPS++.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

TinyGPSPlus gps;

// Intializing GPS & GSM Pins
static const int RX_GPS = 5, TX_GPS = 6, FONA_VR = 2, FONA_VT = 3, FONA_RST = 4;

// Baud rates
static const uint32_t GPSBaud = 9600;

// Location variables
double latitudeNumeric;
double longitudeNumeric;

// Instances
SoftwareSerial fonaSS = SoftwareSerial(FONA_VT, FONA_VR);
SoftwareSerial ssGPS = SoftwareSerial(TX_GPS, RX_GPS);

SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint16_t vbat;
String dweetThing = "gps_tracker"; // Dweet.io Thing

void setup() {
  Serial.begin(9600);
  ssGPS.begin(GPSBaud);
  delay(100);

  initGSM();
  initGPRS();

  printBatteryStatus();
  testAPI();
}

void loop() {
  getGPSInfo();
  delay(500);
  //testAPI();
}
 
void getGPSInfo() {
  while (ssGPS.available() > 0) {
    gps.encode(ssGPS.read());
    if (gps.location.isUpdated()) {
      latitudeNumeric = (gps.location.lat(), 6);
      Serial.print("Latitude = ");
      Serial.print(gps.location.lat(), 6);

      longitudeNumeric = (gps.location.lng(), 6);
      Serial.print(" Longitude = ");
      Serial.println(gps.location.lng());
    }
  }
}

void initGSM() {
  while (!Serial);
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));
}

void initGPRS() {
  fona.setGPRSNetworkSettings(F("Web")); //fona.setGPRSNetworkSettings(F("your_APN"), F("your_username"), F("your_password"));
  fona.enableGPRS(true);
}

void printBatteryStatus() {
  if (! fona.getBattPercent(&vbat)) {
    Serial.println(F("Failed to read Batt"));
  } else {
    Serial.print(F("Battery status: ")); Serial.print(vbat); Serial.println(F("%"));
  }
}

void testAPI() {  
  // Prepare request
  uint16_t statuscode;
  int16_t length;
  char url[80];
  String request = "www.dweet.io/dweet/for/";
  request += dweetThing;
  request += "?latitude=" + String(latitudeNumeric);
  request += "&longitude=" + String(longitudeNumeric);
  request += "&battery=" + String(vbat);
  request.toCharArray(url, request.length());

  // Send location to Dweet.io
  if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
    Serial.println("Failed!");
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();
      // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
      loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
      UDR0 = c;
#else
      Serial.write(c);
#endif
      length--;
    }
  }
  fona.HTTP_GET_end();
}
