#include "Particle.h"

// Port of TinyGPS for the Particle AssetTracker
// https://github.com/mikalhart/TinyGPSPlus
#include "TinyGPS++.h"
#include <ArduinoJson.h>
#include "LIS3DH.h"

PRODUCT_ID(5435);
PRODUCT_VERSION(5);

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object directly.
 */

LIS3DHSPI accel(SPI, A2, WKP);

unsigned long PUBLISH_PERIOD = 15 * 60 * 1000;
unsigned long ALARM_PERIOD = 1 * 60 * 1000;

const unsigned long SERIAL_PERIOD = 5000;
const unsigned long MAX_GPS_AGE_MS = 10000; // GPS location must be newer than this to be considered valid

FuelGauge fuel;

// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastSerial = 0;
unsigned long lastPublish = 0;
unsigned long lastAlarm = millis();
unsigned long startFix = 0;
bool gettingFix = false;

volatile bool positionInterrupt = false;
int movementThreshold = 96;


void setup()
{
	Serial.begin(9600);

	// The GPS module on the AssetTracker is connected to Serial1 and D6
	Serial1.begin(9600);

	// Settings D6 LOW powers up the GPS module
  pinMode(D6, OUTPUT);
  digitalWrite(D6, LOW);
  startFix = millis();
  gettingFix = true;

  delay(5000);

  attachInterrupt(WKP, positionInterruptHandler, RISING);
  // Initialize sensors
  LIS3DHConfig config;
  config.setAccelMode(LIS3DH::RATE_100_HZ);
  config.setPositionInterrupt(movementThreshold);

  bool setupSuccess = accel.setup(config);
  Serial.printlnf("accel setup success: %d", setupSuccess);

  Particle.connect();
  Particle.function("delay", delayMin);
	Particle.function("alarmDelay", alarmDelayMin);
	//Particle.function("moThresh", moThresh);
  Particle.function("batt", batteryStatus);
  Particle.function("pubGps", pubGps);
  Particle.syncTime();
}

void loop()
{
	while (Serial1.available() > 0) {
		if (gps.encode(Serial1.read())) {
			process();
		}
		Particle.process();
	}

	//Serial.println("go to sleep");
	//System.sleep(20000);
	//delay(10000);
	//Serial.println("waked up");
	//if(!Particle.connected)
	//	Particle.connect();
	//Serial1.flush();
}

void process() {
	if (millis() - lastSerial >= SERIAL_PERIOD) {
		lastSerial = millis();

		char buf[128];
		if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS) {
			snprintf(buf, sizeof(buf), "%f,%f,%f", gps.location.lat(), gps.location.lng(), gps.altitude.meters());
			if (gettingFix) {
				gettingFix = false;
				unsigned long elapsed = millis() - startFix;
				Serial.printlnf("%lu milliseconds to get GPS fix", elapsed);
			}
		}
		else {
			strcpy(buf, "no location");
			if (!gettingFix) {
				gettingFix = true;
				startFix = millis();
			}
		}
		Serial.println(buf);

		if (Particle.connected()) {
			if ((millis() - lastPublish >= PUBLISH_PERIOD)||(positionInterrupt == true)) {
				lastPublish = millis();
        pubJson();
        positionInterrupt = false;
			}
		}
	}
}

int pubJson() {
  if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS) {
    StaticJsonBuffer<700> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();

    char lat[30];
    sprintf(lat, "%f", gps.location.lat());
    root["la"] = lat;

    char lon[30];
    sprintf(lon, "%f", gps.location.lng());
    root["lo"] = lon;

    //root["s"] = t.getSpeed();
    root["a"] = gps.altitude.meters();

    LIS3DHSample sample;
		if (accel.getSample(sample)) {
      //root["acm"] = t.readXYZmagnitude();
      root["acx"] = sample.x;
      root["acy"] = sample.y;
      root["acz"] = sample.z;
    }

    root["fv"] = fuel.getVCell();
    root["fc"] = fuel.getSoC();
    root["pi"] = positionInterrupt;

		root["sp"] = gps.speed.kmph();
		root["co"] = gps.course.deg();

		if(gps.satellites.isValid()) {
			root["sa"] = gps.satellites.value();
			root["saa"] = gps.satellites.age();
		}

		if(gps.hdop.isValid()) {
			root["hd"] = gps.hdop.value();
			root["hda"] = gps.hdop.age();
		}

    time_t time = Time.now();
    String ts = Time.format(time, "%Y-%m-%dT%H:%M:%S.000Z");
    char tsBuff[50];
    sprintf(tsBuff, "%s", (const char*) ts);
    root["ts"] = tsBuff;

    char jsonChar[700];
    root.printTo(jsonChar, sizeof(jsonChar));
    Particle.publish("JG", jsonChar, 300, PRIVATE);
    return 1;
  }
  else
    return 0;
}

int pubGps(String command){
  return pubJson();
}

// Lets you remotely check the battery status by calling the function "batt"
// Triggers a publish with the info (so subscribe or watch the dashboard)
// and also returns a '1' if there's >10% battery left and a '0' if below
int batteryStatus(String command){
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("B",
          "v:" + String::format("%.2f",fuel.getVCell()) +
          ",c:" + String::format("%.2f",fuel.getSoC()),
          60, PRIVATE
    );
    // if there's more than 10% of the battery left, then return 1
    if (fuel.getSoC()>10){ return 1;}
    // if you're running out of battery, return 0
    else { return 0;}
}

int delayMin(String command) {
	  int ci = atoi(command);
		if (ci > 0)
    	PUBLISH_PERIOD = ci * 60 * 1000;
    return PUBLISH_PERIOD;
}

int alarmDelayMin(String command) {
		int ci = atoi(command);
		if (ci > 0)
    	ALARM_PERIOD = ci * 60 * 1000;
    return ALARM_PERIOD;
}

void positionInterruptHandler() {
	if(gettingFix) {
		Serial.println("alarm triggerd - no fix");
	}
	else {
		Serial.println("alarm triggerd - with fix");
		if ((millis() - lastAlarm) > ALARM_PERIOD) {
			lastAlarm = millis();
			Serial.println("alarm timeout");
			positionInterrupt = true;
		}
	}
}
