/*
   Authors :    Ömer Şiar Baysal
                ESP-RFID Community

   Released to Public Domain

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
 */

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TimeLib.h>
#include <Ticker.h>
#include "Ntp.h"
#include <AsyncMqttClient.h>

#define DEBUG

#ifdef OFFICIALBOARD

#include <Wiegand.h>

WIEGAND wg;
int relayPin = 13; //D7


#endif

#ifndef OFFICIALBOARD

#include <MFRC522.h>
#include "PN532.h"
#include <Wiegand.h>

MFRC522 mfrc522 = MFRC522();
PN532 pn532;
WIEGAND wg;

int rfidss;
int readerType;
int relayPin;
int relayPinClose = 14; //D5
int relayPinBell = 12; //D6
int relayAlarmPin = 13; //D7
int watchdogPin = 10; //D8
int relayExtendPin = HIGH;
#endif

// these are from vendors
#include "webh/glyphicons-halflings-regular.woff.gz.h"
#include "webh/required.css.gz.h"
#include "webh/required.js.gz.h"

// these are from us which can be updated and changed
#include "webh/esprfid.js.gz.h"
#include "webh/esprfid.htm.gz.h"
#include "webh/index.html.gz.h"

#ifdef ESP8266
extern "C"
{
#include "user_interface.h"
}
#endif

NtpClient NTP;
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiDisconnectHandler;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Variables for whole scope
const char *http_username = "admin";
char *http_pass = NULL;
unsigned long previousMillis = 0;
unsigned long previousLoopMillis = 0;
unsigned long currentMillis = 0;
unsigned long cooldown = 0;
unsigned long deltaTime = 0;
unsigned long uptime = 0;
bool shouldReboot = false;
bool activateRelay = false;
bool activateRelayClose = false;
bool activateRelayBell = false;
bool deactivateRelay = false;
bool deactivateRelayClose = false;
bool deactivateRelayBell = false;
bool LatchingRelay = false;
bool dealatchRelay = false;
bool inAPMode = false;
bool isWifiConnected = false;
unsigned long autoRestartIntervalSeconds = 0;

bool wifiDisabled = true;
bool doDisableWifi = false;
bool doEnableWifi = false;
bool timerequest = false;
bool formatreq = false;
unsigned long wifiTimeout = 0;
unsigned long wiFiUptimeMillis = 0;
char *deviceHostname = NULL;


int WrongInputs = 0;
int WrongInputsAllowed = 3;
unsigned long WrongInputsBlock = 10000;
unsigned long WrongInputsBlockStartMillis = 0;
bool LockInput = false;

unsigned long WatchdogTime = 5000;
unsigned long WatchdogTimeStartMillis = millis ();
bool WatchdogEn = false;

int mqttenabled = 0;
char *mqttTopic = NULL;
char *mhs = NULL;
int mport;

int relayType;
unsigned long activateTime;
int timeZone;
int hitung;
int count = 1;

unsigned long nextbeat = 0;
unsigned long interval = 1800;

#include "log.esp"
#include "mqtt.esp"
#include "helpers.esp"
#include "wsResponses.esp"
#include "rfid.esp"
#include "wifi.esp"
#include "config.esp"
#include "websocket.esp"
#include "webserver.esp"

void ICACHE_FLASH_ATTR setup() {
#ifdef OFFICIALBOARD
	// Set relay pin to LOW signal as early as possible
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	delay(200);
#endif

#ifndef OFFICIALBOARD
	// Set relay pin to LOW signal as early as possible
	pinMode(13, OUTPUT);
	digitalWrite(13, relayExtendPin);
	
	pinMode(relayPinClose, OUTPUT);
	digitalWrite(relayPinClose, relayExtendPin);
	
	pinMode(relayPinBell, OUTPUT);
	digitalWrite(relayPinBell, relayExtendPin);
	
	pinMode(relayAlarmPin, OUTPUT);
	digitalWrite(relayAlarmPin, relayExtendPin);
	
	pinMode(watchdogPin, OUTPUT);
	digitalWrite(watchdogPin, LOW);
	
	delay(200);
#endif

#ifdef DEBUG
	Serial.begin(115200);
	Serial.println();
	Serial.println(F("[ INFO ] ESP RFID v0.8"));
	
	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();
	Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
	Serial.printf("Flash real size: %u\n\n", realSize);
	Serial.printf("Flash ide  size: %u\n", ideSize);
	Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
	Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
	if (ideSize != realSize) {
		Serial.println("Flash Chip configuration wrong!\n");
	} else {
		Serial.println("Flash Chip configuration ok.\n");
	}
#endif

	if (!SPIFFS.begin()) {
#ifdef DEBUG
		Serial.print(F("[ WARN ] Formatting filesystem..."));
#endif
		if (SPIFFS.format()) {
			writeEvent("WARN", "sys", "Filesystem formatted", "");

#ifdef DEBUG
			Serial.println(F(" completed!"));
#endif
		} else {
#ifdef DEBUG
			Serial.println(F(" failed!"));
			Serial.println(F("[ WARN ] Could not format filesystem!"));
#endif
		}
	}
	wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
	if (!loadConfiguration()) {
		fallbacktoAPMode();
	}
	setupWebServer();
	writeEvent("INFO", "sys", "System setup completed, running", "");
}

void ICACHE_RAM_ATTR loop() {
	currentMillis = millis();
	deltaTime = currentMillis - previousLoopMillis;
	uptime = NTP.getUptimeSec();
	previousLoopMillis = currentMillis;
	

	if ((currentMillis - WrongInputsBlockStartMillis >=  WrongInputsBlock) && WrongInputsBlockStartMillis >0)
  {
		WrongInputsBlockStartMillis = 0;
		LockInput = false;
		WrongInputs = 0;
		#ifdef DEBUG
			Serial.print("mili : ");
			Serial.println(millis());
			Serial.println("Lock Reset");
			Serial.print("WrongInputs : ");
			Serial.println(WrongInputs);
		#endif

  }

	if ((currentMillis - WatchdogTimeStartMillis >= WatchdogTime) && !WatchdogEn)
  {
		digitalWrite(watchdogPin, HIGH);
		WatchdogEn = true;
		#ifdef DEBUG
			Serial.print("mili : ");
			Serial.println(millis());
			Serial.println("Watchdog");
			Serial.print("WrongInputs : ");
			Serial.println(WrongInputs);
		#endif
		
  }
  	if ((currentMillis - WatchdogTimeStartMillis >= WatchdogTime + 100) && WatchdogEn)
  {
		digitalWrite(watchdogPin, LOW);
		WatchdogEn = false;
		#ifdef DEBUG
			Serial.print("mili : ");
			Serial.println(millis());
			Serial.println("Watchdog Release");
		#endif
		WatchdogTimeStartMillis = currentMillis;
  }

	if (currentMillis >= cooldown) {
		
		
		rfidloop();
	

	switch (hitung){
		case 1 :
		if	(LatchingRelay){
			digitalWrite(relayPin, !relayType);
			LatchingRelay = false;
			dealatchRelay = true;
		}
		break;

		case 2:
		if (dealatchRelay){
			digitalWrite(relayPin, relayType);
		}
	default :
	count = 1 ;
	break;
	
		}
	}
		if (activateRelay) {
		#ifdef DEBUG
		Serial.print("mili : ");
		Serial.println(millis());
		Serial.println("activating relay now");
		#endif
		digitalWrite(relayPin, !relayType);
		previousMillis = millis();
		activateRelay = false;
		deactivateRelay = true;
	} else if((currentMillis - previousMillis >= activateTime) && (deactivateRelay)) {
		#ifdef DEBUG
		Serial.println(currentMillis);
		Serial.println(previousMillis);
		Serial.println(activateTime);
		Serial.println(activateRelay);
		Serial.println("deactivate relay after this");
		Serial.print("mili : ");
		Serial.println(millis());
		#endif
		digitalWrite(relayPin, relayType);
		deactivateRelay = false;
	}
		if (activateRelayClose) {
		#ifdef DEBUG
		Serial.print("mili : ");
		Serial.println(millis());
		Serial.println("activating close relay now");
		#endif
		digitalWrite(relayPinClose, !relayExtendPin);
		previousMillis = millis();
		activateRelayClose = false;
		deactivateRelayClose = true;
	} else if((currentMillis - previousMillis >= activateTime) && (deactivateRelayClose)) {
		#ifdef DEBUG
		Serial.println(currentMillis);
		Serial.println(previousMillis);
		Serial.println(activateTime);
		Serial.println(activateRelay);
		Serial.println("deactivate close relay after this");
		Serial.print("mili : ");
		Serial.println(millis());
		#endif
		digitalWrite(relayPinClose, relayExtendPin);
		deactivateRelayClose = false;
	}
		if (activateRelayBell) {
		#ifdef DEBUG
		Serial.print("mili : ");
		Serial.println(millis());
		Serial.println("activating Bell relay now");
		#endif
		digitalWrite(relayPinBell, !relayExtendPin);
		previousMillis = millis();
		activateRelayBell = false;
		deactivateRelayBell = true;
	} else if((currentMillis - previousMillis >= activateTime) && (deactivateRelayBell)) {
		#ifdef DEBUG
		Serial.println(currentMillis);
		Serial.println(previousMillis);
		Serial.println(activateTime);
		Serial.println(activateRelay);
		Serial.println("deactivate Bell relay after this");
		Serial.print("mili : ");
		Serial.println(millis());
		#endif
		digitalWrite(relayPinBell, relayExtendPin);
		deactivateRelayBell = false;
	}	

	if (formatreq) {
#ifdef DEBUG
		Serial.println(F("[ WARN ] Factory reset initiated..."));
#endif
		SPIFFS.end();
		ws.enable(false);
		SPIFFS.format();
		ESP.restart();
	}
	
	if (timerequest) {
		timerequest = false;
		sendTime();
	}

	if (autoRestartIntervalSeconds > 0 && uptime > autoRestartIntervalSeconds * 1000) {
		writeEvent("INFO", "sys", "System is going to reboot", "");
#ifdef DEBUG
		Serial.println(F("[ WARN ] Auto restarting..."));
#endif
		shouldReboot = true;
	}
	
	if (shouldReboot) {
		writeEvent("INFO", "sys", "System is going to reboot", "");
#ifdef DEBUG
		Serial.println(F("[ INFO ] Rebooting..."));
#endif
		ESP.restart();
	}
	
	if (isWifiConnected) {
		wiFiUptimeMillis += deltaTime;
	}
	
	if (wifiTimeout > 0 && wiFiUptimeMillis > (wifiTimeout * 1000) && isWifiConnected == true) {
		writeEvent("INFO", "wifi", "WiFi is going to be disabled", "");
		doDisableWifi = true;
	}
	
	if (doDisableWifi == true) {
		doDisableWifi = false;
		wiFiUptimeMillis = 0;
		disableWifi();
	} else if (doEnableWifi == true) {
		writeEvent("INFO", "wifi", "Enabling WiFi", "");
		doEnableWifi = false;
		if (!isWifiConnected) {
			wiFiUptimeMillis = 0;
			enableWifi();
		}
	}
	
	if (mqttenabled == 1) {
		if (mqttClient.connected()) {
				if ((unsigned)now() > nextbeat) {
				mqtt_publish_heartbeat(now());
				nextbeat = (unsigned)now() + interval;
#ifdef DEBUG
				Serial.print("[ INFO ] Nextbeat=");
				Serial.println(nextbeat);
#endif
			}
		}
	}
	
}
