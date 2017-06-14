

#include <ESP8266WiFi.h>

#include <DHT.h>
#include "fauxmoESP.h"

fauxmoESP fauxmo;

#define WIFI_SSID "entropy"
#define WIFI_PASS "xxxxxx"

#define DHTPIN 2          // What digital pin we're connected to
#define DHTTYPE DHT11     // DHT 11
DHT dht(DHTPIN, DHTTYPE);

int garageEvent = 3; // undefined

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}


void setup()
{

  Serial.begin(115200);
  Serial.println("Starting....");
    Serial.println("garage_04_alexa 070617");
  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, INPUT);

  // -----------------------------------------------------------------------------
  // Wifi
  // -----------------------------------------------------------------------------

  // Set WIFI module to STA mode
  WiFi.mode(WIFI_STA);

  // Connect
  //Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  //WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait
  int wifiwait = 0;

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    wifiwait++;
    digitalWrite(BUILTIN_LED, LOW);
   // digitalWrite(5, HIGH);
    delay(200);
    digitalWrite(BUILTIN_LED, HIGH);
  //  digitalWrite(5, LOW);
    delay(200);
    if (wifiwait > 100) {
      wifiwait = 0;
      WiFi.disconnect();
      delay(100);
      Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }


  Serial.println();

  // Connected!
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());


  //digitalWrite(BUILTIN_LED, HIGH);


  //  timer.setInterval(600000L, sendSensor);  // every 10min send DHT metric
  //  timer.setInterval(1000L, checkDoorSensor);  // every 1s check door sensor

  fauxmo.addDevice("garage");


  //
  fauxmo.onMessage([](unsigned char device_id, const char * device_name, bool state) {

    Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");

    if ((strcmp(device_name, "garage") == 0)) {
      garageEvent = state ? 1 : 0;

    }
  
});

}




  //      if (state == true) {
  //        Serial.println("Garage: received request for HIGH - open garage");
  //
  //        if (DoorState == 1) {
  //          Serial.println("Door already open - no action");
  //        }  else {
  //
  //          Serial.println("Opening garage - toggle for 250ms");
  //          digitalWrite(5, HIGH);
  //          digitalWrite(BUILTIN_LED, HIGH);
  //
  //          static unsigned long last = millis();
  //          while (last + 300 > millis()) {
  //            ESP.wdtFeed();
  //          }
  //
  //          digitalWrite(5, LOW);
  //          digitalWrite(BUILTIN_LED, LOW);
  //          Serial.println("toggle done.");
  //        }
  //      } else {
  //
  //        Serial.println("Garage: received request for LOW - close garage");
  //
  //        if (DoorState == 0) {
  //          Serial.println("Door already closed - no action");
  //        }  else {
  //
  //          Serial.println("Closing garage - toggle for 250ms");
  //          digitalWrite(5, HIGH);
  //          digitalWrite(BUILTIN_LED, HIGH);
  //
  //          static unsigned long last = millis();
  //          while (last + 300 > millis()) {
  //            ESP.wdtFeed();
  //          }
  //
  //          digitalWrite(5, LOW);
  //          digitalWrite(BUILTIN_LED, LOW);
  //          digitalWrite(BUILTIN_LED, 0);
  //          Serial.println("toggle done.");
  //        }
  //
  //
  //      }





void loop()
{
  //  timer.run(); // Initiates SimpleTimer

  fauxmo.handle();
  ESP.wdtFeed();
  static unsigned long last_loop = millis();
  if (millis() - last_loop > 5000) {
    last_loop = millis();
    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
    int DoorState = digitalRead(4);
    Serial.print("Door Status: "); Serial.println (DoorState);
  }

  if (garageEvent != 3) {
  bool state = garageEvent ? true : false;
  int DoorState = digitalRead(4);
    Serial.print("Door Status: "); Serial.print (DoorState); Serial.print(" State: "); Serial.println (state);

    // state: false = door to close, true = to open
    // DoorState" 1 = open, 0 = closed

    if (((DoorState == 1) and (state == false) or (DoorState == 0) and (state == true))) {

      Serial.println("###### Toggeling garage");
      pinMode(5, OUTPUT);
      //pinMode(4, INPUT);
      digitalWrite(5, HIGH);
      digitalWrite(BUILTIN_LED, LOW);

      //        static unsigned long last = millis();
      //        while (last + 300 > millis()) {
      //          ESP.wdtFeed();
      //        }

      delay(300);

      digitalWrite(5, LOW);
      digitalWrite(BUILTIN_LED, HIGH);
      //    digitalWrite(BUILTIN_LED, 0);
      Serial.println("toggle done.");

    } else {Serial.println("No action...");}





    garageEvent = 3;
  }

}
