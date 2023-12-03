#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <Preferences.h> // Add the Preferences library for ESP32
#endif

#include <PubSubClient.h>
#include <AccelStepper.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Pinmap
#define S2MINI_LED 15
#define STEPPER_STEP_PIN 16
#define STEPPER_DIR_PIN 17
#define STEPPER_EN_PIN 18

// Stepper Parameter
#define STEPPER_STEPS_PER_REVELAZION 200
#define STEPPER_MICROSTEPS 64

//#define STEPPER_GEAR_RATIO 3.455 //91/200
//#define STEPPER_GEAR_RATIO 3.456 //57/125
#define STEPPER_GEAR_RATIO 3.46    //50/173 //because a 90 dev rev is divisable by 64, its 173 full steps

#define STEPPER_ACCEL 1000
#define STEPPER_ONE_ROT STEPPER_STEPS_PER_REVELAZION * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO
#define STEPPER_MAX_SPEED 200.0
//STEPPER_ONE_ROT / 0.2
#define STEPPER_INVERT_DIRECTION false

WiFiManager wifiManager; // Create a WiFiManager instance
WiFiClient espClient;
String deviceID;

const char* mqtt_server = "192.168.181.108";
const int mqtt_port = 1883;

int stepsPer90Degrees = STEPPER_ONE_ROT  /   4;
int stepsPer1Degrees  = STEPPER_ONE_ROT  / 360;

PubSubClient client(espClient);
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
Preferences preferences;

// Topic variables
String motorTopic;
String angleTopic;
String sleepTopic;
String movementDoneTopic;
String bootTopic;
String calibTopic;
int sleepTime = 0;

bool isMoving = false;
int lastDirection = 0;
int direction;

// Callback function: Processes incoming MQTT messages and performs corresponding actions
void callback(char* topic, byte* payload, unsigned int length) {
  led_flash();

  if (isMoving) {
    led_flash();
    Serial.println("Still moving!");
  } else {
    String message;
    for (unsigned int i = 0; i < length; i++) {
      message += (char)payload[i];
    }

    if (String(topic) == motorTopic) {
      Serial.println("Motor command recieved!");
      Serial.println(String(message));
      // Restore position from preferences
      preferences.begin("StepperPrefs"); // Use a namespace for preferences
      int currentPosition = preferences.getInt("currentPosition", 0);
      bool positionLost = preferences.getBool("isMoving", false);
      if (positionLost) {
        client.publish(movementDoneTopic.c_str(), "0");
        return;
      }
      stepper.setCurrentPosition(currentPosition);

      int newPosition = String(message).toInt() * stepsPer90Degrees * direction;
      Serial.println("runToPosition " + String(newPosition));
      stepper.moveTo(newPosition);
      
    } else if (String(topic) == calibTopic) {
      int calibPosition = String(message).toInt() * stepsPer90Degrees * direction;
      // Reset moving state if device crashes during movement.
      preferences.begin("StepperPrefs");
      preferences.putBool("isMoving", false);
      preferences.putInt("currentPosition", calibPosition);
      preferences.end();

    } else if (String(topic) == sleepTopic) {
      // Send device into deepSleep
      sleepTime = message.toInt() * 1000000;
      Serial.println("Sleep command recieved!");

    } else if (String(topic) == angleTopic) {
      int moveRel = String(message).toInt() * STEPPER_MICROSTEPS * 8; //full steps
      stepper.move(moveRel * direction);
    }

    isMoving = stepper.isRunning();
    if (isMoving) {
      int d = stepper.distanceToGo();
      Serial.println(d);
      if (d > 0) { lastDirection =  1; } 
      if (d < 0) { lastDirection = -1; }
      preferences.putBool("isMoving", true);
      stepper.enableOutputs();
    } else {
      // Report command done
      client.publish(movementDoneTopic.c_str(), "done");
    }
  }
}

void stepperUnload() {
  int d = 0;
  if (lastDirection == -1) { d =  1; }
  if (lastDirection ==  1) { d = -1; }
  stepper.move(d * STEPPER_MICROSTEPS * 2); // teke load from shaft before turning off
  isMoving = stepper.isRunning();
  Serial.println(stepper.distanceToGo());
}

void stepperDone() {
  stepper.disableOutputs();
  preferences.putBool("isMoving", false);
  Serial.println("runToPosition done");

  // Save position in preferences
  int currentPosition = stepper.currentPosition();
  preferences.putInt("currentPosition", currentPosition);
  Serial.println("settings saved");

  preferences.end();
  Serial.println("preferences end");

  // Report movement done
  char msg_out[20];
  dtostrf(currentPosition / STEPPER_STEPS_PER_REVELAZION / STEPPER_MICROSTEPS / STEPPER_GEAR_RATIO, 2, 2, msg_out);
  client.publish(movementDoneTopic.c_str(), msg_out);
}

// Reconnect function: Connects to the MQTT server and subscribes to the required topics
void reconnect() {
  Serial.println("Reconnect!");
  while (!client.connected()) {
    if (client.connect(deviceID.c_str())) {
      client.subscribe(motorTopic.c_str());
      client.subscribe(angleTopic.c_str());
      client.subscribe(sleepTopic.c_str());
      client.subscribe(calibTopic.c_str());
      client.publish(bootTopic.c_str(),"0");
    } else {
      delay(5000);
    }
  }
}

void led_flash() {
  digitalWrite(S2MINI_LED, HIGH);
  delay(70);
  digitalWrite(S2MINI_LED, LOW);
  delay(140);
}

void setup() {
  //Pins init
  pinMode(S2MINI_LED, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);

  //Stepper init
  stepper.setPinsInverted(false,false,true);
  stepper.setEnablePin(STEPPER_EN_PIN);
  stepper.disableOutputs();  //Disable stepper on Boot.
  stepper.setMaxSpeed(STEPPER_MAX_SPEED); // Set max speed to 1 RPM
  stepper.setAcceleration(STEPPER_ACCEL);

  if (!STEPPER_INVERT_DIRECTION) {
    direction = 1;
  } else {
    direction = -1;
  }

  //Serial init
  Serial.begin(115200);
  led_flash();
  delay(4000);

  //Read DeviceID
  Serial.println("Initializing...");
  deviceID = String(ESP.getEfuseMac(), HEX); // Use the unique MAC address as deviceID for ESP32
  Serial.println("ESP ID is: " + deviceID);

  // Connect to Wi-Fi or start a configuration portal
  Serial.println("Connecting to Wifi");
  if (!wifiManager.autoConnect()) {
    //Serial.println("Failed to connect and hit timeout.");
    Serial.println("Failed to connect and hit timeout.");
    delay(3000);
    // Reset and try again or put your code here to handle the failure case
    ESP.restart();
    delay(5000);
  }

  led_flash();
  Serial.println("Wifi Connected!");

  // MQTT Init
  // Initialize topic variables
  String topicPrefix = String("mk_") + deviceID;
  //in
  motorTopic = topicPrefix + "/Motor";
  angleTopic = topicPrefix + "/Angle";
  sleepTopic = topicPrefix + "/Sleep";
  calibTopic = topicPrefix + "/Calib";
  //out
  movementDoneTopic = topicPrefix + "/Done";
  bootTopic = topicPrefix + "/Boot";

  if (WiFi.status() == WL_CONNECTED) {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    
    //Enter deep sleep mode
    //ESP.deepSleep(sleepTime);
  } else {
    // Here you can perform additional actions if there is no WiFi connection.
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

bool wasMoving = false;
bool isUnloading = false;
void loop() {
  ArduinoOTA.handle();  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  isMoving = stepper.run();
  if (wasMoving && !isMoving) {
    if (isUnloading) {
      isUnloading = false;
      stepperDone();
    } else {
      stepperUnload();
      isUnloading = true;
    }
  }
  wasMoving = isMoving;
}