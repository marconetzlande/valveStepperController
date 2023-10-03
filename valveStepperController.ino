#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <Preferences.h> // Add the Preferences library for ESP32
#endif

#include <PubSubClient.h>
#include <AccelStepper.h>
#include <WiFiManager.h>

#define S2MINI_LED 15

#define STEPPER_STEPS_PER_REVELAZION 200
#define STEPPER_MICROSTEPS 64
#define STEPPER_GEAR_RATIO 3.75
#define STEPPER_ACCEL 1000
#define STEPPER_ONE_ROT STEPPER_STEPS_PER_REVELAZION * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO
#define STEPPER_MAX_SPEED STEPPER_ONE_ROT / 1.0

#define STEPPER_INVERT_DIRECTION false

WiFiManager wifiManager; // Create a WiFiManager instance
WiFiClient espClient;
String deviceID;

const char* mqtt_server = "192.168.181.108";
const int mqtt_port = 1883;

int stepsPer90Degrees = STEPPER_ONE_ROT / 4;

#define STEPPER_STEP_PIN 16
#define STEPPER_DIR_PIN 17
#define STEPPER_EN_PIN 18

PubSubClient client(espClient);
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
Preferences preferences;

// Topic variables
String motorTopic;
String sleepTopic;
String movementDoneTopic;
String bootTopic;
String calibTopic;
int sleepTime = 0;

bool isMoving = false;

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
      int direction;
      if (!STEPPER_INVERT_DIRECTION) {
        direction = 1;
      } else {
        direction = -1;
      }
      int newPosition = String(message).toInt() * stepsPer90Degrees * direction;
      Serial.println("runToPosition " + String(newPosition));
      stepper.moveTo(newPosition);
      isMoving = stepper.isRunning();
      if (isMoving) {
        preferences.putBool("isMoving", true);
        stepper.enableOutputs();
      } else {
        client.publish(movementDoneTopic.c_str(), "1");
      }
    } else if (String(topic) == calibTopic) {
      // Reset moving state if device crashes during movement.
      preferences.begin("StepperPrefs");
      preferences.putBool("isMoving", false);
      preferences.end();
    } else if (String(topic) == sleepTopic) {
      // Send device into deepSleep
      sleepTime = message.toInt() * 1000000;
      Serial.println("Sleep command recieved!");
    }
  }
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
  client.publish(movementDoneTopic.c_str(), "1");
}


// Reconnect function: Connects to the MQTT server and subscribes to the required topics
void reconnect() {
  Serial.println("Reconnect!");
  while (!client.connected()) {
    if (client.connect(deviceID.c_str())) {
      client.subscribe(motorTopic.c_str());
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
}

bool wasMoving = false;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  isMoving = stepper.run();
  if (wasMoving && !isMoving) {
    stepperDone();
  }
  wasMoving = isMoving;
}