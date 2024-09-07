#include "PubSubClient.h" // Connect and publish to the MQTT broker

// Code for the ESP32
#include "WiFi.h" // Enables the ESP32 to connect to the local network (via WiFi)
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <utility/imumaths.h>

// #include <M5Core2.h>

// WiFi
const char *ssid = "SG4-3202";          // personal network SSID
const char *wifi_password = "SG4-3202"; // personal network password

int _moisture, sensor_analog;
const int sensor_pin = A0;

char combo[200];
StaticJsonDocument<5000> payload;
StaticJsonDocument<5000> payload_fields;

// MQTT
const char *mqtt_server = "mqtt.things.ph";             // IP of the MQTT broker
const char *mqtt_username = "64c93f454117d46beb4677d9"; // MQTT username
const char *mqtt_password = "UAnvMtslycoTolEQ2r6HahUq"; // MQTT password
const char *clientID = "64c9435000a350d7c96c35f2";      // MQTT client ID

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient);

// Custom function to connet to the MQTT broker via WiFi
void connect_MQTT() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  client.connect(clientID, mqtt_username, mqtt_password);
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was
  // successful. If the connection is failing, make sure you are using the
  // correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  } else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

int RelayPin = 17;

void setup() {

  Serial.begin(9600);
  client.setServer(mqtt_server, 1883);
  pinMode(RelayPin, OUTPUT);
  Serial.begin(115200);

  while (!Serial)
    delay(10); // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(
        "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  connect_MQTT();

  delay(1000);
}

void printEvent(sensors_event_t *event) {
  double x = -1000000, y = -1000000,
         z = -1000000; // dumb values, easy to spot problem

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;

    payload_fields["tilt"] = z;
    /*payload["hardware_serial"] = "espRASPI";
    payload["payload_fields"] = payload_fields;
    serializeJson(payload, combo);
    client.publish("espRASPI", combo);*/

    if (z > 15 || z < -15 || y > 15 || y < -15) {
      Serial.println("Sprinkler titled- turn off!!!");
      digitalWrite(RelayPin, LOW);
    } else {
      Serial.println("Sprinkler is not tilted too far");
      if (_moisture < 15) {
        Serial.println("Moisture levels low - SPRINKLER ON");
        digitalWrite(RelayPin, HIGH);
      } else {
        Serial.println("Moisture levels okay");
        digitalWrite(RelayPin, LOW);
      }

      sensor_analog = analogRead(sensor_pin);
      _moisture = (100 - ((sensor_analog / 4095.00) * 100));
      payload_fields["moisture"] = _moisture;
      Serial.print("MOISTURE: ");
      Serial.println(_moisture);
      payload_fields["tilt"] = z;
      payload_fields["ytilt"] = y;
      payload["hardware_serial"] = "espRASPI";
      payload["payload_fields"] = payload_fields;
      serializeJson(payload, combo);
      client.publish("espRASPI", combo);
    }

  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
  Serial.print("Moisture: ");
  Serial.println(_moisture);
}

void loop() {

  sensors_event_t orientationData, angVelocityData, linearAccelData,
      magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  // printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  // printEvent(&gravityData);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print(" Gyro=");
  Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  //  Serial.print(" Mag=");
  // Serial.println(mag);

  Serial.println("--");

  Serial.setTimeout(50);
}
