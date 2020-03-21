/*
 ESP8266 w BME680 connecting to MQTT by Tim Herden (https://www.timherden.com)

 Use this sketch to connect your ESP8266 (Tested with D1 Mini and ESP8266-12E) via Wifi to a MQTT server 
 and transmitting sensor data from a BME680 (Tested with BME680 breakoputs from Adafruit and Pimoroni)  

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// Update these with values suitable for your network.
const char* ssid = "YOUR WIFI SSID";
const char* password = "YOUR WIFI PASSWORD";
const char* mqtt_server = "192.168.1.1"; // IP of your MQTT server

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi() {

  delay(10);
  // Connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  // Printing connection parameters to serial monitor
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_bme680() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("BME680 test"));

  // change to !bme.begin() for Adafruit breakout since they use 0x70 as default address
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("esp8266/status", "Hello from ESP8266");
      // ... and resubscribe
      // client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  setup_bme680();
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();

    if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Sending sensor data to MQTT and priting to serial monitor
  client.publish("esp8266/bme680/temperature_c", String(bme.temperature).c_str(), true); 
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  client.publish("esp8266/bme680/pressure_hpa", String(bme.pressure / 100.0).c_str(), true); 
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  client.publish("esp8266/bme680/humidity_p", String(bme.humidity).c_str(), true); 
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  client.publish("esp8266/bme680/gas_kohms", String(bme.gas_resistance / 1000.0).c_str(), true); 
  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  
  // Wait for 10 000msec = 10sec
  delay(10000);
 
}
