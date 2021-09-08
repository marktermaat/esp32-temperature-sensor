#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <arduino_secrets.h>
#include <WiFi.h>
#include <SPI.h>
#include <MQTT.h>

// Sensor definitions
#define DHTPIN 32
#define DHTTYPE DHT22
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
DHT_Unified dht(DHTPIN, DHTTYPE);
sensor_t sensor;

// Wifi and MQTT definitions
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
WiFiClient wifiClient;
MQTTClient mqttClient;

struct Measurement {
  double temperature;
  double humidity;
};

void initMQTT();
void initWifi();
void initSensor();
Measurement getMeasurement();
void sendMeasurement(Measurement m);

void setup()
{
  Serial.begin(115200);
  initWifi();
  initMQTT();
  initSensor();

  try
  {
    Measurement m = getMeasurement();
    sendMeasurement(m);
    Serial.print("Temperature: ");
    Serial.println(m.temperature);
    Serial.print("Humidity: ");
    Serial.println(m.humidity);
  }
  catch (...)
  {
    Serial.println("Could not get a good measurement from the sensor");
  }

  Serial.println("Going to sleep");
  Serial.flush();
  delay(1000);

  esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop()
{
}

void initWifi()
{
  Serial.print("Connecting to Wifi SSID: ");
  Serial.println(ssid);
  Serial.print("Connecting.");

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" Connected");
}

void initMQTT()
{
  IPAddress ip(192, 168, 2, 4);
  
  Serial.print("Connecting to MQTT: ");
  mqttClient.begin(ip, wifiClient);
  while(!mqttClient.connect("ESP32-sensor")) {
    Serial.print(".");
    delay(100); 
  }
  Serial.println("MQTT Connected");
}

void initSensor()
{
  dht.begin();
  dht.temperature().getSensor(&sensor);
}

Measurement getSingleMeasurement() {
  Measurement m;
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    m.temperature = event.temperature;
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    m.humidity = event.relative_humidity;
  }

  return m; 
}

Measurement getMeasurement() {
  double totalTemperature = 0;
  double totalHumidity = 0;
  for (int i = 0; i < 3; i++) {
    Measurement m = getSingleMeasurement();
    if(isnan(m.temperature) || isnan(m.humidity)) {
      throw("Could not read sensor"); 
    }
    totalTemperature += m.temperature;
    totalHumidity += m.humidity;
    delay(3000);
  }
  return Measurement {
    totalTemperature / 3,
    totalHumidity / 3
  };
}

String createEvent(String name, double value) {
  return "{\"data_type\":\"" + name + "\",\"value\":" + value + "}";
}

void sendMeasurement(Measurement m) {
  mqttClient.publish("data/temperature/living_room", createEvent("Temperature", m.temperature), false, 1);
  mqttClient.publish("data/humidity/living_room", createEvent("Humidity", m.humidity), false, 1);
}