#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <arduino_secrets.h>
#include <WiFi.h>
#include <SPI.h>
#include <MQTT.h>
#include <NTPClient.h>
#include <ctime>

// Sensor definitions
#define DHTPIN 32
#define DHTTYPE DHT22
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
DHT_Unified dht(DHTPIN, DHTTYPE);
sensor_t sensor;
WiFiUDP ntpUDP;

// Wifi and MQTT definitions
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
WiFiClient wifiClient;
MQTTClient mqttClient;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

struct Measurement
{
  double temperature;
  double humidity;
};

void readAndSendData();
bool initMQTT();
bool initWifi();
void initSensor();
String getCurrentTime();
Measurement getMeasurement();
void sendMeasurement(Measurement m);

void setup()
{
  readAndSendData();

  Serial.println("Going to sleep");
  Serial.flush();
  delay(1000);

  esp_sleep_enable_timer_wakeup(60 * 5 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop()
{
}

void readAndSendData()
{
  try
  {
    Serial.begin(115200);
    if(!initWifi()) return;
    if(!initMQTT()) return;
    initSensor();

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
}

bool initWifi()
{
  Serial.print("Connecting to Wifi SSID: ");
  Serial.println(ssid);
  Serial.print("Connecting.");

  WiFi.begin(ssid, pass);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
    counter++;
    if (counter > 100)
    {
      return false;
    }
  }
  Serial.println(" Connected");
  return true;
}

bool initMQTT()
{
  IPAddress ip(192, 168, 2, 4);

  Serial.print("Connecting to MQTT: ");
  mqttClient.begin(ip, wifiClient);
  int counter = 0;
  while (!mqttClient.connect("ESP32-sensor"))
  {
    Serial.print(".");
    delay(500);
    counter++;
    if (counter > 100)
    {
      return false;
    }
  }
  Serial.println("MQTT Connected");
  return true;
}

void initSensor()
{
  dht.begin();
  dht.temperature().getSensor(&sensor);
}

Measurement getSingleMeasurement()
{
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

String getCurrentTime()
{

  timeClient.begin();

  int counter = 0;
  while(!timeClient.forceUpdate()) {
    delay(500);
    counter++;
    if(counter > 10) {
      throw("Could not retrieve time");
    }
  }
  time_t now = timeClient.getEpochTime();
  char buf[sizeof "0000-00-00T00:00:00Z"];
  strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
  return buf;
}

Measurement getMeasurement()
{
  double totalTemperature = 0;
  double totalHumidity = 0;
  for (int i = 0; i < 3; i++)
  {
    Measurement m = getSingleMeasurement();
    if (isnan(m.temperature) || isnan(m.humidity))
    {
      throw("Could not read sensor");
    }
    totalTemperature += m.temperature;
    totalHumidity += m.humidity;
    delay(3000);
  }
  return Measurement{
      totalTemperature / 3,
      totalHumidity / 3};
}

String createEvent(String name, double value)
{
  return "{\"data_type\":\"" + name + "\",\"value\":" + value + "}";
}

void sendMeasurement(Measurement m)
{
  String now = getCurrentTime();
  String event = String("{\"data_type\":\"climate\",\"timestamp\":\"" + now + "\",\"temperature\":") + m.temperature + ",\"humidity\":" + m.humidity + "}";
  mqttClient.publish("data/climate/living_room", event);
}
