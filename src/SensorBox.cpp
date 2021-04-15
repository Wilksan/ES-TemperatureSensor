#include "SensorBox.hpp"
#include <vector>
#include <string>
#include "DatabaseHelper.hpp"
#include "NTP_Connection.hpp"
#include <Wire.h>
//Display Usage
#include <TFT_eSPI.h>
#include <SPI.h>
//Bme680
#include <EEPROM.h>
#include "bsec.h"
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};
#include <sys/time.h>
//WIFI
#include <WiFi.h>
//Telegram
#include <HTTPClient.h>
//Settings from Config Page
#include <AsyncJson.h>
#include <ArduinoJson.h>

//Telegram variables
const String token = "";
const String chatId = "";

float temperatureTrigger;
float aqiTrigger;

inline uint16_t GET_ADDRESS_OF_TEMPERATUR_TRIGGER(uint16_t startingAddress = 1) { return (SensorBase::GET_MAX_MY_SENSOR_SIZE_IN_BYTE() + startingAddress); }
inline uint16_t GET_ADDRESS_OF_AQI_TRIGGER(uint16_t startingAddress = 1) { return (GET_ADDRESS_OF_TEMPERATUR_TRIGGER(startingAddress) + sizeof(float)); }

inline void setTemperatureTrigger(float temperatureTrigger, uint16_t startingAddress = 1)
{
  EEPROM.writeFloat(GET_ADDRESS_OF_TEMPERATUR_TRIGGER(startingAddress), temperatureTrigger);
  EEPROM.commit();
}

inline void setTemperatureTrigger_oc(float temperatureTrigger, uint16_t startingAddress = 1)
{
  EEPROM.writeFloat(GET_ADDRESS_OF_TEMPERATUR_TRIGGER(startingAddress), temperatureTrigger);
}

inline void setAQITrigger(float AQITrigger, uint16_t startingAddress = 1)
{
  EEPROM.writeFloat(GET_ADDRESS_OF_AQI_TRIGGER(startingAddress), AQITrigger);
  EEPROM.commit();
}

inline void setAQITrigger_oc(float AQITrigger, uint16_t startingAddress = 1)
{
  EEPROM.writeFloat(GET_ADDRESS_OF_AQI_TRIGGER(startingAddress), AQITrigger);
}

inline float getTemperatureTrigger(uint16_t startingAddress = 1)
{
  float result = EEPROM.readFloat(GET_ADDRESS_OF_TEMPERATUR_TRIGGER(startingAddress));
  return result;
}

inline float getAQITrigger(uint16_t startingAddress = 1)
{
  float result = EEPROM.readFloat(GET_ADDRESS_OF_AQI_TRIGGER(startingAddress));
  return result;
}

const int MAX_VALUES = 20;
int values[MAX_VALUES];

//variables
float temperature = 0;
float humidity = 0;
float pressure = 0;
float aqi = 0;
float aqi_accuracy = 0;
String output;

//Data to overdue the DeepSleep
RTC_DATA_ATTR int deepSleepCounter = 0;
RTC_DATA_ATTR int viewCase;
//Initiate Wifi and MQTT
static WiFiClient espClient;
PubSubClient client(espClient);

//Buttons
#define ButtonT 35
long lastDebounceButtonT = 0;
long debounceDelay = 500;

//DeepSleep
#define uS_TO_S_FACTOR 1000000 //Microsekunden in Sekunden
#define TIME_TO_SLEEP 30       //900 Sekunden = 15 Min

// Create an object of the class Bsec
Bsec bme680;
RTC_DATA_ATTR uint8_t bmeState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
RTC_DATA_ATTR int64_t sensor_state_time = 0;

int64_t GetTimestamp()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

bsec_virtual_sensor_t sensorList[7] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
};

//Display instance
static TFT_eSPI tft = TFT_eSPI();

// Helper functions
void checkSensorStatus()
{
  if (bme680.status != BSEC_OK)
  {
    if (bme680.status < BSEC_OK)
    {
      output = "BSEC error code : " + String(bme680.status);
      Serial.println(output);
    }
    else
    {
      output = "BSEC warning code : " + String(bme680.status);
      Serial.println(output);
    }
  }

  if (bme680.bme680Status != BME680_OK)
  {
    if (bme680.bme680Status < BME680_OK)
    {
      output = "BME680 error code : " + String(bme680.bme680Status);
      Serial.println(output);
    }
    else
    {
      output = "BME680 warning code : " + String(bme680.bme680Status);
      Serial.println(output);
    }
  }
  bme680.status = BSEC_OK;
}

void oneMinuteDisplay()
{
  //1 Minute Display aktiv halten
  delay(15000);
  delay(15000);
  delay(15000);
  delay(15000);
  Serial.println("1 Minute ist um");
}

void getReadings()
{
  if (bme680.run())
  {
    temperature = bme680.rawTemperature;
    humidity = bme680.rawHumidity;
    pressure = bme680.pressure / 100.0;
    aqi = bme680.iaq;
    aqi_accuracy = bme680.iaqAccuracy;

    if (aqi_accuracy >= 1)
    {
      sensor_state_time = GetTimestamp();
      bme680.getState(bmeState);
      Serial.println("Saved State");
    }
    checkSensorStatus();
  }
  else
  {
    checkSensorStatus();
  }
}

void displayTemp()
{
  //Temperatur auf dem Display ausgeben
  tft.setFreeFont(&Orbitron_Light_24);
  tft.setCursor(0, 30);
  tft.print("Temp: ");
  tft.print(temperature);
  tft.print("c");
  tft.println();
  tft.println();
  tft.drawLine(0, 35, 250, 35, TFT_BLUE);
}

void displayHumidity()
{
  //Feuchtigkeit auf dem Display ausgeben
  tft.setFreeFont(&Orbitron_Light_24);
  tft.setCursor(0, 60);
  tft.print("Feuchte: ");
  tft.print(humidity);
  tft.print("%");
  tft.drawLine(0, 65, 250, 65, TFT_BLUE);
}

void displayPressure()
{
  //Druck auf dem Display ausgeben
  tft.setFreeFont(&Orbitron_Light_24);
  tft.setCursor(0, 90);
  tft.print("hPa: ");
  tft.print(pressure);
  tft.drawLine(0, 95, 250, 95, TFT_BLUE);
}

void displayAQI()
{

  tft.setFreeFont(&Orbitron_Light_24);
  tft.setCursor(0, 120);
  if (aqi <= 50)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("AQI: ");
    tft.print(aqi);
  }
  else if (aqi > 50 && aqi <= 100)
  {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print("AQI: ");
    tft.print(aqi);
  }
  else if (aqi > 100 && aqi <= 150)
  {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.print("AQI: ");
    tft.print(aqi);
  }
  else if (aqi > 150 && aqi <= 200)
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("AQI: ");
    tft.print(aqi);
  }
  else if (aqi > 200 && aqi <= 300)
  {
    tft.setTextColor(TFT_PURPLE, TFT_BLACK);
    tft.print("AQI: ");
    tft.print(aqi);
  }
  else if (aqi > 300 && aqi <= 500)
  {
    tft.setTextColor(TFT_BROWN, TFT_BLACK);
    tft.print("AQI: ");
    tft.print(aqi);
  }
}

void displaySerial()
{
  Serial.print("Temperatur: ");
  Serial.print(temperature);
  Serial.println("c");
  Serial.print("Feuchte: ");
  Serial.print(humidity);
  Serial.println("%");
  Serial.print("Druck: ");
  Serial.print(pressure);
  Serial.println("hPa");
  Serial.print("AQI: ");
  Serial.println(aqi);
  Serial.print("Genauigkeit: ");
  Serial.println(aqi_accuracy);
}

void drawGraph()
{
  //(0/0) define
  int x = 30;
  int y = 105; //15 px to 105px are tempreture 90 px a 3px per degree
  int yOld = 105;
  int z = 20;

  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setFreeFont();
  //x-Axis
  tft.drawLine(30, 105, 230, 105, TFT_BLUE);
  //y-Axis
  tft.drawLine(30, 15, 30, 105, TFT_BLUE);
  //Array Y coordinates for temperature
  int tempKey[7] = {105, 90, 75, 60, 45, 30, 15};
  int tempKeyValue[7] = {0, 5, 10, 15, 20, 25, 30};
  int tempKeyXCord = 27;

  //vertical lines x axis
  for (int i = 0; i < 20; i++)
  {
    z = z + 10;
    tft.drawLine(z, 103, z, 108, TFT_BLUE);
  }

  //horizontal lines and value y-axis
  for (int i = 0; i <= 6; i++)
  {
    tft.drawLine(tempKeyXCord, tempKey[i], tempKeyXCord + 6, tempKey[i], TFT_BLUE);
    tft.drawNumber(tempKeyValue[i], 0, tempKey[i]);
  }

  //real graph
  for (int i = 0; i < MAX_VALUES; i++)
  {
    int value = values[i];
    //height from temperature and subtract from actual height
    y = 105 - value * 3;

    if (i == 0)
    {
      yOld = y;
    }

    //circle at temperature
    tft.drawCircle(x, y, 2, TFT_GREEN);
    //connect points
    //if x>30 subtract 10 to get old x axis else x for first circle
    tft.drawLine(x, y, x > 30 ? x - 10 : x, yOld, TFT_GREEN);
    //move x position by 10
    x = x + 10;
    //save old y-coordinate
    yOld = y;
  }
}

void telegramWork()
{
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + token + "/sendMessage?chat_id=" + chatId + "&text=";
  if (temperature >= getTemperatureTrigger() && aqi <= getAQITrigger())
  {
    url += "Temperatur im Raum bei: " + String(temperature) + "°C ! Luftqualität in Ordnung.";
  }
  else if (temperature <= getTemperatureTrigger() && aqi > getAQITrigger())
  {
    url += "AQI bei: " + String(aqi) + " ! Bitte den Raum lüften. Temperatur mit " + temperature + "°C in Ordnung.";
  }
  else if (temperature >= getTemperatureTrigger() && aqi > getAQITrigger())
  {
    url += "Temperatur bei: " + String(temperature) + "°C und AQI bei: " + String(aqi) + "!!! Dringend Lüften.";
  }
  else
  {
    Serial.println("Werte sind in Ordnung keine Nachricht versendet");
  }

  http.begin(url); //Specify the URL
  int httpCode = http.GET();
  if (httpCode > 0)
  { //Check for the returning code
    String payload = http.getString();
    Serial.println(httpCode);
    Serial.println(payload);
  }
  else
  {
    Serial.println("Error on HTTP request");
  }
  http.end();
}

void deepSleep()
{

  uint64_t time_us = ((bme680.nextCall - GetTimestamp()) * 1000) - esp_timer_get_time();
  esp_sleep_enable_timer_wakeup(time_us);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 1);
  esp_deep_sleep_start();
}

void viewGraph(SensorBox *sensorBox)
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  sensorBox->getFromDB();
  drawGraph();
  delay(1000);
}

void viewQuickInfo()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  delay(1000);
  getReadings();
  displaySerial();
  displayTemp();
  displayHumidity();
  displayPressure();
  displayAQI();
}

void IRAM_ATTR toggleButtonT()
{
  Serial.println("Button Top getriggert");
  if ((millis() - lastDebounceButtonT) > debounceDelay)
  {
    if (viewCase == 0)
    {
      viewCase = 2;
      Serial.println("Wechsel auf Graph");
    }
    else if (viewCase == 1)
    {
      viewCase = 0;
      Serial.println("Wechsel auf Beide");
    }
    else if (viewCase == 2)
    {
      viewCase = 1;
      Serial.println("Wechsel auf Info");
    }
    Serial.println("ButtonT aktiviert");
    lastDebounceButtonT = millis();
    Serial.println("ViewCase auf: " + String(viewCase) + " gesetzt!");
    deepSleep();
  }
}

void SensorBox::sendToDB()
{
  // Establish a connection to the database server.
  if (!m_conn.connected())
  {
    connectToDatabase(m_conn, m_databaseIP, m_databaseUsername.c_str(), m_databasePassword.c_str());
  }
  m_cursor = new MySQL_Cursor(&m_conn);
  char bufferDatabase[256] = {};
  char bufferDatabaseValues[128] = {};
  sprintf(bufferDatabaseValues, "(%d, '%s', %f, %f, %f, %f, %f)", m_room.getID(), m_deviceMAC.c_str(), temperature, humidity, pressure, aqi, aqi_accuracy);
  DATABASE_INSERT(
      "es_datenbank.hat_raumwert_aufgenommen",
      "(RaeumeID, GeraeteMAC, Temperatur, Luftfeuchtigkeit, Luftdruck, AQI, AQI_Accuracy)",
      bufferDatabaseValues,
      bufferDatabase,
      256);

  Serial.printf("> [D]: %s\n", bufferDatabase);
  m_cursor->execute(bufferDatabase);
}

void SensorBox::getFromDB()
{
  vector<string> databaseResponse = getDatabaseSelectResponse(&m_conn, "es_datenbank.hat_raumwert_aufgenommen", "Temperatur", "1", "ORDER BY Zeitpunkt DESC LIMIT 20");
  for (int i = 0; i < databaseResponse.size(); i++)
  {
    if (i < MAX_VALUES)
    {
      values[i] = int(atof(databaseResponse.at(19 - i).c_str()));
    }
  }
}

SensorBox::SensorBox()
    : SensorBase()
{
}

SensorBox::SensorBox(TFT_eSPI* tft)
    : SensorBase(tft);
{

}

SensorBox::~SensorBox()
{
}

void SensorBox::preSetupState()
{
}

void SensorBox::setupSetup()
{
  // Handle the recieved new device data from the client
  server.on(
      "/html/redirect/api/posts/new_device_data",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        char *loginData = reinterpret_cast<char *>(data);
        DynamicJsonDocument doc(512);
        deserializeJson(doc, loginData);
        Serial.println("\n -> Data for the wifi recieved:");
        serializeJson(doc, Serial);
        setTemperatureTrigger_oc(doc["trigger"].as<float>());
        Serial.printf("Test= %2.f", doc["aqi"].as<float>());
        Serial.println(sizeof(float));
        setAQITrigger_oc(doc["aqi"].as<float>());
        EEPROM.commit();
        // chatId = doc["charID"].as<String>();
        //token = doc["token"].as<String>();

        printf("[S]> Trigger = %.2f \t Aqi = %.2f \t Char-ID = %s \t Token = %s\n", getTemperatureTrigger(), getAQITrigger(), chatId.c_str(), token.c_str());

        Serial.println("=====================================================\n\n");

        request->send(200);
      });
}
void SensorBox::setupRuntime()
{
  tft.begin();
  tft.setRotation(1);
  pinMode(ButtonT, INPUT);
  attachInterrupt(ButtonT, toggleButtonT, FALLING);
  Serial.println(F("BME680 Starten:"));
  Wire.begin(21, 15);
  bme680.begin(0x77, Wire);
  checkSensorStatus();
  bme680.setConfig(bsec_config_iaq);
  checkSensorStatus();
  bme680.setState(bmeState);
  bme680.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
  checkSensorStatus();
}

void SensorBox::setupSleep()
{
}
void SensorBox::setupReset()
{
}

void SensorBox::loopSetup()
{
}
void SensorBox::loopRuntime()
{
  delay(1000);
  if (deepSleepCounter == 0)
  {
    viewCase = 0;
  }
  else if (deepSleepCounter != 0)
  {
    Serial.println("ViewCase: " + String(viewCase));
  }
  switch (viewCase)
  {
  case 0:
  {
    deepSleepCounter++;
    viewQuickInfo();
    sendToDB();
    for (int i = 0; i <= 100; i++)
    {
      getReadings();
      //displaySerial();
      Serial.println(i);
      delay(3000);
    }
    viewQuickInfo();
    sendToDB();
    telegramWork();
    oneMinuteDisplay();
    viewGraph(this);
    oneMinuteDisplay();
    deepSleep();
  }
  break;
  case 1:
  {
    deepSleepCounter++;
    viewQuickInfo();
    for (int i = 0; i <= 100; i++)
    {
      getReadings();
      //displaySerial();
      delay(3000);
    }
    viewQuickInfo();
    sendToDB();
    telegramWork();
    deepSleep();
  }
  break;
  case 2:
  {
    deepSleepCounter++;
    Serial.println("Ansicht Graph");
    viewGraph(this);
    oneMinuteDisplay();
    getReadings();
    telegramWork();
    deepSleep();
  }
  break;
  }
}

void SensorBox::loopSleep()
{
}

void SensorBox::loopReset()
{
}
