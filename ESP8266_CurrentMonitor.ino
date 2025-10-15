#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include "private_user_cfg.h"

/* Notes 
    - Use WESMOS D1 R2 in Arduino IDE if using Plusivo ESP8266 board 
    - D0 is connected to RST to enable wakeup from deep sleep 
*/

/* General Stuff */
#define SERIAL_BRATE      115200
#define HALF_SECOND       500
#define FULL_SECOND       1000
#define ZERO_INIT         0
#define DEFAULT_INIT      ZERO_INIT
#define E_OK              0
#define E_NOT_OK          1
#define TRUE              true
#define FALSE             false
static uint8 serialOnly = E_NOT_OK; /* Set to E_OK to skip Network Stuff and use Serial for Data Aquisition; Will also skip sleeping as it's embedded in Network Stack */ 

/* I2C Stuff */
#define I2C_SDA           SDA /* I2C SDA pin for INA219 sensor */
#define I2C_SCL           SCL /* I2C SCL pin for INA219 sensor */
#define I2C_POWER_PIN     D5  /* Use this pin to power INA219 sensor only when needed */
#define I2C_E_UNKOWN      4   /* Unknown Error */

/* Sensor Stuff */
/* Will use INA219 DC Current Monitor */
/* INA219 GND must be also connected to Load GND */
#define SENSOR_I2C_ADDRESS  0x40
#define INA_INIT_RETRIES    3
Adafruit_INA219 ina219;
static uint8 inaInit = E_NOT_OK;

/* MQTT Stuff */
#define MQTT_DEVICE_NAME      PRIVATE_CFG_HA_DEVICE_NAME
#define MQTT_DEVICE_VERSION   "25.42.00"
const char* wifi_ssid = PRIVATE_CFG_WIFI_SSID;
const char* wifi_password = PRIVATE_CFG_WIFI_PASSWORD;
const char* mqtt_user = PRIVATE_CFG_MQTT_USER;
const char* mqtt_password = PRIVATE_CFG_MQTT_PASSWORD;
IPAddress mqtt_broker = PRIVATE_CFG_MQTT_BROKER_ADDRESS;
IPAddress device_static_ip = PRIVATE_CFG_WIFI_STATIC_IP;
IPAddress gateway_static_ip = PRIVATE_CFG_WIFI_GATEWAY;
IPAddress network_mask = PRIVATE_CFG_WIFI_SUBMASK;
IPAddress dns_static_ip = PRIVATE_CFG_WIFI_DNS;
byte macAddress[WL_MAC_ADDR_LENGTH] = { PRIVATE_CFG_MQTT_MAC_ADDRESS };
WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HASensorNumber voltageSensor("fVoltage", HASensorNumber::PrecisionP2);
HASensorNumber currentSensor("fCurrent", HASensorNumber::PrecisionP2);
HASensorNumber powerSensor("fPower", HASensorNumber::PrecisionP2);
HASensorNumber batterySensor("fBattery", HASensorNumber::PrecisionP2);
HASensorNumber sleepSensor("fSleep");

/* Sleep Stuff */
#define RTC_MAGIC_NUMBER          0xDEADBEEFu
#define RTC_MAGIC_ADDRESS         0x00000000u
#define RTC_SLEEP_TIME_ADDRESS    0x00000004u
#define RTC_SLEEP_REMAIN_ADDRESS  0x00000008u
#define SLEEP_SECONDS_DEFAULT     10
#define SLEEP_FULL_SECOND         1e6
#define SLEEP_FIVE_SECOND         5e6
#define SLEEP_QUARTER_MINUTE      15e6
#define SLEEP_HALF_MINUTE         30e6
#define SLEEP_FULL_MINUTE         60e6
#define SLEEP_FIVE_MINUTE         300e6
#define SLEEP_QUARTER_HOUR        900e6
#define SLEEP_HALF_HOUR           1800e6
byte sleepInsomnia = E_OK; /* With E_OK you can go to sleep */
uint32 magicPattern = RTC_MAGIC_NUMBER;
uint32 sleepTime = SLEEP_SECONDS_DEFAULT;
uint32 sleepRemaining = DEFAULT_INIT;

/* Battery Stuff */
ADC_MODE(ADC_VCC);

void Sleep_HandleSleep(RFMode wake_mode)
{
  /* Check magic pattern for data loss */
  ESP.rtcUserMemoryRead(RTC_MAGIC_ADDRESS, &magicPattern, sizeof(magicPattern));
  if(RTC_MAGIC_NUMBER == magicPattern)
  {
    /* No data loss, read Sleep Data */
    ESP.rtcUserMemoryRead(RTC_SLEEP_TIME_ADDRESS, &sleepTime, sizeof(sleepTime));
    ESP.rtcUserMemoryRead(RTC_SLEEP_REMAIN_ADDRESS, &sleepRemaining, sizeof(sleepRemaining));
  }
  else
  {
    /* Data loss likely to power loss */
    /* Restore to defaults */
    magicPattern = RTC_MAGIC_NUMBER;
    sleepTime = SLEEP_SECONDS_DEFAULT;
    sleepRemaining = sleepTime;
    ESP.rtcUserMemoryWrite(RTC_SLEEP_TIME_ADDRESS, &sleepTime, sizeof(sleepTime));
    ESP.rtcUserMemoryWrite(RTC_SLEEP_REMAIN_ADDRESS, &sleepRemaining, sizeof(sleepRemaining));
    ESP.rtcUserMemoryWrite(RTC_MAGIC_ADDRESS, &magicPattern, sizeof(magicPattern));
  }

  /* Ensure the timing is correct */
  if(sleepRemaining > sleepTime)
  {
    /* Something is sketchy, fix it */
    sleepRemaining = sleepTime;
  }

  /* Still tired? */
  if(sleepRemaining != 0)
  {
    /* Must Sleep, decide how much */
    uint32 toSleep = DEFAULT_INIT;
    if(1800 <= sleepRemaining)
    {
      toSleep = SLEEP_HALF_HOUR;
      sleepRemaining -= 1800;
    }
    else if(900 <= sleepRemaining)
    {
      toSleep = SLEEP_QUARTER_HOUR;
      sleepRemaining -= 900;
    }
    else if(300 <= sleepRemaining)
    {
      toSleep = SLEEP_FIVE_MINUTE;
      sleepRemaining -= 300;
    }
    else if(60 <= sleepRemaining)
    {
      toSleep = SLEEP_FULL_MINUTE;
      sleepRemaining -= 60;
    }
    else if(30 <= sleepRemaining)
    {
      toSleep = SLEEP_HALF_MINUTE;
      sleepRemaining -= 30;
    }
    else if(15 <= sleepRemaining)
    {
      toSleep = SLEEP_QUARTER_MINUTE;
      sleepRemaining -= 15;
    }
    else if(5 <= sleepRemaining)
    {
      toSleep = SLEEP_FIVE_SECOND;
      sleepRemaining -= 5;
    }
    else if(1 <= sleepRemaining)
    {
      toSleep = SLEEP_FULL_SECOND;
      sleepRemaining -= 1;
    }

    /* Save updated Sleep Data */
    ESP.rtcUserMemoryWrite(RTC_SLEEP_REMAIN_ADDRESS, &sleepRemaining, sizeof(sleepRemaining));

    /* Go back to sleep */
    if(E_OK == sleepInsomnia)
    {
      if(sleepRemaining == 0)
      {
        /* Last Sleep, Wakeup with Radio with calibration */
        wake_mode = WAKE_RFCAL;
      }
      else
      {
        /* We have to sleep more, disable Radio */
        wake_mode = WAKE_RF_DISABLED;
      }

      /* Prepare for Deep Sleep */
      WiFi.setAutoReconnect(false); 
      WiFi.disconnect(true);
      delay(1);
      digitalWrite(I2C_POWER_PIN, LOW);

      /* Disable Pins */
      Pins_Disable();

      //Serial.print("DEBUG:: I go to Sleep, remaining: "); Serial.println(sleepRemaining);
      ESP.deepSleep(toSleep, wake_mode);
    }
    else
    {
      /* I can't get no sleep */
      //Serial.println("Insomniac");
    }
  }
  else
  {
    /* Sleep is complete, Continue */
    //Serial.println("DEBUG:: No need to sleep");
  }
}

void Sleep_HandleWakeup()
{
  /* Decide what kind of wakeup was happening */
  bool waking_from_sleep = ESP.getResetReason() == "Deep-Sleep Wake";
  /* Only calibrate if wakeup si not sleep related */
  /* Do not wakeup with WiFi if we still need to sleep further */
  RFMode wake_mode = waking_from_sleep ? WAKE_RF_DISABLED : WAKE_RFCAL;

  /* Go back to sleep ? */
  Sleep_HandleSleep(wake_mode);
}

void Sleep_ParseTime(const uint8* payload, uint16 length)
{
  /* Ensure correct parsing */
  char buffer[length + 1];
  strncpy(buffer, (const char*)payload, length);
  buffer[length] = '\0';

  /* Parse time from ASCII to uint32 */
  uint32 parsed = strtoul((const char*)buffer, NULL, 10);

  /* Set the time and persist it */
  sleepTime = parsed;
  if(sleepRemaining > sleepTime)
  {
    sleepRemaining = sleepTime;
    /* Save updated Sleep Data */
    ESP.rtcUserMemoryWrite(RTC_SLEEP_REMAIN_ADDRESS, &sleepRemaining, sizeof(sleepRemaining));
  }

  /* Save updated Sleep Data */
  ESP.rtcUserMemoryWrite(RTC_SLEEP_TIME_ADDRESS, &sleepTime, sizeof(sleepTime));
}

void Pins_Init()
{
  /* Builtin LED */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); /* HIGH will turn it off */

  /* Power I2C Bus */
  pinMode(I2C_POWER_PIN, OUTPUT);
  digitalWrite(I2C_POWER_PIN, LOW);
}

void Pins_Disable()
{
  /* Set all pins to INPUT with no pull-up/down */
  pinMode(0, INPUT); /* GPIO 0 */
  pinMode(2, INPUT); /* GPIO 2 */
  pinMode(4, INPUT); /* GPIO 4 / SDA */
  pinMode(5, INPUT); /* GPIO 5 / SCL */
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  pinMode(14, INPUT); /* GPIO 14 / SCLK */
  pinMode(12, INPUT); /* GPIO 12 / MISO */
  pinMode(13, INPUT); /* GPIO 13 / MOSI */
  pinMode(15, INPUT); /* GPIO 15 / CS */
  pinMode(10, INPUT); /* GPIO 10 */
  pinMode(9, INPUT); /* GPIO 9 */
  pinMode(LED_BUILTIN, INPUT);
  pinMode(A0, INPUT); /* ADC 0 */
}

void Serial_Init()
{
  /* Serial Initialization */
  Serial.begin(SERIAL_BRATE);

  /* Wait or get stuck */
  while(!Serial);

  Serial.println("");
}

void Wifi_Init()
{
  /* Validity Checks */
  if(E_OK == serialOnly)
  {
    /* Skip Network Init */
    return;
  }
  else
  {
    /* Implementation */
    /* Enable Wifi */
    WiFi.forceSleepWake();
    delay(1);

    /* Try to speedup WiFi, but keep consumption low */
    WiFi.persistent(false); /* Faster Boot time => Less Battery consumption */
    WiFi.setAutoReconnect(true); 
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.mode(WIFI_STA);

    /* Connect */
    Serial.print("Connecting to "); Serial.println(wifi_ssid);
    WiFi.begin(wifi_ssid, wifi_password);
    WiFi.config(device_static_ip, gateway_static_ip, network_mask, dns_static_ip);

    /* Wait for it */
    uint32 wifiTimeout = ZERO_INIT;
    while(WL_CONNECTED != WiFi.status())
    {
      /* Maximum 10 seconds */
      if(1000 == wifiTimeout)
      {
        /* Timeout occured */
        return;
      }
      else
      {
        wifiTimeout++;
      }

      /* Idle Animation */
      Serial.print(".");

      /* Wait 10 ms */
      delay(10);
    }

    Serial.println("Connected to WiFi!");
  }
}

void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length) 
{
  // This callback is called when message from MQTT broker is received.
  // Please note that you should always verify if the message's topic is the one you expect.
  // For example: if (memcmp(topic, "myCustomTopic") == 0) { ... }
  String compTopic = "";

  Serial.print("New message on topic: ");
  Serial.println(topic);
  Serial.print("Data Length: ");
  Serial.println(length);
  Serial.print("Data: ");
  for(int offset = DEFAULT_INIT; offset < length; offset++)
  {
    Serial.print((char)payload[offset]);
  }
  Serial.println("");

  /* Check if Sensor Data is requested */
  compTopic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/s/read";
  if(E_OK == memcmp(topic, compTopic, sizeof(compTopic)))
  {
    /* Empty Data to exclude Retained Messages */
    if(DEFAULT_INIT != length)
    {
      /* Read Sensor Data */
      Sensors_UpdateAll();
    }
  }

  /* Check if Device Sleep Time is requested */
  compTopic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/sleep/read";
  if(E_OK == memcmp(topic, compTopic, sizeof(compTopic)))
  {
    /* Empty Data to exclude Retained Messages */
    if(DEFAULT_INIT != length)
    {
      /* Publish Sleep time */
      sleepSensor.setValue(sleepTime);
    }
  }

  /* Check if Device Sleep Time is set */
  compTopic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/sleep/set";
  if(E_OK == memcmp(topic, compTopic, sizeof(compTopic)))
  {
    /* Empty Data to exclude Retained Messages */
    if(DEFAULT_INIT != length)
    {
      /* Try to set value */
      Sleep_ParseTime(payload, length);
    }
  }

  /* Check if Sleep shall be prevented */
  compTopic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/insomnia/set";
  if(E_OK == memcmp(topic, compTopic, sizeof(compTopic)))
  {
    /* Empty Data to exclude Retained Messages */
    if(DEFAULT_INIT != length)
    {
      /* Prevent Sleeping */
      sleepInsomnia = E_NOT_OK;
    }
  }

  /* Check if Sleep shall be resumed */
  compTopic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/insomnia/clear";
  if(E_OK == memcmp(topic, compTopic, sizeof(compTopic)))
  {
    /* Empty Data to exclude Retained Messages */
    if(DEFAULT_INIT != length)
    {
      /* Resume Sleeping */
      sleepInsomnia = E_OK;
      if(DEFAULT_INIT == sleepRemaining)
      {
        sleepRemaining = sleepTime;
        /* Save updated Sleep Data */
        ESP.rtcUserMemoryWrite(RTC_SLEEP_REMAIN_ADDRESS, &sleepRemaining, sizeof(sleepRemaining));
      }

      /* Try to sleep */
      Sleep_HandleSleep(WAKE_RF_DISABLED);
    }
  }
}

void onMqttConnected() 
{
  Serial.println("---------- MQTT Connected ----------");

  /* Subscribe to relevant topics */
  String topic = "";
  topic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/s/read";
  mqtt.subscribe(topic);
  topic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/sleep/read";
  mqtt.subscribe(topic);
  topic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/sleep/set";
  mqtt.subscribe(topic);
  topic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/insomnia/set";
  mqtt.subscribe(topic);
  topic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/insomnia/clear";
  mqtt.subscribe(topic);
  topic = PRIVATE_CFG_MQTT_BASE_TOPIC + "/battery/read";
  mqtt.subscribe(topic);

  /* Publish Data */
  Sensors_UpdateAll();

  /* Read the MQTT news */
  uint32 startTime = millis();
  while((3 * FULL_SECOND) > (millis() - startTime))
  {
    /* MQTT Mainfunction */
    mqtt.loop();
  }

  /* Go to Sleep */
  if(E_OK == sleepInsomnia)
  {
    /* Start the alarm */
    sleepRemaining = sleepTime;

    /* Save updated Sleep Data */
    ESP.rtcUserMemoryWrite(RTC_SLEEP_REMAIN_ADDRESS, &sleepRemaining, sizeof(sleepRemaining));

    /* Try to sleep */
    Sleep_HandleSleep(WAKE_RF_DISABLED);
  }
}

void onMqttDisconnected() 
{
  Serial.println("Disconnected from the broker!");
}

void Mqtt_Init()
{
  /* Validity Checks */
  if(E_OK == serialOnly)
  {
    /* Skip Network Stack */
    return;
  }
  else
  {
    /* Implementation */
    /* Set Device Info */
    WiFi.macAddress(macAddress);
    device.setUniqueId(macAddress, sizeof(macAddress));
    device.setName(MQTT_DEVICE_NAME);
    device.setSoftwareVersion(MQTT_DEVICE_VERSION);

    /* Set Sensors Info */
    voltageSensor.setIcon("mdi:flash-triangle");
    voltageSensor.setName("Load Voltage");
    voltageSensor.setUnitOfMeasurement("V");
    currentSensor.setIcon("mdi:current-dc");
    currentSensor.setName("Load Current");
    currentSensor.setUnitOfMeasurement("mA");
    powerSensor.setIcon("mdi:lightning-bolt");
    powerSensor.setName("Load Power");
    powerSensor.setUnitOfMeasurement("W");
    sleepSensor.setIcon("mdi:bed-clock");
    sleepSensor.setName("Sleep Time");
    sleepSensor.setUnitOfMeasurement("s");
    batterySensor.setIcon("mdi:battery-charging");
    batterySensor.setName("Supply Voltage");
    batterySensor.setUnitOfMeasurement("V");

    /* Set Callbacks */
    mqtt.onMessage(onMqttMessage);
    mqtt.onConnected(onMqttConnected);
    mqtt.onDisconnected(onMqttDisconnected);

    /* Begin MQTT */
    mqtt.begin(mqtt_broker, mqtt_user, mqtt_password);
  }
}

void I2c_Init()
{
  /* I2C Initialization */
  digitalWrite(I2C_POWER_PIN, HIGH);
  Wire.begin();
  Serial.println("---------- I2C Powered ----------");
}

void Sensors_Init()
{
  /* Initialize INA219 Sensor */
  uint8 retryCounter = INA_INIT_RETRIES;
  uint8 retVal = E_OK;

  do
  {
    retVal = ina219.begin();
    retryCounter--;
  } while((FALSE == retVal) && (0 < retryCounter));

  if(FALSE == retVal)
  {
    /* Something went wrong */
    Serial.println("Failed to initialize INA219!");
  }
  else
  {
    /* Calibrate Sensor */
    inaInit = E_OK;
    //ina219.setCalibration_32V_1A();
    ina219.setCalibration_16V_400mA();
    Serial.println("---------- INA219 Started ----------");
  }
}

void Battery_Read()
{
  /* Update Battery reading */
  float vcc = ESP.getVcc() / 1000.0; /* Also convert to Volts */

  /* Print Data */
  Serial.println("---------- Battery Management ----------");
  Serial.print("Supply Voltage: "); Serial.println(vcc);

  /* Publish Data */
  batterySensor.setValue(vcc);
}

void INA219_Read()
{
  if(E_NOT_OK == inaInit)
  {
    /* Unitialized */
    Serial.println("---------- INA219 Not Initialized ----------");
    return;
  }

  /* Read Sensor Data */
  float shuntVoltage = ina219.getShuntVoltage_mV();
  float busVoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  if(DEFAULT_INIT > current_mA)
  {
    /* Reading is likely wrong or no connection */
    current_mA = DEFAULT_INIT;
  }
  float loadVoltage = busVoltage + (shuntVoltage / 1000);
  float power_mW = loadVoltage * current_mA;
  float power_W = (float)(power_mW / 1000.0);

  /* Print Data */
  Serial.println("---------- INA219 Readings ----------");
  Serial.print("Voltage: "); Serial.print(loadVoltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power: "); Serial.print(power_W); Serial.println(" W");

  /* Validity Checks */
  if(E_NOT_OK == serialOnly)
  {
    /* Publish Data */
    voltageSensor.setValue(loadVoltage);
    currentSensor.setValue(current_mA);
    powerSensor.setValue(power_W); /* Show in W */
  }
  else
  {
    /* Skip Network Stack */
  }
}

void Sensors_UpdateAll()
{
  /* Read INA219 */
  INA219_Read();

  /* Update Sleep Sensor */
  sleepSensor.setValue(sleepTime);

  /* Read Supply */
  Battery_Read();
}

void LowLevel_Startup()
{
  /* Disable WiFi to save power */
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
}

void setup() 
{
  /* Time critical conifiguration */
  LowLevel_Startup();
  Pins_Init();

  /* Init */
  Serial_Init();
  Sleep_HandleWakeup();
  I2c_Init();
  Sensors_Init();
  Wifi_Init();
  Mqtt_Init();
}

// the loop function runs over and over again forever
void loop() 
{
  /* Validity Checks */
  if(E_NOT_OK == serialOnly)
  {
    /* MQTT Mainfunction */
    mqtt.loop();
  }
  else
  {
    /* Skip Network Stack */
    INA219_Read();

    /* Wait a bit */
    delay(FULL_SECOND);
  }
}
