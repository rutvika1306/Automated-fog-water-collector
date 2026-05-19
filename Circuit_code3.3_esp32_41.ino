#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <EEPROM.h>



#define EEPROM_SIZE 4
#define EEPROM_STATE_ADDR 0

#include "esp_task_wdt.h"


int WDT_TIMEOUT = 30;
// #define MOTOR_MAX_RUN_MS 12000UL
#define timer_open 8000UL
#define timer_close 6000UL
#define EARLY_TRIGGER_MS 2000UL
#define PULSES_PER_MM 5.0

//wifi
const char* ssid = "iitk";
const char* password = "";

//MQTT
const char* mqttServer = "172.27.21.121";
const uint16_t mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPass = "";
const char* mqttTopic1 = "datavis";

//Google sheet
const char* GAS_URL = "https://script.google.com/macros/s/AKfycbwAMmZ4DzhfjLAkK96flxP-vfSb2ealxR00HjlWljfiQUlIa1PNB9BCP7NaxFYnu2Bp/exec";
const char* SECRET_KEY = "my_secret_key_here";

unsigned long lastSend = 0;
const unsigned long sendInterval = 5UL * 60UL * 1000UL;  // 5 minute


//Pins
// Motor driver
#define SLP1_PIN 16
#define DIR1_PIN 17
#define PWM_PIN 18

// Proximity sensors
#define PROX_UP_PIN 25
#define PROX_DOWN_PIN 26

// Encoder
#define ENC_A_PIN 32
#define ENC_B_PIN 33

// DHT22
#define DHTPIN 4
#define DHTTYPE DHT22


#define PWM_CHANNEL 0
#define PWM_FREQ 20000
#define PWM_RESOLUTION 10


/*OBJECTS */
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

/*GLOBALS*/
volatile bool proxUpTriggered = false;
volatile bool proxDownTriggered = false;

volatile long encoderCount = 0;
volatile uint8_t lastAB = 0;

volatile long visibilityValue = 0;

WiFiClientSecure secureClient;

//VISIBILITY TIMER CONTROL
unsigned long visLowStartTime  = 0;
unsigned long visHighStartTime = 0;

const unsigned long VIS_DELAY_MS = 5UL * 60UL * 1000UL;  // 5 minutes



// Curtain state
// false = CLOSED, true = OPEN
bool state_change = false;


// bool once_state = false;
//ISR
void IRAM_ATTR proxISR1() {
  proxUpTriggered = true;
}
void IRAM_ATTR proxISR2() {
  proxDownTriggered = true;
}


void saveStateChange(bool state) {
  EEPROM.write(EEPROM_STATE_ADDR, state ? 1 : 0);
  EEPROM.commit();
}
void loadStateChange() {
  state_change = EEPROM.read(EEPROM_STATE_ADDR) == 1;
}


void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(ENC_A_PIN);
  uint8_t b = digitalRead(ENC_B_PIN);
  uint8_t ab = (a << 1) | b;

  if ((lastAB == 0b00 && ab == 0b01) ||
      (lastAB == 0b01 && ab == 0b11) ||
      (lastAB == 0b11 && ab == 0b10) ||
      (lastAB == 0b10 && ab == 0b00)) {
    encoderCount++;
  } else if ((lastAB == 0b00 && ab == 0b10) ||
             (lastAB == 0b10 && ab == 0b11) ||
             (lastAB == 0b11 && ab == 0b01) ||
             (lastAB == 0b01 && ab == 0b00)) {
    encoderCount--;
  }

  lastAB = ab;
}


//HELPERS 
float getDistanceMM() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return (float)count / PULSES_PER_MM;
}

void resetEncoder() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}


void motorStart(uint16_t speed, bool direction) {
  // int speed = min(speed, 1023);   // 10-bit limit
  digitalWrite(SLP1_PIN, HIGH);
  digitalWrite(DIR1_PIN, direction);
  // ledcWrite(PWM_CHANNEL, speed);
  analogWrite(PWM_PIN, 700);
}

void motorStop() {
  // ledcWrite(PWM_CHANNEL, 0);
  analogWrite(PWM_PIN, 0);
  digitalWrite(SLP1_PIN, LOW);
}



//WIFI 
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.begin(ssid, password);
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - start > 20000) {
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      start = millis();
    }
  }
}

//MQTT
long parseVisibilityMetersFromCsv6(const String& json) {
  int start = 0;
  for (int i = 1; i < 6; i++) {
    start = json.indexOf(',', start) + 1;
    if (start <= 0) return -1;
  }

  int end = json.indexOf(',', start);
  if (end < 0) end = json.length();

  String col6 = json.substring(start, end);
  col6.replace("KM", "");
  col6.replace("km", "");
  col6.replace("+", "");
  col6.trim();

  float km = col6.toFloat();
  return (km >= 0) ? (long)(km * 1000) : -1;
}

void mqttCallback(char* topic, byte* json, unsigned int length) {
  Serial.print("MQTT message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)json[i];
  }

  Serial.println(msg);

  if (String(topic) == mqttTopic1) {
    long v = parseVisibilityMetersFromCsv6(msg);
    Serial.print("Parsed visibility = ");
    Serial.println(v);

    if (v >= 0) {
      visibilityValue = v;

      // if (once_state == false) {
      //   if (visibilityValue < 500) {
      //     state_change = true;
      //     once_state = true;
      //   }
      // }
    }
  } else {
    Serial.println("Topic does not match subscription!");
  }
}


void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT... ");

    // UNIQUE CLIENT ID
    String clientId = "ESP32_" + String((uint32_t)ESP.getEfuseMac(), HEX);

    bool connected;
    if (mqttUser[0] == '\0') {
      connected = client.connect(clientId.c_str());
    } else {
      connected = client.connect(clientId.c_str(), mqttUser, mqttPass);
    }

    if (connected) {
      Serial.println("connected");
      client.subscribe(mqttTopic1);
      Serial.print("Subscribed to: ");
      Serial.println(mqttTopic1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5 sec");
      delay(5000);
    }
  }
}


//CURTAIN CONTROL 
void runCurtain(bool direction, volatile bool& limitFlag, unsigned long MOTOR_MAX_RUN_MS) {
  resetEncoder();
  unsigned long start = millis();

  while (true) {
    esp_task_wdt_reset();
    Serial.println("moving-------------------");
    Serial.println(MOTOR_MAX_RUN_MS);
    client.loop();  // 🔴 REQUIRED
    motorStart(200, direction);

    if (millis() - start >= MOTOR_MAX_RUN_MS) break;
    int DIS = getDistanceMM();
    Serial.println(DIS);
    if (DIS > 1000) break;

    if (limitFlag) {
      limitFlag = false;  // clear ISR flag
      break;
    }
  }
  Serial.println("stopping-------------------");
  motorStop();
}




bool postToGoogleSheet(String topic, String visibility, String humidity, String tips, String rawmsg) {
  if (WiFi.status() != WL_CONNECTED) return false;

  // JSON body
  String json = "{";
  json += "\"key\":\"" + String(SECRET_KEY) + "\",";
  json += "\"topic\":\"" + topic + "\",";
  json += "\"visibility\":\"" + visibility + "\",";
  json += "\"humidity\":\"" + humidity + "\",";
  json += "\"tips\":\"" + tips + "\",";
  json += "\"raw\":\"" + rawmsg + "\"";
  json += "}";

  WiFiClientSecure *client = new WiFiClientSecure;
  client->setInsecure(); // disables certificate verification - simpler but less secure

  HTTPClient https;
  https.begin(*client, GAS_URL);
  https.addHeader("Content-Type", "application/json");

  int httpCode = https.POST(json);
  if (httpCode > 0) {
    String payload = https.getString();
    Serial.printf("HTTP %d, resp: %s\n", httpCode, payload.c_str());
  } else {
    Serial.printf("POST failed, error: %s\n", https.errorToString(httpCode).c_str());
  }

  https.end();
  delete client;

  return (httpCode == 200);
}








//SETUP
void setup() {
  Serial.begin(115200);

  secureClient.setInsecure();

  EEPROM.begin(EEPROM_SIZE);
  Serial.println("//start");
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  dht.begin();

  pinMode(SLP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  pinMode(PROX_UP_PIN, INPUT_PULLUP);
  pinMode(PROX_DOWN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PROX_UP_PIN), proxISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_DOWN_PIN), proxISR2, FALLING);
  Serial.println("//start-------1");
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  lastAB = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);




  connectWiFi();
  Serial.println("//start------2");
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
  Serial.println("//start--------3");
  connectMQTT();
  Serial.println("//start-------------4");


  loadStateChange();

  Serial.print("Restored state_change = ");
  Serial.println(state_change);
}

//LOOP
void loop() {
  esp_task_wdt_reset();
  Serial.println("//start------------5");
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!client.connected()) connectMQTT();
  client.loop();

  Serial.println("//start===============6");
  
unsigned long now = millis();

//VISIBILITY < 500 == OPEN 
if (visibilityValue < 500) {

  visHighStartTime = 0;   // reset opposite timer

  if (visLowStartTime == 0) {
    visLowStartTime = now;   // start low-vis timer
  }

  // if low visibility persists for 5 minutes
  if ((now - visLowStartTime >= VIS_DELAY_MS) && !state_change) {
    Serial.println("Visibility < 500 for 5 min → OPEN curtain");

    proxDownTriggered = false;
    runCurtain(true, proxUpTriggered, timer_open);

    state_change = true;
    saveStateChange(true);
  }
}

//VISIBILITY >= 500 == CLOSE
else {

  visLowStartTime = 0;   // reset opposite timer

  if (visHighStartTime == 0) {
    visHighStartTime = now;  // start high-vis timer
  }

  // if high visibility persists for 5 minutes
  if ((now - visHighStartTime >= VIS_DELAY_MS) && state_change) {
    Serial.println("Visibility >= 500 for 5 min → CLOSE curtain");

    proxUpTriggered = false;
    runCurtain(false, proxDownTriggered, timer_close);

    state_change = false;
    saveStateChange(false);
  }
}


  Serial.println("//start------------7");
  // DHT22
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && !isnan(t)) {
    Serial.print("H: ");
    Serial.print(h);
    Serial.print(" % | T: ");
    Serial.print(t);
    Serial.println(" C");
  }



  // GOOGLE SHEET LOGGING 
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    long vis;
    noInterrupts();
    vis = visibilityValue;
    interrupts();

    String topic = "datavis";
    String visibilityStr = String(vis);
    String humidityStr   = isnan(h) ? "" : String(h, 1);
    String curtainStr    = state_change ? "OPEN" : "CLOSED";

    String rawmsg = "";
    rawmsg += "vis=" + visibilityStr;
    rawmsg += ", curtain=" + curtainStr;
    rawmsg += ", encoder=";
    rawmsg += String(getDistanceMM(), 1);   // float → 1 decimal

    bool ok = postToGoogleSheet(
                topic,
                visibilityStr,
                humidityStr,
                curtainStr,
                rawmsg
              );

    Serial.println(ok ? "Google Sheet POST OK" : "Google Sheet POST FAILED");
  }




  delay(2000);
}
