/*
  Sistema de monitoramento de GLP para espaços confinados
  
  Projeto para a disciplina de Microprocessadores da UPE/Poli
  Outubro de 2022
*/

// ---------- Importacao de Bibliotecas ----------
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

#include "env.h"


// ---------- Diretivas ----------
// Debug
// #define DEBUG_SENSOR_MQ2
#define DEBUG_WIFI
#define DEBUG_MQTT
// #define DEBUG_LCD

// Sirene
#define LED_PIN              2
#define BUZZER_PIN           18
#define TOOGLE_INTERVAL_MS   50

//LCD
#define LCD_COLUMNS   16
#define LCD_ROWS      2
#define I2C_ADDR      0x27

// MQTT
#define MQTT_CLIENT_NAME            "uPProject_Station1"
#define MQTT_BROKER_ADDR            "test.mosquitto.org"

#define TOPIC_LED                   "uPProject/station/1/led/value"
#define TOPIC_LCD                   "uPProject/station/1/lcd/value"
#define TOPIC_GAS_SENSOR_VALUE      "uPProject/station/1/mq2/value"
#define TOPIC_GAS_SENSOR_ALERT      "uPProject/station/1/mq2/alert"

// Sensor MQ-2
#define MQ2_SENSOR_PIN                  34
#define ADC_RESOLUTION                  4095
#define INTERVAL_MS_BETWEEN_READINGS    10
#define NUMBER_OF_READINGS              100

#define CRITICAL_GAS_VALUE              10
#define GAS_PUBLISH_INTERVAL            5000
#define GAS_CRITICAL_ALERT_INTERVAL     30000


// ---------- Declaracao de variaveis ----------
long previousTime = 0, previousTime2 = 0, previousTime3 = 0;
bool panic = false, firstValue = true;

// WiFi
const char* ssid = ENV_SSID;
const char* password = ENV_PASSWORD;

// MQTT
const char* mqtt_server = MQTT_BROKER_ADDR;     // Endereco do Broker MQTT
bool alreadyResetAlert = false;

// Sensor MQ-2
unsigned int mq2SumValue = 0, reading = 0;
float mq2ValuePercent = 0, mq2Average = 0, lastMq2ValuePercent = 0;


// ---------- Instanciando objetos ----------
WiFiClient espClient;

PubSubClient client(espClient);
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_ROWS);  


// ---------- Declarando Prototipo das Funcoes ----------
void setupWifi();
void callback(char*, byte*, unsigned int);
void reconnect();
void leituraDoSensorMQ2();
void trataLcd(String);


// ---------- Setup ----------
void setup() {
  // Configurando LCD
  lcd.init();
  lcd.backlight();

  // Informacoes
  lcd.clear();
  lcd.setCursor(0, 0);    // col, row
  lcd.print(" Inicializando");
  lcd.setCursor(0, 1);
  lcd.print("   sistema...");

  // Configura IOs
  pinMode(MQ2_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Inicializando comunicacao serial
  Serial.begin(115200);

  // Configurando WiFi
  setupWifi();

  // Configurando MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}


// ---------- Loop Infinito ----------
void loop() {
  // Mantem conexao MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  leituraDoSensorMQ2();

  if (panic && (millis() - previousTime3 >= TOOGLE_INTERVAL_MS)) {
    previousTime3 = millis();
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  if ((mq2ValuePercent != lastMq2ValuePercent) || (firstValue)) {
    firstValue = false;
    lastMq2ValuePercent = mq2ValuePercent;
    
    // Convertendo valor para um array de caracteres
    char mq2ValuePercentString[10];
    dtostrf(mq2ValuePercent, 5, 2, mq2ValuePercentString);
    
    bool pubState = client.publish(TOPIC_GAS_SENSOR_VALUE, mq2ValuePercentString);
    
    #ifdef DEBUG_MQTT
      if(pubState) {
        Serial.print("[MQTT] Publicado valor do Sensor de gás: ");
        Serial.println(mq2ValuePercentString);
      } else {
        Serial.print("[MQTT] Falha ao publicar valor do Sensor de gás: ");
        Serial.println(mq2ValuePercentString);
      }
    #endif

    lcd.setCursor(0, 1);
    lcd.print("GLP(%):         ");
    lcd.setCursor(9, 1);
    lcd.print(mq2ValuePercent);
    
    if (mq2ValuePercent >= CRITICAL_GAS_VALUE) {
      alreadyResetAlert = false;      
      panic = true;

      lcd.setCursor(0, 0);    // col, row
      lcd.print("    Perigo!     ");

      if (millis() - previousTime2 >= GAS_CRITICAL_ALERT_INTERVAL) {
        previousTime2 = millis();
        
        bool pubState = client.publish(TOPIC_GAS_SENSOR_ALERT, "Perigo de incêndio!");

        #ifdef DEBUG_MQTT
          if(pubState) {
            Serial.println("[MQTT] Publicada mensagem de socorro!");
          } else {
            Serial.println("[MQTT] Falha ao publicar a mensagem de socorro");
          }
        #endif
      }

    } else {
      panic = false;

      if (!alreadyResetAlert) {
        alreadyResetAlert = true;
        previousTime2 = 0;      // Para que uma publicacao em TOPIC_GAS_SENSOR_ALERT possa ser feita imediatamente
      
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        
        lcd.setCursor(0, 0);    // col, row
        lcd.print("Ambiente seguro!");

        bool pubState = client.publish(TOPIC_GAS_SENSOR_ALERT, "Situação sob controle");

        #ifdef DEBUG_MQTT
          if (pubState) {
            Serial.println("[MQTT] Publicada mensagem: Situação sob controle");
          } else {
            Serial.println("[MQTT] Falha ao publicar a mensagem: Situação sob controle");
          }
        #endif
      }

    }
  }
}


// ---------- Funcoes Auxiliares para WiFi ----------
void setupWifi() {
  delay(10);

  // Conectando a rede WiFi
  #ifdef DEBUG_WIFI
    Serial.println();
    Serial.print("Conectando a ");
    Serial.println(ssid);
  #endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

    #ifdef DEBUG_WIFI
      Serial.print(".");
    #endif
  }

  #ifdef DEBUG_WIFI
    Serial.println("");
    Serial.println("WiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  #endif

  // Informacoes sobre o WiFi
  lcd.clear();
  lcd.setCursor(0, 0);    // col, row
  lcd.print(" WiFi Conectado");

}


// ---------- Funcoes Auxiliares para MQTT ----------
void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;

  #ifdef DEBUG_MQTT
    Serial.print("Mensagem recebida no tópico ");
    Serial.print(topic);
    Serial.print("  |  Conteúdo: ");
  #endif
  
  for (int i = 0; i < length; i++) {
    #ifdef DEBUG_MQTT
      Serial.print((char)message[i]);
    #endif
    messageTemp += (char)message[i];
  }
  #ifdef DEBUG_MQTT
    Serial.println();
  #endif

  if (String(topic) == TOPIC_LCD) {
    trataLcd(messageTemp);
  }
}

void trataLcd(String receivedMessage){
  #ifdef DEBUG_LCD
    Serial.print("Mensagem recebida: ");
    Serial.println(receivedMessage);
  #endif

  if (receivedMessage == "SOS") {
    lcd.setCursor(0, 0);    // col, row
    lcd.print("Equipe a caminho");
  }
}

void reconnect() {
  while (!client.connected()) {

    #ifdef DEBUG_MQTT
      Serial.println("Tentativa de conexão MQTT...");
    #endif

    if (client.connect(MQTT_CLIENT_NAME)) {
  
      #ifdef DEBUG_MQTT
        Serial.println("    Conectado!");
      #endif
      
      // Subscribe
      client.subscribe(TOPIC_LED);
      client.subscribe(TOPIC_LCD);

    } else {

      #ifdef DEBUG_MQTT
        Serial.print("    Falha na conexao, rc=");
        Serial.print(client.state());
        Serial.println(" nova tentativa em 5 segundos...");
      #endif

      delay(5000);
    }
  }
}

void leituraDoSensorMQ2(){
  if (reading >= NUMBER_OF_READINGS) {
    reading = 0;

    mq2Average = mq2SumValue / NUMBER_OF_READINGS;
    mq2SumValue = 0;

    mq2ValuePercent = map(mq2Average, 0, ADC_RESOLUTION, 0, 100);

    #ifdef DEBUG_SENSOR_MQ2
      Serial.print("[Sensor MQ-2] Média de leituras: ");
      Serial.print(mq2ValuePercent);
      Serial.println("%");
    #endif
  
  } else {
    
    if (millis() - previousTime >= INTERVAL_MS_BETWEEN_READINGS) {
      previousTime = millis();

      mq2SumValue += analogRead(MQ2_SENSOR_PIN);

      reading++;
    }
  }
}
