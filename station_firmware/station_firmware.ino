/*
  Sistema de alerta de riscos para espaços confinados
  
  Projeto para a disciplina de Microprocessadores da UPE/Poli
  Outubro de 2022
*/

// ---------- Importacao de Bibliotecas ----------
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>


#include "env.h"

// ---------- Diretivas ----------
// Hardware
#define GAS_SENSOR_PIN    34
#define LED_PIN           2

#define CRITICAL_GAS_VALUE            60

#define GAS_PUBLISH_INTERVAL          5000
#define GAS_CRITICAL_ALERT_INTERVAL   30000

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


// ---------- Declaracao de variaveis ----------
// Sensor de gas
float gasSensorValue = 0, gasSensorValuePercent = 0;

// WiFi
const char* ssid = ENV_SSID;
const char* password = ENV_PASSWORD;

// MQTT
const char* mqtt_server = MQTT_BROKER_ADDR;     // Endereco do Broker MQTT
long previousTime = 0, previousTime2 = 0;
bool alreadyResetAlert = false;

// ---------- Instanciando objetos ----------
WiFiClient espClient;

PubSubClient client(espClient);
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_ROWS);  


// ---------- Declarando Prototipo das Funcoes ----------
void setupWifi();
void callback(char*, byte*, unsigned int);
void reconnect();

void controlaLed(String);
void trataLcd(String);


// ---------- Setup ----------
void setup() {
  // Configura IOs
  pinMode(LED_PIN, OUTPUT);
  pinMode(GAS_SENSOR_PIN, INPUT);
  
  // Inicializando comunicacao serial
  Serial.begin(115200);

  // Configurando WiFi
  setupWifi();

  // Configurando LCD
  lcd.init();
  lcd.backlight();

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

  // Mantem conexao MQTT
  long currentTime = millis();
  if (currentTime - previousTime >= GAS_PUBLISH_INTERVAL) {
    previousTime = currentTime;

    // Leitura do sensor de gas
    gasSensorValue = analogRead(GAS_SENSOR_PIN);
    gasSensorValuePercent = map(gasSensorValue, 0, 4095, 0, 100);
    
    // Convertendo valor para um array de caracteres
    char gasSensorValuePercentString[10];
    dtostrf(gasSensorValuePercent, 5, 2, gasSensorValuePercentString);
    
    client.publish(TOPIC_GAS_SENSOR_VALUE, gasSensorValuePercentString);
    Serial.print("Sensor de gás: ");
    Serial.println(gasSensorValuePercentString);
    
    if (gasSensorValuePercent >= CRITICAL_GAS_VALUE) {
      alreadyResetAlert = false;
      
      long currentTime2 = millis();
      if (currentTime2 - previousTime2 >= GAS_CRITICAL_ALERT_INTERVAL) {
        previousTime2 = currentTime2;
        
        client.publish(TOPIC_GAS_SENSOR_ALERT, "Perigo de incêndio!");
        Serial.println("Publicada mensagem de socorro");
      }
      
    } else {

      if (!alreadyResetAlert) {
        alreadyResetAlert = true;
        
        client.publish(TOPIC_GAS_SENSOR_ALERT, "Situação sob controle");
        Serial.println("Publicada mensagem: Situação sob controle");
      }        

    }
  }
}


// ---------- Funcoes Auxiliares para WiFi ----------
void setupWifi() {
  delay(10);

  // Conectando a rede WiFi
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}


// ---------- Funcoes Auxiliares para MQTT ----------
void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;

  Serial.print("Mensagem recebida no tópico ");
  Serial.print(topic);
  Serial.print("  |  Conteúdo: ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  
  if (String(topic) == TOPIC_LED) {
    controlaLed(messageTemp);
  } else if (String(topic) == TOPIC_LCD) {
    trataLcd(messageTemp);
  }
}

void controlaLed(String receivedMessage){
  if (receivedMessage == "1") {
    Serial.println("Ligando LED...");
    digitalWrite(LED_PIN, HIGH);
  } else if (receivedMessage == "0") {
    Serial.println("Desligando LED...");
    digitalWrite(LED_PIN, LOW);      
  }
}

void trataLcd(String receivedMessage){
  Serial.print("Mensagem recebida: ");
  Serial.println(receivedMessage);

  if (receivedMessage == "SOS") {
    lcd.clear();
    lcd.setCursor(0, 0);    // col, row
    lcd.print("    Socorro");
    lcd.setCursor(0, 1);
    lcd.print("   a caminho!");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Tentativa de conexão MQTT...");
    
    if (client.connect(MQTT_CLIENT_NAME)) {
      Serial.println("    Conectado!");
      
      // Subscribe
      client.subscribe(TOPIC_LED);
      client.subscribe(TOPIC_LCD);
    } else {
      Serial.print("    Falha na conexao, rc=");
      Serial.print(client.state());
      Serial.println(" nova tentativa em 5 segundos...");
      delay(5000);
    }
  }
}
