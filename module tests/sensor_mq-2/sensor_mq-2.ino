#define MQ2_SENSOR_PIN                  34
#define ADC_RESOLUTION                  4095
#define INTERVAL_MS_BETWEEN_READINGS    10
#define NUMBER_OF_READINGS              100

unsigned int mq2SumValue = 0, reading = 0;
float mq2ValuePercent = 0, mq2Average = 0;
long currentTime = 0, previousTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(MQ2_SENSOR_PIN, INPUT);
}

void loop() {
  
  if (reading >= NUMBER_OF_READINGS) {
    reading = 0;

    mq2Average = mq2SumValue / NUMBER_OF_READINGS;
    mq2SumValue = 0;

    mq2ValuePercent = map(mq2Average, 0, ADC_RESOLUTION, 0, 100);

    Serial.print("Leitura do Sensor MQ-2: ");
    Serial.print(mq2ValuePercent);
    Serial.println("%");
  
  } else {
    
    long currentTime = millis();
    if (currentTime - previousTime >= INTERVAL_MS_BETWEEN_READINGS) {
      previousTime = currentTime;

      mq2SumValue += analogRead(MQ2_SENSOR_PIN);

      reading++;
    }
  }

}
