#define BUZZER_PIN           18
#define TOOGLE_INTERVAL_MS   100

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  toggle(BUZZER_PIN);
}

void toggle(uint8_t pin){
  static long previousTime = 0;

  if (millis() - previousTime >= TOOGLE_INTERVAL_MS) {
    previousTime = millis();
    digitalWrite(pin, !digitalRead(pin));
  }
}