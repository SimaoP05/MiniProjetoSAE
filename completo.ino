#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

// Configurações
const int buzzerPin = 8;
const int emgPin = A0;

const float accelThreshold = 15.0;  // m/s²
const int emgThreshold = 50;         // valor mínimo de EMG

const unsigned long buzzerOnTime = 1000;  // 1 segundo
const unsigned long buzzerOffTime = 3000; // 3 segundos

unsigned long previousMillis = 0;
bool buzzerState = false;
bool alarmActive = false;

// Offsets de gravidade
float gravityOffset = 9.81;  // simples, para este exemplo

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Erro: MPU6050 não detectado!");
    while (1);
  }

  Serial.println("Sistema pronto!");
}

void loop() {
  // --- Leitura do acelerómetro ---
  int16_t ax_raw = mpu.getAccelerationX();
  int16_t ay_raw = mpu.getAccelerationY();
  int16_t az_raw = mpu.getAccelerationZ();

  // Conversão para m/s²
  float ax = (ax_raw / 16384.0) * 9.80665;
  float ay = (ay_raw / 16384.0) * 9.80665;
  float az = (az_raw / 16384.0) * 9.80665;

  // Magnitude e remoção da gravidade
  float magnitude = sqrt(ax*ax + ay*ay + az*az);
  float accelLinear = magnitude - gravityOffset;
  if (accelLinear < 0) accelLinear = 0;

  Serial.print("Accel sem gravidade: ");
  Serial.println(accelLinear);

  // --- Leitura do EMG ---
  int emgValue = analogRead(emgPin);
  Serial.print("EMG: ");
  Serial.println(emgValue);

  unsigned long currentMillis = millis();

  // --- Detecção de impacto brusco ---
  if (accelLinear > accelThreshold) {
    Serial.println("Impacto brusco detectado!");

    // Se não houver sinal muscular suficiente
    if (emgValue < emgThreshold) {
      alarmActive = true;
      previousMillis = currentMillis;
      buzzerState = true;
      digitalWrite(buzzerPin, HIGH);
      Serial.println("Sem sinais musculares -> alarme ativado");
    }
  }

  // --- Lógica do buzzer intermitente ---
  if (alarmActive) {
    if (buzzerState && currentMillis - previousMillis >= buzzerOnTime) {
      buzzerState = false;
      previousMillis = currentMillis;
      digitalWrite(buzzerPin, LOW);
    } else if (!buzzerState && currentMillis - previousMillis >= buzzerOffTime) {
      buzzerState = true;
      previousMillis = currentMillis;
      digitalWrite(buzzerPin, HIGH);
    }
  }

  delay(100);
}