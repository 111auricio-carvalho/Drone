#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t macControle[6] = {0x7C, 0x4F, 0xAD, 0x1D, 0xEF, 0xD4};

// ==========================================
// 1. PINAGEM DO DRONE (ESP32-S3 MINI Nano)
// ==========================================
const int PIN_I2C_SDA = 8;
const int PIN_I2C_SCL = 9;
const uint8_t MPU_ADDR = 0x68;

const int PIN_M1_PWM = 4;  // Dianteiro Esquerdo (CW)
const int PIN_M2_PWM = 5;  // Dianteiro Direito (CCW)
const int PIN_M3_PWM = 6;  // Traseiro Direito (CW)
const int PIN_M4_PWM = 7;  // Traseiro Esquerdo (CCW)

const int PIN_M1_LED = 10;
const int PIN_M2_LED = 11;
const int PIN_M3_LED = 12;
const int PIN_M4_LED = 13;

const int PWM_FREQ = 500;
const int PWM_RES  = 10;
const int ESC_MIN  = 512;
const int ESC_IDLE = 535;
const int ESC_MAX  = 1023;

// ==========================================
// 2. ESTRUTURAS ESP-NOW
// ==========================================
typedef struct struct_comando {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t modo;
  uint8_t aux1;
  uint8_t aux2;
  uint8_t aux3;
} struct_comando;

typedef struct struct_telemetria {
  float tensaoBateriaDrone;
  bool bateriaBaixa;
} struct_telemetria;

struct_comando comandoRecebido;
struct_telemetria telemetriaEnviar;

// ==========================================
// 3. VARIÁVEIS DA IMU (MPU-6050)
// ==========================================
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accOffsetRoll = 0, accOffsetPitch = 0;
float anguloRoll = 0, anguloPitch = 0;
float taxaGyroRoll = 0, taxaGyroPitch = 0, taxaGyroYaw = 0;

// ==========================================
// 4. GANHOS PID EQUILIBRADOS (100g)
// ==========================================
float Kp_ang = 1.60;
float Ki_ang = 0.01;
float Kd_ang = 0.45;

float Kp_yaw = 0.40;

float integralRoll = 0, integralPitch = 0;
const float INTEGRAL_MAX = 35.0;

// ==========================================
// 5. CONTROLE DE ESTADO
// ==========================================
bool motoresArmados = false;
bool modoPairarAtivo = false;
int throttleTravado = ESC_IDLE;

unsigned long ultimoPacote = 0;
unsigned long ultimoLoopUs = 0;
unsigned long ultimoPrint = 0;
uint8_t aux1Ant = 0, aux2Ant = 0, aux3Ant = 0;

void escreverMotores(int m1, int m2, int m3, int m4) {
  ledcWrite(PIN_M1_PWM, constrain(m1, ESC_MIN, 1023));
  ledcWrite(PIN_M2_PWM, constrain(m2, ESC_MIN, 1023));
  ledcWrite(PIN_M3_PWM, constrain(m3, ESC_MIN, 1023));
  ledcWrite(PIN_M4_PWM, constrain(m4, ESC_MIN, 1023));
}

void iniciarMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); // +-500 deg/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10); // +-8g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03); // DLPF 44Hz
  Wire.endTransmission(true);
}

void calibrarMPU6050() {
  long somaGX = 0, somaGY = 0, somaGZ = 0;
  float somaRoll = 0, somaPitch = 0;
  const int amostras = 300;

  for (int i = 0; i < amostras; i++) {
    bool pisca = (i / 15) % 2;
    digitalWrite(PIN_M1_LED, pisca);
    digitalWrite(PIN_M2_LED, pisca);
    digitalWrite(PIN_M3_LED, pisca);
    digitalWrite(PIN_M4_LED, pisca);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    float fax = ax / 4096.0;
    float fay = ay / 4096.0;
    float faz = az / 4096.0;

    // Pitch mantido + Roll invertido (removido o '-' de -fax)
    somaPitch += (atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0 / PI);
    somaRoll  += (atan2(fax, sqrt(fay * fay + faz * faz)) * 180.0 / PI);

    somaGX += gx;
    somaGY += gy;
    somaGZ += gz;
    delay(3);
  }

  gyroOffsetX = somaGX / (float)amostras;
  gyroOffsetY = somaGY / (float)amostras;
  gyroOffsetZ = somaGZ / (float)amostras;
  accOffsetRoll  = somaRoll / (float)amostras;
  accOffsetPitch = somaPitch / (float)amostras;
  anguloRoll = 0;
  anguloPitch = 0;
}

void atualizarIMU(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() == 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    // Roll invertido para M2 e M3 acelerarem quando baixarem
    taxaGyroPitch =  (gy - gyroOffsetY) / 65.5;
    taxaGyroRoll  = -(gx - gyroOffsetX) / 65.5;
    taxaGyroYaw   =  (gz - gyroOffsetZ) / 65.5;

    float fax = ax / 4096.0;
    float fay = ay / 4096.0;
    float faz = az / 4096.0;

    float accPitch = (atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0 / PI) - accOffsetPitch;
    float accRoll  = (atan2(fax, sqrt(fay * fay + faz * faz)) * 180.0 / PI) - accOffsetRoll;

    anguloRoll  = 0.985 * (anguloRoll  + taxaGyroRoll  * dt) + 0.015 * accRoll;
    anguloPitch = 0.985 * (anguloPitch + taxaGyroPitch * dt) + 0.015 * accPitch;
  }
}

void aoReceberComando(const esp_now_recv_info_t *info, const uint8_t *dados, int len) {
  if (len == sizeof(comandoRecebido)) {
    memcpy(&comandoRecebido, dados, sizeof(comandoRecebido));
    ultimoPacote = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  ledcAttach(PIN_M1_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_M2_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_M3_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_M4_PWM, PWM_FREQ, PWM_RES);
  escreverMotores(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);

  pinMode(PIN_M1_LED, OUTPUT);
  pinMode(PIN_M2_LED, OUTPUT);
  pinMode(PIN_M3_LED, OUTPUT);
  pinMode(PIN_M4_LED, OUTPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  iniciarMPU6050();
  calibrarMPU6050();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(aoReceberComando);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macControle, 6);
  esp_now_add_peer(&peerInfo);

  ultimoLoopUs = micros();
}

void loop() {
  unsigned long agoraUs = micros();
  float dt = (agoraUs - ultimoLoopUs) / 1000000.0;
  if (dt < 0.002) return;
  ultimoLoopUs = agoraUs;

  atualizarIMU(dt);

  unsigned long agoraMs = millis();
  bool sinalOk = (agoraMs - ultimoPacote < 300);

  if (!sinalOk || abs(anguloRoll) > 65.0 || abs(anguloPitch) > 65.0) {
    motoresArmados = false;
    modoPairarAtivo = false;
    integralRoll = integralPitch = 0;
    escreverMotores(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
  } else {
    if (comandoRecebido.aux3 == 1 && aux3Ant == 0 && !motoresArmados) {
      calibrarMPU6050();
      ultimoLoopUs = micros();
    }
    aux3Ant = comandoRecebido.aux3;

    if (comandoRecebido.aux1 == 1 && aux1Ant == 0) {
      if (!motoresArmados) {
        if (comandoRecebido.throttle < -750 && abs(anguloRoll) < 15.0 && abs(anguloPitch) < 15.0) {
          motoresArmados = true;
          modoPairarAtivo = false;
          integralRoll = integralPitch = 0;
        }
      } else {
        motoresArmados = false;
        modoPairarAtivo = false;
      }
    }
    aux1Ant = comandoRecebido.aux1;

    if (comandoRecebido.aux2 == 1 && aux2Ant == 0 && motoresArmados) {
      modoPairarAtivo = !modoPairarAtivo;
      if (modoPairarAtivo) {
        int thrAtual = ESC_IDLE;
        if (comandoRecebido.throttle > -900) {
          thrAtual = map(comandoRecebido.throttle, -900, 1000, ESC_IDLE, 960);
        }
        throttleTravado = constrain(thrAtual, ESC_IDLE + 30, 920);
      }
    }
    aux2Ant = comandoRecebido.aux2;
  }

  bool estadoLed = LOW;
  if (!motoresArmados) estadoLed = ((agoraMs / 400) % 2);
  else if (modoPairarAtivo) estadoLed = ((agoraMs / 100) % 2);
  else estadoLed = HIGH;

  digitalWrite(PIN_M1_LED, estadoLed);
  digitalWrite(PIN_M2_LED, estadoLed);
  digitalWrite(PIN_M3_LED, estadoLed);
  digitalWrite(PIN_M4_LED, estadoLed);

  int m1 = ESC_MIN, m2 = ESC_MIN, m3 = ESC_MIN, m4 = ESC_MIN;

  if (motoresArmados) {
    float anguloMax = 12.0;
    float taxaYawMax = 80.0;
    if (comandoRecebido.modo == 2) { anguloMax = 25.0; taxaYawMax = 130.0; }
    if (comandoRecebido.modo == 3) { anguloMax = 40.0; taxaYawMax = 180.0; }

    int throttleBase = ESC_IDLE;
    float alvoRoll = 0, alvoPitch = 0, alvoYaw = 0;

    if (modoPairarAtivo) {
      if (abs(comandoRecebido.throttle) > 250) {
        modoPairarAtivo = false;
      } else {
        throttleBase = throttleTravado;
        alvoRoll  = (comandoRecebido.roll / 1000.0) * 10.0;
        alvoPitch = (comandoRecebido.pitch / 1000.0) * 10.0;
        alvoYaw   = (comandoRecebido.yaw / 1000.0) * 60.0;
      }
    }

    if (!modoPairarAtivo) {
      alvoRoll  = (comandoRecebido.roll / 1000.0) * anguloMax;
      alvoPitch = (comandoRecebido.pitch / 1000.0) * anguloMax;
      alvoYaw   = (comandoRecebido.yaw / 1000.0) * taxaYawMax;

      if (comandoRecebido.throttle > -900) {
        throttleBase = map(comandoRecebido.throttle, -900, 1000, ESC_IDLE, 960);
      }
    }

    bool emVoo = (throttleBase > ESC_IDLE + 40);

    float erroRoll = alvoRoll - anguloRoll;
    if (emVoo) integralRoll = constrain(integralRoll + erroRoll * Ki_ang * dt * 100.0, -INTEGRAL_MAX, INTEGRAL_MAX);
    else integralRoll = 0;
    float pidRoll = (Kp_ang * erroRoll) + integralRoll - (Kd_ang * taxaGyroRoll);

    float erroPitch = alvoPitch - anguloPitch;
    if (emVoo) integralPitch = constrain(integralPitch + erroPitch * Ki_ang * dt * 100.0, -INTEGRAL_MAX, INTEGRAL_MAX);
    else integralPitch = 0;
    float pidPitch = (Kp_ang * erroPitch) + integralPitch - (Kd_ang * taxaGyroPitch);

    float erroYaw = alvoYaw - taxaGyroYaw;
    float pidYaw = (Kp_yaw * erroYaw);

    m1 = throttleBase + pidRoll - pidPitch - pidYaw;
    m2 = throttleBase - pidRoll - pidPitch + pidYaw;
    m3 = throttleBase - pidRoll + pidPitch - pidYaw;
    m4 = throttleBase + pidRoll + pidPitch + pidYaw;

    m1 = constrain(m1, ESC_IDLE, ESC_MAX);
    m2 = constrain(m2, ESC_IDLE, ESC_MAX);
    m3 = constrain(m3, ESC_IDLE, ESC_MAX);
    m4 = constrain(m4, ESC_IDLE, ESC_MAX);
  }

  escreverMotores(m1, m2, m3, m4);

  if (agoraMs - ultimoPrint >= 100) {
    ultimoPrint = agoraMs;
    Serial.printf("R:%5.1f P:%5.1f | M1:%4d M2:%4d M3:%4d M4:%4d\n",
                  anguloRoll, anguloPitch, m1, m2, m3, m4);

    telemetriaEnviar.tensaoBateriaDrone = 4.10;
    telemetriaEnviar.bateriaBaixa = false;
    esp_now_send(macControle, (uint8_t *)&telemetriaEnviar, sizeof(telemetriaEnviar));
  }
}