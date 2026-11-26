#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t macDrone[6] = {0xD8, 0x85, 0xAC, 0xD4, 0x3A, 0x94};

const int PIN_JOY_LX = 1; // Yaw (Girar)
const int PIN_JOY_LY = 2; // Throttle (Aceleracao)
const int PIN_JOY_RX = 3; // Roll (Lados)
const int PIN_JOY_RY = 4; // Pitch (Frente/Tras)

const int PIN_BTN_LEVE     = 5;
const int PIN_BTN_MEDIO    = 6;
const int PIN_BTN_AVANCADO = 7;
const int PIN_BTN_AUX1     = 8;
const int PIN_BTN_AUX2     = 9;
const int PIN_BTN_AUX3     = 10;

const int PIN_LED_VERDE       = 11;
const int PIN_LED_AZUL        = 12;
const int PIN_LED_AMARELO     = 13;
const int PIN_LED_VERM_MODO   = 14;
const int PIN_LED_VERM_BAT_DR = 15;

const int TODOS_LEDS[5] = {
  PIN_LED_VERDE, PIN_LED_AZUL, PIN_LED_AMARELO,
  PIN_LED_VERM_MODO, PIN_LED_VERM_BAT_DR
};

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

struct_comando pacoteControle;
struct_telemetria pacoteTelemetria;

enum ModoVoo { MODO_LEVE = 1, MODO_MEDIO = 2, MODO_AVANCADO = 3 };
ModoVoo modoAtual = MODO_LEVE;

bool droneConectado = false;
bool batDroneBaixa  = false;
unsigned long ultimoAckDrone = 0;

int centroLX = 2048, centroLY = 2048, centroRX = 2048, centroRY = 2048;
const int DEADZONE = 160;

unsigned long ultimoPiscaVerde = 0;
bool estadoPiscaVerde = false;
unsigned long ultimoEnvioEspNow = 0;
unsigned long ultimoDebounce = 0;

void aoEnviarDados(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    droneConectado = true;
    ultimoAckDrone = millis();
  }
}

void aoReceberTelemetria(const esp_now_recv_info_t *info, const uint8_t *dados, int len) {
  if (len == sizeof(pacoteTelemetria)) {
    memcpy(&pacoteTelemetria, dados, sizeof(pacoteTelemetria));
    batDroneBaixa = pacoteTelemetria.bateriaBaixa;
  }
}

void atualizarLEDsMultiplexados() {
  static int indiceLed = 0;
  static unsigned long ultimoTrocaLed = 0;

  if (micros() - ultimoTrocaLed < 2000) return;
  ultimoTrocaLed = micros();

  for (int i = 0; i < 5; i++) digitalWrite(TODOS_LEDS[i], LOW);

  bool acender = false;
  switch (indiceLed) {
    case 0: acender = droneConectado ? true : estadoPiscaVerde; break;
    case 1: acender = (modoAtual == MODO_LEVE); break;
    case 2: acender = (modoAtual == MODO_MEDIO); break;
    case 3: acender = (modoAtual == MODO_AVANCADO); break;
    case 4: acender = batDroneBaixa; break;
  }

  if (acender) digitalWrite(TODOS_LEDS[indiceLed], HIGH);
  indiceLed = (indiceLed + 1) % 5;
}

void calibrarCentroJoysticks() {
  long somaLX = 0, somaLY = 0, somaRX = 0, somaRY = 0;
  for (int i = 0; i < 100; i++) {
    somaLX += analogRead(PIN_JOY_LX);
    somaLY += analogRead(PIN_JOY_LY);
    somaRX += analogRead(PIN_JOY_RX);
    somaRY += analogRead(PIN_JOY_RY);
    delay(5);
  }
  centroLX = somaLX / 100;
  centroLY = somaLY / 100;
  centroRX = somaRX / 100;
  centroRY = somaRY / 100;
}

// Mapeia o curso completo (0 a 4095) e aplica curva exponencial opcional
int processarEixo(int leituraBruta, int centro, bool aplicarExpo) {
  int diferenca = leituraBruta - centro;
  if (abs(diferenca) < DEADZONE) return 0;

  float norm = 0.0f;
  if (diferenca > 0) {
    norm = (float)(leituraBruta - (centro + DEADZONE)) / (float)(4090 - (centro + DEADZONE));
  } else {
    norm = -(float)((centro - DEADZONE) - leituraBruta) / (float)((centro - DEADZONE) - 5);
  }
  norm = constrain(norm, -1.0f, 1.0f);

  // Curva Exponencial: em 50% do stick entrega ~27% de comando; em 100% entrega 100%
  if (aplicarExpo) {
    float expo = 0.60f;
    norm = norm * (1.0f - expo) + (norm * norm * norm) * expo;
  }

  return (int)(norm * 1000.0f);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PIN_BTN_LEVE,     INPUT_PULLUP);
  pinMode(PIN_BTN_MEDIO,    INPUT_PULLUP);
  pinMode(PIN_BTN_AVANCADO, INPUT_PULLUP);
  pinMode(PIN_BTN_AUX1,     INPUT_PULLUP);
  pinMode(PIN_BTN_AUX2,     INPUT_PULLUP);
  pinMode(PIN_BTN_AUX3,     INPUT_PULLUP);

  for (int i = 0; i < 5; i++) {
    pinMode(TODOS_LEDS[i], OUTPUT);
    digitalWrite(TODOS_LEDS[i], LOW);
  }

  delay(500);
  calibrarCentroJoysticks();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_register_send_cb(aoEnviarDados);
  esp_now_register_recv_cb(aoReceberTelemetria);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macDrone, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  unsigned long agora = millis();
  atualizarLEDsMultiplexados();

  if (agora - ultimoAckDrone > 500) droneConectado = false;

  if (agora - ultimoPiscaVerde >= 500) {
    ultimoPiscaVerde = agora;
    estadoPiscaVerde = !estadoPiscaVerde;
  }

  if (agora - ultimoDebounce > 200) {
    if (digitalRead(PIN_BTN_LEVE) == LOW) {
      modoAtual = MODO_LEVE;
      ultimoDebounce = agora;
    } else if (digitalRead(PIN_BTN_MEDIO) == LOW) {
      modoAtual = MODO_MEDIO;
      ultimoDebounce = agora;
    } else if (digitalRead(PIN_BTN_AVANCADO) == LOW) {
      modoAtual = MODO_AVANCADO;
      ultimoDebounce = agora;
    }
  }

  if (agora - ultimoEnvioEspNow >= 20) {
    ultimoEnvioEspNow = agora;

    // Throttle linear usando curso completo (false) | Direcao e Yaw com curva Expo suave (true)
    pacoteControle.yaw      = -processarEixo(analogRead(PIN_JOY_LX), centroLX, true);
    pacoteControle.throttle = -processarEixo(analogRead(PIN_JOY_LY), centroLY, false);
    pacoteControle.roll     = -processarEixo(analogRead(PIN_JOY_RX), centroRX, true);
    pacoteControle.pitch    = -processarEixo(analogRead(PIN_JOY_RY), centroRY, true);
    pacoteControle.modo     = modoAtual;
    pacoteControle.aux1     = (digitalRead(PIN_BTN_AUX1) == LOW) ? 1 : 0;
    pacoteControle.aux2     = (digitalRead(PIN_BTN_AUX2) == LOW) ? 1 : 0;
    pacoteControle.aux3     = (digitalRead(PIN_BTN_AUX3) == LOW) ? 1 : 0;

    esp_now_send(macDrone, (uint8_t *)&pacoteControle, sizeof(pacoteControle));
  }
}