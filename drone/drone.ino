// ============================================================================
//  DRONE - Controladora de voo (ESP32-S3 + MPU-6050 + 4 ESC PWM)
//
//  Controle em cascata:
//    ângulo (P)  ->  setpoint de taxa  ->  PID de taxa (P, I, D filtrado)  ->  mixer
//
//  Feedback e correção dinâmica:
//    * Telemetria (CSV "T,...") pela USB e pelo ESP-NOW (o controle repassa ao PC).
//    * Ganhos ajustáveis em tempo real por comandos de texto (USB ou via controle).
//    * Detector de oscilação: em voo, se detectar tremor, reduz automaticamente
//      a escala dos ganhos de taxa (P e D). "aplicar" + "salvar" grava o resultado.
//    * "orient": calibração guiada da orientação do MPU (descobre eixos e sinais).
//
//  Placa (Arduino IDE / arduino-cli): ESP32S3 Dev Module, "USB CDC On Boot: Enabled"
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <stdarg.h>

uint8_t macControle[6] = {0x7C, 0x4F, 0xAD, 0x1D, 0xEF, 0xD4};

// ==========================================
// 1. PINAGEM
// ==========================================
const int PIN_I2C_SDA = 8;
const int PIN_I2C_SCL = 9;
const uint8_t MPU_ADDR = 0x68;

const int PIN_M1_PWM = 4;  // Dianteiro Esquerdo (CW)
const int PIN_M2_PWM = 5;  // Dianteiro Direito  (CCW)
const int PIN_M3_PWM = 6;  // Traseiro Direito   (CW)
const int PIN_M4_PWM = 7;  // Traseiro Esquerdo  (CCW)

const int PIN_LEDS[4] = {10, 11, 12, 13};

const int PWM_FREQ = 500;
const int PWM_RES  = 10;
const int ESC_MIN  = 512;   // ~1000 us
const int ESC_IDLE = 535;
const int ESC_MAX  = 1023;  // ~2000 us
const int THR_MAX  = 960;   // throttle máximo do stick (deixa margem p/ correção)

// ==========================================
// 2. PACOTES ESP-NOW
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

#define PKT_TELEMETRIA 0xA1
#define PKT_TEXTO      0xA3
#define PKT_CMD_TEXTO  0xB1

typedef struct __attribute__((packed)) {
  uint8_t  tipo;          // PKT_TELEMETRIA
  uint8_t  flags;         // b0 armado, b1 pairar, b2 bateria baixa, b3 oscilando
  uint32_t ms;
  int16_t  roll, pitch;   // graus x100
  int16_t  gR, gP, gY;    // graus/s x10
  int16_t  spR, spP;      // alvo de ângulo x100
  int16_t  m[4];
  int16_t  thr;
  uint8_t  escala;        // escala dinâmica dos ganhos x100
  uint8_t  oscR, oscP;    // frequência de oscilação detectada (Hz)
  uint16_t loopHz;
  float    tensao;
} pkt_telemetria_t;

typedef struct __attribute__((packed)) {
  uint8_t tipo;           // PKT_TEXTO ou PKT_CMD_TEXTO
  char    texto[200];
} pkt_texto_t;

struct_comando comando;            // cópia segura usada pelo loop
volatile struct_comando comandoRx; // escrito pelo callback do Wi-Fi
portMUX_TYPE muxCmd = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long ultimoPacote = 0;

char cmdRemoto[64];
volatile bool cmdRemotoPendente = false;

// ==========================================
// 3. CONFIGURAÇÃO / GANHOS (persistidos em NVS)
// ==========================================
struct Config {
  // Orientação do MPU: eixo do sensor (0=X, 1=Y, 2=Z) e sinal para cada eixo do drone.
  // Convenção usada pelo mixer:
  //   roll  > 0 : lado DIREITO desce
  //   pitch > 0 : NARIZ desce
  //   yaw   > 0 : nariz gira para a DIREITA (horário visto de cima)
  int8_t eixoR, sinalR;
  int8_t eixoP, sinalP;
  int8_t eixoY, sinalY;

  float angP;     // (graus/s) por grau de erro
  float rP, rI, rD;   // PID de taxa roll/pitch (saída em contagens de PWM)
  float yP, yI;       // PID de taxa yaw
  float dtermHz;      // filtro do termo D
  float iMax;         // limite do integrador (contagens)
  bool  autoAjuste;   // redução automática de ganho ao detectar oscilação
};

const Config CONFIG_PADRAO = {
  0, -1,   // roll  = -X (mesmo sinal do firmware antigo)
  1, +1,   // pitch = +Y
  2, +1,   // yaw   = +Z
  4.0f,
  0.30f, 0.50f, 0.0020f,
  0.80f, 0.30f,
  30.0f,
  60.0f,
  true
};

Config cfg = CONFIG_PADRAO;
Preferences prefs;
float escalaGanho = 1.0f;   // escala do P de taxa, ajustada pelo detector (oscilação 6-25 Hz)
float escalaD = 1.0f;       // escala do D de taxa, ajustada pelo detector (vibração >= 25 Hz)

// Filtros do giroscópio (chaves separadas na NVS para não perder a Config gravada)
uint8_t mpuDlpf = 3;        // registrador 0x1A do MPU: 2=94Hz 3=44Hz 4=21Hz 5=10Hz
float gyroHz = 0;           // PT1 extra no giro usado pelo PID (0 = desligado)
float filtG[3] = {0, 0, 0};

// ---- Notch dinâmico: rastreia a frequência dominante de vibração e a remove do giro ----
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  float aplicar(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
};
bool  notchAtivo = true;
float notchHz = 0;          // frequência atual (0 = ainda não detectou)
float notchQ = 2.5f;
Biquad notch[3];
float fsLoop = 1000;        // taxa de amostragem real (filtrada)
// estimador de frequência (cruzamentos por zero do giro passa-alta)
float estLento[3] = {0, 0, 0};
int8_t estLado = 0;
uint16_t estCruz = 0;
float estPico = 0;
unsigned long estT0 = 0;
float notchHzSoma = 0;
uint32_t notchHzN = 0;

// ---- Curva de throttle: stick central = pairar, expo suaviza em volta do centro ----
float thrHover = 736;       // PWM com o stick no centro (ponto de pairar)
float thrExpo = 0.6f;       // 0 = linear, 1 = muito suave no centro
float thrHz = 4.0f;         // suavização do throttle (0 = desligada)
float thrFilt = ESC_IDLE;
float thrMax = 860;         // limite do stick no máximo ("tmax"): acima de ~830 o M1 causou eventos violentos

// ---- TPA: atenua P e D com throttle alto (mais rotação = mais autoridade por contagem) ----
float tpa = 0.4f;           // fração de redução de P/D no throttle máximo (0 = desligado)
float tpaBreak = 760;       // throttle a partir do qual começa a reduzir
float tpaFator = 1.0f;

// Analógico 1 horizontal (yaw): desligado por padrão — o analógico 1 controla só a velocidade.
// O laço de yaw continua ativo com alvo 0, segurando a direção do drone.
bool yawStick = false;

// Ajuste fino de nível (graus): corrige deriva quando o "zero" do sensor não é o nível real
// (calibração em superfície inclinada / placa torta). Positivo em nr inclina para a DIREITA,
// positivo em np inclina o NARIZ para BAIXO (vai para a frente).
float nivelR = 0, nivelP = 0;

// "Subida reta": no modo LEVE (botão do controle) o analógico 2 é ignorado —
// o drone fica travado no nível (com nr/np) e o analógico 1 só sobe/desce.
bool modoReto = true;

// ---- Trim aprendido: base do integrador que compensa motor/CG assimétricos ----
float trimR = 0, trimP = 0, trimY = 0;      // contagens de PWM, salvas na NVS
float aprendR = 0, aprendP = 0, aprendY = 0;
float tempoEstavel = 0;                     // s de voo estável neste voo

// ==========================================
// 4. IMU
// ==========================================
float gyroOff[3] = {0, 0, 0};
float sinalGravidade = 1.0f;   // +1 se o MPU está com a face para cima, -1 se invertido
float accOffR = 0, accOffP = 0;
float gyroS[3];                // taxas do sensor (graus/s), sem offset
float angS[2];                 // ângulos do acelerômetro em torno de X e Y do sensor
bool  accValido = false;

float anguloRoll = 0, anguloPitch = 0;
float taxaRoll = 0, taxaPitch = 0, taxaYaw = 0;
const float TAU_COMPLEMENTAR = 0.5f;   // s (no chão: nivela rápido)
// Em voo o acelerômetro NÃO mede inclinação enquanto o drone acelera para o lado (ele "lê nivelado").
// Com 0,5 s o estimador apagava a inclinação real e o drone derivava cada vez mais.
// Em voo, confia no giro e corrige pelo acelerômetro bem devagar.
float tauVoo = 5.0f;                   // s ("tau")
const float GYRO_LSB = 16.4f;          // LSB por grau/s em +-2000 graus/s
const float ACC_LSB = 2048.0f;         // LSB por g em +-16 g
const float ACC_LPF_HZ = 8.0f;         // filtra a vibração ANTES do atan2 (senão ela vira desvio de ângulo)
float accF[3] = {0, 0, 1};             // acelerômetro filtrado (g)

// ==========================================
// 5. ESTADO DO CONTROLE
// ==========================================
bool motoresArmados = false;
bool modoPairarAtivo = false;
int  throttleTravado = ESC_IDLE;
uint8_t aux1Ant = 0, aux2Ant = 0, aux3Ant = 0;

float iR = 0, iP = 0, iY = 0;
float dFiltR = 0, dFiltP = 0;
float taxaAntR = 0, taxaAntP = 0;
float alvoRollLog = 0, alvoPitchLog = 0;
int   mOut[4] = {ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN};
int   thrLog = 0;

unsigned long ultimoLoopUs = 0;
unsigned long ultimaTelemetria = 0;
uint32_t contLoops = 0, loopHz = 0;
unsigned long ultimoContLoops = 0;
uint8_t modoLog = 2;          // 0 = sem CSV na USB, 1 = 10 Hz, 2 = 50 Hz (padrão: 50 Hz p/ análise)

// Detector de oscilação (por eixo roll/pitch)
struct DetectorOsc {
  float lento = 0;      // passa-baixa lento da taxa
  int8_t lado = 0;      // último lado da histerese
  uint16_t cruzamentos = 0;
  float pico = 0;
  float hz = 0;
};
DetectorOsc oscR, oscP;
unsigned long inicioJanelaOsc = 0;
unsigned long ultimaReducao = 0;
bool oscilando = false;

// Calibração guiada de orientação
enum EstadoOrient { OR_INATIVO, OR_ROLL, OR_PITCH, OR_YAW, OR_ESPERA };
EstadoOrient orientEstado = OR_INATIVO, orientProximo = OR_INATIVO;
float orientInt[3];
unsigned long orientT0 = 0;
int8_t orientEixo[3], orientSinal[3];

// ==========================================
// 6. SAÍDA DE TEXTO (USB + ESP-NOW)
// ==========================================
void msg(const char *fmt, ...) {
  pkt_texto_t p;
  p.tipo = PKT_TEXTO;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(p.texto, sizeof(p.texto), fmt, ap);
  va_end(ap);
  Serial.print("# ");
  Serial.println(p.texto);
  esp_now_send(macControle, (uint8_t *)&p, 1 + strlen(p.texto) + 1);
}

// ==========================================
// 7. NVS
// ==========================================
void carregarConfig() {
  prefs.begin("drone", true);
  if (prefs.getBytesLength("cfg") == sizeof(Config)) {
    prefs.getBytes("cfg", &cfg, sizeof(Config));
  }
  mpuDlpf = constrain(prefs.getUChar("dlpf", 3), 1, 6);
  gyroHz = prefs.getFloat("ghz", 0);
  notchAtivo = prefs.getBool("notch", true);
  trimR = prefs.getFloat("tR", 0);
  trimP = prefs.getFloat("tP", 0);
  trimY = prefs.getFloat("tY", 0);
  thrHover = prefs.getFloat("thov", 736);
  thrExpo = prefs.getFloat("texp", 0.6f);
  thrHz = prefs.getFloat("thz", 4.0f);
  tpa = prefs.getFloat("tpa", 0.4f);
  tpaBreak = prefs.getFloat("tpab", 760);
  yawStick = prefs.getBool("yaws", false);
  thrMax = prefs.getFloat("tmax", 860);
  nivelR = prefs.getFloat("nivR", 0);
  nivelP = prefs.getFloat("nivP", 0);
  modoReto = prefs.getBool("reto", true);
  tauVoo = prefs.getFloat("tau", 5.0f);
  prefs.end();
}

// Stick (-1000..1000, centro 0) -> PWM. Centro = pairar; expo deixa o centro suave;
// as pontas continuam chegando ao mínimo (idle) e ao máximo (THR_MAX).
float curvaThrottle(int16_t stick) {
  float s = constrain(stick / 1000.0f, -1.0f, 1.0f);
  float u = fabsf(s);
  float f = u * (1.0f - thrExpo) + u * u * u * thrExpo;
  if (s >= 0) return thrHover + (thrMax - thrHover) * f;
  return thrHover - (thrHover - ESC_IDLE) * f;
}

void salvarTrim() {
  prefs.begin("drone", false);
  prefs.putFloat("tR", trimR);
  prefs.putFloat("tP", trimP);
  prefs.putFloat("tY", trimY);
  prefs.end();
}

void salvarConfig() {
  prefs.begin("drone", false);
  prefs.putBytes("cfg", &cfg, sizeof(Config));
  prefs.putUChar("dlpf", mpuDlpf);
  prefs.putFloat("ghz", gyroHz);
  prefs.putBool("notch", notchAtivo);
  prefs.putFloat("thov", thrHover);
  prefs.putFloat("texp", thrExpo);
  prefs.putFloat("thz", thrHz);
  prefs.putFloat("tpa", tpa);
  prefs.putFloat("tpab", tpaBreak);
  prefs.putBool("yaws", yawStick);
  prefs.putFloat("tmax", thrMax);
  prefs.putFloat("nivR", nivelR);
  prefs.putFloat("nivP", nivelP);
  prefs.putBool("reto", modoReto);
  prefs.putFloat("tau", tauVoo);
  prefs.end();
  msg("Configuracao salva na memoria.");
}

void mostrarGanhos() {
  const char *nomes = "XYZ";
  msg("ang=%.2f | rp=%.3f ri=%.3f rd=%.4f | yp=%.3f yi=%.3f | dhz=%.0f imax=%.0f | auto=%d escalaP=%.2f escalaD=%.2f",
      cfg.angP, cfg.rP, cfg.rI, cfg.rD, cfg.yP, cfg.yI, cfg.dtermHz, cfg.iMax, cfg.autoAjuste, escalaGanho, escalaD);
  msg("estimador: tau em voo=%.1f s (no chao 0.5 s)", tauVoo);
  msg("filtros: dlpf=%d (MPU %s) ghz=%.0f | notch %s (atual %.0f Hz)", mpuDlpf,
      mpuDlpf == 1 ? "188Hz" : mpuDlpf == 2 ? "94Hz" : mpuDlpf == 3 ? "44Hz" : mpuDlpf == 4 ? "21Hz" : mpuDlpf == 5 ? "10Hz" : "5Hz",
      gyroHz, notchAtivo ? "ligado" : "desligado", notchHz);
  msg("trim aprendido: R %.1f P %.1f Y %.1f (contagens PWM)", trimR, trimP, trimY);
  msg("throttle: thov=%.0f (pairar no centro) tmax=%.0f texp=%.2f thz=%.1f | TPA tpa=%.2f a partir de tpab=%.0f",
      thrHover, thrMax, thrExpo, thrHz, tpa, tpaBreak);
  msg("sticks: analogico 1 = velocidade%s | analogico 2 = lados + frente/tras",
      yawStick ? " + girar" : " (girar desligado)");
  msg("ajuste de nivel: nr=%.1f (+ = direita) np=%.1f (+ = frente) graus", nivelR, nivelP);
  msg("modo LEVE: %s | MEDIO/AVANCADO: analogico 2 ativo",
      modoReto ? "SUBIDA RETA (analogico 2 ignorado)" : "normal (12 graus)");
  msg("orientacao: roll=%c%c pitch=%c%c yaw=%c%c | gravidade=%s",
      cfg.sinalR > 0 ? '+' : '-', nomes[cfg.eixoR],
      cfg.sinalP > 0 ? '+' : '-', nomes[cfg.eixoP],
      cfg.sinalY > 0 ? '+' : '-', nomes[cfg.eixoY],
      sinalGravidade > 0 ? "face p/ cima" : "face p/ baixo");
}

// ==========================================
// 8. MOTORES E MPU
// ==========================================
void escreverMotores(const int m[4]) {
  ledcWrite(PIN_M1_PWM, constrain(m[0], ESC_MIN, ESC_MAX));
  ledcWrite(PIN_M2_PWM, constrain(m[1], ESC_MIN, ESC_MAX));
  ledcWrite(PIN_M3_PWM, constrain(m[2], ESC_MIN, ESC_MAX));
  ledcWrite(PIN_M4_PWM, constrain(m[3], ESC_MIN, ESC_MAX));
}

void motoresDesligados() {
  for (int i = 0; i < 4; i++) mOut[i] = ESC_MIN;
  escreverMotores(mOut);
}

void escreverReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void iniciarMPU6050() {
  escreverReg(0x6B, 0x00);  // acorda
  escreverReg(0x1B, 0x18);  // giro +-2000 graus/s (16.4 LSB/grau/s): +-500 saturava com a vibração
  escreverReg(0x1C, 0x18);  // acel +-16 g (2048 LSB/g): a vibração do M1 pode saturar +-8 g
  escreverReg(0x1A, mpuDlpf);  // DLPF (padrão 3 = 44 Hz)
}

bool lerMPU(int16_t a[3], int16_t g[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  a[0] = (b[0] << 8) | b[1];
  a[1] = (b[2] << 8) | b[3];
  a[2] = (b[4] << 8) | b[5];
  g[0] = (b[8] << 8) | b[9];
  g[1] = (b[10] << 8) | b[11];
  g[2] = (b[12] << 8) | b[13];
  return true;
}

// Ângulos do acelerômetro em torno dos eixos X e Y do sensor, com o MESMO sinal
// da rotação medida pelo giroscópio nesses eixos (regra da mão direita).
void angulosAcel(float ax, float ay, float az, float out[2]) {
  out[0] = atan2f( ay * sinalGravidade, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
  out[1] = atan2f(-ax * sinalGravidade, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

float eixoDrone(const float v[3], int8_t eixo, int8_t sinal) { return sinal * v[eixo]; }

float accRollBruto() { return cfg.eixoR < 2 ? cfg.sinalR * angS[cfg.eixoR] : 0; }
float accPitchBruto() { return cfg.eixoP < 2 ? cfg.sinalP * angS[cfg.eixoP] : 0; }

void calibrarMPU6050() {
  const int amostras = 400;
  double sg[3] = {0, 0, 0}, sa[3] = {0, 0, 0};
  int validas = 0;
  for (int i = 0; i < amostras; i++) {
    bool pisca = (i / 15) % 2;
    for (int k = 0; k < 4; k++) digitalWrite(PIN_LEDS[k], pisca);
    int16_t a[3], g[3];
    if (lerMPU(a, g)) {
      for (int k = 0; k < 3; k++) { sg[k] += g[k]; sa[k] += a[k] / ACC_LSB; }
      validas++;
    }
    delay(3);
  }
  if (validas < amostras / 2) {
    msg("ERRO: MPU-6050 nao responde (%d/%d leituras).", validas, amostras);
    return;
  }
  for (int k = 0; k < 3; k++) gyroOff[k] = sg[k] / validas;
  float ax = sa[0] / validas, ay = sa[1] / validas, az = sa[2] / validas;
  accF[0] = ax; accF[1] = ay; accF[2] = az;   // filtro já começa assentado
  sinalGravidade = (az >= 0) ? 1.0f : -1.0f;
  if (fabsf(az) < 0.8f) msg("AVISO: gravidade nao esta no eixo Z do MPU (az=%.2f g). Drone nivelado?", az);
  angulosAcel(ax, ay, az, angS);
  accOffR = accRollBruto();
  accOffP = accPitchBruto();
  anguloRoll = 0;
  anguloPitch = 0;
  msg("Calibrado: giro off=(%.0f,%.0f,%.0f) acel=(%.2f,%.2f,%.2f)g", gyroOff[0], gyroOff[1], gyroOff[2], ax, ay, az);
}

// Calcula os coeficientes do notch (RBJ) para a frequência f na taxa fsLoop
void configurarNotch(float f) {
  float w0 = 2.0f * PI * f / fsLoop;
  float alpha = sinf(w0) / (2.0f * notchQ);
  float c = cosf(w0);
  float a0 = 1.0f + alpha;
  for (int k = 0; k < 3; k++) {
    notch[k].b0 = 1.0f / a0;
    notch[k].b1 = -2.0f * c / a0;
    notch[k].b2 = 1.0f / a0;
    notch[k].a1 = -2.0f * c / a0;
    notch[k].a2 = (1.0f - alpha) / a0;
  }
}

// Estima a frequência dominante de vibração (giro SEM notch) e aplica o notch nas 3 taxas.
// Usa o eixo roll+pitch passa-alta (corta o movimento do voo abaixo de ~40 Hz) e conta
// cruzamentos por zero em janelas de 100 ms. Só rastreia quando a vibração é relevante.
void atualizarNotch(float dt) {
  float bruto[3] = {taxaRoll, taxaPitch, taxaYaw};
  float hp[3];
  float aHP = dt / (dt + 1.0f / (2.0f * PI * 40.0f));
  for (int k = 0; k < 3; k++) {
    estLento[k] += (bruto[k] - estLento[k]) * aHP;
    hp[k] = bruto[k] - estLento[k];
  }
  float s = hp[0] + hp[1];   // vibração aparece nos dois eixos; somar reforça o tom comum
  estPico = max(estPico, fabsf(s));
  int8_t lado = (s > 4.0f) ? 1 : (s < -4.0f ? -1 : 0);
  if (lado != 0 && lado != estLado) {
    if (estLado != 0) estCruz++;
    estLado = lado;
  }
  unsigned long agora = millis();
  if (agora - estT0 >= 100) {
    float janela = (agora - estT0) / 1000.0f;
    float f = estCruz / (2.0f * janela);
    float fMax = fsLoop * 0.45f;
    if (estPico > 20.0f && f >= 50.0f && f <= fMax) {
      notchHz = (notchHz == 0) ? f : notchHz + (f - notchHz) * 0.3f;
      configurarNotch(notchHz);
      if (motoresArmados) { notchHzSoma += notchHz; notchHzN++; }
    }
    estCruz = 0;
    estPico = 0;
    estT0 = agora;
  }
  if (notchAtivo && notchHz > 0) {
    taxaRoll  = notch[0].aplicar(taxaRoll);
    taxaPitch = notch[1].aplicar(taxaPitch);
    taxaYaw   = notch[2].aplicar(taxaYaw);
  }
}

void atualizarIMU(float dt) {
  int16_t a[3], g[3];
  if (!lerMPU(a, g)) return;

  for (int k = 0; k < 3; k++) gyroS[k] = (g[k] - gyroOff[k]) / GYRO_LSB;
  float aA = dt / (dt + 1.0f / (2.0f * PI * ACC_LPF_HZ));
  for (int k = 0; k < 3; k++) accF[k] += (a[k] / ACC_LSB - accF[k]) * aA;
  float ax = accF[0], ay = accF[1], az = accF[2];
  angulosAcel(ax, ay, az, angS);

  taxaRoll  = eixoDrone(gyroS, cfg.eixoR, cfg.sinalR);
  taxaPitch = eixoDrone(gyroS, cfg.eixoP, cfg.sinalP);
  taxaYaw   = eixoDrone(gyroS, cfg.eixoY, cfg.sinalY);

  fsLoop += (1.0f / dt - fsLoop) * 0.01f;
  atualizarNotch(dt);

  if (gyroHz > 0) {
    float aG = dt / (dt + 1.0f / (2.0f * PI * gyroHz));
    filtG[0] += (taxaRoll - filtG[0]) * aG;  taxaRoll = filtG[0];
    filtG[1] += (taxaPitch - filtG[1]) * aG; taxaPitch = filtG[1];
    filtG[2] += (taxaYaw - filtG[2]) * aG;   taxaYaw = filtG[2];
  }

  // Só confia no acelerômetro quando a aceleração total está perto de 1 g
  float norma = sqrtf(ax * ax + ay * ay + az * az);
  accValido = (norma > 0.8f && norma < 1.2f);

  bool voando = motoresArmados && thrLog > ESC_IDLE + 40;
  float tau = voando ? tauVoo : TAU_COMPLEMENTAR;
  float alpha = tau / (tau + dt);
  anguloRoll  += taxaRoll * dt;
  anguloPitch += taxaPitch * dt;
  if (accValido) {
    anguloRoll  = alpha * anguloRoll  + (1 - alpha) * (accRollBruto()  - accOffR);
    anguloPitch = alpha * anguloPitch + (1 - alpha) * (accPitchBruto() - accOffP);
  }
}

// ==========================================
// 9. DETECTOR DE OSCILAÇÃO (correção dinâmica)
// ==========================================
const float OSC_HISTERESE = 15.0f;  // graus/s
const float OSC_AMPLITUDE = 40.0f;  // graus/s pico para considerar tremor
const float OSC_HZ_MIN    = 6.0f;
const unsigned long OSC_JANELA_MS = 500;

void alimentarDetector(DetectorOsc &d, float taxa, float dt) {
  d.lento += (taxa - d.lento) * (dt / (0.04f + dt));   // ~4 Hz
  float hp = taxa - d.lento;
  if (fabsf(hp) > d.pico) d.pico = fabsf(hp);
  int8_t lado = (hp > OSC_HISTERESE) ? 1 : (hp < -OSC_HISTERESE ? -1 : 0);
  if (lado != 0 && lado != d.lado) {
    if (d.lado != 0) d.cruzamentos++;
    d.lado = lado;
  }
}

void avaliarOscilacao(bool emVoo, unsigned long agoraMs) {
  if (agoraMs - inicioJanelaOsc < OSC_JANELA_MS) return;
  float janela = (agoraMs - inicioJanelaOsc) / 1000.0f;
  inicioJanelaOsc = agoraMs;

  DetectorOsc *dets[2] = {&oscR, &oscP};
  oscilando = false;
  float hzMax = 0, picoMax = 0;
  for (auto d : dets) {
    d->hz = d->cruzamentos / (2.0f * janela);
    if (d->hz >= OSC_HZ_MIN && d->pico > OSC_AMPLITUDE) {
      oscilando = true;
      hzMax = max(hzMax, d->hz);
    }
    picoMax = max(picoMax, d->pico);
    d->cruzamentos = 0;
    d->pico = 0;
  }

  if (oscilando && emVoo && cfg.autoAjuste && agoraMs - ultimaReducao > 1000) {
    ultimaReducao = agoraMs;
    if (hzMax >= 25.0f) {
      // Vibração rápida: o PID não gera isso, é ruído dos motores amplificado pelo D.
      escalaD = max(0.2f, escalaD * 0.75f);
      escalaGanho = max(0.5f, escalaGanho * 0.95f);
      msg("Vibracao %.0f Hz (pico %.0f graus/s) -> escalaD %.2f escalaP %.2f", hzMax, picoMax, escalaD, escalaGanho);
    } else {
      // Oscilação de controle: P alto demais.
      escalaGanho = max(0.35f, escalaGanho * 0.85f);
      escalaD = max(0.2f, escalaD * 0.9f);
      msg("Oscilacao %.0f Hz (pico %.0f graus/s) -> escalaP %.2f escalaD %.2f", hzMax, picoMax, escalaGanho, escalaD);
    }
  }
}

// ==========================================
// 10. CALIBRAÇÃO GUIADA DE ORIENTAÇÃO
// ==========================================
void iniciarPassoOrient(EstadoOrient passo) {
  orientEstado = passo;
  orientInt[0] = orientInt[1] = orientInt[2] = 0;
  orientT0 = millis();
  if (passo == OR_ROLL)  msg("ORIENT 1/2: incline o lado DIREITO para BAIXO (~30 graus) e volte.");
  if (passo == OR_PITCH) msg("ORIENT 2/2: incline o NARIZ para BAIXO (~30 graus) e volte.");
}

// Espera o drone ficar parado (giro < 8 graus/s por 1 s) antes do próximo passo,
// para que o movimento de volta do passo anterior não seja confundido com o seguinte.
unsigned long orientParadoDesde = 0;

void finalizarOrient() {
  orientEstado = OR_INATIVO;
  if (orientEixo[0] == orientEixo[1] || orientEixo[0] > 1 || orientEixo[1] > 1) {
    msg("ORIENT falhou: roll=%c pitch=%c (precisam ser X e Y distintos). Repita.",
        "XYZ"[orientEixo[0]], "XYZ"[orientEixo[1]]);
    return;
  }
  // Frente x Esquerda deve apontar para CIMA (= sentido medido pela gravidade no eixo Z)
  int8_t cruz = orientSinal[0] * orientSinal[1] * (orientEixo[0] == 0 ? 1 : -1);
  if (cruz != (int8_t)sinalGravidade) {
    msg("ORIENT falhou: movimentos inconsistentes com a gravidade (um deles foi no sentido contrario?). Repita.");
    return;
  }
  cfg.eixoR = orientEixo[0]; cfg.sinalR = orientSinal[0];
  cfg.eixoP = orientEixo[1]; cfg.sinalP = orientSinal[1];
  cfg.eixoY = 2;             cfg.sinalY = -(int8_t)sinalGravidade;  // yaw+ = rotação em torno de "baixo"
  msg("ORIENT concluido. Apoie o drone nivelado e parado: recalibrando em 3 s...");
  delay(3000);
  calibrarMPU6050();
  ultimoLoopUs = micros();
  salvarConfig();
  mostrarGanhos();
}

void processarOrient(float dt) {
  if (orientEstado == OR_INATIVO) return;
  if (orientEstado == OR_ESPERA) {
    bool parado = fabsf(gyroS[0]) < 8 && fabsf(gyroS[1]) < 8 && fabsf(gyroS[2]) < 8;
    if (!parado) orientParadoDesde = millis();
    else if (millis() - orientParadoDesde > 1000) {
      if (orientProximo == OR_INATIVO) finalizarOrient();
      else iniciarPassoOrient(orientProximo);
    }
    return;
  }
  if (millis() - orientT0 > 45000) {
    msg("ORIENT cancelado (tempo esgotado). Orientacao nao alterada.");
    orientEstado = OR_INATIVO;
    return;
  }
  for (int k = 0; k < 3; k++) orientInt[k] += gyroS[k] * dt;
  int melhor = 0;
  for (int k = 1; k < 3; k++) if (fabsf(orientInt[k]) > fabsf(orientInt[melhor])) melhor = k;
  if (fabsf(orientInt[melhor]) < 25.0f) return;

  int idx = (orientEstado == OR_ROLL) ? 0 : 1;
  orientEixo[idx] = melhor;
  orientSinal[idx] = orientInt[melhor] > 0 ? 1 : -1;
  msg("  detectado: eixo %c, sinal %c. Volte ao nivel e segure parado...", "XYZ"[melhor], orientSinal[idx] > 0 ? '+' : '-');

  orientProximo = (orientEstado == OR_ROLL) ? OR_PITCH : OR_INATIVO;
  orientEstado = OR_ESPERA;
  orientParadoDesde = millis();
}

// ==========================================
// 10a. TRIM APRENDIDO
// ==========================================
// Em voo estável, a média do integrador é exatamente a correção fixa que o drone precisa
// (motor mais fraco, CG deslocado). Ao pousar, essa média vira o ponto de partida do
// integrador no próximo voo — o drone já decola compensando, sem cair para o lado fraco.
void aprenderTrim(float dt, bool emVoo) {
  bool estavel = emVoo && abs(comando.roll) < 150 && abs(comando.pitch) < 150 &&
                 fabsf(anguloRoll) < 10 && fabsf(anguloPitch) < 10;
  if (!estavel) return;
  float a = dt / 1.5f;   // média móvel de ~1,5 s
  if (tempoEstavel == 0) { aprendR = iR; aprendP = iP; aprendY = iY; }
  aprendR += (iR - aprendR) * a;
  aprendP += (iP - aprendP) * a;
  aprendY += (iY - aprendY) * a;
  tempoEstavel += dt;
}

void aoDesarmar() {
  if (tempoEstavel >= 2.0f) {
    float lim = cfg.iMax * 0.8f;
    // mistura 60% do aprendido para convergir em poucos voos sem saltos
    trimR = constrain(trimR + 0.6f * (aprendR - trimR), -lim, lim);
    trimP = constrain(trimP + 0.6f * (aprendP - trimP), -lim, lim);
    trimY = constrain(trimY + 0.6f * (aprendY - trimY), -lim, lim);
    salvarTrim();
    msg("Trim aprendido (%.1f s estavel): R %.1f P %.1f Y %.1f", tempoEstavel, trimR, trimP, trimY);
  }
  if (notchHzN > 0) msg("Notch medio no voo: %.0f Hz", notchHzSoma / notchHzN);
  tempoEstavel = 0;
  notchHzSoma = 0;
  notchHzN = 0;
}

// ==========================================
// 10b. TESTE DE VIBRAÇÃO (PID desligado, um motor por vez)
// ==========================================
int VIB_PWM = 640;   // "vib <pwm>" muda (560-820)
const unsigned long VIB_PASSO_MS = 2500;
int vibPasso = -1;              // 0..3 = motor 1..4, 4 = todos, -1 = inativo
bool vibSoIndividual = false;   // "vibm": drone solto -> não liga os 4 juntos (decolaria sem PID)
unsigned long vibT0 = 0;
double vibSoma[3];
float vibPico = 0;
uint32_t vibN = 0;
float vibRms[5];
// Diagnóstico de estimador: com o drone parado e nivelado, os dois deveriam ficar em ~0
double vibAccR = 0, vibAccP = 0;   // soma do ângulo só do acelerômetro
float vibGirR = 0, vibGirP = 0;    // integral só do giro (bruto, sem notch) desde o início do passo

void zerarVib() {
  vibSoma[0] = vibSoma[1] = vibSoma[2] = 0;
  vibPico = 0;
  vibN = 0;
  vibAccR = vibAccP = 0;
  vibGirR = vibGirP = 0;
  vibT0 = millis();
}

void pararVib(const char *motivo) {
  vibPasso = -1;
  motoresDesligados();
  escreverReg(0x1A, mpuDlpf);
  if (motivo) msg("VIB abortado: %s", motivo);
}

void iniciarVib() {
  msg("VIB: PRENDA O DRONE. Cada motor gira 2,5 s (PWM %d), depois os 4 juntos. PID desligado.", VIB_PWM);
  escreverReg(0x1A, 1);   // DLPF 188 Hz: mede a vibração real que chega ao sensor
  vibPasso = 0;
  zerarVib();
}

void processarVib(float dt) {
  if (vibPasso < 0) return;
  vibGirR += eixoDrone(gyroS, cfg.eixoR, cfg.sinalR) * dt;
  vibGirP += eixoDrone(gyroS, cfg.eixoP, cfg.sinalP) * dt;
  if (motoresArmados) { pararVib("drone armado"); return; }
  float limite = vibSoIndividual ? 20 : 35;
  if (fabsf(anguloRoll) > limite || fabsf(anguloPitch) > limite) { pararVib("drone inclinou (nao estava preso?)"); return; }

  unsigned long t = millis() - vibT0;
  if (t > 600) {   // ignora a aceleração inicial do motor
    for (int k = 0; k < 3; k++) {
      vibSoma[k] += gyroS[k] * gyroS[k];
      vibPico = max(vibPico, fabsf(gyroS[k]));
    }
    vibAccR += accRollBruto() - accOffR;
    vibAccP += accPitchBruto() - accOffP;
    vibN++;
  }
  for (int i = 0; i < 4; i++) mOut[i] = (vibPasso == 4 || vibPasso == i) ? VIB_PWM : ESC_MIN;
  escreverMotores(mOut);

  if (t < VIB_PASSO_MS) return;
  float r[3];
  for (int k = 0; k < 3; k++) r[k] = vibN ? sqrtf(vibSoma[k] / vibN) : 0;
  vibRms[vibPasso] = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  if (vibPasso < 4) msg("VIB motor %d: RMS %.0f graus/s (X%.0f Y%.0f Z%.0f) pico %.0f", vibPasso + 1, vibRms[vibPasso], r[0], r[1], r[2], vibPico);
  else              msg("VIB todos:   RMS %.0f graus/s (X%.0f Y%.0f Z%.0f) pico %.0f", vibRms[4], r[0], r[1], r[2], vibPico);
  msg("   angulo so acel: R %+.1f P %+.1f | so giro: R %+.1f P %+.1f | estimado: R %+.1f P %+.1f | acel=(%.2f,%.2f,%.2f)g",
      vibN ? vibAccR / vibN : 0, vibN ? vibAccP / vibN : 0, vibGirR, vibGirP, anguloRoll, anguloPitch,
      accF[0], accF[1], accF[2]);
  vibPasso++;
  zerarVib();
  if (vibPasso < 4 || (vibPasso == 4 && !vibSoIndividual)) return;

  pararVib(nullptr);
  int pior = 0;
  float media = 0;
  for (int i = 0; i < 4; i++) { media += vibRms[i] / 4; if (vibRms[i] > vibRms[pior]) pior = i; }
  msg("VIB concluido. Pior: motor %d (%.0f vs media %.0f graus/s). >2x a media = helice/motor/fixacao desse braco.",
      pior + 1, vibRms[pior], media);
}

// ==========================================
// 11. COMANDOS DE TEXTO (USB ou via controle)
// ==========================================
void processarComando(char *linha) {
  while (*linha == ' ') linha++;
  char nome[16] = {0};
  float v = 0;
  int n = sscanf(linha, "%15s %f", nome, &v);
  if (n < 1) return;
  String c = String(nome);
  c.toLowerCase();
  bool temValor = (n == 2);

  if (c == "ajuda" || c == "help") {
    msg("ganhos | ang rp ri rd yp yi dhz imax <valor> | dlpf 1-6 | ghz <Hz> | notch 0/1 | thov texp thz | trim [0] | auto 0/1 | escala 1 | aplicar | salvar | padrao");
    msg("log 0/1/2 | cal | orient | vib (teste de vibracao por motor) | parar | estado");
    return;
  }
  if (c == "ganhos") { mostrarGanhos(); return; }
  if (c == "estado") {
    msg("armado=%d pairar=%d R=%.1f P=%.1f taxa=(%.0f,%.0f,%.0f) loop=%lu Hz notch=%.0f Hz radio=%s",
        motoresArmados, modoPairarAtivo, anguloRoll, anguloPitch, taxaRoll, taxaPitch, taxaYaw,
        (unsigned long)loopHz, notchHz, ((long)(millis() - ultimoPacote) < 300) ? "ok" : "sem sinal");
    return;
  }
  if (c == "log" && temValor) { modoLog = constrain((int)v, 0, 2); msg("log=%d", modoLog); return; }
  if (c == "salvar") { salvarConfig(); return; }
  if (c == "padrao") {
    Config orient = cfg;
    cfg = CONFIG_PADRAO;
    cfg.eixoR = orient.eixoR; cfg.sinalR = orient.sinalR;   // mantém a orientação medida
    cfg.eixoP = orient.eixoP; cfg.sinalP = orient.sinalP;
    cfg.eixoY = orient.eixoY; cfg.sinalY = orient.sinalY;
    escalaGanho = escalaD = 1;
    msg("Ganhos padrao carregados, orientacao mantida (use 'salvar')."); mostrarGanhos(); return;
  }
  if (c == "escala") { escalaGanho = escalaD = temValor ? constrain(v, 0.2f, 1.5f) : 1.0f; msg("escala=%.2f", escalaGanho); return; }
  if (c == "aplicar") {
    cfg.rP *= escalaGanho; cfg.rD *= escalaD; escalaGanho = escalaD = 1.0f;
    msg("Escalas aplicadas aos ganhos (use 'salvar')."); mostrarGanhos(); return;
  }
  if (c == "notch" && temValor) {
    notchAtivo = v != 0;
    msg("notch %s (atual %.0f Hz)", notchAtivo ? "ligado" : "desligado", notchHz);
    return;
  }
  if ((c == "nr" || c == "np") && temValor && fabsf(v) <= 10) {
    (c == "nr" ? nivelR : nivelP) = v;
    msg("ajuste de nivel: nr=%.1f (+ = direita) np=%.1f (+ = frente) graus (use 'salvar')", nivelR, nivelP);
    return;
  }
  if (c == "reto" && temValor) {
    modoReto = v != 0;
    msg("subida reta no modo LEVE: %s (use 'salvar')", modoReto ? "ligada" : "desligada");
    return;
  }
  if (c == "yaw" && temValor) {
    yawStick = v != 0;
    msg("girar pelo analogico 1: %s (use 'salvar')", yawStick ? "ligado" : "desligado");
    return;
  }
  if (c == "trim") {
    if (temValor && v == 0) { trimR = trimP = trimY = 0; salvarTrim(); msg("Trim zerado."); }
    else msg("trim aprendido: R %.1f P %.1f Y %.1f", trimR, trimP, trimY);
    return;
  }
  if (c == "dlpf" && temValor && v >= 1 && v <= 6) {
    mpuDlpf = (uint8_t)v;
    escreverReg(0x1A, mpuDlpf);
    msg("dlpf=%d", mpuDlpf); return;
  }
  if (c == "ghz" && temValor && v >= 0 && v <= 200) {
    gyroHz = v;
    filtG[0] = taxaRoll; filtG[1] = taxaPitch; filtG[2] = taxaYaw;
    msg("ghz=%.0f", gyroHz); return;
  }
  if (c == "auto" && temValor) { cfg.autoAjuste = v != 0; msg("auto=%d", cfg.autoAjuste); return; }

  if (motoresArmados && (c == "cal" || c == "orient" || c == "vib" || c == "vibm")) { msg("Desarme antes."); return; }
  if (c == "cal") { calibrarMPU6050(); ultimoLoopUs = micros(); return; }
  if (c == "orient") { iniciarPassoOrient(OR_ROLL); return; }
  if (c == "vib" || c == "vibm") {
    VIB_PWM = temValor ? constrain((int)v, 560, 820) : 640;
    vibSoIndividual = (c == "vibm");
    iniciarVib();
    return;
  }
  if (c == "parar") { pararVib("pedido do usuario"); return; }

  float *alvo = nullptr;
  if (c == "ang") alvo = &cfg.angP;
  else if (c == "rp") alvo = &cfg.rP;
  else if (c == "ri") alvo = &cfg.rI;
  else if (c == "rd") alvo = &cfg.rD;
  else if (c == "yp") alvo = &cfg.yP;
  else if (c == "yi") alvo = &cfg.yI;
  else if (c == "dhz") alvo = &cfg.dtermHz;
  else if (c == "imax") alvo = &cfg.iMax;
  else if (c == "thov" && v >= ESC_IDLE + 50 && v <= THR_MAX - 50) alvo = &thrHover;
  else if (c == "texp" && v <= 1.0f) alvo = &thrExpo;
  else if (c == "thz") alvo = &thrHz;
  else if (c == "tpa" && v <= 0.8f) alvo = &tpa;
  else if (c == "tau" && v >= 0.3f && v <= 30) alvo = &tauVoo;
  else if (c == "tmax" && v >= thrHover + 30 && v <= THR_MAX) alvo = &thrMax;
  else if (c == "tpab" && v >= ESC_IDLE && v <= THR_MAX) alvo = &tpaBreak;
  if (alvo && temValor && v >= 0 && v < 1000) {
    *alvo = v;
    msg("%s = %.4f", nome, v);
    return;
  }
  msg("Comando invalido: '%s' (digite ajuda)", linha);
}

void lerSerialUSB() {
  static char buf[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) { buf[n] = 0; processarComando(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

// ==========================================
// 12. ESP-NOW
// ==========================================
void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados, int len) {
  if (len == sizeof(struct_comando)) {
    portENTER_CRITICAL(&muxCmd);
    memcpy((void *)&comandoRx, dados, sizeof(struct_comando));
    ultimoPacote = millis();
    portEXIT_CRITICAL(&muxCmd);
  } else if (len >= 2 && dados[0] == PKT_CMD_TEXTO && !cmdRemotoPendente) {
    int n = min(len - 1, (int)sizeof(cmdRemoto) - 1);
    memcpy(cmdRemoto, dados + 1, n);
    cmdRemoto[n] = 0;
    cmdRemotoPendente = true;
  }
}

void enviarTelemetria(unsigned long agoraMs) {
  pkt_telemetria_t t;
  t.tipo = PKT_TELEMETRIA;
  t.flags = (motoresArmados ? 1 : 0) | (modoPairarAtivo ? 2 : 0) | (oscilando ? 8 : 0);
  t.ms = agoraMs;
  t.roll = anguloRoll * 100;
  t.pitch = anguloPitch * 100;
  t.gR = constrain(taxaRoll * 10, -32000, 32000);
  t.gP = constrain(taxaPitch * 10, -32000, 32000);
  t.gY = constrain(taxaYaw * 10, -32000, 32000);
  t.spR = alvoRollLog * 100;
  t.spP = alvoPitchLog * 100;
  for (int i = 0; i < 4; i++) t.m[i] = mOut[i];
  t.thr = thrLog;
  t.escala = escalaGanho * 100;
  t.oscR = min(255.0f, oscR.hz);
  t.oscP = min(255.0f, oscP.hz);
  t.loopHz = loopHz;
  t.tensao = 4.10f;  // TODO: medir a bateria com o ADC
  esp_now_send(macControle, (uint8_t *)&t, sizeof(t));

  if (modoLog > 0) {
    Serial.printf("T,%lu,%u,%d,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.1f,%.1f,%lu\n",
                  agoraMs, t.flags, thrLog, anguloRoll, anguloPitch, taxaRoll, taxaPitch, taxaYaw,
                  alvoRollLog, alvoPitchLog, mOut[0], mOut[1], mOut[2], mOut[3],
                  escalaGanho, oscR.hz, oscP.hz, (unsigned long)loopHz);
  }
}

// ==========================================
// 13. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  ledcAttach(PIN_M1_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_M2_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_M3_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_M4_PWM, PWM_FREQ, PWM_RES);
  motoresDesligados();

  for (int k = 0; k < 4; k++) pinMode(PIN_LEDS[k], OUTPUT);
  delay(1000);

  carregarConfig();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(aoReceber);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macControle, 6);
  esp_now_add_peer(&peerInfo);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  iniciarMPU6050();
  calibrarMPU6050();

  msg("Drone pronto. MAC %s. Digite 'ajuda'.", WiFi.macAddress().c_str());
  mostrarGanhos();
  ultimoLoopUs = micros();
  inicioJanelaOsc = millis();
}

// ==========================================
// 14. LOOP (~500 Hz)
// ==========================================
void loop() {
  lerSerialUSB();
  if (cmdRemotoPendente) { processarComando(cmdRemoto); cmdRemotoPendente = false; }

  unsigned long agoraUs = micros();
  float dt = (agoraUs - ultimoLoopUs) / 1000000.0f;
  if (dt < 0.001f) return;   // loop de 1 kHz (o MPU entrega 1 kHz com DLPF ligado)
  ultimoLoopUs = agoraUs;
  dt = min(dt, 0.01f);

  contLoops++;
  unsigned long agoraMs = millis();
  if (agoraMs - ultimoContLoops >= 1000) { loopHz = contLoops; contLoops = 0; ultimoContLoops = agoraMs; }

  atualizarIMU(dt);
  processarOrient(dt);

  portENTER_CRITICAL(&muxCmd);
  memcpy(&comando, (const void *)&comandoRx, sizeof(comando));
  unsigned long pacote = ultimoPacote;
  portEXIT_CRITICAL(&muxCmd);
  // Com sinal: o pacote pode ter chegado DEPOIS de agoraMs (callback do Wi-Fi roda em paralelo),
  // então a diferença tem que ser tratada com sinal; sem isso, o failsafe dispara falso.
  bool sinalOk = pacote != 0 && (long)(agoraMs - pacote) < 300;

  // ---------- Segurança / arme ----------
  if (!sinalOk || fabsf(anguloRoll) > 65.0f || fabsf(anguloPitch) > 65.0f || orientEstado != OR_INATIVO || vibPasso >= 0) {
    if (motoresArmados) {
      msg("Desarmado: %s", !sinalOk ? "sem sinal do radio" : (orientEstado != OR_INATIVO ? "orient" : (vibPasso >= 0 ? "teste vib" : "inclinacao > 65 graus")));
      // só aprende de pousos normais; queda/perda de sinal não são voo estável confiável
      if (sinalOk && fabsf(anguloRoll) <= 65.0f && fabsf(anguloPitch) <= 65.0f) aoDesarmar();
      else tempoEstavel = 0;
    }
    motoresArmados = false;
    modoPairarAtivo = false;
    aux1Ant = comando.aux1; aux2Ant = comando.aux2; aux3Ant = comando.aux3;
  } else {
    if (comando.aux3 == 1 && aux3Ant == 0 && !motoresArmados) {
      calibrarMPU6050();
      ultimoLoopUs = micros();
    }
    aux3Ant = comando.aux3;

    if (comando.aux1 == 1 && aux1Ant == 0) {
      if (!motoresArmados) {
        if (comando.throttle < -750 && fabsf(anguloRoll) < 15.0f && fabsf(anguloPitch) < 15.0f) {
          motoresArmados = true;
          modoPairarAtivo = false;
          iR = trimR; iP = trimP; iY = trimY;   // parte da compensação aprendida
          tempoEstavel = 0;
          // As escalas aprendidas NÃO voltam a 1 ao armar: a correção acumula entre tentativas.
          msg("ARMADO (escalaP %.2f escalaD %.2f, trim R %.0f P %.0f)", escalaGanho, escalaD, trimR, trimP);
        }
      } else {
        motoresArmados = false;
        modoPairarAtivo = false;
        msg("Desarmado pelo controle");
        aoDesarmar();
      }
    }
    aux1Ant = comando.aux1;

    if (comando.aux2 == 1 && aux2Ant == 0 && motoresArmados) {
      modoPairarAtivo = !modoPairarAtivo;
      if (modoPairarAtivo) {
        throttleTravado = constrain((int)thrFilt, ESC_IDLE + 30, 920);
      }
    }
    aux2Ant = comando.aux2;
  }

  // ---------- LEDs ----------
  bool estadoLed;
  if (orientEstado != OR_INATIVO) estadoLed = (agoraMs / 50) % 2;
  else if (!motoresArmados) estadoLed = (agoraMs / 400) % 2;
  else if (modoPairarAtivo) estadoLed = (agoraMs / 100) % 2;
  else estadoLed = HIGH;
  for (int k = 0; k < 4; k++) digitalWrite(PIN_LEDS[k], estadoLed);

  // ---------- Controle ----------
  bool emVoo = false;
  if (motoresArmados) {
    float anguloMax = 12.0f, taxaYawMax = 80.0f;
    if (comando.modo == 2) { anguloMax = 25.0f; taxaYawMax = 130.0f; }
    if (comando.modo == 3) { anguloMax = 40.0f; taxaYawMax = 180.0f; }

    int throttleBase = ESC_IDLE;
    float alvoRoll, alvoPitch, alvoYaw;

    if (modoPairarAtivo && abs(comando.throttle) > 250) modoPairarAtivo = false;
    if (modoPairarAtivo) {
      throttleBase = throttleTravado;
      alvoRoll  = (comando.roll / 1000.0f) * 10.0f;
      alvoPitch = (comando.pitch / 1000.0f) * 10.0f;
      alvoYaw   = (comando.yaw / 1000.0f) * 60.0f;
    } else {
      alvoRoll  = (comando.roll / 1000.0f) * anguloMax;
      alvoPitch = (comando.pitch / 1000.0f) * anguloMax;
      alvoYaw   = (comando.yaw / 1000.0f) * taxaYawMax;
      float alvoThr = curvaThrottle(comando.throttle);
      if (thrHz > 0) thrFilt += (alvoThr - thrFilt) * (dt / (dt + 1.0f / (2.0f * PI * thrHz)));
      else thrFilt = alvoThr;
      throttleBase = (int)thrFilt;
    }
    if (!yawStick) alvoYaw = 0;   // analógico 1 só controla a velocidade
    if (modoReto && comando.modo == 1) { alvoRoll = 0; alvoPitch = 0; }   // LEVE: só sobe/desce, nivelado
    alvoRoll += nivelR;
    alvoPitch += nivelP;
    alvoRollLog = alvoRoll;
    alvoPitchLog = alvoPitch;
    thrLog = throttleBase;
    emVoo = (throttleBase > ESC_IDLE + 40);

    // Laço externo: ângulo -> taxa desejada
    float spR = constrain(cfg.angP * (alvoRoll - anguloRoll), -250.0f, 250.0f);
    float spP = constrain(cfg.angP * (alvoPitch - anguloPitch), -250.0f, 250.0f);

    // Laço interno: PID de taxa
    tpaFator = 1.0f - tpa * constrain((throttleBase - tpaBreak) / (float)(THR_MAX - tpaBreak), 0.0f, 1.0f);
    float kp = cfg.rP * escalaGanho * tpaFator, kd = cfg.rD * escalaD * tpaFator;
    float eR = spR - taxaRoll, eP = spP - taxaPitch, eY = alvoYaw - taxaYaw;

    if (emVoo) {
      iR = constrain(iR + cfg.rI * eR * dt, -cfg.iMax, cfg.iMax);
      iP = constrain(iP + cfg.rI * eP * dt, -cfg.iMax, cfg.iMax);
      iY = constrain(iY + cfg.yI * eY * dt, -cfg.iMax, cfg.iMax);
    } else {
      iR = trimR; iP = trimP; iY = trimY;   // no chão: integrador parado na compensação aprendida
    }

    // Termo D sobre a medida (sem "chute" do setpoint), filtrado
    float aD = dt / (dt + 1.0f / (2.0f * PI * cfg.dtermHz));
    dFiltR += (((taxaRoll - taxaAntR) / dt) - dFiltR) * aD;
    dFiltP += (((taxaPitch - taxaAntP) / dt) - dFiltP) * aD;

    float uR = constrain(kp * eR + iR - kd * dFiltR, -200.0f, 200.0f);
    float uP = constrain(kp * eP + iP - kd * dFiltP, -200.0f, 200.0f);
    float uY = constrain(cfg.yP * eY + iY, -120.0f, 120.0f);

    // Mixer quad X: M1 DE(CW)  M2 DD(CCW)  M3 TD(CW)  M4 TE(CCW)
    float mix[4] = {
      +uR - uP - uY,
      -uR - uP + uY,
      -uR + uP - uY,
      +uR + uP + uY,
    };

    // Preserva a correção diferencial quando bate nos limites (evita assimetria)
    float mx = -1e9, mn = 1e9;
    for (int i = 0; i < 4; i++) { mx = max(mx, mix[i]); mn = min(mn, mix[i]); }
    float base = throttleBase;
    if (emVoo) {
      if (base + mx > ESC_MAX) base = ESC_MAX - mx;
      if (base + mn < ESC_IDLE) base = min((float)ESC_IDLE - mn, (float)THR_MAX);
    }
    for (int i = 0; i < 4; i++) mOut[i] = constrain((int)(base + mix[i]), ESC_IDLE, ESC_MAX);
    escreverMotores(mOut);
    aprenderTrim(dt, emVoo);
  } else {
    iR = trimR; iP = trimP; iY = trimY;
    dFiltR = dFiltP = 0;
    thrFilt = ESC_IDLE;
    thrLog = 0;
    alvoRollLog = alvoPitchLog = 0;
    if (vibPasso >= 0) processarVib(dt);
    else motoresDesligados();
  }
  taxaAntR = taxaRoll;
  taxaAntP = taxaPitch;

  alimentarDetector(oscR, taxaRoll, dt);
  alimentarDetector(oscP, taxaPitch, dt);
  avaliarOscilacao(emVoo, agoraMs);

  unsigned long periodo = (modoLog == 2) ? 20 : 100;
  if (agoraMs - ultimaTelemetria >= periodo) {
    ultimaTelemetria = agoraMs;
    enviarTelemetria(agoraMs);
  }
}
