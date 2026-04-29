#include "Arduino.h"
#include "WiFi.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"

// ################### Motor TB6600 ###################
const int dirPin = 5;    // Pino direção conectado ao DIR+ do TB6600
const int stepPin = 4;   // Pino passo conectado ao PUL+ do TB6600
const int sw1Pin = 34;
const int sw2Pin = 35;

float Pot = 0, Pot_f = 0, k1 = 100, vel = 0;

typedef enum {
  m_fdc,
  m_brd,
  m_mov
} MotorMode_t;

MotorMode_t motorMode = m_fdc;
MotorMode_t motorModePrev = m_fdc;

const bool ENABLE_MODO_BORDA = true;
const unsigned int MOTOR_VEL_BAIXA_US = 1200;
const float BORDA_VARIACAO_PCT = 0.10;
const int MOV_SEGMENTOS = 30;
// const int MOV_STEPS_10CM = 1600; // 30 mm por trecho
const int MOV_STEPS_10CM = 1355; // 25,4 mm por trecho
const unsigned long MOV_MEDICAO_MS = 5000; // Tempo que o sensor fica parado medindo e no final faz a media
float MED_DIF = 0;


bool systemActive = false;
float bordaBase = 0;
bool bordaBaseReady = false;

extern float mediaSensor2;
extern float somaMedidas;
extern int contadorMedidas;
extern unsigned long inicioMedia_s2;

float x_ida[MOV_SEGMENTOS] = {0};
float x_volta[MOV_SEGMENTOS] = {0};

// ################### Heltec LoRa Config and Display ###################
#define RF_FREQUENCY 450000000  // Hz
#define TX_OUTPUT_POWER 10      // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define hallPin 07
#define espessuraPin 47

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 256

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

unsigned long timeMovelPackage = 5000;

static RadioEvents_t RadioEvents;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX
} States_t;

int16_t txNumber;
int16_t rxNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi, rxSize;

// ******************************** Display Setup ********************************
// #define SDA_OLED 4
#define SCL_OLED 15
#define RST_OLED 16
// #define LED 2

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void drawMultilineText(SSD1306Wire &display, int x, int y, const String &text) {
  int lineHeight = 10;
  int currentY = y;
  int maxWidth = 128;

  String remainingText = text;
  while (remainingText.length() > 0) {
    int splitIndex = remainingText.length();
    while (splitIndex > 0 && display.getStringWidth(remainingText.substring(0, splitIndex)) > maxWidth) {
      splitIndex--;
    }
    display.drawString(x, currentY, remainingText.substring(0, splitIndex));
    currentY += lineHeight;
    remainingText = remainingText.substring(splitIndex);
    remainingText.trim();
  }
}

// ******************************** Interrupções e Deep Sleep ********************************
volatile bool interrupt_flag = false;
volatile bool deepsleepflag = false;
volatile bool resendflag = false;

void IRAM_ATTR interrupt_GPIO0() {
  interrupt_flag = true;
}

void interrupt_handle(void) {
  if (interrupt_flag) {
    interrupt_flag = false;
    if (digitalRead(0) == 0) {
      if (rxNumber <= 2) {
        resendflag = true;
      } else {
        deepsleepflag = true;
      }
    }
  }
}

#define Vext 36

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

// ******************************** Motor ********************************
void motor_step_once(int dir, unsigned int stepDelayUs) {
  digitalWrite(dirPin, dir);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelayUs);
  vel = stepDelayUs;
}

bool podeMoverDir(int dir) {
  if (dir == 1) {
    return digitalRead(sw1Pin) == 0;
  }
  return digitalRead(sw2Pin) == 0;
}

void moverTrechoSteps(int dir, int steps) {
  for (int i = 0; i < steps; i++) {
    if (!podeMoverDir(dir)) {
      break;
    }
    motor_step_once(dir, MOTOR_VEL_BAIXA_US);
  }
}

float medirLaserPorTempo(unsigned long duracaoMs) {
  unsigned long t0 = millis();
  float soma = 0;
  int amostras = 0;

  while ((millis() - t0) < duracaoMs) {
    sensor1_update();
    sensor2_update();
    if (mediaSensor2 > 0) {
      soma += mediaSensor2;
      amostras++;
    }
    delay(5);
  }

  if (amostras == 0) {
    return mediaSensor2;
  }

  return soma / amostras;
}

void enviarPacoteMovimento(const char *tag, float *dados) {
  snprintf(txpacket, BUFFER_SIZE,
           "MOV %s %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
          // "MOV %s %.1f %.1f %.1f",

           tag,
          //  dados[0], dados[1], dados[2]);
           dados[0], dados[1], dados[2], dados[3], dados[4],
           dados[5], dados[6], dados[7], dados[8], dados[9],
           dados[10], dados[11], dados[12], dados[13], dados[14],
           dados[15], dados[16], dados[17], dados[18], dados[19],
           dados[20], dados[21], dados[22], dados[23], dados[24],
           dados[25], dados[26], dados[27], dados[28], dados[29]);

  // Serial.println(txpacket);
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  state = LOWPOWER;
}

void executarVarreduraMovimento(int dir, float *destino) {
  for (int i = 0; i < MOV_SEGMENTOS; i++) {
    moverTrechoSteps(dir, MOV_STEPS_10CM);
    destino[i] = medirLaserPorTempo(MOV_MEDICAO_MS);
  }
}

void motor_update() {
  Pot = analogRead(2);
  Pot_f = (Pot_f * k1 + Pot) / (k1 + 1);

  // Serial.print(digitalRead(sw1Pin)); Serial.println(digitalRead(sw2Pin));
  // Serial.print(Pot);
  // Serial.println(Pot_f);

  // Modo fim de curso = m_fdc
  // Modo borda = m_brd
  // Modo movimento = m_mov

  if (motorMode != motorModePrev) {
    if (motorMode == m_fdc) {
      systemActive = false;
      bordaBaseReady = false;
      bordaBase = 0;
    } else {
      systemActive = true;

      // === LIMPEZA DO SENSOR AO ENTRAR NO MODO BORDA ===
      if (motorMode == m_brd) {
        bordaBaseReady = false;
        bordaBase = 0;
        mediaSensor2 = 0;
        somaMedidas = 0;
        contadorMedidas = 0;
        inicioMedia_s2 = millis();
      }
      // ==================================================
    }
    motorModePrev = motorMode;
  }

  if (motorMode == m_fdc) {
    // Modo fim de curso: tudo desligado, procura fim de curso da direita (sw1Pin)
    systemActive = false;
    if (podeMoverDir(1)) {
      motor_step_once(1, MOTOR_VEL_BAIXA_US);
    } else {
      motorMode = ENABLE_MODO_BORDA ? m_brd : m_mov;
    }
  }

  // ===== MODO BORDA (pode comentar este bloco se nao quiser usar) =====
  else if (motorMode == m_brd) {
    systemActive = true;
    MED_DIF = 1000;

    if (!bordaBaseReady && mediaSensor2 > 0) {
      bordaBase = mediaSensor2;
      bordaBaseReady = true;
    }

    bool bordaDetectada = false;
    if (bordaBaseReady && mediaSensor2 > 0) {
      if (mediaSensor2 <= (bordaBase * (1.0 - BORDA_VARIACAO_PCT))) {
        bordaDetectada = true;
      }
    }

    if (!bordaDetectada) {
      if (podeMoverDir(0)) {
        motor_step_once(0, MOTOR_VEL_BAIXA_US);
      }
    } else {
      motorMode = m_mov;
    }
  }

  else if (motorMode == m_mov) {
    // Modo movimento: 10 trechos de 30 mm (ida), envia; repete inverso (volta), envia
    systemActive = true;

    MED_DIF = 20000 - 50;

    executarVarreduraMovimento(0, x_ida);
    enviarPacoteMovimento("Espessura", x_ida);

    // ===== VOLTA COM MEDICAO (DESABILITADA) =====
    // executarVarreduraMovimento(1, x_volta);
    // enviarPacoteMovimento("VOLTA", x_volta);
    // ===========================================

    motorMode = m_fdc;
  }

  // Serial.print(digitalRead(sw1Pin));
  // Serial.print(" ");
  // Serial.print(digitalRead(sw2Pin));
  // Serial.print(" ");
  // Serial.println(vel);
  


  // if((digitalRead(sw1Pin)==1)||(digitalRead(sw2Pin)==1))
  // { digitalWrite(stepPin, 0); }
}

// ******************************** Sensor 1 – Medição via analogRead ********************************
int cont = 0, inicio = 0, atraso = 500;
float sum = 0, med = 0;
float ta = 0, tb = 0, tc = 0, td = 0, t1 = 0, t2 = 0, t3 = 0;
float Distan_pc = 0, Deform_pc = 0;
float hall = 0;
int ima = 0;
float Deform_pc_final = 0;
float med_t12 = 0;
float med_t3 = 0;

void sensor1_update() {
  
  cont++;
  hall = analogRead(hallPin);
  sum += analogRead(hallPin);

  if (cont > 4) {
    med = sum / cont;
    cont = 0;
    sum = 0;
  }

  if (millis() < 2000) {
    ima = 0;
    inicio = 0;
  }

  if ((med < 1850) && (inicio == 0) && (ima == 0)) {
    ta = millis();
    inicio = 1;
    ima = 1;
  }

  if ((med > 2050) && (inicio == 1) && (ima == 1) && ((millis() - ta) > atraso)) {
    tb = millis();
    t1 = tb - ta;
    ima = 2;
  }

  if ((med > 2050) && (inicio == 1) && (ima == 2) && ((millis() - tb) > atraso)) {
    tc = millis();
    t2 = tc - tb;
    ima = 3;
  }

  if ((med > 2050) && (inicio == 1) && (ima == 3) && ((millis() - tc) > atraso)) {
    td = millis();
    t3 = td - tc;
    ima = 0;
    inicio = 0;

    med_t12 = (t1 + t2) / 2;

    if (med_t12 > 0) {
      med_t3 = t3 / med_t12;
      Deform_pc = (med_t3 - 1) * 1000;
    } else {
      Deform_pc = 0;
    }
  }

  // Serial.print(t1);  Serial.print(" ");
  // Serial.print(t2);  Serial.print(" ");
  // Serial.print(t3);  Serial.print(" ");
  // Serial.print(med); Serial.print(" ");
  // Serial.print(hall); Serial.print(" ");
  // Serial.print(ima); Serial.print(" ");
  // Serial.println(Deform_pc);
}

// ******************************** Sensor 2 – Medição via pulseIn ********************************
float leitura = 0;
float distanciaMM = 0;
unsigned long tempoAtual_s2 = 0;
unsigned long tempoAnterior_s2 = 0;
unsigned long inicioMedia_s2 = 0;
int contadorMedidas = 0;
float somaMedidas = 0;
float mediaSensor2 = 0;

float interpolar(float x, float x1, float y1, float x2, float y2) {
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

float converterParaMM(float valor) {
  if (valor >= 4854) return 25;
  if (valor >= 3625) return interpolar(valor, 3625, 30, 4854, 25);
  if (valor >= 3045) return interpolar(valor, 3045, 34, 3625, 30);
  if (valor >= 2505) return interpolar(valor, 2505, 39, 3045, 34);
  if (valor >= 2220) return interpolar(valor, 2220, 43, 2505, 39);
  if (valor >= 1913) return interpolar(valor, 1913, 49, 2220, 43);
  if (valor >= 1738) return interpolar(valor, 1738, 53, 1913, 49);
  if (valor >= 1520) return interpolar(valor, 1520, 59, 1738, 53);
  if (valor >= 1379) return interpolar(valor, 1379, 64, 1520, 59);
  if (valor >= 1243) return interpolar(valor, 1243, 70, 1379, 64);
  if (valor >= 1172) return interpolar(valor, 1172, 74, 1243, 70);
  if (valor >= 1075) return interpolar(valor, 1075, 79, 1172, 74);
  if (valor >= 997)  return interpolar(valor, 997, 85, 1075, 79);
  if (valor >= 918)  return interpolar(valor, 918, 91, 997, 85);
  return 91;
}

void sensor2_update() {
  tempoAtual_s2 = millis();

  if (tempoAtual_s2 - tempoAnterior_s2 >= 100) {
    tempoAnterior_s2 = tempoAtual_s2;

    // leitura = pulseIn(espessuraPin, HIGH, 20000);
    leitura = pulseIn(espessuraPin, HIGH);
    distanciaMM = converterParaMM(leitura);
    somaMedidas += distanciaMM;
    contadorMedidas++;
  }

  if (tempoAtual_s2 - inicioMedia_s2 >= MED_DIF) {
    inicioMedia_s2 = tempoAtual_s2;
    mediaSensor2 = somaMedidas / contadorMedidas;

    somaMedidas = 0;
    contadorMedidas = 0;
  }
}

// ******************************** Função de Inicialização LoRa ********************************
void lora_init(void) {
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  txNumber = 0;
  Rssi = 0;
  rxNumber = 0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  state = STATE_TX;
}

// ******************************** Handlers de Eventos LoRa ********************************
void OnTxDone(void) {
  state = STATE_RX;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout...");
  state = STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  rxNumber++;
  Rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  state = STATE_TX;
}

// ******************************** Setup ********************************
void setup() {
  Serial.begin(250000);

  // Motor
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sw1Pin, INPUT);
  pinMode(sw2Pin, INPUT);
  digitalWrite(dirPin, 0);

  // Heltec / LoRa / Display
  VextON();
  delay(100);

  factory_display.init();
  factory_display.clear();
  factory_display.display();

  attachInterrupt(0, interrupt_GPIO0, FALLING);

  lora_init();

  String packet = "Esperando por leitura!";
  factory_display.drawString(0, 10, packet);
  factory_display.display();
  delay(100);
  factory_display.clear();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Sensores
  pinMode(espessuraPin, INPUT);
  inicioMedia_s2 = millis();
  tempoAnterior_s2 = millis();
}

// ******************************** Loop Principal ********************************
void loop() {
  interrupt_handle();

  if (deepsleepflag) {
    VextOFF();
    Radio.Sleep();
    SPI.end();
    pinMode(RADIO_DIO_1, ANALOG);
    pinMode(RADIO_NSS, ANALOG);
    pinMode(RADIO_RESET, ANALOG);
    pinMode(LORA_CLK, ANALOG);
    pinMode(LORA_MISO, ANALOG);
    pinMode(LORA_MOSI, ANALOG);
    esp_sleep_enable_timer_wakeup(600 * 1000 * (uint64_t)1000);
    esp_deep_sleep_start();
  }

  // Atualiza motor e sensores
  motor_update();
  
  // Atualiza sempre
  sensor1_update();


  if (systemActive && (motorMode != m_mov)) {
    sensor2_update();
  }

  // Envia pacote LoRa a cada 5 segundos
  static unsigned long lastSendTime = 0;
  // if (systemActive && (millis() - lastSendTime) > timeMovelPackage) {
  if ((millis() - lastSendTime) > timeMovelPackage) {
    lastSendTime = millis();
    txNumber++;

    // sprintf(txpacket, "F 1 %.1f %.1f", mediaSensor2, Deform_pc); // Envia os dois sensores
    sprintf(txpacket, "Elongamento %.1f", Deform_pc); // Envia apenas o sensor de elongamento
    // Serial.println(txpacket);

    unsigned long T_Lora1 = millis();
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    unsigned long T_Lora2 = millis();
    float T_envio = T_Lora2 - T_Lora1;

    factory_display.clear();
    factory_display.drawString(0, 0, "MEXTEC - Enviando...");
    drawMultilineText(factory_display, 0, 20, String(txpacket));
    factory_display.drawString(0, 50, "T envio: ");
    factory_display.drawString(40, 50, String(T_envio, 0));
    factory_display.display();

    state = LOWPOWER;
  }

  switch (state) {
    case STATE_TX:
      state = LOWPOWER;
      break;

    case LOWPOWER:
      Radio.IrqProcess();
      break;

    default:
      break;
  }
}