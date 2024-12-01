/*
 ============================================================================
 Projeto: Sistema de Monitoramento Andador
 Autoras: Gabriela Kazumi Ono Leal & Leticia Miyuki Kimoto
 Última Atualização: 01/12/2024

 ============================================================================
 Descrição:
 Sistema de monitoramento de sinais vitais e movimento no andador
 - Botão: Ativa e desativa a leitura dos dados
 - MAX30102: Monitora frequência cardíaca e oxigenação do sangue (SpO2)
 - Sensores de Força (FSR402): Calculam o baricentro
 - MPU6050: Calcula velocidade e conta passos
 - Display OLED: Exibe alertas em tempo real
 - Buzzer: Indica eventos críticos por som

 Créditos:
 https://www.instructables.com/Guide-to-Using-MAX30102-Heart-Rate-and-Oxygen-Sens/ - base para configuração do MAX30102
 
 */

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>

// Configuração Display Oled
#define LARGURA_TELA 128
#define ALTURA_TELA 64
#define RESET_OLED -1
Adafruit_SSD1306 display(LARGURA_TELA, ALTURA_TELA, &Wire, RESET_OLED);

// Configuração Botão
#define pinoBotao 2  
volatile bool estadoLeitura = true; 

// Configuração FSR402
const int sensorForcaEsq = A0;
const int sensorForcaDir = A1;

// Configuração Buzzer
const int pinoBuzzer = 6;

// Configuração MAX30102
MAX30105 sensorParticulas;
const int tamBuffer = 20;
uint32_t bufferInfravermelho[tamBuffer];  // Buffer de dados infravermelhos
uint32_t bufferVermelho[tamBuffer];      // Buffer de dados vermelhos
int32_t oxigenacao;                // Valor de SpO2 calculado
int8_t oxigenacaoValida;           // Indicador de validade do SpO2
int32_t frequenciaCardiaca;        // Valor da frequência cardíaca calculada
int8_t frequenciaValida;           // Indicador de validade da frequência cardíaca

// Configuração MPU6050
MPU6050 mpu;
float acelX, acelY;
float acelX_filtrada = 0, acelY_filtrada = 0;
float velX = 0, velY = 0;
float vel_total = 0;
float offsetX = 0, offsetY = 0;
const float alfa = 0.2;
unsigned long ultimoTempo = 0;
int contadorPassos = 0;
float limitePasso = 0.1;
float magnitudeAnterior = 0;
#define TAMANHO_BUFFER 10
float bufferAceleracao[TAMANHO_BUFFER];
int indiceBuffer = 0;

void calibrarSensorMPU() {
  int leituras = 100;
  long somaX = 0, somaY = 0;

  for (int i = 0; i < leituras; i++) {
    int16_t acelX_bruto, acelY_bruto;
    mpu.getAcceleration(&acelX_bruto, &acelY_bruto, nullptr);
    somaX += acelX_bruto;
    somaY += acelY_bruto;
    delay(10);
  }

  offsetX = somaX / (float)leituras / 16384;
  offsetY = somaY / (float)leituras / 16384;
}

void alternarEstado() {
  estadoLeitura = !estadoLeitura;  
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configuração Botão
  pinMode(pinoBotao, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(pinoBotao), alternarEstado, FALLING);

  // Configuração Display Oled
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Falha ao iniciar o display OLED");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.print("INIT");
  display.display();

  // Configuração Buzzer
  pinMode(pinoBuzzer, OUTPUT);
  digitalWrite(pinoBuzzer, HIGH);

  // Configuração MAX30102
  if (!sensorParticulas.begin(Wire, I2C_SPEED_FAST, 0x57)) {
    Serial.println("Falha ao iniciar o MAX30102");
    while (true);
  }
  sensorParticulas.setup(60, 4, 2, 100, 411, 4096);

  // Configuração MPU6050
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  if (!mpu.testConnection()) {
    Serial.println("Falha ao iniciar o MPU6050");
    while (1);
  }

  calibrarSensorMPU();
  ultimoTempo = micros();
}

void loop() {
  if (estadoLeitura) {
    unsigned long tempoAtual = micros();
    float dt = (tempoAtual - ultimoTempo) / 1000000.0;
    ultimoTempo = tempoAtual;

    // Leitura FSR402
    int forcaEsquerda = analogRead(sensorForcaEsq);
    int forcaDireita = analogRead(sensorForcaDir);
    int baricentro = abs(25 * (forcaEsquerda - forcaDireita) / (forcaEsquerda + forcaDireita));

    // Leitura MAX30102
    if (sensorParticulas.available()) {
      int vermelho = sensorParticulas.getRed();
      int infravermelho = sensorParticulas.getIR();
      sensorParticulas.nextSample();

      static int indiceBuffer = 0;
      bufferVermelho[indiceBuffer] = vermelho;
      bufferInfravermelho[indiceBuffer] = infravermelho;
      indiceBuffer++;

      if (indiceBuffer == tamBuffer) {
        maxim_heart_rate_and_oxygen_saturation(bufferInfravermelho, tamBuffer, bufferVermelho, &oxigenacao, &oxigenacaoValida, &frequenciaCardiaca, &frequenciaValida);
        indiceBuffer = 0;
      }
    }

    // Leitura MPU6050
    int16_t acelX_bruto, acelY_bruto;
    mpu.getAcceleration(&acelX_bruto, &acelY_bruto, nullptr);
    acelX = acelX_bruto / 16384 - offsetX;
    acelY = acelY_bruto / 16384 - offsetY;

    acelX_filtrada = alfa * acelX + (1 - alfa) * acelX_filtrada;
    acelY_filtrada = alfa * acelY + (1 - alfa) * acelY_filtrada;

    float acelX_SI = acelX_filtrada * 9.81;
    float acelY_SI = acelY_filtrada * 9.81;

    // Calculo Velocidade
    if (abs(acelX_SI) > 0.02) velX += acelX_SI * dt;
    if (abs(acelY_SI) > 0.02) velY += acelY_SI * dt;

    vel_total = sqrt(velX * velX + velY * velY);

    if (sqrt(acelX * acelX + acelY * acelY) < 0.05) {
      velX = 0;
      velY = 0;
    }

    // Calculo de Passos
    float mediaAceleracao = calcularMedia();
    float magnitude = sqrt(acelX_SI * acelX_SI + acelY_SI * acelY_SI) - mediaAceleracao;

    if (magnitude > limitePasso && magnitudeAnterior <= limitePasso) {
      contadorPassos++;
    }

    magnitudeAnterior = magnitude;
    bufferAceleracao[indiceBuffer] = magnitude;
    indiceBuffer = (indiceBuffer + 1) % TAMANHO_BUFFER;

    // Sinais de alerta e impressão de dados
    controleAlerta(baricentro, oxigenacao, oxigenacaoValida, frequenciaCardiaca, frequenciaValida, vel_total);

    imprimirDados(baricentro, frequenciaCardiaca, frequenciaValida, oxigenacao, oxigenacaoValida, vel_total, contadorPassos);

    delay(50);

  } else {
    delay(100);  
  }
}

float calcularMedia() {
  float soma = 0.0;
  for (int i = 0; i < TAMANHO_BUFFER; i++) {
    soma += bufferAceleracao[i];
  }
  return soma / TAMANHO_BUFFER;
}

void controleAlerta(int baricentro, int oxigenacao, int oxigenacaoValida, int frequenciaCardiaca, 
                    int frequenciaValida, int vel_total) {
  display.clearDisplay();
  if (baricentro > 16.67) {
    digitalWrite(pinoBuzzer, LOW);
    display.setCursor(0, 20);
    display.print("BARI");
  } else if (oxigenacaoValida && oxigenacao < 90) {
    digitalWrite(pinoBuzzer, LOW);
    display.setCursor(0, 20);
    display.print("SPO2");
  } else if (frequenciaValida && (frequenciaCardiaca < 75 || frequenciaCardiaca > 136)) {
    digitalWrite(pinoBuzzer, LOW);
    display.setCursor(0, 20);
    display.print("BC");
  } else if (vel_total > 1.5) {
    digitalWrite(pinoBuzzer, LOW);
    display.setCursor(0, 20);
    display.print("VEL");
  } else {
    digitalWrite(pinoBuzzer, HIGH);
  }
  display.display();
}

void imprimirDados(int baricentro, int frequenciaCardiaca, int frequenciaValida, int oxigenacao, 
                   int oxigenacaoValida, float vel_total, int contadorPassos) {
  
  Serial.print("Baricentro (cm): "); // FSR402
  Serial.print(baricentro);
  Serial.print(" | Frequência Cardíaca: ");

  if (frequenciaValida) { // MAX30102
    Serial.print(frequenciaCardiaca);
    Serial.print(" bpm");
  } else {
    Serial.print("Erro");
  }
  Serial.print(" | Oxigenação: ");
  if (oxigenacaoValida) {
    Serial.print(oxigenacao);
    Serial.print(" %");
  } else {
    Serial.print("Erro");
  }

  Serial.print(" | Velocidade: "); //MPU6050
  Serial.print(vel_total);
  Serial.print(" m/s");
  Serial.print(" | Total de Passos: ");
  Serial.println(contadorPassos);

}
