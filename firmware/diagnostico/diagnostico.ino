// Diagnóstico de bancada — Arduino Nano + PCA9685
//
// Sketch mínimo, para quando o firmware de produção responde OK e o servo não
// se mexe. Aqui não há suavização, lote, limite de junta, sensor nem máquina
// de estados: só o caminho mais curto entre o comando e o pulso.
//
// O firmware de produção já avisa quando o PCA9685 não responde no 0x40
// (`ERR SEM_PCA9685` no boot). O que este sketch acrescenta é o caminho curto:
// escrever contagem de pulso crua, sem suavização, sem lote, sem limite de
// junta e sem máquina de estados no meio. Quando o de produção responde OK e
// mesmo assim nada se move, é aqui que se descobre se o problema está no
// pulso, na alimentação ou no servo.
//
//   115200 baud  (o mesmo do firmware de produção)
//
// COMANDOS
//   ?              estado: canal ativo, último pulso, leitura do A0
//   i              varre o barramento I2C de novo
//   c <0-15>       escolhe o canal
//   a <0-180>      move por ângulo (converte por 102-492, correto a 50 Hz)
//   p <0-4095>     escreve a contagem de pulso CRUA, sem conversão
//   u <500-2500>   escreve por microssegundos (converte pela frequência real)
//   s              varredura lenta de 60 a 120 graus, uma ida e volta
//   o              desliga o PWM do canal (servo solta)
//   z              lê o A0 vinte vezes e mostra média, mínimo e máximo
//
// O comando `p` é o teste mais direto que existe: ele não passa por conta
// nenhuma. Se `p 102` e `p 492` não movem o servo, o problema está abaixo do
// software (I2C, alimentação, fiação ou o próprio servo).

#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const uint8_t ENDERECO_PCA = 0x40;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(ENDERECO_PCA);

// 50 Hz: cada contagem vale 4,883 us. 102 = 500 us, 492 = 2400 us, que é a
// faixa que SG90 e MG90S aceitam. NÃO use 150-600 aqui: aquele par é do
// exemplo da Adafruit a 60 Hz e, a 50 Hz, manda 732-2930 us, empurrando o
// servomotor contra o batente na parte de cima do curso.
const int PULSO_MIN = 102;    // ~0 grau   (500 us)
const int PULSO_MAX = 492;    // ~180 graus (2400 us)
const uint8_t PINO_ACS = A0;

uint8_t canal = 0;
int ultimo_pulso = -1;
bool pca_presente = false;

char linha[32];
uint8_t n_linha = 0;


// Varre o barramento e relata tudo que responde. Um PCA9685 sem jumper de
// endereço aparece em 0x40. Barramento mudo costuma ser SDA/SCL trocados,
// soltos, ou o módulo sem alimentação lógica no VCC.
void varreI2C() {
  Serial.println(F("--- varredura I2C ---"));
  uint8_t achados = 0;

  for (uint8_t end = 1; end < 127; end++) {
    Wire.beginTransmission(end);
    if (Wire.endTransmission() == 0) {
      achados++;
      Serial.print(F("  dispositivo em 0x"));
      if (end < 16) Serial.print('0');
      Serial.print(end, HEX);
      if (end == ENDERECO_PCA) Serial.print(F("  <- PCA9685"));
      Serial.println();
    }
  }

  pca_presente = false;
  Wire.beginTransmission(ENDERECO_PCA);
  if (Wire.endTransmission() == 0) pca_presente = true;

  if (achados == 0) {
    Serial.println(F("  NADA responde no barramento."));
    Serial.println(F("  Confira: SDA no A4, SCL no A5, VCC do PCA no 5V do"));
    Serial.println(F("  Nano, e GND comum entre Nano, PCA e fonte."));
  } else if (!pca_presente) {
    Serial.println(F("  Ha dispositivo, mas NAO em 0x40."));
    Serial.println(F("  Algum jumper de endereco do PCA esta soldado/fechado?"));
  } else {
    Serial.println(F("  PCA9685 OK."));
  }
  Serial.println(F("---------------------"));
}


void escreve(uint8_t ch, int pulso) {
  if (pulso < 0) pulso = 0;
  if (pulso > 4095) pulso = 4095;
  pwm.setPWM(ch, 0, (uint16_t)pulso);
  ultimo_pulso = pulso;

  Serial.print(F("canal "));
  Serial.print(ch);
  Serial.print(F("  pulso "));
  Serial.println(pulso);

  if (!pca_presente) {
    Serial.println(F("  AVISO: o PCA nao respondeu na ultima varredura."));
    Serial.println(F("  O comando saiu, mas provavelmente nao chegou a lugar nenhum."));
  }
}


void leA0() {
  long soma = 0;
  int menor = 1023, maior = 0;
  for (uint8_t i = 0; i < 20; i++) {
    int v = analogRead(PINO_ACS);
    soma += v;
    if (v < menor) menor = v;
    if (v > maior) maior = v;
    delay(10);
  }
  Serial.print(F("A0  media "));
  Serial.print(soma / 20.0, 1);
  Serial.print(F("  min "));
  Serial.print(menor);
  Serial.print(F("  max "));
  Serial.print(maior);
  Serial.print(F("  (espalhamento "));
  Serial.print(maior - menor);
  Serial.println(')');
  Serial.println(F("  Parado e sem carga, a media fica perto de 512."));
  Serial.println(F("  Espalhamento grande com o servo parado sugere ruido ou"));
  Serial.println(F("  GND do sensor no lugar errado."));
}


void estado() {
  Serial.print(F("canal ativo "));
  Serial.print(canal);
  Serial.print(F("  ultimo pulso "));
  if (ultimo_pulso < 0) Serial.print(F("(nenhum)"));
  else Serial.print(ultimo_pulso);
  Serial.print(F("  PCA "));
  Serial.println(pca_presente ? F("presente") : F("AUSENTE"));
}


void ajuda() {
  Serial.println(F("? estado | i varre I2C | c <0-15> canal | a <0-180> angulo"));
  Serial.println(F("p <0-4095> pulso cru | u <us> microsegundos | s sweep"));
  Serial.println(F("o desliga | z le A0"));
}


void processa(char *t) {
  while (*t == ' ' || *t == '\t') t++;
  if (*t == '\0') return;

  char cmd = *t;
  char *arg = t + 1;
  while (*arg == ' ') arg++;

  switch (cmd) {
    case '?': estado(); break;
    case 'i': case 'I': varreI2C(); break;
    case 'z': case 'Z': leA0(); break;

    case 'c': case 'C': {
      int v = atoi(arg);
      if (v < 0 || v > 15) { Serial.println(F("canal 0 a 15")); break; }
      canal = v;
      Serial.print(F("canal ativo "));
      Serial.println(canal);
      break;
    }

    case 'a': case 'A': {
      int g = atoi(arg);
      if (g < 0) g = 0;
      if (g > 180) g = 180;
      long pulso = PULSO_MIN + (long)(g) * (PULSO_MAX - PULSO_MIN) / 180;
      Serial.print(F("angulo "));
      Serial.print(g);
      Serial.print(F(" -> "));
      escreve(canal, pulso);
      break;
    }

    case 'p': case 'P':
      escreve(canal, atoi(arg));
      break;

    // Pensar em microssegundos é mais seguro que em contagens, porque a folha
    // de dados do servomotor fala em us e a contagem depende da frequência.
    case 'u': case 'U': {
      long us = atol(arg);
      if (us < 200 || us > 3000) { Serial.println(F("us entre 200 e 3000")); break; }
      long cont = us * 4096L / 20000L;        // 50 Hz
      Serial.print(us);
      Serial.print(F(" us -> "));
      escreve(canal, cont);
      break;
    }

    // Bloqueante de propósito: é sketch de diagnóstico, e o que se quer aqui
    // é ver o servo andar sem nenhuma lógica de interpolação no meio.
    case 's': case 'S': {
      Serial.println(F("sweep 60 -> 120 -> 60"));
      for (int g = 60; g <= 120; g++) {
        pwm.setPWM(canal, 0, PULSO_MIN + (long)g * (PULSO_MAX - PULSO_MIN) / 180);
        delay(25);
      }
      for (int g = 120; g >= 60; g--) {
        pwm.setPWM(canal, 0, PULSO_MIN + (long)g * (PULSO_MAX - PULSO_MIN) / 180);
        delay(25);
      }
      Serial.println(F("fim"));
      break;
    }

    case 'o': case 'O':
      pwm.setPWM(canal, 0, 4096);      // bit de full-off
      ultimo_pulso = -1;
      Serial.print(F("PWM desligado no canal "));
      Serial.println(canal);
      break;

    default:
      ajuda();
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  Wire.begin();

  Serial.println();
  Serial.println(F("=== DIAGNOSTICO PCA9685 ==="));
  varreI2C();

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println(F("PWM a 50 Hz. Nenhum canal foi acionado ainda."));
  ajuda();
}


void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      linha[n_linha] = '\0';
      if (n_linha > 0) processa(linha);
      n_linha = 0;
    } else if (c != '\r' && n_linha < sizeof(linha) - 1) {
      linha[n_linha++] = c;
    }
  }
}
