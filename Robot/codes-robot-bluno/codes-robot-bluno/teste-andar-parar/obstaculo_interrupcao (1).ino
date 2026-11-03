// Parar, verificar e virar diante de obstáculo — robô ST5 VAC (4 motores).
// URM37 em modo automático (eco no pino 2, lido por interrupção).
// Baseado em test_ultrasonic_autoINT.ino e test_motors.ino (CentraleSupélec).
//
// Arquivos da disciplina que precisam estar na MESMA pasta deste sketch:
//   Motor.h, Motor.cpp, Parameters.h, MyServo.h, MyServo.cpp
//
// Ligações (pinos em Parameters.h):
//   URM37 eco -> pino 2 (interrupção), 5V, GND
//   Motores   -> FR: PWM 3/DIR 4, FL: PWM 11/DIR 12, RR: PWM 5/DIR 8, RL: PWM 6/DIR 7
//   Servo frontal (onde o URM37 está montado) -> pino 26
//
// Comportamento: anda até o sensor ler THRESHOLD cm ou menos, para, observa por 2 s;
// se o obstáculo saiu, volta a andar; se continua, gira no eixo e tenta de novo.
//
// Comandos pela serial (115200):
//   G       anda em modo autônomo        X       para tudo
//   T       testa a virada (bloqueante)  V<n>    velocidade 0..255
//   R<ms>   duração da virada (calibre até dar ~180 graus)
//
// Mensagens enviadas pela serial:
//   BLOQUEADO    obstáculo estático por 2 s: o carro vai virar
//   SEM_SENSOR   o sensor parou de mandar medidas: o carro para
//   D,<cm>,<estado>   diagnóstico a cada 500 ms (-1 = sem eco válido, -2 = sem sinal)

#include "Motor.h"
#include "Parameters.h"

#ifndef USAR_SERVO
#define USAR_SERVO 1   // 1: centraliza o servo frontal em 90 graus (o URM37 está montado nele)
#endif

#if USAR_SERVO
#include "MyServo.h"
#endif

/* ---------- Pinos ---------- */
const int URM_ECHO = 2;      // saída de eco do URM37: pulsos em LOW, 50 us por cm
const int SERVO_PIN = 26;

/* ---------- Motores (convenção do test_motors.ino: valor positivo = para frente) ----------
   Se uma roda girar ao contrário, troque o último parâmetro (0 <-> 1) só dela. */
Motor FRmotor(vitMotor1, dirMotor1, 0);   // frente direita
Motor FLmotor(vitMotor2, dirMotor2, 0);   // frente esquerda
Motor RRmotor(vitMotor3, dirMotor3, 0);   // trás direita
Motor RLmotor(vitMotor4, dirMotor4, 0);   // trás esquerda

/* ---------- Parâmetros ---------- */
const int THRESHOLD = 30;                      // cm: abaixo disso é obstáculo
const unsigned long TEMPO_VERIFICACAO = 2000;  // ms observando se o obstáculo é estático
const unsigned long VALIDADE_US = 300000UL;    // sem medida nova por este tempo = sensor sem sinal
const int INVALIDOS_PARA_LIVRE = 2;            // leituras inválidas seguidas = nada à frente
const bool INICIAR_ANDANDO = true;
const int DELTA_MAX = 50;                      // rampa: máx. variação do PWM a cada 50 ms (só ao acelerar)

int velocidade = 150;                          // 0..255
unsigned int tempoVirar = 1200;                // ms para ~180 graus (calibrar)

const int DIST_INVALIDA = -1;                  // pulso inválido (> 900 cm): sem eco útil
const int DIST_SEM_SINAL = -2;                 // nenhuma medida recente

#if USAR_SERVO
MyServo frontServo(SERVO_PIN, 20, 160, 0);
#endif

/* ---------- Sensor (interrupção) ---------- */
volatile unsigned long pulseStart = 0;
volatile unsigned long pulseDuration = 0;
volatile unsigned long lastMeasure = 0;   // micros() da última medida completa
volatile unsigned long nMedidas = 0;      // quantas medidas completas já chegaram
volatile bool pulsoAberto = false;

void timeEcho() {
  unsigned long agora = micros();
  if (digitalRead(URM_ECHO) == LOW) {
    pulseStart = agora;
    pulsoAberto = true;
  } else if (pulsoAberto) {
    pulseDuration = agora - pulseStart;
    lastMeasure = agora;
    nMedidas++;
    pulsoAberto = false;
  }
}

int amostras[3] = {DIST_INVALIDA, DIST_INVALIDA, DIST_INVALIDA};
int idxAmostra = 0;
unsigned long ultimoN = 0;

// Mediana das 3 últimas medidas válidas, sem bloquear.
// Devolve DIST_INVALIDA (-1) ou DIST_SEM_SINAL (-2) quando não há valor útil.
int distanciaFiltrada() {
  noInterrupts();                           // copia atômica (4 bytes)
  unsigned long p = pulseDuration;
  unsigned long t = lastMeasure;
  unsigned long n = nMedidas;
  interrupts();

  if (n != ultimoN) {                       // chegou uma medida nova
    ultimoN = n;
    unsigned long cm = p / 50;
    amostras[idxAmostra] = (cm > 900) ? DIST_INVALIDA : (int)cm;
    idxAmostra = (idxAmostra + 1) % 3;
  }

  if (n == 0 || (micros() - t) > VALIDADE_US) return DIST_SEM_SINAL;

  int v[3];
  int k = 0;
  for (int i = 0; i < 3; i++)
    if (amostras[i] >= 0) v[k++] = amostras[i];
  if (k == 0) return DIST_INVALIDA;

  for (int i = 1; i < k; i++) {             // ordenação simples
    int x = v[i], j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[k / 2];
}

/* ---------- Controle dos 4 motores ---------- */
// Lado direito = FR + RR; lado esquerdo = FL + RL. Valores de -255 a 255 (positivo = frente).
int alvoEsq = 0, alvoDir = 0;               // o que se quer
int valEsq = 0, valDir = 0;                 // o que está aplicado (sobe com rampa)

void aplicarMotores() {
  FRmotor.setvalue(valDir);
  RRmotor.setvalue(valDir);
  FLmotor.setvalue(valEsq);
  RLmotor.setvalue(valEsq);
}

int passoRampa(int atual, int alvo) {
  if (atual < alvo) return (atual + DELTA_MAX < alvo) ? atual + DELTA_MAX : alvo;
  if (atual > alvo) return (atual - DELTA_MAX > alvo) ? atual - DELTA_MAX : alvo;
  return atual;
}

void atualizarMotores() {                   // chamada a cada 50 ms
  valEsq = passoRampa(valEsq, alvoEsq);
  valDir = passoRampa(valDir, alvoDir);
  aplicarMotores();
}

void irPara(int esq, int dir) { alvoEsq = esq; alvoDir = dir; }

void pararMotores() {                       // imediato, sem rampa (segurança)
  alvoEsq = alvoDir = 0;
  valEsq = valDir = 0;
  aplicarMotores();
}

void andar() { irPara(velocidade, velocidade); }
void virar() { irPara(velocidade, -velocidade); }   // esquerda para frente, direita para trás: gira para a direita

/* ---------- Tarefas ---------- */
#define numtask 3
unsigned long deltask[numtask] = {0, 50, 500};  // período de cada tarefa (ms)
unsigned long timtask[numtask];
bool taskon[numtask] = {false, false, false};
void task0();  void task1();  void task2();
void (*NameTask[numtask])() = {task0, task1, task2};

void TaskOn(int task) {
  taskon[task] = true;
  timtask[task] = millis() + deltask[task];
}

/* ---------- Máquina de estados ---------- */
enum Estado { ANDANDO, VERIFICANDO, VIRANDO, SEM_SENSOR };
Estado estado = ANDANDO;
unsigned long tEstado = 0;
bool ativo = INICIAR_ANDANDO;
int distAtual = DIST_SEM_SINAL;
int invalidos = 0;

const char* nomeEstado() {
  switch (estado) {
    case ANDANDO:     return "ANDANDO";
    case VERIFICANDO: return "VERIFICANDO";
    case VIRANDO:     return "VIRANDO";
    default:          return "SEM_SENSOR";
  }
}

void decidir(int d) {
  switch (estado) {
    case ANDANDO:
      if (d == DIST_SEM_SINAL) {
        pararMotores();
        estado = SEM_SENSOR;
        Serial.println("SEM_SENSOR");
      } else if (d >= 0 && d <= THRESHOLD) {
        pararMotores();
        estado = VERIFICANDO;
        tEstado = millis();
        invalidos = 0;
      } else {
        andar();                            // repetir é inofensivo
      }
      break;

    case VERIFICANDO:
      if (d == DIST_SEM_SINAL) {            // já está parado
        estado = SEM_SENSOR;
        Serial.println("SEM_SENSOR");
        break;
      }
      invalidos = (d == DIST_INVALIDA) ? invalidos + 1 : 0;
      if (d > THRESHOLD || invalidos >= INVALIDOS_PARA_LIVRE) {
        andar();                            // saiu da frente: não era estático
        estado = ANDANDO;
      } else if (millis() - tEstado >= TEMPO_VERIFICACAO) {
        Serial.println("BLOQUEADO");        // aqui o Pi marcaria a aresta no grafo
        virar();
        estado = VIRANDO;
        tEstado = millis();
      }
      break;

    case VIRANDO:
      if (millis() - tEstado >= tempoVirar) {
        pararMotores();                     // ANDANDO decide: anda ou verifica de novo
        estado = ANDANDO;
      }
      break;

    case SEM_SENSOR:
      if (d != DIST_SEM_SINAL) estado = ANDANDO;   // o sensor voltou
      break;
  }
}

/* TAREFA 1 (50 ms): mede, decide e atualiza os motores */
void task1() {
  int d = distanciaFiltrada();
  distAtual = d;
  if (ativo) decidir(d);
  atualizarMotores();
}

/* TAREFA 2 (500 ms): diagnóstico */
void task2() {
  Serial.print("D,");
  Serial.print(distAtual);
  Serial.print(",");
  Serial.println(nomeEstado());
}

/* TAREFA 0: comandos pela serial */
int lerNumero(int padrao) {
  char c = Serial.peek();
  if (c == '-' || (c >= '0' && c <= '9')) return Serial.parseInt();
  return padrao;
}

void task0() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    delay(2);                               // deixa chegar os dígitos seguintes
    switch (c) {
      case 'G':
        ativo = true;
        estado = ANDANDO;
        break;
      case 'X':
        ativo = false;
        estado = ANDANDO;
        pararMotores();
        break;
      case 'T': {
        virar();
        unsigned long ini = millis();
        while (millis() - ini < tempoVirar) {
          atualizarMotores();
          delay(50);
        }
        pararMotores();
        break;
      }
      case 'V':
        velocidade = constrain(lerNumero(velocidade), 0, 255);
        break;
      case 'R':
        tempoVirar = constrain(lerNumero(tempoVirar), 100, 5000);
        break;
    }
  }
}

/* ---------- Setup e loop ---------- */
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);                     // timeout curto para o parseInt()
  pararMotores();

  pinMode(URM_ECHO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(URM_ECHO), timeEcho, CHANGE);

#if USAR_SERVO
  frontServo.start();
  frontServo.setpos(90);                    // sensor olhando para a frente
#endif

  TaskOn(0);
  TaskOn(1);
  TaskOn(2);
}

void loop() {
  for (int k = 0; k < numtask; k++)
    if (taskon[k] && (long)(millis() - timtask[k]) >= 0) {   // vale também para período 0
      NameTask[k]();
      timtask[k] = millis() + deltask[k];                    // sem recuperar atrasos em rajada
    }
}
