#include <Arduino.h>
#include <Wire.h>
#include <ESP32Encoder.h>
#include <Adafruit_VL53L0X.h>
#include "driver/ledc.h"

// =========================
// Pines ESP32
// =========================
#define AIN1 18
#define AIN2 19
#define PWMA 23

#define BIN1 17
#define BIN2 16
#define PWMB 4

#define ENC_DER_A 34
#define ENC_DER_B 35
#define ENC_IZQ_A 36
#define ENC_IZQ_B 39

#define S0 27
#define S1 26
#define S2 25
#define S3 33
#define SIG 32

// =========================
//  Parámetros PID (N)
// =========================
int cont_c = 0;

double Kp_v = 1.5;
double Ki_v = 0.5;
double kd_v = 0.01;

double Kp_d = 0.5;
double Ki_d = 0.1;
double kd_d = 0.01;

double Kp_o = 1.5;
double Ki_o = 0.0; // No se utilizará en el controlador PD de orientación
double kd_o = 0.05;

// =========================
// Parámetros vehículo
// =========================
#define RADIO_RUEDA_CM 2.15f
#define L_CM 5.6f
#define CPR  1452.0f

// =========================
// PWM
// =========================
const int PWM_FREQ = 5000;
const ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
const ledc_mode_t PWM_MODE = LEDC_HIGH_SPEED_MODE;
const ledc_channel_t CH_A = LEDC_CHANNEL_0;
const ledc_channel_t CH_B = LEDC_CHANNEL_1;

// =========================
// Sensores IR
// =========================
#define NUM_SENSORES 16
const int THR_HIGH_FIJO = 4090;
const int THR_LOW_FIJO  = 4060;

int minV[NUM_SENSORES];
int maxV[NUM_SENSORES];
int thrHigh[NUM_SENSORES];
int thrLow[NUM_SENSORES];
uint8_t binState[NUM_SENSORES] = {0};
int irRaw[NUM_SENSORES];
int irBin[NUM_SENSORES];

// =========================
// Objetos globales
// =========================
ESP32Encoder encoder_der;
ESP32Encoder encoder_izq;
Adafruit_VL53L0X vl53;

// =========================
// Muestreo
// =========================
const unsigned long TS_MS = 20;
const unsigned long DEBUG_MS = 150;
unsigned long tLoop = 0;
unsigned long tDebug = 0;

// =========================
// Variables medidas
// =========================
float velocidadDer = 0.0f;
float velocidadIzq = 0.0f;
float velocidadMedida = 0.0f;
float distanciaMedida = 0.0f;
float orientacionMedida = 0.0f;

// =========================
// Referencias
// =========================
float referenciaVelocidad = 15.0f;
float referenciaDistancia = 0.0f;
float referenciaOrientacion = 0.0f;

// =========================
// Salidas de control
// =========================
float salidaVelocidad = 0.0f;
float salidaDistancia = 0.0f;
float salidaOrientacion = 0.0f;

// =========================
// Comandos motores
// =========================
int cmdMotorIzq = 0;
int cmdMotorDer = 0;

// =========================
// Variables encoder
// =========================
long prevCountDer = 0;
long prevCountIzq = 0;
unsigned long prevVelMs = 0;

// =========================
// Motores
// =========================
void motor(int velIzq, int velDer) {
  velIzq = constrain(velIzq, -255, 255);
  velDer = constrain(velDer, -255, 255);

  int pwmIzq = abs(velIzq);
  int pwmDer = abs(velDer);

  if (velDer > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (velDer < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmDer = 0;
  }

  ledc_set_duty(PWM_MODE, CH_A, pwmDer);
  ledc_update_duty(PWM_MODE, CH_A);

  if (velIzq > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (velIzq < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmIzq = 0;
  }

  ledc_set_duty(PWM_MODE, CH_B, pwmIzq);
  ledc_update_duty(PWM_MODE, CH_B);
}

void motorStop() {
  motor(0, 0);
}

// =========================
// Conversión velocidad a PWM
// =========================
int velToPWM(float vel_cm_s, float vel_max_cm_s, int pwm_min, int pwm_max) {
  float s = (vel_cm_s >= 0.0f) ? 1.0f : -1.0f;
  float v = fabs(vel_cm_s);

  if (v < 1.0f) return 0;
  if (v > vel_max_cm_s) v = vel_max_cm_s;

  int pwm = (int)((v / vel_max_cm_s) * (pwm_max - pwm_min) + pwm_min);
  return (int)(s * pwm);
}

// =========================
// MUX y sensores IR
// =========================
static inline void setMuxChannel(byte channel) {
  digitalWrite(S0, channel & 0x01);
  digitalWrite(S1, (channel >> 1) & 0x01);
  digitalWrite(S2, (channel >> 2) & 0x01);
  digitalWrite(S3, (channel >> 3) & 0x01);
  delayMicroseconds(15);
}

static inline int leerSensor(uint8_t canal) {
  setMuxChannel(canal);
  delayMicroseconds(20);

  analogRead(SIG);
  delayMicroseconds(10);

  int a = analogRead(SIG);
  delayMicroseconds(5);
  int b = analogRead(SIG);

  return (a + b) / 2;
}

void leerIR16(int out[NUM_SENSORES], bool binary) {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int v = leerSensor(i);

    if (binary) {
      if (binState[i] == 0) {
        if (v >= thrHigh[i]) binState[i] = 1;
      } else {
        if (v <= thrLow[i]) binState[i] = 0;
      }

      out[i] = binState[i];
    } else {
      out[i] = v;
    }
  }
}

void calibrarSensoresIR() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    minV[i] = 0;
    maxV[i] = 4095;
    binState[i] = 0;
    thrHigh[i] = THR_HIGH_FIJO;
    thrLow[i] = THR_LOW_FIJO;
  }
}

// =========================
// Sensor distancia
// =========================
float distancia() {
  VL53L0X_RangingMeasurementData_t measure;
  vl53.rangingTest(&measure, false);

  if (measure.RangeStatus == 0 || measure.RangeStatus == 1) {
    return measure.RangeMilliMeter * 0.1f;
  }

  return -1.0f;
}

// =========================
// Encoders y velocidad
// =========================
void resetEncoders() {
  encoder_der.clearCount();
  encoder_izq.clearCount();
  prevCountDer = 0;
  prevCountIzq = 0;
  prevVelMs = millis();
}

void actualizarVelocidad() {
  unsigned long now = millis();
  float dt = (now - prevVelMs) * 0.001f;
  if (dt <= 0.0f) return;

  long countDer = encoder_der.getCount();
  long countIzq = encoder_izq.getCount();

  long deltaDer = countDer - prevCountDer;
  long deltaIzq = countIzq - prevCountIzq;

  float revDer = (float)deltaDer / CPR;
  float revIzq = (float)deltaIzq / CPR;

  velocidadDer = (2.0f * PI * RADIO_RUEDA_CM * revDer) / dt;
  velocidadIzq = (2.0f * PI * RADIO_RUEDA_CM * revIzq) / dt;
  velocidadMedida = 0.5f * (velocidadDer + velocidadIzq);

  prevCountDer = countDer;
  prevCountIzq = countIzq;
  prevVelMs = now;
}

// =========================
// Mediciones
// =========================
void actualizarMediciones() {
  leerIR16(irRaw, false);
  leerIR16(irBin, true);
  actualizarVelocidad();
  
  cont_c += 1;
  if (cont_c == 4) {
    cont_c = 0;
    distanciaMedida = distancia();
    orientacionMedida = 0;

    for (int i = 0; i < 8; i++) {
      if (i == 0) {
        orientacionMedida = orientacionMedida + 4 * irBin[i + 8];
      }
      else if (i == 1) {
        orientacionMedida = orientacionMedida + 3 * irBin[i + 8];
      }
      else if (i == 2) {
        orientacionMedida = orientacionMedida + 2 * irBin[i + 8];
      }
      else if (i == 3 || i == 4) { // <-- Corregido de && a ||
        orientacionMedida = orientacionMedida + irBin[i + 8];
      }
      else if (i == 5) {
        orientacionMedida = orientacionMedida - 2 * irBin[i + 8];
      }
      else if (i == 6) {
        orientacionMedida = orientacionMedida - 3 * irBin[i + 8];
      }
      else if (i == 7) {
        orientacionMedida = orientacionMedida - 4 * irBin[i + 8];
      }
    }
  }
}
  
// =========================
// Controladores Implementados
// =========================

float ControladorVelocidad(float referencia, float medicion) {
  static float error_ant = 0.0f;
  static float integral = 0.0f;
  float Ts = TS_MS * 0.001f; // Tiempo de muestreo en segundos

  float error = referencia - medicion;
  integral += error * Ts;
  float derivada = (error - error_ant) / Ts;

  float u = (Kp_v * error) + (Ki_v * integral) + (kd_v * derivada);

  error_ant = error;
  return u;
}

float ControladorDistancia(float referencia, float medicion) {
  static float error_ant = 0.0f;
  static float integral = 0.0f;
  float Ts = TS_MS * 0.001f; 

  float error = referencia - medicion;
  integral += error * Ts;
  float derivada = (error - error_ant) / Ts;

  float u = (Kp_d * error) + (Ki_d * integral) + (kd_d * derivada);

  error_ant = error;
  return u;
}

float ControladorOrientacion(float referencia, float medicion) {
  static float error_ant = 0.0f;
  float Ts = TS_MS * 0.001f; 

  float error = referencia - medicion;
  float derivada = (error - error_ant) / Ts;

  // Controlador PD: Sin accion integral
  float u = (Kp_o * error) + (kd_o * derivada);

  error_ant = error;
  return u;
}

// =========================
// Actuación
// =========================
void actualizarActuacion() {
  salidaVelocidad   = ControladorVelocidad(referenciaVelocidad, velocidadMedida);
  salidaDistancia   = ControladorDistancia(referenciaDistancia, distanciaMedida);
  salidaOrientacion = ControladorOrientacion(referenciaOrientacion, orientacionMedida);

  cmdMotorIzq = velToPWM(salidaVelocidad - salidaOrientacion, 50.6, 35, 255);
  cmdMotorDer = velToPWM(salidaVelocidad + salidaOrientacion, 50.6, 35, 255);

  motor(cmdMotorIzq, cmdMotorDer);
}

// =========================
// Debug
// =========================
void imprimirDebug() {
  Serial.print("IR: ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print(irRaw[i]);
    if (i < NUM_SENSORES - 1) Serial.print(" ");
  }

  Serial.print(" | Vder: ");
  Serial.print(velocidadDer);

  Serial.print(" | Vizq: ");
  Serial.print(velocidadIzq);

  Serial.print(" | Vmed: ");
  Serial.print(velocidadMedida);

  Serial.print(" | Dmed: ");
  Serial.print(distanciaMedida);

  Serial.print(" | Omed: ");
  Serial.print(orientacionMedida);

  Serial.print(" | uVel: ");
  Serial.print(salidaVelocidad);

  Serial.print(" | uDis: ");
  Serial.print(salidaDistancia);

  Serial.print(" | uOri: ");
  Serial.print(salidaOrientacion);

  Serial.print(" | MI: ");
  Serial.print(cmdMotorIzq);

  Serial.print(" | MD: ");
  Serial.println(cmdMotorDer);
}
// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);

  ledc_timer_config_t tcfg = {
    .speed_mode = PWM_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&tcfg);

  ledc_channel_config_t chA = {
    .gpio_num = PWMA,
    .speed_mode = PWM_MODE,
    .channel = CH_A,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&chA);

  ledc_channel_config_t chB = {
    .gpio_num = PWMB,
    .speed_mode = PWM_MODE,
    .channel = CH_B,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&chB);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoder_der.attachFullQuad(ENC_DER_A, ENC_DER_B);
  encoder_izq.attachFullQuad(ENC_IZQ_A, ENC_IZQ_B);

  resetEncoders();
  calibrarSensoresIR();

  if (!vl53.begin()) {
    Serial.println("VL53L0X no detectado");
  }

  tLoop = millis();
  tDebug = millis();
}

// =========================
// Loop
// =========================
void loop() {
  if (millis() - tLoop >= TS_MS) {
    tLoop = millis();
    actualizarMediciones();
    actualizarActuacion();
  }

  if (millis() - tDebug >= DEBUG_MS) {
    tDebug = millis();
    imprimirDebug();
  }
}