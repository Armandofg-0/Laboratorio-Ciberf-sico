#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include "driver/ledc.h"
#include <FastIMU.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

// ************************************************************
// 0. SELECCIÓN DE ROBOT (adaptar)
// ************************************************************
#define ROBOT_ID  0   //  Único valor que cambia por robot

// Las ESP32 fuerzan una ip estatica (no se usa DHCP):
// ------------------------------------------------------------
#if ROBOT_ID == 0
  #define EtiquetaRobot     "L" //L
  #define IP_LOCAL          IPAddress(192, 168, 1, 169)
  #define IP_SUCESOR        "192.168.1.178" 
  const bool esLider = true;
// ------------------------------------------------------------
#elif ROBOT_ID == 1
  #define EtiquetaRobot     "S" //S
  #define IP_LOCAL          IPAddress(192, 168, 1, 178)
  #define IP_SUCESOR        "192.168.1.188"
  const bool esLider = false;
// ------------------------------------------------------------
#elif ROBOT_ID == 2
  #define EtiquetaRobot     "T" // C
  #define IP_LOCAL          IPAddress(192, 168, 1, 188)
  #define IP_SUCESOR        "192.168.1.19" // IP cualquiera, no envia a nadie
  const bool esLider = false;
#else
  #error "ID de Robot no configurado correctamente"
#endif

// ************************************************************
// 1. Definiciones del código
// ************************************************************

// 1.1. Configuración WiFi / UDP
// ============================================================
#define ssid              "Linksys02764" 
#define password          "ddqvcdkki7"
#define IP_monitoreo      "192.168.1.111"
#define puerto_monitoreo   1304
#define puerto_sucesor     1111
#define puerto_local       1111

// ============================================================
// 1.2. Parámetros del vehículo
// ============================================================
#define radio_rueda   2.15f   
#define l             5.6f    
#define N             1       
#define CPR           1600    // cuentas por revolución
#define VEL_MAX_CM_S  50.0f   // velocidad lineal máxima (cm/s)
#define PWM_MAX       255.0f
#define PWM_MIN_MOV    35.0f  // PWM mínimo para vencer estática
#define TS_MS        20 // Tiempo de muestreo
// ============================================================
// 1.3. Pines ESP32
// ============================================================
#define ain1       18
#define ain2       19
#define pwm_a      23
#define bin1       17
#define bin2       16
#define pwm_b       4
#define enc1_pinA  34
#define enc1_pinB  35
#define enc2_pinA  36
#define enc2_pinB  39
#define S0         27
#define S1         26
#define S2         25
#define S3         33
#define SIG        32
// ============================================================
// 1.4. VL53L0X / IMU
// ============================================================
#define IMU_ADDRESS      0x68
#define WHO_AM_I_REG     0x75
#define DIST_BLOQUEO_CM  3.0f   // distancia mínima de seguridad: si dist < esto → frenar
#define DIST_SENSOR_MAX  30.0f  // rango máximo útil del VL53L0X (cm)

// ============================================================
// 1.5. Sensores IR  (MUX 16 canales)
// ============================================================
#define NUM_SENSORES  16
#define IR_FRONT_BASE  8   // sensores frontales: canales 8..15
#define IR_REAR_BASE   0   // sensores traseros:  canales 0..7

// ============================================================
// 1.6. Máquina de estados
// ============================================================
#define inicio       0
#define calibracion  1
#define controlLoop  2
// ************************************************************
// 2. Declaración de variables, structs globales
// ************************************************************

// ============================================================
// 2.1. Variables sensores IR
// ============================================================
const int THR_HIGH_FIJO = 4090;
const int THR_LOW_FIJO  = 4060;

static int     maxV[NUM_SENSORES];
static int     minV[NUM_SENSORES];
static int     thrHigh[NUM_SENSORES];
static int     thrLow[NUM_SENSORES];
static uint8_t binState[NUM_SENSORES] = {0};

int irRaw[NUM_SENSORES];
int irBin[NUM_SENSORES];

// ============================================================
// 2.2. Variables máquina de estados
// ============================================================
volatile byte estado            = inicio;
volatile byte estado_siguiente  = inicio;
volatile byte calibrar          = 0;

// ============================================================
// 2.3. Objetos globales
// ============================================================
WiFiUDP          udp;
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();
ESP32Encoder     encoder_der;
ESP32Encoder     encoder_izq;

unsigned long    lastMicros = 0;

// ============================================================
// 2.4. IMU (MPU6050 o MPU6500 autodetectado)
// ============================================================
enum TipoIMU {
  IMU_NONE = 0,
  IMU_IS_MPU6050,
  IMU_IS_MPU6500
};

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
};

TipoIMU   tipoIMU = IMU_NONE;

MPU6050   imu6050;
MPU6500   imu6500;

calData   calib = { 0 };

AccelData accelData;
GyroData  gyroData;

bool      imu_ok = false;
float     gz_imu = 0.0f;   // actualizado en TaskControl

// ============================================================
// 2.5. PWM y distancia
// ============================================================
int     pwm_cmd_R = 0;
int     pwm_cmd_L = 0;

float   dist_cm   = 0.0f;
uint8_t cont_dist = 0;

// ============================================================
// 2.6. Estructura de debug / telemetría
// ============================================================
struct ControlDebug {
  double input;
  double setpoint;
  double error;
  double output;
  double kp, ki, kd;
};

ControlDebug dbg_velocidad   = {0,0,0,0,0,0,0};
ControlDebug dbg_distancia   = {0,0,0,0,0,0,0};
ControlDebug dbg_orientacion = {0,0,0,0,0,0,0};

// ============================================================
// 2.7. Encoders y velocidad
// ============================================================
long          prevCountDer = 0;
long          prevCountIzq = 0;
unsigned long prevVelMs    = 0;

float velocidadDer    = 0.0f;
float velocidadIzq    = 0.0f;
float velocidadMedida = 0.0f;   // promedio de ambas ruedas

float aceleracionLocal = 0.0f;

// ============================================================
// 2.8. Ganancias PID
// ============================================================

// ── Velocidad ──
float Kp_v = 1.50f;
float Ki_v = 0.50f;
float kd_v = 0.01f;

// ── Distancia ──
float Kp_d = 2.50f;
float Ki_d = 0.10f;
float kd_d = 0.01f;

// ── Orientación ──
float Kp_o = 5.00f;
float kd_o = 0.1f;
float ki_o = 0.00f;

// ============================================================
// 2.9. Referencias y estados internos PID
// ============================================================

// ── Referencias ──
float vel_ref_fija = 20.0f; // Velocidad deseada 
double vel_ref        = 0.0; // Cálculo dinámico
double distancia_ref  = 10.0;   // cm

// ── Estado PID distancia ──
float distanciaFiltrada = 0.0f;
float integral_dist     = 0.0f;
float error_ant_dist    = 0.0f;

// ── Estado PID velocidad ──
float integral_vel  = 0.0f;
float prevError_vel = 0.0f;

// ── Estado PID orientación ──
float integral_ori        = 0.0f;
float errorAnterior_ori   = 0.0f;
float lastValidError_ori  = 0.0f;

// ============================================================
// 2.10. Variables de línea
// ============================================================
float posicion_linea = 0.0f;

// ============================================================
// 2.11. Salidas controladores
// ============================================================
float salidaVelocidad   = 0.0f;
float salidaOrientacion = 0.0f;
float salidaDistancia   = 0.0f;

// ============================================================
// 2.12. Variables CACC (UDP / V2V)
// ============================================================
float    velocidad_lider   = 0.0f;  // cm/s
float    aceleracion_lider = 0.0f;  // cm/s²
float    distancia_lider   = 0.0f;  // cm

bool     lider_detectado   = false;

uint32_t last_rx = 0; // timestamp último paquete válido

// ============================================================
// 2.13. Buffers de comunicación
// ============================================================
char   paquete_entrante[128];
char   msg[256];

String parar = "si";

// ************************************************************
// 3. Declaración de funciones
// ************************************************************
TipoIMU  detectarIMU();
bool     iniciarIMU();
IMUData  leerIMU();
void     calibrarSensoresIR();
void     leerIR16(int out[], bool binary);
uint8_t  readByte(uint8_t address, uint8_t reg);
void     setup_wifi();
void     udp_recep();
void     udp_transm();
void     send_state_to_gui();
void     send_control_to_gui();

float ControladorVelocidad  (float referencia, float medicion);
float ControladorDistancia  (float referencia, float medicion);
float ControladorOrientacion(float referencia, float medicion);
float   calcularReferenciaCACC(float dist_lider, float vel_lider,
                                float acc_lider, float v_local_dist);

double  distancia();
void    actualizarVelocidad();
double  velToPWM(double vel_cm_s);
void    motor(int velIzq, int velDer);

void  TaskEstado (void*);
void  TaskControl(void*);

// ************************************************************
// 4. Definición de funciones
// ************************************************************

// ============================================================
// 4.1. FUNCIONES AUXILIARES DE BAJO NIVEL
// ============================================================

// MUX — selección de canal
// ============================================================
static inline void setMuxChannel(byte ch) {
  digitalWrite(S0,  ch        & 0x01);
  digitalWrite(S1, (ch >> 1)  & 0x01);
  digitalWrite(S2, (ch >> 2)  & 0x01);
  digitalWrite(S3, (ch >> 3)  & 0x01);
  delayMicroseconds(15);
}

// MUX — lectura de un canal con doble muestreo anti-ruido
// ============================================================
static inline int leerSensor(uint8_t canal) {
  setMuxChannel(canal);
  delayMicroseconds(20);
  analogRead(SIG);
  delayMicroseconds(10);
  int a = analogRead(SIG);
  delayMicroseconds(5);
  int b = analogRead(SIG);
  return (a + b) / 2;          // promedio para reducir ruido
}

// LECTURA SENSORES IR  (MUX 16 canales)
// ============================================================
void leerIR16(int out[], bool binary) {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int v = leerSensor(i);
    if (binary) {
      // Histéresis?
      if (binState[i] == 0) { if (v >= thrHigh[i]) binState[i] = 1; }
      else                  { if (v <= thrLow[i])  binState[i] = 0; }
      out[i] = binState[i];
    } else {
      out[i] = v;
    }
  }
}

// CALIBRACIÓN SENSORES IR
// ============================================================
void calibrarSensoresIR() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    minV[i]     = 0;
    maxV[i]     = 4095;
    binState[i] = 0;
    thrHigh[i]  = THR_HIGH_FIJO;
    thrLow[i]   = THR_LOW_FIJO;
  }
}

// DISTANCIA VL53L0X
//   Retorna distancia en cm, o −1.0 si la medición falla.
// ============================================================
double distancia() {
  VL53L0X_RangingMeasurementData_t measure;
  vl53.rangingTest(&measure, false);
  if (measure.RangeStatus == 0 || measure.RangeStatus == 1) {
    return measure.RangeMilliMeter * 0.1;   // mm -> cm
  }
  return -1.0;
}

// IMU — lectura I2C de un registro
// ============================================================
uint8_t readByte(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)address, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// IMU — detección automática MPU6050 / MPU6500
// ============================================================
TipoIMU detectarIMU() {
  uint8_t whoami = readByte(IMU_ADDRESS, WHO_AM_I_REG);
  if      (whoami == 0x68) return IMU_IS_MPU6050;
  else if (whoami == 0x70) return IMU_IS_MPU6500;
  return IMU_NONE;
}

// IMU — inicialización
// ============================================================
bool iniciarIMU() {
  int err = -1;
  if      (tipoIMU == IMU_IS_MPU6050) err = imu6050.init(calib, IMU_ADDRESS);
  else if (tipoIMU == IMU_IS_MPU6500) err = imu6500.init(calib, IMU_ADDRESS);
  return (err == 0);
}

// IMU — lectura de acelerómetro y giroscopio
// ============================================================
IMUData leerIMU() {
  IMUData data = {0, 0, 0, 0, 0, 0};
  if (!imu_ok) return data;
  if (tipoIMU == IMU_IS_MPU6050) {
    imu6050.update();
    imu6050.getAccel(&accelData);
    imu6050.getGyro(&gyroData);
  } else if (tipoIMU == IMU_IS_MPU6500) {
    imu6500.update();
    imu6500.getAccel(&accelData);
    imu6500.getGyro(&gyroData);
  }
  data.ax = accelData.accelX; data.ay = accelData.accelY; data.az = accelData.accelZ;
  data.gx = gyroData.gyroX;   data.gy = gyroData.gyroY;   data.gz = gyroData.gyroZ;
  return data;
}

// ACTUALIZAR VELOCIDAD  (encoders cuadratura)
//   Debe llamarse cada dt aprox 20 ms desde TaskControl.
// ============================================================
void actualizarVelocidad() {
  unsigned long now = millis();
  float dt = (now - prevVelMs) * 0.001f;
  if (dt <= 0.0f) return;

  long countDer = encoder_der.getCount();
  long countIzq = encoder_izq.getCount();

  float revDer = (float)(countDer - prevCountDer) / (float)CPR;
  float revIzq = (float)(countIzq - prevCountIzq) / (float)CPR;

  // v = 2*pi*r*n / dt  (cm/s)
  velocidadDer    = (2.0f * PI * radio_rueda * revDer) / dt;
  velocidadIzq    = (2.0f * PI * radio_rueda * revIzq) / dt;
  velocidadMedida = 0.5f * (velocidadDer + velocidadIzq);

  prevCountDer = countDer;
  prevCountIzq = countIzq;
  prevVelMs    = now;
}

// VEL -> PWM  (velocidad lineal cm/s -> PWM)
// ============================================================
double velToPWM(double vel_cm_s) {
  float s = (vel_cm_s >= 0.0) ? 1.0f : -1.0f;
  float v = fabsf((float)vel_cm_s);

  if (v < 2.0f) return 0.0; // CAMBIAR DE SER NECESARIO: VELOCIDAD NECESARIA PARA EMPEZAR
  if (v > VEL_MAX_CM_S) v = VEL_MAX_CM_S;

  int pwm = (int)((v / VEL_MAX_CM_S) * (PWM_MAX - PWM_MIN_MOV) + PWM_MIN_MOV);
  return (double)(s * (float)pwm);
}

// MOTOR  (velIzq / velDer : −255 ... 255)
// ============================================================
void motor(int velIzq, int velDer) {
  velIzq = constrain(velIzq, -255, 255);
  velDer = constrain(velDer, -255, 255);

  // Motor derecho — canal A
  if      (velDer > 0) { digitalWrite(ain1, HIGH); digitalWrite(ain2, LOW);  }
  else if (velDer < 0) { digitalWrite(ain1, LOW);  digitalWrite(ain2, HIGH); }
  else                 { digitalWrite(ain1, LOW);  digitalWrite(ain2, LOW);  }
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)abs(velDer));
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

  // Motor izquierdo — canal B
  if      (velIzq > 0) { digitalWrite(bin1, HIGH); digitalWrite(bin2, LOW);  }
  else if (velIzq < 0) { digitalWrite(bin1, LOW);  digitalWrite(bin2, HIGH); }
  else                 { digitalWrite(bin1, LOW);  digitalWrite(bin2, LOW);  }
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)abs(velIzq));
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

// ============================================================
// 4.2. CONTROLADORES
// ============================================================

// CONTROLADOR PID VELOCIDAD
//   Funde salidaDistancia en ref_real internamente:
//   si el robot está demasiado cerca, recorta la referencia.
// ============================================================
float ControladorVelocidad(float referencia, float medicion) {
  const float Ts = TS_MS * 0.001f;
  
  // Cota inferior por distancia: solo permite reducir, nunca aumentar
  float ref_real = constrain(referencia + salidaDistancia, 0.0f, VEL_MAX_CM_S); 
  float error = ref_real - medicion;
  integral_vel += error * Ts;
  integral_vel = constrain(integral_vel, -50.0f, 50.0f); // Anti-windup (clamp)
  
  float derivada = (error - prevError_vel) / Ts;
  prevError_vel = error;

  return Kp_v * error + Ki_v * integral_vel + kd_v * derivada;
}

// CONTROLADOR PID DISTANCIA
//   Filtro EMA sobre la medición ruidosa del VL53L0X.
//   Freno de emergencia cuando la distancia filtrada < 3 cm.
// ============================================================
float ControladorDistancia(float referencia, float medicion) {
  if (medicion < 0.0f) return 0.0f; 

  // Filtro EMA persistente
  distanciaFiltrada = 0.7f * distanciaFiltrada + 0.3f * medicion;

  // Freno de emergencia
  if (distanciaFiltrada < 3.0f) {
    integral_dist = 0.0f;
    return -vel_ref_fija; 
  }

  const float Ts = TS_MS * 0.001f;
  float error = distanciaFiltrada - referencia;
  
  integral_dist += error * Ts;
  integral_dist = constrain(integral_dist, -20.0f, 20.0f);
  
  float derivada = (error - error_ant_dist) / Ts;
  error_ant_dist = error;

  return Kp_d * error + Ki_d * integral_dist + kd_d * derivada;
}

// CONTROLADOR PD ORIENTACIÓN
//   referencia: posición deseada de la línea (normalmente 0 = centrado)
//   medicion: posición actual calculada del arreglo IR
//
//   Características:
//     • Detecta línea perdida → congela el último error válido
//     • Feedforward de giroscopio Z para anticipar giros
// ============================================================
float ControladorOrientacion(float referencia, float medicion) {
  const float Ts = TS_MS * 0.001f;
  
  // Detección de línea perdida
  bool linea_perdida = true;
  for (int i = IR_FRONT_BASE; i < IR_FRONT_BASE + 8; i++) {
    if (irBin[i]) { 
      linea_perdida = false;
      break; 
    }
  }

  // Si se pierde la línea, mantenemos el último error conocido para evitar giros erráticos
  float error = linea_perdida ? lastValidError_ori : (referencia - medicion);
  if (!linea_perdida) lastValidError_ori = error;

  float derivada = (error - errorAnterior_ori) / Ts;
  errorAnterior_ori = error;

  // PD + feedforward del giroscopio
  return Kp_o * error + kd_o * derivada + 0.02f * gz_imu;
}

// CACC — Cooperative Adaptive Cruise Control
//  Combina en tiempo real:
//    * Datos V2V del líder (velocidad, aceleración, distancia) recibidos por UDP
//    * Medición local de distancia (VL53L0X)
//    * Control local de distancia (PID)
//  Cuando lider_detectado = false  ->  w approx 0  entonces  el robot usa solo control local.
//  Cuando lider_detectado = true   ->  w = 1  entonces  domina la componente cooperativa.
//
//  Retorna: referencia de velocidad lineal (cm/s) para el PID de velocidad.
// ============================================================
float h = 0.0f;
float calcularReferenciaCACC(float dist_lider, float vel_lider,
                              float acc_lider,  float v_local_dist) {

  // -- Parámetros de tiempo de seguimiento (headway) --
  // Reducimos un poco el incremento en curva para no generar una distancia deseada excesiva
  const float h_base       = 0.20f; 
  const float h_curva_max  = 0.15f; // Antes 0.30f: menos conservador en curvas 
  
  // -- Ganancias cooperativas aumentadas --
  // Kv > 1 permite al seguidor ser proactivo para cerrar brechas
  const float Kv_cacc = 1.15f; 
  const float Kd_cacc = 0.65f;

  // ---- Headway adaptativo ----
  float curvaIntensidad = fabs(posicion_linea); 
  h = h_base + h_curva_max * curvaIntensidad; 

  // ---- Distancia deseada (headway dinámico) ----
  float distancia_deseada = (float)distancia_ref + h * velocidadMedida; 

  // ---- Medición de distancia real ----
  float dist_real = (dist_cm > 0.0f && dist_cm <= DIST_SENSOR_MAX)
                    ? dist_cm : dist_lider;

  // ---- Errores cooperativos ----
  float error_distancia = dist_real - distancia_deseada; 
  float error_velocidad = vel_lider - velocidadMedida; 

  // ---- Reducción de confianza en curvas (AJUSTADA) ----
  // Aumentamos el límite inferior de 0.40 a 0.85 para no ignorar el error de distancia en giros
  float curvaFactor = constrain(1.0f - curvaIntensidad, 0.85f, 1.0f); 
  error_distancia  *= curvaFactor;

  // ---- Componente cooperativa ----
  // Se suma la velocidad del líder como Feedforward para mejorar el seguimiento dinámico
  float v_coop = vel_lider + (Kv_cacc * error_velocidad) + (Kd_cacc * error_distancia);

  // ---- Referencia final ----
  return constrain(v_coop, 0.0f, VEL_MAX_CM_S);
}

// ============================================================
// 4.3. COMUNICACIÓN
// ============================================================

// Nombre del estado actual (usado en send_state_to_gui)
// ============================================================
const char* fsmName(byte st) {
  switch (st) {
    case inicio:      return "inicio";
    case calibracion: return "calibracion";
    case controlLoop: return "controlLoop";
    default:          return "desconocido";
  }
}

// Configuración WiFi + UDP
// ============================================================
void setup_wifi() {
  IPAddress gateway(192, 168, 1, 1); 
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(IP_LOCAL, gateway, subnet); // Asigna la IP definida arriba
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[OK] WiFi conectado");
  udp.begin(puerto_local);
}

// Recepción y descompresión de datos desde otro vehículo (V2V)
// ============================================================
void udp_recep_from_robot() {
  String s = String(paquete_entrante);
  s.trim();

  // Se asume el protocolo: "RX/ID_ROBOT/VEL_LIDER/ACCEL_LIDER/DIST_LIDER"
  int firstSlash  = s.indexOf('/', 3);
  int secondSlash = s.indexOf('/', firstSlash + 1);
  int thirdSlash  = s.indexOf('/', secondSlash + 1);

  if (firstSlash != -1 && secondSlash != -1 && thirdSlash != -1) {
    // Actualización de variables globales para el cálculo del CACC
    velocidad_lider   = s.substring(firstSlash + 1, secondSlash).toFloat();
    aceleracion_lider = s.substring(secondSlash + 1, thirdSlash).toFloat();
    distancia_lider   = s.substring(thirdSlash + 1).toFloat();
    
    last_rx = millis();      // Timestamp para validar la frescura del dato
    lider_detectado = true;  // Activa la componente cooperativa en el CACC
    
    // Notificar a la GUI sobre la recepción de datos V2V
    send_comunication_to_gui("RX_R", s.c_str());
  }
}

// Recepción UDP: comandos de la GUI y paquetes V2V
// ============================================================
void udp_recep() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(paquete_entrante, 255);
    if (len > 0) paquete_entrante[len] = '\0';

    String cmd = String(paquete_entrante);

    // --- COMANDOS DE CONTROL DE FLUJO ---
    if (cmd.startsWith("GUI/START")) {
      parar = "no";
    } 
    else if (cmd.startsWith("GUI/STOP")) {
      parar = "si";
    } 
    else if (cmd.startsWith("GUI/CALIBRAR")) {
      calibrar = 1;
    }

    // --- COMANDOS DE CONFIGURACIÓN DE PARÁMETROS ---
    
    // 1. Cambiar Referencia de Velocidad (GUI/VEL_REF/valor)
    else if (cmd.startsWith("GUI/VEL_REF/")) {
      float nueva_vel = cmd.substring(12).toFloat();
      if (nueva_vel >= 0) vel_ref_fija = nueva_vel; 
    }

    // 2. Cambiar Referencia de Distancia (GUI/DIST_REF/valor)
    else if (cmd.startsWith("GUI/DIST_REF/")) {
      float nueva_dist = cmd.substring(13).toFloat();
      if (nueva_dist > 0) distancia_ref = nueva_dist;
    }

    // 3. Actualiza PID Dinámicamente (GUI/PID/controlador/kp/ki/kd)
    else if (cmd.startsWith("GUI/PID/")) {
      char tipo[15];
      float p, i, d;
      
      // sscanf extrae: el nombre del controlador (%[^/]) y las tres constantes (%f)
      if (sscanf(paquete_entrante, "GUI/PID/%[^/]/%f/%f/%f", tipo, &p, &i, &d) == 4) {
        String ctrl = String(tipo);

        if (ctrl == "velocidad") {
          Kp_v = p; 
          Ki_v = i; 
          kd_v = d;
        } 
        else if (ctrl == "distancia") {
          Kp_d = p; 
          Ki_d = i; 
          kd_d = d;
        } 
        else if (ctrl == "orientacion") {
          Kp_o = p; 
          //
          kd_o = d; 
        }
      }
    }

    // --- COMUNICACIÓN ENTRE ROBOTS ---
    else if (cmd.startsWith("RX/")) {
      udp_recep_from_robot();
    }
  }
}

// Transmisión UDP: telemetría al robot sucesor (V2V)
// ============================================================
void udp_transm() {
  if (estado != controlLoop) return;

  String payload = "RX/";
  payload += String(EtiquetaRobot)    + "/";
  payload += String(velocidadMedida)  + "/";
  payload += String(aceleracionLocal) + "/"; 
  payload += String(dist_cm);

  udp.beginPacket(IP_SUCESOR, puerto_sucesor);
  udp.print(payload);
  udp.endPacket();
}

// Actualiza los buffers de debug con los datos del último ciclo de control
// ============================================================
void preparar_datos_telemetria() {
  // Datos del controlador de Velocidad
  dbg_velocidad.input    = velocidadMedida;
  dbg_velocidad.setpoint = vel_ref;
  dbg_velocidad.error    = vel_ref - velocidadMedida;
  dbg_velocidad.output   = salidaVelocidad;
  dbg_velocidad.kp = Kp_v; dbg_velocidad.ki = Ki_v; dbg_velocidad.kd = kd_v;

  // Datos del controlador de Distancia
  dbg_distancia.input    = dist_cm;
  dbg_distancia.setpoint = distancia_ref;
  dbg_distancia.error    = dist_cm - (float)distancia_ref;
  dbg_distancia.output   = salidaDistancia;
  dbg_distancia.kp = Kp_d; dbg_distancia.ki = Ki_d; dbg_distancia.kd = kd_d;

  // Datos del controlador de Orientación (Línea)
  dbg_orientacion.input    = posicion_linea;
  dbg_orientacion.setpoint = 0.0;
  dbg_orientacion.error    = -posicion_linea;
  dbg_orientacion.output   = salidaOrientacion;
  dbg_orientacion.kp = Kp_o; dbg_orientacion.kd = kd_o;
}

// Envía el estado actual de la FSM a la GUI
// ============================================================
void send_state_to_gui() {
  String payload = "STATE/";
  payload += "velocidad/" + String(velocidadMedida) + "/";
  payload += "parar/"      + parar  + "/";
  payload += "curva/"         + String(h);                        

  udp.beginPacket(IP_monitoreo, puerto_monitoreo);
  udp.print(payload);
  udp.endPacket();
}

// Envía telemetría de los controladores a la GUI
//   Formato: CTRL/VEL_IN/VEL_SET/VEL_OUT/DIST_IN/DIST_SET/DIST_OUT/ORI_IN/ORI_OUT
// ============================================================
void send_control_to_gui() {
  String payload;

  // -------------------------
  // Control velocidad
  // -------------------------
  payload  = "CONTROL/";
  payload += "controlador/velocidad/";
  payload += "input/"    + String(dbg_velocidad.input, 6) + "/";
  payload += "setpoint/" + String(dbg_velocidad.setpoint, 6) + "/";
  payload += "error/"    + String(dbg_velocidad.error, 6) + "/";
  payload += "output/"   + String(dbg_velocidad.output, 6) + "/";
  payload += "kp/"       + String(dbg_velocidad.kp, 6) + "/";
  payload += "ki/"       + String(dbg_velocidad.ki, 6) + "/";
  payload += "kd/"       + String(dbg_velocidad.kd, 6);

  udp.beginPacket(IP_monitoreo, puerto_monitoreo);
  udp.print(payload);
  udp.endPacket();

  // -------------------------
  // Control orientacion
  // -------------------------
  payload  = "CONTROL/";
  payload += "controlador/orientacion/";
  payload += "input/"    + String(dbg_orientacion.input, 6) + "/";
  payload += "setpoint/" + String(dbg_orientacion.setpoint, 6) + "/";
  payload += "error/"    + String(dbg_orientacion.error, 6) + "/";
  payload += "output/"   + String(dbg_orientacion.output, 6) + "/";
  payload += "kp/"       + String(dbg_orientacion.kp, 6) + "/";
  payload += "ki/"       + String(dbg_orientacion.ki, 6) + "/";
  payload += "kd/"       + String(dbg_orientacion.kd, 6);

  udp.beginPacket(IP_monitoreo, puerto_monitoreo);
  udp.print(payload);
  udp.endPacket();

  // -------------------------
  // Control distancia
  // -------------------------
  payload  = "CONTROL/";
  payload += "controlador/distancia/";
  payload += "input/"    + String(dbg_distancia.input, 6) + "/";
  payload += "setpoint/" + String(dbg_distancia.setpoint, 6) + "/";
  payload += "error/"    + String(dbg_distancia.error, 6) + "/";
  payload += "output/"   + String(dbg_distancia.output, 6) + "/";
  payload += "kp/"       + String(dbg_distancia.kp, 6) + "/";
  payload += "ki/"       + String(dbg_distancia.ki, 6) + "/";
  payload += "kd/"       + String(dbg_distancia.kd, 6);

  udp.beginPacket(IP_monitoreo, puerto_monitoreo);
  udp.print(payload);
  udp.endPacket();
}

// Envía un mensaje de comunicación genérico a la GUI
// ============================================================
void send_comunication_to_gui(const char* tipo, const char* mensaje) {
  String p = String(tipo) + "/" + String(mensaje);
  udp.beginPacket(IP_monitoreo, puerto_monitoreo);
  udp.print(p);
  udp.endPacket();
}

// ============================================================
// 4.4. MÁQUINA DE ESTADOS Y TAREAS RTOS
// ============================================================

// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // VL53L0X
  if (!vl53.begin()) Serial.println("[WARN] VL53L0X no detectado");
  else               Serial.println("[OK]  VL53L0X listo");

  // IMU
  tipoIMU = detectarIMU();
  if (tipoIMU == IMU_NONE) {
    Serial.println("[WARN] No se detectó MPU6050/MPU6500");
    imu_ok = false;
  } else {
    imu_ok = iniciarIMU();
    if (imu_ok) { lastMicros = micros(); Serial.println("[OK]  IMU lista"); }
    else          Serial.println("[ERR] Fallo inicialización IMU");
  }

  // WiFi (no bloquea el arranque si no conecta)
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  setup_wifi();

  // Pines MUX + IR
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);

  // Pines dirección de motores
  pinMode(ain1, OUTPUT); pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT); pinMode(bin2, OUTPUT);

  // LEDC — canal A (motor derecho) y canal B (motor izquierdo)
  ledc_timer_config_t timer_conf = {
    .speed_mode      = LEDC_HIGH_SPEED_MODE, 
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num       = LEDC_TIMER_0, 
    .freq_hz         = 5000, 
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t chA = {
    .gpio_num   = pwm_a, 
    .speed_mode = LEDC_HIGH_SPEED_MODE, 
    .channel    = LEDC_CHANNEL_0,
    .intr_type  = LEDC_INTR_DISABLE, 
    .timer_sel  = LEDC_TIMER_0, 
    .duty       = 0, 
    .hpoint     = 0
  };
  ledc_channel_config(&chA);

  ledc_channel_config_t chB = {
    .gpio_num   = pwm_b, 
    .speed_mode = LEDC_HIGH_SPEED_MODE, 
    .channel    = LEDC_CHANNEL_1,
    .intr_type  = LEDC_INTR_DISABLE, 
    .timer_sel  = LEDC_TIMER_0, 
    .duty       = 0, 
    .hpoint     = 0
  };
  ledc_channel_config(&chB);

  // Encoders cuadratura
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_der.attachFullQuad(enc1_pinA, enc1_pinB);
  encoder_izq.attachFullQuad(enc2_pinA, enc2_pinB);
  encoder_der.clearCount();
  encoder_izq.clearCount();
  prevVelMs = millis();

  calibrarSensoresIR();

  // FreeRTOS tasks
  //   TaskEstado  : core 0, prioridad 1 — FSM + UDP
  //   TaskControl : core 1, prioridad 2 — bucle de control 50 Hz
  xTaskCreatePinnedToCore(TaskEstado,  "TaskEstado",  10240, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskControl, "TaskControl",  4096, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// TASK DE ESTADO  (core 0, 5 ms)
//   Maneja la FSM del robot y la comunicación UDP con la GUI
// ============================================================
void TaskEstado(void* parameter) {
  for (;;) {
    udp_recep();   // recibir comandos GUI + datos V2V

    switch (estado) {

      case inicio: {
        motor(0, 0);
        encoder_der.clearCount();
        encoder_izq.clearCount();

        // --- Limpieza total de integradores y términos derivativos ---
        integral_vel      = 0.0f;
        prevError_vel     = 0.0f;
        
        integral_dist     = 0.0f;
        error_ant_dist    = 0.0f;
        distanciaFiltrada = 0.0f;
        
        integral_ori       = 0.0f;
        errorAnterior_ori  = 0.0f;
        lastValidError_ori = 0.0f;

        salidaVelocidad = salidaOrientacion = salidaDistancia = 0.0f;
        vel_ref = 0.0;
        
        send_state_to_gui();
        break;
      }

      case calibracion: {
        calibrarSensoresIR();
        calibrar = 0;
        // Regresa automáticamente a inicio (ver transiciones abajo)
        break;
      }

      case controlLoop: {
        udp_transm();  // enviar telemetría + debug a GUI
        send_state_to_gui();
        preparar_datos_telemetria();
        send_control_to_gui();
        break;
      }
    }

    // Transiciones FSM
    if      (estado == inicio      && calibrar)       estado_siguiente = calibracion;
    else if (estado == inicio      && parar == "si")  estado_siguiente = inicio;
    else if (estado == inicio      && parar == "no")  estado_siguiente = controlLoop;
    else if (estado == calibracion)                   estado_siguiente = inicio;
    else if (estado == controlLoop && parar == "no")  estado_siguiente = controlLoop;
    else                                              estado_siguiente = inicio;

    estado = estado_siguiente;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// TASK DE CONTROL  (core 1, período = 20 ms -> 50 Hz)
//
//  Pipeline:
//    IR -> velocidad -> distancia -> orientación -> distancia PID
//    -> CACC -> velocidad PID -> mezcla -> motores
// ============================================================
void TaskControl(void* parameter) {
  const TickType_t periodo      = pdMS_TO_TICKS(TS_MS);
  TickType_t       lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (estado == controlLoop && parar == "no") {

      // ── 1. Leer IR (siempre, cada ciclo) ──
      leerIR16(irRaw, false);
      leerIR16(irBin, true);

      // ── 2. Velocidad ──
      static float velAnterior = 0;
      
      velAnterior = velocidadMedida;
      actualizarVelocidad();
      aceleracionLocal = (velocidadMedida - velAnterior) / (TS_MS * 0.001f);

      // ── 3. Distancia: solo cada 4 ciclos (VL53L0X es lento 33–50 ms) ──
      cont_dist++;
      if (cont_dist >= 4) {
        cont_dist = 0;
        dist_cm = (float)distancia();
      }
      // dist_cm conserva su último valor válido entre lecturas

      // Freno de seguridad (usa el último dist_cm válido, ok)
      if (dist_cm > 0.0f && dist_cm < DIST_BLOQUEO_CM) {
        motor(0, 0);
        vTaskDelayUntil(&lastWakeTime, periodo);
        continue;
      }

      // ── 4. Posición de la línea ──
      //    Sensores frontales: irBin[8] ... irBin[15]
      //    +4,+3,+2,+1,+1,−2,−3,−4  (asimétrico, sin peso 0)
      //    orientacionMedida > 0 -> línea hacia la izquierda del robot
      //    orientacionMedida < 0 -> línea hacia la derecha
      posicion_linea = 0.0f;
      for (int i = 0; i < 8; i++) {
        if (irBin[IR_FRONT_BASE + i]) {
          if      (i == 0)            posicion_linea += 4.0f;
          else if (i == 1)            posicion_linea += 3.0f;
          else if (i == 2)            posicion_linea += 2.0f;
          else if (i == 3 || i == 4)  posicion_linea += 1.0f;
          else if (i == 5)            posicion_linea -= 2.0f;
          else if (i == 6)            posicion_linea -= 3.0f;
          else if (i == 7)            posicion_linea -= 4.0f;
        }
      }

      // ── Leer giroscopio Z para feedforward de orientación ──
      IMUData imuData = leerIMU();
      gz_imu = imuData.gz;

      // ── 5. Orientación ──
      salidaOrientacion = ControladorOrientacion(0.0f, posicion_linea);

      // ── 6. Distancia ──
      salidaDistancia = ControladorDistancia((float)distancia_ref, dist_cm);

      // ── 7. CACC -> vel_ref ──
      if (esLider) {
        // El líder siempre intenta mantener la velocidad establecida
        vel_ref = (double)vel_ref_fija; 
      } else {
        // El seguidor calcula su velocidad dinámicamente para no chocar
        vel_ref = (double)calcularReferenciaCACC(
            distancia_lider, velocidad_lider, aceleracion_lider,
            (float)salidaDistancia);
      }

      // ── 8. Velocidad (salidaDistancia ya es global, se usa internamente) ──
      salidaVelocidad = ControladorVelocidad((float)vel_ref, velocidadMedida);

      // ── 9. Mezcla + motores ──
      float cmdIzq = salidaVelocidad - salidaOrientacion;
      float cmdDer = salidaVelocidad + salidaOrientacion;
      pwm_cmd_L = (int)velToPWM((double)cmdIzq);
      pwm_cmd_R = (int)velToPWM((double)cmdDer);
      motor(pwm_cmd_L, pwm_cmd_R);

      // ── 10. Debug ──
      

    } else {
      motor(0, 0);
    }
    vTaskDelayUntil(&lastWakeTime, periodo);
  }
}