#ifndef BOURAK_H
#define BOURAK_H

#include <U8g2lib.h>
#include <Wire.h>
#include "ElBourak.h"  // La nouvelle bibliothèque ultra-rapide


/*
  forward_brake_fast(-200, 200);   // a droite
  forward_brake_fast(200, -200);//a gauche 


*/
//==================================

// ========== CONFIGURATION PID ==========
const int ConstantCount = 4;
static const float Kp[ConstantCount] = { 0.1, 0.15, 0.065, 0 };
static const float Ki[ConstantCount] = { 0.0005, 0.0005, 0.0003, 0 };
static const float Kd[ConstantCount] = { 0.21, 0.19, 0.32, 0 };

// Vitesse
int maxspeeda = 240;
int maxspeedb = 240;
int basespeeda = 225;
int basespeedb = 225;

// Variables PID
int P, I, D;
int lastError = 0;
int lastError2 = 0;
long I_accumulated = 0;
int f1 = 0;
unsigned long loopCounter = 0;
unsigned long lastFreqCheck = 0;

// ========== CONFIGURATION CAPTEURS (MUX) ==========
int SensorCount = 16;

#define S0 25
#define S1 26
#define S2 27
#define S3 14
#define SIG 36

ElBourak pid(SensorCount, 7500, SIG, S0, S1, S2, S3);

#define UL0 pid.Tab1[0]
#define UL1 pid.Tab1[1]
#define UL2 pid.Tab1[2]
#define UL3 pid.Tab1[3]
#define UL4 pid.Tab1[4]
#define UL5 pid.Tab1[5]
#define UL6 pid.Tab1[6]
#define UL7 pid.Tab1[7]
#define UL8 pid.Tab1[8]
#define UL9 pid.Tab1[9]
#define UL10 pid.Tab1[10]
#define UL11 pid.Tab1[11]
#define UL12 pid.Tab1[12]
#define UL13 pid.Tab1[13]
#define UL14 pid.Tab1[14]
#define UL15 pid.Tab1[15]

#define OFF forward_brake_fast(0, 0)
#define WAKT (millis() - t)
#define AKRA_WAKT t = millis()

// ========== CONFIGURATION OLED ==========
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 21, 22);

// ========== CONFIGURATION MPU ==========
#define I2CMPU Wire

bool oledActif = true;
int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
int16_t gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

unsigned long lastMicros = 0;
float angleZ = 0;
float gyroBiasZ = 0;

float rotZFiltered = 0;
float ang, ang1 = 0;

int x = 0, y = 0, z = 0, k = 0, f = 0;
long t = 0, t1 = 0, t2 = 0;

// ========== CONFIGURATION MOTEURS BTS7960 (Inversés) ==========
#define MOT_R_PWM1 18
#define MOT_R_PWM2 19
#define MOT_L_PWM1 5
#define MOT_L_PWM2 17

// ========== LED & BOUTON ==========

#define led 2
#define LED digitalRead(led)
#define LEDON digitalWrite(led, 1)
#define LEDOFF digitalWrite(led, 0)

// ========== FONCTIONS AFFICHAGE OPTIMISÉES ==========
void afficherTexte(String message, byte style) {
  u8g2.firstPage();
  do {
    switch (style) {
      case 1:
        u8g2.setFont(u8g2_font_10x20_tr);
        u8g2.drawStr(0, 35, message.c_str());
        break;
      case 2:
        u8g2.setFont(u8g2_font_helvR12_te);
        u8g2.drawStr(0, 40, message.c_str());
        break;
      case 3:
        u8g2.setFont(u8g2_font_8x13_tr);
        u8g2.drawFrame(0, 10, 128, 44);
        u8g2.drawStr(5, 35, message.c_str());
        break;
      default:
        u8g2.setFont(u8g2_font_8x13_tr);
        u8g2.drawStr(0, 32, message.c_str());
    }
  } while (u8g2.nextPage());
}

// Fonction unique pour afficher une valeur numérique (float ou long)
void afficherValeur(double valeur, byte style) {
  char buffer[16];
  dtostrf(valeur, 1, 2, buffer);
  u8g2.firstPage();
  do {
    switch (style) {
      case 1:
        u8g2.setFont(u8g2_font_10x20_tr);
        u8g2.drawStr(0, 35, buffer);
        break;
      case 2:
        u8g2.setFont(u8g2_font_helvR12_te);
        u8g2.drawStr(0, 40, buffer);
        break;
      case 3:
        u8g2.setFont(u8g2_font_8x13_tr);
        u8g2.drawFrame(0, 10, 128, 44);
        u8g2.drawStr(5, 35, buffer);
        break;
      default: u8g2.setFont(u8g2_font_8x13_tr); u8g2.drawStr(0, 32, buffer);
    }
  } while (u8g2.nextPage());
}

// Compatibilité avec l'ancien code
inline void afficherTexte1(float valeur, byte style) {
  afficherValeur(valeur, style);
}
inline void afficherTexte2(long valeur, byte style) {
  afficherValeur((double)valeur, style);
}

void eteindreOLED() {
  u8g2.setPowerSave(true);
  oledActif = false;
}

void allumerOLED() {
  u8g2.setPowerSave(false);
  oledActif = true;
}

void afficherBarresCapteurs(int mode, int delayMs = 10) {
  if (mode == 0) {
    pid.readRawAll();
  } else if (mode == 1) {
    pid.readCalibrated();
  } else if (mode == 3) {
    pid.readDigitalAll();
  }

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);

    if (mode == 0) u8g2.drawStr(0, 10, "Mode : Brut");
    else if (mode == 1) u8g2.drawStr(0, 10, "Mode : Calibre");
    else if (mode == 3) u8g2.drawStr(0, 10, "Mode : Digital");

    for (uint8_t i = 0; i < SensorCount; i++) {
      uint8_t hauteurBarre;
      if (mode == 3) {
        hauteurBarre = pid.Tab1[i] ? 50 : 0;
      } else {
        hauteurBarre = map(pid.Tab[i], 0, 1000, 0, 50);
      }
      u8g2.drawBox(5 + (i * 7), 64 - hauteurBarre, 5, hauteurBarre);
      if (i % 2 == 0) {
        u8g2.setCursor(5 + (i * 7), 63);
        u8g2.print(i);
      }
    }
  } while (u8g2.nextPage());
  delay(delayMs);  // délai réduit si nécessaire
}

// ========== FONCTIONS MPU OPTIMISÉES ==========
void setupMPU() {
  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(0x6B);
  I2CMPU.write(0x00);
  I2CMPU.endTransmission();

  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(0x1B);
  I2CMPU.write(0x08);
  I2CMPU.endTransmission();

  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(0x1C);
  I2CMPU.write(0x00);
  I2CMPU.endTransmission();
}

// Lecture générique des registres MPU
void recordMPURegisters(uint8_t startReg, uint8_t numBytes, int16_t* destX, int16_t* destY, int16_t* destZ) {
  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(startReg);
  I2CMPU.endTransmission();
  I2CMPU.requestFrom(0x68, numBytes);

  unsigned long timeout = micros() + 1000;
  while (I2CMPU.available() < numBytes && micros() < timeout)
    ;

  if (I2CMPU.available() >= numBytes) {
    *destX = I2CMPU.read() << 8 | I2CMPU.read();
    *destY = I2CMPU.read() << 8 | I2CMPU.read();
    *destZ = I2CMPU.read() << 8 | I2CMPU.read();
  }
}

inline void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

inline void processGyroData() {
  rotX = gyroX / 65.5;
  rotY = gyroY / 65.5;
  rotZ = gyroZ / 65.5;
}

void recordAccelRegisters() {
  recordMPURegisters(0x3B, 6, &accelX, &accelY, &accelZ);
  processAccelData();
}

void recordGyroRegisters() {
  recordMPURegisters(0x43, 6, &gyroX, &gyroY, &gyroZ);
  processGyroData();
}

void calibrateGyro() {
  long sumZ = 0;
  int samples = 1000;

  afficherTexte("Calibration...", 1);

  for (int i = 0; i < samples; i++) {
    recordGyroRegisters();
    sumZ += gyroZ;
    delayMicroseconds(900);
  }

  gyroBiasZ = (float)sumZ / samples;
  lastMicros = micros();
  angleZ = 0;

  afficherTexte("Pret!", 2);
  delay(500);
}

void calcANG() {
  unsigned long now = micros();

  if (lastMicros == 0) {
    lastMicros = now;
    return;
  }

  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  recordGyroRegisters();

  float rawZ = gyroZ - gyroBiasZ;
  if (abs(rawZ) < 20) rawZ = 0;
  rotZ = rawZ / 65.5;
  angleZ += rotZ * dt;

  angleZ = fmod(angleZ, 360.0);
  if (angleZ < 0) angleZ += 360;
}

// Gyrodour avec affichage moins fréquent
void gyrodour(bool x, int angl) {
  calcANG();
  ang = angleZ;
  ang1 = 0;
  unsigned long lastDisplay = millis();
  while (ang1 < angl) {
    calcANG();
    float diff = angleZ - ang;
    if (diff < -180) diff += 360;
    else if (diff > 180) diff -= 360;
    ang1 = diff;
    if (millis() - lastDisplay > 100) {
      afficherValeur(ang1, 3);
      lastDisplay = millis();
    }
  }
}

// ⚡ FONCTION MOTEUR BTS7960 (inchangée)
inline void forward_brake_fast(int posb, int posa) {
  if (posb > 0) {
    analogWrite(MOT_R_PWM1, posb);
    analogWrite(MOT_R_PWM2, 0);
  } else if (posb < 0) {
    analogWrite(MOT_R_PWM1, 0);
    analogWrite(MOT_R_PWM2, -posb);
  } else {
    analogWrite(MOT_R_PWM1, 0);
    analogWrite(MOT_R_PWM2, 0);
  }

  if (posa > 0) {
    analogWrite(MOT_L_PWM1, posa);
    analogWrite(MOT_L_PWM2, 0);
  } else if (posa < 0) {
    analogWrite(MOT_L_PWM1, 0);
    analogWrite(MOT_L_PWM2, -posa);
  } else {
    analogWrite(MOT_L_PWM1, 0);
    analogWrite(MOT_L_PWM2, 0);
  }
}

// ⚡ PID ULTRA-OPTIMISÉ (version unique)
void PID_control_fast(int constantIndex, int st, bool whiteLine) {
  int position;
  if (whiteLine) {
    position = pid.ReadLineWhiteFast();
  } else {
    position = pid.ReadLineBlackFast();
  }
  int error = st - position;

  D = ((error - lastError) + (lastError - lastError2)) >> 1;
  lastError2 = lastError;
  lastError = error;

  I_accumulated += error;
  if ((error > 0 && I_accumulated < 0) || (error < 0 && I_accumulated > 0)) {
    I_accumulated = I_accumulated >> 1;
  }
  I_accumulated = constrain(I_accumulated, -100000, 100000);
  I = I_accumulated / 100;

  P = error;
  int motorspeed = (P * Kp[constantIndex]) + (I * Ki[constantIndex]) + (D * Kd[constantIndex]);

  if (abs(error) < 500) {
    int sign = (error > 0) ? 1 : -1;
    motorspeed += sign * 10;
  }

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda) {
    int overflow = motorspeeda - maxspeeda;
    motorspeeda = maxspeeda;
    motorspeedb = constrain(motorspeedb + (overflow >> 1), -80, maxspeedb);
  }
  if (motorspeedb > maxspeedb) {
    int overflow = motorspeedb - maxspeedb;
    motorspeedb = maxspeedb;
    motorspeeda = constrain(motorspeeda + (overflow >> 1), -80, maxspeeda);
  }

  motorspeeda = constrain(motorspeeda, -80, maxspeeda);
  motorspeedb = constrain(motorspeedb, -80, maxspeedb);

  forward_brake_fast(motorspeedb, motorspeeda);
}

// Compatibilité avec l'ancien code
inline void PID_controlW_fast(int constantIndex, int st) {
  PID_control_fast(constantIndex, st, true);
}
inline void PID_controlB_fast(int constantIndex, int st) {
  PID_control_fast(constantIndex, st, false);
}

// Compteurs (inline pour vitesse)
inline int count() {
  pid.readDigitalAll();
  int x = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (pid.Tab1[i] == 1) x++;
  }
  return x;
}

inline int countR() {
  pid.readDigitalAllR();
  int x = 0;
  for (int i = 8; i < SensorCount; i++) {
    if (pid.Tab1[i] == 1) x++;
  }
  return x;
}

inline int countL() {
  pid.readDigitalAllL();
  int x = 0;
  for (int i = 0; i < 8; i++) {
    if (pid.Tab1[i] == 1) x++;
  }
  return x;
}

// ===============================
// PID MPU (Optimisé)
// ===============================
float Kp_mpu = 7;
float Ki_mpu = 0.1;
float Kd_mpu = 2;

float mpuError = 0;
float mpuIntegral = 0;
float mpuLastError = 0;
float targetAngle = 0;

unsigned long lastDisplay = 0;
bool mpuInitialized = false;

int baseSpeed = 230;
int maxSpeed = 255;
int currentSpeed = 0;

void initMPU_PID(int base, int max, float angleOffset) {
  calcANG();
  targetAngle = angleZ + angleOffset;
  mpuIntegral = 0;
  mpuLastError = 0;

  baseSpeed = constrain(base, 0, 255);
  maxSpeed = constrain(max, 0, 255);
  currentSpeed = baseSpeed;

  mpuInitialized = true;
  lastDisplay = millis();
}

void runMPU_PID() {
  if (!mpuInitialized) return;

  calcANG();

  mpuError = angleZ - targetAngle;
  if (mpuError > 180) mpuError -= 360;
  else if (mpuError < -180) mpuError += 360;

  float P = Kp_mpu * mpuError;

  mpuIntegral += mpuError;
  mpuIntegral = constrain(mpuIntegral, -400, 400);
  float I = Ki_mpu * mpuIntegral;

  float derivative = mpuError - mpuLastError;
  float D = Kd_mpu * derivative;
  mpuLastError = mpuError;

  float correction = P + I + D;
  correction = constrain(correction, -180, 180);

  currentSpeed = baseSpeed;

  int speedRight = currentSpeed + correction;
  int speedLeft = currentSpeed - correction;

  speedRight = constrain(speedRight, 0, 255);
  speedLeft = constrain(speedLeft, 0, 255);

  forward_brake_fast(speedRight, speedLeft);
}

void stopMPU_PID() {
  forward_brake_fast(0, 0);
  mpuInitialized = false;
  currentSpeed = 0;
}

void setMPU_PID(float kp, float ki, float kd) {
  Kp_mpu = kp;
  Ki_mpu = ki;
  Kd_mpu = kd;
}

void gyroTurnPID(float angle, int maxSpeed, float t) {
  float Kp = 2.7;
  float Ki = 0.0001;
  float Kd = 0.03;

  float error, previousError = 0;
  float derivative;
  float output;

  float tolerance = t;

  calcANG();
  float startAngle = angleZ;

  float targetAngle = startAngle + angle;
  targetAngle = fmod(targetAngle + 360.0, 360.0);

  unsigned long lastTime = millis();

  while (true) {
    calcANG();

    error = targetAngle - angleZ;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) <= tolerance) break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt == 0) dt = 0.001;
    lastTime = now;

    derivative = (error - previousError) / dt;
    previousError = error;

    output = Kp * error + Kd * derivative;
    output = constrain(output, -maxSpeed, maxSpeed);

    forward_brake_fast(output, -output);
  }
}

// 1. Lire et afficher les valeurs brutes (0 - 4095) sur le Serial
void lireEtAfficherBrutsSerial() {
  pid.readRawAll(); // Met à jour le tableau pid.Tab1
  
  Serial.print("Valeurs Brutes : ");
  for (int i = 0; i < 16; i++) {
    Serial.print(pid.Tab1[i]);
    Serial.print("\t"); // Espace pour aligner les colonnes
  }
  Serial.println(); // Retour à la ligne à la fin
  delay(100);
}

// 2. Lire et afficher les valeurs calibrées (0 - 1000) sur le Serial
void lireEtAfficherCalibresSerial() {
  pid.readCalibrated(); // Met à jour le tableau pid.Tab
  
  Serial.print("Valeurs Calibrees : ");
  for (int i = 0; i < 16; i++) {
    Serial.print(pid.Tab[i]);
    Serial.print("\t");
  }
  Serial.println();
  delay(100);
}

#endif