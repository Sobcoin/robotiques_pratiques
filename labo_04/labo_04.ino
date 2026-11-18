#include <Arduino.h>
#include <MeAuriga.h>
#include <Adafruit_seesaw.h>
#include "labo3.h"
#define NB_IR 5
#define JAUNE 0x643200
#define ROUGE 0x640000
enum stateLabo { WAITING,
                 LAB3,
                 CALIB,
                 NOCALIB,
                 FOLLOWLINE,
                 INVALIDCMD
};
struct Capteur {
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};
stateLabo stateLine = WAITING;
Capteur capteurs[NB_IR];
int ledPin = 44;
int numberLed = 12;
int speedAverage = 70;
int pulse = 9;
int lightLevel = 100;
int speedMax = 255;
float ratio = 39.627;
int sensorValues[NB_IR];  // Tableau pour stocker les valeurs des capteurs
uint16_t valMin[NB_IR];
uint16_t valMax[NB_IR];
uint16_t seuil[NB_IR];
static bool calibrationDone = false;
String serialMessage;
MeUltrasonicSensor sensorDistance(PORT_6);
MeEncoderOnBoard leftMotor(SLOT1);
MeEncoderOnBoard rightMotor(SLOT2);
MeRGBLed ringLed(0, numberLed);
MeGyro gyro(0, 0x69);
Adafruit_seesaw ss;

void encodeRight() {
  rightMotor.pulsePosPlus();
}
void encodeLeft() {
  leftMotor.pulsePosPlus();
}

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(rightMotor.getIntNum(), encodeRight, RISING);
  attachInterrupt(leftMotor.getIntNum(), encodeLeft, RISING);
  leftMotor.setPulse(pulse);
  leftMotor.setRatio(ratio);
  rightMotor.setPulse(pulse);
  rightMotor.setRatio(ratio);
  ringLed.setpin(ledPin);
  ringLed.setNumber(numberLed);
  Serial.begin(115200);
  gyro.begin();
  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }
  Serial.println("Connexion réussie au LyneTracker!");
  Serial.println("ETAT : BOOT");
  delay(3000);
}

void loop() {
  unsigned long currentTime = millis();
  leftMotor.loop();
  rightMotor.loop();
  gyro.update();
  state(currentTime);
  // put your main code here, to run repeatedly:
}

void laboThree(unsigned long ct) {
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    stateRobot = START;
    Serial.println("ETAT : LABO 3");
  }
  stateManager(ct);
  if (stateRobot == END) {
    firstTime = true;
    stateRobot = START;
    stateLine = WAITING;
  }
}
void calibrationTask() {
  static bool firstTime = true;
  static int halfTurn = 0;
  if (firstTime) {
    firstTime = false;
    halfTurn = 0;
    calibrationDone = false;
    Serial.println("ETAT : CALIBRATION");
    firstTime = false;
    for (int i = 0; i < NB_IR; i++) {
      valMin[i] = 1023;
      valMax[i] = 0;
    }
  }
  for (int i = 0; i < NB_IR; i++) {
    sensorValues[i] = ss.analogRead(i);
    if (sensorValues[i] < valMin[i]) {
      valMin[i] = sensorValues[i];
    }
    if (sensorValues[i] > valMax[i]) {
      valMax[i] = sensorValues[i];
    }
    seuil[i] = (valMin[i] + valMax[i]) / 2;
    capteurs[i].valeurLue = sensorValues[i];
    capteurs[i].valeurMin = valMin[i];
    capteurs[i].valeurMax = valMax[i];
  }
  pivot(180, END);
  if (stateRobot == END) {
    stateRobot = START;
    halfTurn++;
    if (halfTurn == 2) {
      calibrationDone = true;
      firstTime = true;
      stateLine = WAITING;
    }
  }
}
void noCalibrationTask(unsigned long ct, uint32_t couleur) {
  static unsigned long lastTime = 0;
  static unsigned long lastTimeBlink = 0;
  static bool blink = true;
  static bool firstTime = true;
  int rate = 3000;
  int rateBlink = 300;
  if (firstTime) {
    firstTime = false;
    lastTime = ct;
    if (couleur == JAUNE) {
      Serial.println("ETAT : NON-CALIBRATION");
    } else if (couleur == ROUGE) {
      Serial.println("ETAT : COMMANDE INVALIDE");
    }
  }
  if (ct - lastTimeBlink >= rateBlink) {
    lastTimeBlink = ct;
    if (blink) {
      ringLed.setColor(0, couleur);
      ringLed.show();
    } else {
      ringLed.setColor(0, 0, 0);
      ringLed.show();
    }
    blink = !blink;
  }
  if (ct - lastTime >= rate) {
    lastTime = ct;
    stateLine = WAITING;
    firstTime = true;
  }
}
void waitingTask() {
  leftMotor.setMotorPwm(0);
  rightMotor.setMotorPwm(0);
  ringLed.setColor(0, 0, 0);
  ringLed.show();
}
void bluetoothMessage() {
  if (!Serial.available()) return;
  serialMessage = Serial.readStringUntil('\n');
  if (serialMessage.length() > 2) {
    // Retrait de l'entête 0xFF55
    //serialMessage.remove(0, 2);
    serialMessage.trim();
    if (serialMessage == "stop") {
      stateLine = WAITING;
    } else if (serialMessage == "lab3") {
      stateLine = LAB3;
    } else if (serialMessage == "cal") {
      stateLine = CALIB;
    } else if (serialMessage == "ligne") {
      stateLine = FOLLOWLINE;
    } else {
      stateLine = INVALIDCMD;
    }
  }
  Serial.print("cmd reçu : ");
  Serial.println(serialMessage);
}

float captorReadingNormalize(int index) {
  // On multiplie par 1.0 pour forcer la division en décimal
  return ((capteurs[index].valeurLue - capteurs[index].valeurMin) * 1.0) / (capteurs[index].valeurMax - capteurs[index].valeurMin) * 1000.0;
}

void normalizeValues() {
  for (int i = 0; i < NB_IR; i++) {
    capteurs[i].valeurNormalisee = captorReadingNormalize(i);
  }
}

float calculPosition() {
  float numerateur = 0;
  float denominateur = 0;
  for (int i = 0; i < NB_IR; i++) {
    numerateur += capteurs[i].valeurNormalisee * (i - 2);
    denominateur += capteurs[i].valeurNormalisee;
  }
  float position = numerateur / denominateur * 1000;
  return position;
}

float computePID() {
  float consigne = 0.0f;
  normalizeValues();
  float position = calculPosition();
  // Ajustement des coefficients
  static float kp = 0.1;   // Coefficient proportionnel
  static float ki = 0.01;  // Coefficient intégral
  static float kd = 0.01;  // Coefficient dérivé

  static float integral = 0;
  static float derivative = 0;
  static float lastError = 0;

  float error = position - consigne;

  integral += error;

  // Adapter cette valeur selon les besoins de votre application
  const float integralLimit = 1000;

  // Limiter l'intégrale pour éviter l'emballement intégral
  integral = constrain(integral, -integralLimit, integralLimit);

  derivative = error - lastError;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;

  return output;
}

void followLine(float adjust) {
  for (int i = 0; i < NB_IR; i++) {
    sensorValues[i] = ss.analogRead(i);
    //capteurs[i].valeurLue = sensorValues[i];
  }

  // Ajuster la direction en fonction des valeurs
  if (sensorValues[2] < seuil[2]) {
    // Le capteur du milieu voit la ligne, avancer
    leftMotor.setMotorPwm(-speedAverage);  // négatif car le moteur gauche est inversé
    rightMotor.setMotorPwm(speedAverage);
  } else if (sensorValues[0] < seuil[0] || sensorValues[1] < seuil[1]) {
    // Les capteurs de gauche voient la ligne, tourner à gauche
    leftMotor.setMotorPwm(-constrain(speedAverage - adjust, -speedMax, speedMax));
    rightMotor.setMotorPwm(constrain(speedAverage + adjust, -speedMax, speedMax));
  } else if (sensorValues[3] < seuil[3] || sensorValues[4] < seuil[4]) {
    // Les capteurs de droite voient la ligne, tourner à droite
    leftMotor.setMotorPwm(-constrain(speedAverage + adjust, -speedMax, speedMax));
    rightMotor.setMotorPwm(constrain(speedAverage - adjust, -speedMax, speedMax));
  } else {
    // Aucun capteur ne voit la ligne, arrêter ou chercher la ligne
    pivot(180, START);
  }
}

void state(unsigned long ct) {
  bluetoothMessage();
  switch (stateLine) {
    case WAITING:
      waitingTask();
      break;
    case LAB3:
      laboThree(ct);
      break;
    case CALIB:
      calibrationTask();
      break;
    case FOLLOWLINE:
      if (calibrationDone) {
        float adjustment = computePID();
        followLine(adjustment);
      } else {
        stateLine = NOCALIB;
      }
      break;
    case INVALIDCMD:
      noCalibrationTask(ct, ROUGE);
      break;
    case NOCALIB:
      noCalibrationTask(ct, JAUNE);
      break;
  }
}