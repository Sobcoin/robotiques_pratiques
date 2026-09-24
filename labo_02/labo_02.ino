#include <MeAuriga.h>
int ledPin = 44;
int numberOfLed = 12;
int sizeLed = 7;
int speedAverage = 200;
int dist = 0;
int lightLevel = 255;
MeEncoderOnBoard leftMotor(SLOT1);
MeEncoderOnBoard rightMotor(SLOT2);
MeUltrasonicSensor sensorDistance(PORT_9);
MeRGBLed ringLed(0, numberOfLed);
enum stateMachine { MOVE,
                    SLOW,
                    STOP,
                    BACK,
                    PIVOT };
stateMachine stateRobot = MOVE;

void setup() {
  ringLed.setpin(ledPin);
  ringLed.setNumber(numberOfLed);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  dist = catchDistance(currentTime);
  displayDistance(currentTime);
  stateManager(currentTime);
}

int catchDistance(unsigned long ct) {
  static unsigned long lastTime = 0;
  int minDistanceValid = 2;
  int maxDistanceValid = 400;
  int rate = 100;
  if (ct - lastTime >= rate) {
    lastTime = ct;
    int newDistance = sensorDistance.distanceCm();
    if (newDistance >= minDistanceValid && newDistance <= maxDistanceValid) {
      dist = newDistance;
    }
  }
  return dist;
}
void displayDistance(unsigned long ct) {
  int rate = 250;
  static unsigned long lastTime = 0;
  if (ct - lastTime >= rate) {
    lastTime = ct;
    Serial.print("Distance : ");
    Serial.print(dist);
    Serial.println(" cm");
  }
}

void moveState() {
  static bool firstTime = true;
  int limitObject = 100;
  int indexFront[sizeLed] = { 11, 0, 1, 2, 3, 4, 5 };
  if (dist >= limitObject && firstTime) {
    firstTime = false;
    for (int e = 0; e < numberOfLed; e++) {
      ringLed.setColorAt(e, 0, 0, 0);
    }
    for (int e = 0; e < sizeLed; e++) {
      ringLed.setColorAt(indexFront[e], 0, lightLevel, 0);
    }
    ringLed.show();
  }
  leftMotor.setMotorPwm(-speedAverage);
  rightMotor.setMotorPwm(speedAverage);
  if (dist < limitObject) {
    firstTime = true;
    stateRobot = SLOW;
  }
}

void slowState() {
  static bool firstTime = true;
  int limitObject = 30;
  int indexBack[sizeLed] = { 5, 6, 7, 8, 9, 10, 11 };
  int speedSlow = 100;
  if (firstTime) {
    firstTime = false;
    for (int e = 0; e < 12; e++) {
      ringLed.setColorAt(e, 0, 0, 0);
    }
    for (int e = 0; e < sizeLed; e++) {
      ringLed.setColorAt(indexBack[e], lightLevel, lightLevel, 0);
    }
    ringLed.show();
  }
  leftMotor.setMotorPwm(-speedSlow);
  rightMotor.setMotorPwm(speedSlow);
  if (dist <= limitObject) {
    stateRobot = STOP;
  } else if (dist >= 100) {
    stateRobot = MOVE;
  }
  firstTime = true;
}

void stopState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  int rate = 2000;
  int speedStop = 0;
  if (firstTime) {
    firstTime = false;
    lastTime = ct;
    ringLed.setColor(0, 0, 0);
    ringLed.setColor(lightLevel, 0, 0);
    ringLed.show();
  }
  leftMotor.setMotorPwm(speedStop);
  rightMotor.setMotorPwm(speedStop);
  if (ct - lastTime >= rate) {
    stateRobot = BACK;
    firstTime = true;
  }
}

void backState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  int rate = 1000;
  int speedBack = 100;
  if (firstTime) {
    lastTime = ct;
    firstTime = false;
  }
  leftMotor.setMotorPwm(speedBack);
  rightMotor.setMotorPwm(-speedBack);
  if (ct - lastTime >= rate) {
    firstTime = true;
    stateRobot = PIVOT;
  }
}

void pivotState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  int rate = 2600;
  int speedPivot = 100;
  if (firstTime) {
    lastTime = ct;
    firstTime = false;
  }
  leftMotor.setMotorPwm(speedPivot);
  rightMotor.setMotorPwm(speedPivot);
  if (ct - lastTime >= rate) {
    firstTime = true;
    stateRobot = MOVE;
  }
}

void stateManager(unsigned long ct) {
  switch (stateRobot) {
    case MOVE:
      moveState();
      break;
    case SLOW:
      slowState();
      break;
    case STOP:
      stopState(ct);
      break;
    case BACK:
      backState(ct);
      break;
    case PIVOT:
      pivotState(ct);
      break;
  }
}
