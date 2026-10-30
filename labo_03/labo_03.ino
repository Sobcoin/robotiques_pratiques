#include <MeAuriga.h>
int ledPin = 44;
int maxLed = 6;
int minLed = 0;
int sizeLed = 7;
int numberLed = 12;
int speedAverage = 90;
int pulse = 9;
int lightLevel = 100;
int speedMax = 255;
int dist = 0;
int angleRight = 90;
int angleLeft = -90;
int semiAngle = 180;
float ratio = 39.627;
float angleStart = 0.0;
float angleTarget = 0.0;
float distanceActual=0.0;
float pulsationActual=0.0;
MeUltrasonicSensor sensorDistance(PORT_6);
MeEncoderOnBoard leftMotor(SLOT1);
MeEncoderOnBoard rightMotor(SLOT2);

MeRGBLed ringLed(0, numberLed);
MeGyro gyro(0, 0x69);

enum stateMachine { START,
                    SEGMENTONE,
                    PIVOTONE,
                    SEGMENTWO,
                    PIVOTWO,
                    SEGMENTHREE,
                    SEMIPIVOT,
                    END
};
stateMachine stateRobot = START;
void encodeRight() {
  rightMotor.pulsePosPlus();
}
void encodeLeft() {
  leftMotor.pulsePosPlus();
}

void setup() {
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
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  leftMotor.loop();
  rightMotor.loop();
  gyro.update();
  stateManager(currentTime);
}
void segmentTask(float distanceTarget = 0.0) {

  static float previousError = 0.0;
  float Kp = 3.5;
  float kd = 1.0;
  float diameterWheel = 6.5;
  float pi = 3.14159;

   pulsationActual = ((abs(leftMotor.getCurPos()) + abs(rightMotor.getCurPos())) / 2);
  gyro.update();
  angleTarget = gyro.getAngleZ();
  float error = angleTarget - angleStart;
  float correction = Kp * error + kd * (error - previousError);
  previousError = error;
   distanceActual = (pulsationActual / (pulse * ratio)) * diameterWheel * pi;
  int leftSide = speedAverage + correction;
  int rightSide = speedAverage - correction;
  leftMotor.setMotorPwm(-constrain(leftSide, -speedMax, speedMax));
  rightMotor.setMotorPwm(constrain(rightSide, -speedMax, speedMax));
  int mapLed = map(distanceActual, 0, distanceTarget, minLed, maxLed);
  for (int i = minLed; i <= constrain(mapLed, minLed, maxLed); i++) {
    ringLed.setColorAt(i, 0, lightLevel, 0);
  }
  ringLed.show();
}
void starting(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  int rate = 3000;
  if (firstTime) {
    firstTime = false;
    lastTime = ct;
    ringLed.setColor(0, 0, 0);
    ringLed.show();
    gyro.resetData();
    leftMotor.setPulsePos(0);
    rightMotor.setPulsePos(0);
    return;
  }
  if (ct - lastTime >= rate) {
    lastTime = ct;
    stateRobot = SEGMENTONE;
    firstTime = true;
  }
}
void firstSegment(float distanceTarget, stateMachine state, unsigned long ct, int lightSegment = 11) {
  static bool firstTime = true;
  static unsigned long lastTime = ct;
  if (firstTime) {
    firstTime = false;
    gyro.resetData();
    leftMotor.setPulsePos(0);
    rightMotor.setPulsePos(0);
    angleStart = gyro.getAngleZ();
    ringLed.setColor(0, 0, 0);
    ringLed.setColorAt(lightSegment, 0, lightLevel, 0);
    ringLed.show();
    return;
  }
  segmentTask(distanceTarget);
  if (distanceActual >= distanceTarget) {
    leftMotor.setMotorPwm(0);
    rightMotor.setMotorPwm(0);
    ringLed.setColor(0, 0, 0);
    ringLed.show();
    firstTime = true;
    stateRobot = state;
  }
}

void pivot(int turn, stateMachine state) {
  static bool firstTime = true;
  static float angleStart = 0.0;
  static float angleTarget = 0.0;
  if (firstTime) {
    firstTime = false;
    gyro.resetData();
    angleStart = gyro.getAngleZ() + turn;
    ringLed.setColor(0, 0, 0);
    ringLed.show();
    return;
  }
  angleTarget = gyro.getAngleZ();
  int leftSide = turn > 0 ? speedAverage : -speedAverage;
  int rightSide = turn > 0 ? speedAverage : -speedAverage;
  leftMotor.setMotorPwm(constrain(leftSide, -speedMax, speedMax));
  rightMotor.setMotorPwm(constrain(rightSide, -speedMax, speedMax));
  leftMotor.setMotorPwm(constrain(leftSide, -speedMax, speedMax));
  rightMotor.setMotorPwm(constrain(rightSide, -speedMax, speedMax));

  if (abs(angleTarget - angleStart) <= 2.0) {
    leftMotor.setMotorPwm(0);
    rightMotor.setMotorPwm(0);
    stateRobot = state;
    firstTime = true;
  }
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

void secondSegment(unsigned long ct) {
  float limitObject = 30;
  int lightSegmentTwo = 10;
  static bool firstTime = true;
  int rate = 100;
  int distanceSonic = catchDistance(ct);
  if (firstTime) {
    firstTime = false;
    gyro.resetData();
    leftMotor.setPulsePos(0);
    rightMotor.setPulsePos(0);
    angleStart = gyro.getAngleZ();
    ringLed.setColor(0, 0, 0);
    ringLed.setColorAt(lightSegmentTwo, 0, lightLevel, 0);
    ringLed.show();
    Serial.println("Segment 2");
    return;
  }
  segmentTask(distanceSonic - limitObject);
  if (distanceSonic <= limitObject) {
    leftMotor.setMotorPwm(0);
    rightMotor.setMotorPwm(0);
    stateRobot = PIVOTWO;
    firstTime = true;
  }
}
void threeSegment(float distanceTarget, unsigned long ct) {
  int lightSegment = 9;
  Serial.println("Segment 3");
  stateMachine state = SEMIPIVOT;
  firstSegment(distanceTarget, state, ct, lightSegment);
}


void endCourse(unsigned long ct) {
  static unsigned long lastTime = 0;
  static bool firstTime = true;
  int rate = 50;
  static bool blink = true;
  if (firstTime) {
    firstTime = false;
    leftMotor.setMotorPwm(0);
    rightMotor.setMotorPwm(0);
    for (int i = 0; i < sizeLed; i++) {
      ringLed.setColorAt(i, 0, 0, 0);
      ringLed.show();
    }
  }
  if (ct - lastTime >= rate) {
    lastTime = ct;
    if (blink) {
      for (int i = 7; i <= 11; i++) {
        ringLed.setColorAt(i, 0, lightLevel, 0);
        ringLed.show();
      }
    } else {
      for (int i = 7; i <= 11; i++) {
        ringLed.setColorAt(i, 0, 0, 0);
        ringLed.show();
      }
    }
    blink = !blink;
  }
}
void stateManager(unsigned long ct) {
  switch (stateRobot) {
    case START:
      starting(ct);
      break;
    case SEGMENTONE:
      firstSegment(120, PIVOTONE, ct);
      break;
    case PIVOTONE:
      pivot(angleRight, SEGMENTWO);
      break;
    case SEGMENTWO:
      secondSegment(ct);
      break;
    case PIVOTWO:
      pivot(angleLeft, SEGMENTHREE);
      break;
    case SEGMENTHREE:
      threeSegment(120.0, ct);
      break;
    case SEMIPIVOT:
      pivot(semiAngle, END);
      break;
    case END:
      endCourse(ct);
      break;
  }
}
