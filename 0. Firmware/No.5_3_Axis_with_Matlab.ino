#include <AccelStepper.h>
//----------------------------------------------------------------------------------------------------------------------------------------------
// [Pre-Setting]
#define baudRate 115200
#define stepAng 1.8  //(degree) Step angle of the stepper motor
#define pitchLgth 4  //(mm) Pitch length of the lead screw
#define maxBuf 64    // Maximum number of buffer
// Before uploading this code, please double-check few changable variables below here

// [Advanced-Setting]
long maxDisX = 16571, maxDisY = 14038, maxDisZ = 9095;  // Maximum distances on each axies measured in steps. This can change by the microstep.
// If you know it, no need to do calibration. Otherwise, let them to be 0 and then do calibration
//----------------------------------------------------------------------------------------------------------------------------------------------

#define STATE1_HOMING 1
#define STATE1_CALIBRATING 2
#define STATE1_RUNNING 3

#define STATE2_XtYtZ 1
#define STATE2_XnYtZ 2
#define STATE2_XnYnZ 3

// Define the stepper motor and the pins we will use
AccelStepper stepper1(1, 2, 5);  // (Type: Driver, Step, DIR)
AccelStepper stepper2(1, 3, 6);  // (Type:driver, STEP, DIR)
AccelStepper stepper3(1, 4, 7);  // (Type:driver, STEP, DIR)
const int enablePin = 8;
const int limitSwitchX = 9, limitSwitchY = 10, limitSwitchZ = 11;

// Set the variables to repeatedly use
int stepperState = 0;
double directionState = 0;
int microstepState = 2;
int testState = 0;
const double direction_CW = 1;  // Clockwise
long targetPosX = 0, targetPosY = 0, targetPosZ = 0;
double targetVelX = 0, targetVelY = 0, targetVelZ = 0;
long maxDis1X = 0, maxDis1Y = 0, maxDis1Z = 0;  // 1st trial
long maxDis2X = 0, maxDis2Y = 0, maxDis2Z = 0;  // 2nd trial
long prevPosX = 0, prevPosY = 0, prevPosZ = 0;
long tempPosX = 0, tempPosY = 0, tempPosZ = 0;
char ready = 0;
double newMaximumSpeed = 0;
double newSpeed = 0;
double newAcceleration = 0;
const double stepperMaxSpeed = 2000;
const double stepperSpeed = 1200;           // stepperSpeed = steps / second
double stepperSpeedZ = -stepperSpeed;       // Because of how the stepper motor is set. Need to put the minus in front of the value of 'stepperSpeed'
double stepperAcceleration = stepperSpeed;  // stepperAcceleration = steps / (second)^2
unsigned int stepperMinPulseWidth = 20;
const long safetyDis = (-1 * stepperSpeed / 5);  // Distance from the limtiswitch
long safetyDisZ = ((-1) * safetyDis);

char received = 0;            // received data will be read in 'received' first
int curBuf = 0;               // Current number of buffer
char string[maxBuf] = { 0 };  // Re-assmble each character of the received data in the array

double revPerMM = ((double)1 / pitchLgth);
double stepsPerRev_Motor = (360 / stepAng);
double stepsPerRev_Micro = ((stepsPerRev_Motor)*pow(2, (microstepState - 1)));
double stepsPerMM = (revPerMM * stepsPerRev_Micro);

void setup() {
  // Set initial seed values for the stepper
  pinMode(enablePin, OUTPUT);
  pinMode(limitSwitchX, INPUT_PULLUP);
  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);

  Serial.begin(baudRate);

  stepper1.setMaxSpeed(stepperMaxSpeed);
  stepper1.setSpeed(stepperSpeed);
  stepper1.setAcceleration(stepperAcceleration);
  stepper1.setMinPulseWidth(stepperMinPulseWidth);  // Prevent more steps drift due to the pulse width by TB6600 driver

  stepper2.setMaxSpeed(stepperMaxSpeed);
  stepper2.setSpeed(stepperSpeed);
  stepper2.setAcceleration(stepperAcceleration);
  stepper2.setMinPulseWidth(stepperMinPulseWidth);

  stepper3.setMaxSpeed(stepperMaxSpeed);
  stepper3.setSpeed(stepperSpeedZ);
  stepper3.setAcceleration(stepperAcceleration);
  stepper3.setMinPulseWidth(stepperMinPulseWidth);

  digitalWrite(enablePin, LOW);  // Enable

  SerialSetup();
}

void loop() {
  Serial.parseInt();
  while (Serial.available() == 0) {}
  stepperState = Serial.parseInt();

  switch (stepperState) {
    case STATE1_HOMING:
      Homing();
      break;

    case STATE1_CALIBRATING:
      break;

    case STATE1_RUNNING:
      Serial.parseInt();
      while (Serial.available() == 0) {}
      testState = Serial.parseInt();

      switch (testState) {
        case STATE2_XtYtZ:
          break;
        case STATE2_XnYtZ:
          break;
        case STATE2_XnYnZ:
          AbsolutePosX();
          AbsolutePosY();
          AbsolutePosZ();
          Positioning();
          break;
      }
  }
}

void SerialSetup() {
  // Double-check whether the serial communication between Arduino and Matlab successfully works or not
  Serial.print('A');
  char A = 'b';
  while (A != 'A') {
    A = Serial.read();
  }
}

void Homing() {  // Move the slider to the initial position - homing
  // Reset the target position if this function is needed after the test-drive = 0; = 0;
  targetPosZ = 0;

  // Run till the first limit-switch is detected
  while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper2.setSpeed(stepperSpeed);
    stepper3.setSpeed(stepperSpeedZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  // Run till the second limit-switch is detected
  if (digitalRead(limitSwitchX) == 0) {  // 1st case: the first limit-switch detected is X-axis one
    HomingBackX();
    while (digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper2.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedZ);
      stepper2.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchY) == 0) {  // the second limit-switch detected is Y-axis one
      HomingBackY();
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedZ);
        stepper3.run();
      }
      HomingBackZ();
    } else {  // the second limit-switch detected is Z-axis one
      HomingBackZ();
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeed);
        stepper2.run();
      }
      HomingBackY();
    }
  } else if (digitalRead(limitSwitchY) == 0) {  // 2nd case: the first limit-switch detected is Y-axis one
    HomingBackY();
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedZ);
      stepper1.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchX) == 0) {  // the second limit-switch detected is X-axis one
      HomingBackX();
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedZ);
        stepper3.run();
      }
      HomingBackZ();
    } else {  // the second limit-switch detected is Z-axis one
      HomingBackZ();
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      HomingBackX();
    }
  } else {  // 3rd case: the first limit-switch detected is Z-axis one
    HomingBackZ();
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper2.setSpeed(stepperSpeed);
      stepper1.run();
      stepper2.run();
    }
    if (digitalRead(limitSwitchX) == 0) {  // the second limit-switch detected is X-axis one
      HomingBackX();
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeed);
        stepper2.run();
      }
      HomingBackY();
    } else {  // the second limit-switch detected is Y-axis one
      HomingBackY();
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      HomingBackX();
    }
  }
}
void HomingBackX() {
  stepper1.setCurrentPosition(0);  // When the limit switch is pressed, set the position to 0 steps
  while (stepper1.currentPosition() != safetyDis) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
}
void HomingBackY() {
  stepper2.setCurrentPosition(0);
  while (stepper2.currentPosition() != safetyDis) {
    stepper2.setSpeed(-stepperSpeed);
    stepper2.run();
  }
}
void HomingBackZ() {
  stepper3.setCurrentPosition(0);
  while (stepper3.currentPosition() != safetyDisZ) {
    stepper3.setSpeed(-stepperSpeedZ);
    stepper3.run();
  }
}

void AbsolutePosX() {
  Serial.parseFloat();
  while (Serial.available() == 0) {}
  targetPosX = ((-1) * Serial.parseFloat() * stepsPerMM);
  stepper1.moveTo(targetPosX);
  if (stepper1.currentPosition() > targetPosX) {
    directionState = (-1 * direction_CW);
  } else {
    directionState = direction_CW;
  }
  targetVelX = (directionState * stepperSpeed);
}
void AbsolutePosY() {
  Serial.parseFloat();
  while (Serial.available() == 0) {}
  targetPosY = ((-1) * Serial.parseFloat() * stepsPerMM);
  stepper2.moveTo(targetPosY);
  if (stepper2.currentPosition() > targetPosY) {
    directionState = (-1 * direction_CW);
  } else {
    directionState = direction_CW;
  }
  targetVelY = (directionState * stepperSpeed);
}
void AbsolutePosZ() {
  Serial.parseFloat();
  while (Serial.available() == 0) {}
  targetPosZ = (Serial.parseFloat() * stepsPerMM);
  stepper3.moveTo(targetPosZ);
  if (stepper3.currentPosition() > targetPosZ) {
    directionState = (-1 * direction_CW);
  } else {
    directionState = direction_CW;
  }
  targetVelZ = (-directionState * stepperSpeedZ);
}
void Positioning() {  // Run X, Y, and Z all together
  while ((stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0)) {
    stepper1.setSpeed(targetVelX);
    stepper2.setSpeed(targetVelY);
    stepper3.setSpeed(targetVelZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  if ((stepper1.currentPosition() == targetPosX) || (targetPosX == 0)) {  // 1st case
    ShowingCurPosX();
    while ((stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper2.setSpeed(targetVelY);
      stepper3.setSpeed(targetVelZ);
      stepper2.run();
      stepper3.run();
    }
    if ((stepper2.currentPosition() == targetPosY) || (targetPosY == 0)) {
      ShowingCurPosY();
      while (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(targetVelZ);
        stepper3.run();
      }
      ShowingCurPosZ();
    } else {
      ShowingCurPosZ();
      while (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(targetVelY);
        stepper2.run();
      }
      ShowingCurPosY();
    }
  } else if ((stepper2.currentPosition() == targetPosY) || (targetPosY == 0)) {  // 2nd case
    ShowingCurPosY();
    while ((stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) && (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper1.setSpeed(targetVelX);
      stepper3.setSpeed(targetVelZ);
      stepper1.run();
      stepper3.run();
    }
    if ((stepper1.currentPosition() == targetPosX) || (targetPosX == 0)) {
      ShowingCurPosX();
      while (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(targetVelZ);
        stepper3.run();
      }
      ShowingCurPosZ();
    } else {
      ShowingCurPosZ();
      while (stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(targetVelX);
        stepper1.run();
      }
      ShowingCurPosX();
    }
  } else {  // 3rd case
    ShowingCurPosZ();
    while ((stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(targetVelX);
      stepper2.setSpeed(targetVelY);
      stepper1.run();
      stepper2.run();
    }
    if ((stepper1.currentPosition() == targetPosX) || (targetPosX == 0)) {
      ShowingCurPosX();
      while (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(targetVelY);
        stepper2.run();
      }
      ShowingCurPosY();
    } else {
      ShowingCurPosY();
      while (stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(targetVelX);
        stepper1.run();
      }
      ShowingCurPosX();
    }
  }
}

void ShowingCurPosX() {
  Serial.print(F("   X-Axis Current Position: "));
  Serial.print(abs(stepper1.currentPosition()) / stepsPerMM);
  Serial.print(F("/"));
  Serial.println(abs(maxDisX) / stepsPerMM);
}
void ShowingCurPosY() {
  Serial.print(F("   Y-Axis Current Position: "));
  Serial.print(abs(stepper2.currentPosition()) / stepsPerMM);
  Serial.print(F("/"));
  Serial.println(abs(maxDisY) / stepsPerMM);
}
void ShowingCurPosZ() {
  Serial.print(F("   Z-Axis Current Position: "));
  Serial.print(abs(stepper3.currentPosition()) / stepsPerMM);
  Serial.print(F("/"));
  Serial.println(maxDisZ / stepsPerMM);
}