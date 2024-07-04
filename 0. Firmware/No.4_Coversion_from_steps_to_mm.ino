#include <AccelStepper.h>
//--------------------------------------------------------------------------
// Before uploading this code, please double-check few variables below here. 
// [Pre-Setting]
#define baudrate 115200
#define stepAng 1.8 //(degree) Step angle of the stepper motor
#define pitchLgth 4 //(mm) Pitch length of the lead screw.
//--------------------------------------------------------------------------

// [State Menu]
#define STATE1_UPDATING 1
#define STATE1_ENABLE 2
#define STATE1_HOMING 3
#define STATE1_CALIBRATING 4
#define STATE1_TESTDRIVING 5
#define STATE1_MANUALSETTING 6
#define STATE1_REWINDING 7

#define STATE2_ABSOULUTE 1
#define STATE2_RELATIVE 2
#define STATE2_BACKTOMENU 3

#define STATE3_X_AXIS 1
#define STATE3_Y_AXIS 2
#define STATE3_Z_AXIS 3
#define STATE3_RUNALL 4
#define STATE3_BACKTOMENU 5

// [Globals]
// 0. Allocate the pins we will use
AccelStepper stepper1(1, 2, 5);  // (Type: Driver, Step, DIR)
AccelStepper stepper2(1, 3, 6);  // (Type:driver, STEP, DIR)
AccelStepper stepper3(1, 4, 7);  // (Type:driver, STEP, DIR)

const int enablePin = 8;
const int limitSwitchX = 9, limitSwitchY = 10, limitSwitchZ = 11;

// 1. Set the variables for the operation
int stepperState = 0;
int testState = 0;
double directionState = 0;
int microstepState = 0;
int settingState = 0;
const double direction_CW = 1;   // Clockwise
const double direction_CCW = 2;  // Counter-Clockwise
long targetDisX = 0, targetDisY = 0, targetDisZ = 0;
long targetPosX = 0, targetPosY = 0, targetPosZ = 0;
double targetVelX = 0, targetVelY = 0, targetVelZ = 0;
long maxDis1X = 0, maxDis1Y = 0, maxDis1Z = 0; // 1st trial
long maxDis2X = 0, maxDis2Y = 0, maxDis2Z = 0; // 2nd trial
long maxDisX = 0, maxDisY = 0, maxDisZ = 0;
long prevPosX = 0, prevPosY = 0, prevPosZ = 0;
long tempPosX = 0, tempPosY = 0, tempPosZ = 0;
char Ready = 0;
double newMaximumSpeed = 0;
double newSpeed = 0;
double newAcceleration = 0;

double stepperMaxSpeed = 0;
double stepperSpeed = 0;         // stepperSpeed = steps / second
double stepperSpeedZ = 0;        // Because of how the stepper motor is set. Need to put the minus in front of the value of 'stepperSpeed'
double stepperAcceleration = 0;  // stepperAcceleration = steps / (second)^2
unsigned int stepperMinPulseWidth = 20;
long safetyDis = 0;              // Distance from the limtiswitch
long safetyDisZ = -1 * safetyDis;

// 2. Convert the measurement unit from steps(pulses) to millimeter(mm)
double revForMM = 0;
double stepsForRev_motor = 0;   // Only consider the stepper motor specification
double stepsForRev_micro = 0;   // Consideration all together with the microstep setting
double stepsForMM = 0;          // (M92) Steps per Millimeter - leadscrew driven systems

void setup() {  // Set initial seed values for the stepper
  pinMode(enablePin, OUTPUT);
  pinMode(limitSwitchX, INPUT_PULLUP);
  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);

  Serial.begin(baudrate);

  SettingDef();

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

  digitalWrite(enablePin, HIGH);  // Disable the motor pin for the safety reason
}

void loop() {  // Main interface
  if (digitalRead(enablePin) == 0) {
    Serial.println(F("※ Present Status: Enabled"));
  } else {
    Serial.println(F("※ Present Status: Disabled"));
  }

  prevPosX = stepper1.currentPosition();
  prevPosY = stepper2.currentPosition();
  prevPosZ = stepper3.currentPosition();

  Serial.println(F("※ [Controller State]"));
  Serial.println(F("   [1] Updating"));
  Serial.println(F("   [2] Enable / Disable"));
  Serial.println(F("   [3] Homing"));
  Serial.println(F("   [4] Step-Calibration"));
  Serial.println(F("   [5] Test Driving"));
  Serial.println(F("   [6] Manual Setting"));
  Serial.println(F("   [7] Rewinding"));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  stepperState = Serial.parseInt();

  Serial.print(F("   Selected ["));
  Serial.print(stepperState);
  Serial.println(F("]"));
  Serial.println(F(""));

  switch (stepperState) {
    case STATE1_UPDATING:
      Updating();
      break;

    case STATE1_ENABLE:
      Enable();
      break;

    case STATE1_HOMING:
      if (digitalRead(enablePin) == 0) {
        Homing();
      } else {
        Serial.println(F("   [Warning] Disabled now. Please change it to be enabled"));
        Serial.println(F(""));
        return;  // Better to use return since if we use 'loop()' it will come back here again after that
      }
      break;

    case STATE1_CALIBRATING:
      if (digitalRead(enablePin) == 0) {
        Calibrating();
      } else {
        Serial.println(F("   [Warning] Disabled now. Please change it to be enabled"));
        Serial.println(F(""));
        return;
      }
      break;

    case STATE1_TESTDRIVING:
      if (digitalRead(enablePin) == 0) {
        if (maxDisX == 0 || maxDisY == 0 || maxDisZ == 0) {
          Serial.println(F("   [Warning] Please do calibration first!"));
          Serial.println(F(""));
          return;  // Cannot use loop();
        } else {
          Serial.println(F("※ [Test State]"));
          Serial.println(F("   [1] Move to the certain location"));
          Serial.println(F("   [2] Move from the current position"));
          Serial.println(F("   [3] Back to menu"));

          Serial.parseInt(); // Clear the buffer
          while (Serial.available() == 0) {}
          testState = Serial.parseInt();

          Serial.print(F("   Selected ["));
          Serial.print(testState);
          Serial.println(F("]"));
          Serial.println(F(""));

          switch (testState) {
            case STATE2_ABSOULUTE:
              AbsolutePos();
              break;

            case STATE2_RELATIVE:
              RelativePos();
              break;

            case STATE2_BACKTOMENU:
              return;
              break;
          }
        }
      } else {
        Serial.println(F("   [Warning] Disabled now. Please change it to be enabled"));
        Serial.println(F(""));
        return;
      }
      break;

    case STATE1_MANUALSETTING:
      if (digitalRead(enablePin) == 1) {
        if (maxDisX == 0 || maxDisY == 0 || maxDisZ == 0) {
          Serial.println(F("   [Warning] Please do calibration first!"));
          Serial.println(F(""));
          return;  // Cannot use loop();
        } else {
          ManualSet();
        }
      } else {
        Serial.println(F("   [Warning] Please disable the motor first. Then, continue this step."));
        return;
      }
      break;

    case STATE1_REWINDING:
      if (digitalRead(enablePin) == 0) {
        Rewinding();
      } else {
        Serial.println(F("   [Warning] Disabled now. Please change it to be enabled"));
        Serial.println(F(""));
        return;
      }
      break;
  }
}

void SettingDef() {  // Pre-set the default microstep setting
  Serial.println(F("※ [Microstep state]"));
  Serial.println(F("   Microstep: 1,  Pulse/rev: 200"));
  Serial.println(F("   Microstep: 2,  Pulse/rev: 400"));
  Serial.println(F("   Microstep: 4,  Pulse/rev: 800"));
  Serial.println(F("   Microstep: 8,  Pulse/rev: 1600"));
  Serial.println(F("   Microstep: 16, Pulse/rev: 3200"));
  Serial.println(F("   Microstep: 32, Pulse/rev: 6400"));
  Serial.println(F(""));

  Serial.println(F("   Moderate maximum speed, constant speed, and acceleration for each microstep will be assigned accordingly"));
  Serial.print(F("※ Type the number of the microstep on your motor driver: "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  microstepState = Serial.parseInt();
  Serial.println(microstepState);
  Serial.println(F(""));

  if (microstepState == 1 || microstepState == 2 || microstepState == 4 || microstepState == 8 || microstepState == 16 || microstepState == 32) {
    revForMM = ((double)1 / pitchLgth);    
    stepsForRev_motor = (360 / stepAng);
    stepsForRev_micro = (stepsForRev_motor * microstepState);
    stepsForMM = (revForMM * stepsForRev_micro);

    switch (microstepState) {
      case 1:
        Serial.println(F("※ Default constant speed: 600 steps/s, default acceleration: 600 steps/s^2."));
        stepperMaxSpeed = 1000;
        stepperSpeed = 600;
        stepperAcceleration = 600;
        break;

      case 2:
        Serial.println(F("※ Default constant speed: 1200 steps/s, default acceleration: 1200 steps/s^2."));
        stepperMaxSpeed = 2000;
        stepperSpeed = 1200;
        stepperAcceleration = 1200;
        break;

      case 4:
        Serial.println(F("※ Default constant speed: 2400 steps/s, default acceleration: 2400 steps/s^2."));
        stepperMaxSpeed = 4000;
        stepperSpeed = 2400;
        stepperAcceleration = 2400;
        break;

      case 8:
        Serial.println(F("※ Default constant speed: 4800 steps/s, default acceleration: 4800 steps/s^2."));
        stepperMaxSpeed = 8000;
        stepperSpeed = 4800;
        stepperAcceleration = 4800;
        break;

      case 16:
        Serial.println(F("※ Default constant speed: 9600 steps/s, default acceleration: 9600 steps/s^2."));
        stepperMaxSpeed = 16000;
        stepperSpeed = 9600;
        stepperAcceleration = 9600;
        break;

      case 32:
        Serial.println(F("※ Default constant speed: 19200 steps/s, default acceleration: 19200 steps/s^2."));
        stepperMaxSpeed = 32000;
        stepperSpeed = 19200;
        stepperAcceleration = 19200;
        break;
    }
    Serial.print(F("   1mm linear motion needs "));
    Serial.print(stepsForMM);
    Serial.println(F(" steps"));

    safetyDis = (-1 * stepperSpeed / 5);
    stepperSpeedZ = (-1 * stepperSpeed);
    safetyDisZ = (-1 * safetyDis);
  } else {
    Serial.println(F("   [Warning] Please type the proper number. Re-try!"));
    Serial.println(F(""));
    SettingDef();
  }

  Serial.println(F("※ Before the operation, please check wether CNC machine is enabled or not"));
  Serial.println(F(""));
}

void Updating() {  // Updating the constant speed and the acceleration for once
  Serial.println(F("   [Updating]"));
  Serial.print(F("   Default maximum speed:"));
  Serial.println(stepperMaxSpeed);
  Serial.print(F("   Present maximum speed:"));
  if (newMaximumSpeed == 0) {
    newMaximumSpeed = stepperMaxSpeed;
  }
  Serial.println(newMaximumSpeed);
  Serial.print("   New maximum speed value will be set as: ");  //confirm update by message

  Serial.parseFloat(); // Clear the buffer
  while (Serial.available() == 0) {}
  newMaximumSpeed = Serial.parseFloat();
  stepper1.setMaxSpeed(newMaximumSpeed);  //update the value of the variable
  Serial.println(newMaximumSpeed);
  Serial.println(F(""));

  Serial.print(F("   Default constant speed:"));
  Serial.println(stepperSpeed);
  Serial.print(F("   Present constant speed:"));
  if (newSpeed == 0) {
    newSpeed = stepperSpeed;
  }
  Serial.println(newSpeed);
  Serial.print("   New constant speed value will be set as: ");  //confirm update by message

  Serial.parseFloat(); // Clear the buffer
  while (Serial.available() == 0) {}
  newSpeed = Serial.parseFloat();  //receive the constant speed from serial
  stepper1.setSpeed(newSpeed);     //update the value of the variable
  Serial.println(newSpeed);
  Serial.println(F(""));

  Serial.print(F("   Default acceleration:"));
  Serial.println(stepperAcceleration);
  Serial.print(F("   Present acceleration:"));
  if (newAcceleration == 0) {
    newAcceleration = stepperAcceleration;
  }
  Serial.println(newAcceleration);
  Serial.print("   New accerelation value will be set as: ");  //confirm update by message

  Serial.parseFloat(); // Clear the buffer
  while (Serial.available() == 0) {}
  newAcceleration = Serial.parseFloat();      //receive the acceleration from serial
  stepper1.setAcceleration(newAcceleration);  //update the value of the variable
  Serial.println(newAcceleration);
  Serial.println(F(""));
}

void Enable() {  // Change the status of Enable/Disable
  if (digitalRead(enablePin) == 0) {
    digitalWrite(enablePin, HIGH);
  } else {
    digitalWrite(enablePin, LOW);
  }
}

void Homing() {  // Move the slider to the initial position - homing
  // Reset the target position if this function is needed after the test-drive
  targetPosX = 0;
  targetPosY = 0;
  targetPosZ = 0; 

  Serial.println(F("※ [Homing]"));
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
  Serial.println(F(""));   
}
void HomingBackX() {
  Serial.println(F("   [X-Axis] limit switch is detected"));
  stepper1.setCurrentPosition(0);  // When the limit switch is pressed, set the position to 0 steps
  Serial.print(F("   Reset X-Position: "));
  Serial.println(abs(stepper1.currentPosition()) / stepsForMM);
  while (stepper1.currentPosition() != safetyDis) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
  Serial.print(F("   X-Axis Current Position: "));
  Serial.println(abs(stepper1.currentPosition()) / stepsForMM);
}
void HomingBackY() {
  Serial.println(F("   [Y-Axis] limit switch is detected"));
  stepper2.setCurrentPosition(0);
  Serial.print(F("   Reset Y-Position: "));
  Serial.println(abs(stepper2.currentPosition()) / stepsForMM);
  while (stepper2.currentPosition() != safetyDis) {
    stepper2.setSpeed(-stepperSpeed);
    stepper2.run();
  }
  Serial.print(F("   Y-Axis Current Position: "));
  Serial.println(abs(stepper2.currentPosition()) / stepsForMM);
}
void HomingBackZ() {
  Serial.println(F("   [Z-axis] limit switch is detected"));
  stepper3.setCurrentPosition(0);
  Serial.print(F("   Reset Z-Position: "));
  Serial.println(abs(stepper3.currentPosition()) / stepsForMM);
  while (stepper3.currentPosition() != safetyDisZ) {
    stepper3.setSpeed(-stepperSpeedZ);
    stepper3.run();
  }
  Serial.print(F("   Z-Axis Current Position: "));
  Serial.println(abs(stepper3.currentPosition()) / stepsForMM);
}

void Calibrating() {
  if (stepper1.currentPosition() != safetyDis || stepper2.currentPosition() != safetyDis || stepper3.currentPosition() != safetyDisZ) {
    Serial.println(F("   [Warning] Please do homing before calibration!"));
    Serial.println(F(""));
    return;
  }
  Serial.println(F("   [Notice] Position calibration in process"));

  // X-Axis
  // Move the slider to the right-hand position
  while (digitalRead(limitSwitchX) != 0) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
  maxDis1X = stepper1.currentPosition();
  stepper1.setCurrentPosition(0);
  while (stepper1.currentPosition() != (-safetyDis)) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }
  // Move the slider to the left-hand position
  while (digitalRead(limitSwitchX) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }
  maxDis2X = stepper1.currentPosition();
  stepper1.setCurrentPosition(0);
  while (stepper1.currentPosition() != safetyDis) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
  Serial.print(F("   The average maximum distance on the X-Axis: "));
  maxDisX = -1 * ((abs(maxDis1X) + abs(maxDis2X)) / 2);
  Serial.print(abs(maxDisX) / stepsForMM);
  Serial.println(F(" (mm)"));

  // Y-Axis
  while (digitalRead(limitSwitchY) != 0) {
    stepper2.setSpeed(-stepperSpeed);
    stepper2.run();
  }
  maxDis1Y = stepper2.currentPosition();
  stepper2.setCurrentPosition(0);
  while (stepper2.currentPosition() != (-safetyDis)) {
    stepper2.setSpeed(stepperSpeed);
    stepper2.run();
  }
  while (digitalRead(limitSwitchY) != 0) {
    stepper2.setSpeed(stepperSpeed);
    stepper2.run();
  }
  maxDis2Y = stepper2.currentPosition();
  stepper2.setCurrentPosition(0);
  while (stepper2.currentPosition() != safetyDis) {
    stepper2.setSpeed(-stepperSpeed);
    stepper2.run();
  }
  Serial.print(F("   The average maximum distance on the Y-Axis: "));
  maxDisY = -1 * ((abs(maxDis1Y) + abs(maxDis2Y)) / 2);
  Serial.print(abs(maxDisY) / stepsForMM);
  Serial.println(F(" (mm)"));

  // Z-Axis
  while (digitalRead(limitSwitchZ) != 0) {
    stepper3.setSpeed(-stepperSpeedZ);
    stepper3.run();
  }
  maxDis1Z = stepper3.currentPosition();
  stepper3.setCurrentPosition(0);
  while (stepper3.currentPosition() != (-safetyDisZ)) {
    stepper3.setSpeed(stepperSpeedZ);
    stepper3.run();
  }
  // Move the slider to the left-hand position
  while (digitalRead(limitSwitchZ) != 0) {
    stepper3.setSpeed(stepperSpeedZ);
    stepper3.run();
  }
  maxDis2Z = stepper3.currentPosition();
  stepper3.setCurrentPosition(0);
  while (stepper3.currentPosition() != safetyDisZ) {
    stepper3.setSpeed(-stepperSpeedZ);
    stepper3.run();
  }
  Serial.print(F("   The average maximum distance on the Z-Axis: "));
  maxDisZ = ((abs(maxDis1Z) + abs(maxDis2Z)) / 2);
  Serial.print(abs(maxDisZ) / stepsForMM);
  Serial.println(F(" (mm)"));
  Serial.println(F(""));
}

void AbsolutePos() {
  Serial.println(F("※ [Setting State]"));
  Serial.println(F("   After setting all X, Y, and Z, please press '[4] Run' at the end"));
  Serial.println(F("   If you don't want to change some positions, then just leave it"));
  Serial.println(F("   [1] X-Axis"));
  Serial.println(F("   [2] Y-Axis"));
  Serial.println(F("   [3] Z-Axis"));
  Serial.println(F("   [4] Run"));
  Serial.println(F("   [5] Back to menu"));
  Serial.println(F(""));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  settingState = Serial.parseInt();

  if (settingState == 1 || settingState == 2 || settingState == 3 || settingState == 4  || settingState == 5) {
    switch (settingState) {
      case STATE3_X_AXIS:
        AbsolutePosX();
        break;
      case STATE3_Y_AXIS:
        AbsolutePosY();
        break;
      case STATE3_Z_AXIS:
        AbsolutePosZ();
        break;
      case STATE3_RUNALL:
        RunAll();
        break;
      case STATE3_BACKTOMENU:
        return;
        break;
    }
  } else {
    Serial.println(F("   [Warning] Please type the proper number. Re-try!"));
    Serial.println(F(""));
    AbsolutePos();
  }
}
void AbsolutePosX() {
  Serial.print(F("   By the calibration, the maximum distance for X-Axis is ["));
  Serial.print(abs(maxDisX) / stepsForMM);
  Serial.println(F(" (mm)]"));
  Serial.print(F("   X-Axis (mm): "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  targetPosX = ((-1 * Serial.parseInt()) * stepsForMM) ;
  Serial.println(abs(targetPosX) / stepsForMM);
  if (targetPosX <= maxDisX) {
    Serial.println(F("   [Warning] Don't exceed the limitation. Re-try!"));
    AbsolutePosX();
  }
  stepper1.moveTo(targetPosX);

  if (stepper1.currentPosition() > targetPosX) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  }
  Serial.println(F(""));
  targetVelX = (directionState * stepperSpeed);
  AbsolutePos();
}
void AbsolutePosY() {
  Serial.print(F("   By the calibration, the maximum distance for Y-Axis is ["));
  Serial.print(abs(maxDisY) / stepsForMM);
  Serial.println(F(" (mm)]"));
  Serial.print(F("   Y-Axis (mm): "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  targetPosY = (-1 * Serial.parseInt() * stepsForMM);
  Serial.println(abs(targetPosY) / stepsForMM);
  if (targetPosY <= maxDisY) {
    Serial.println(F("   [Warning] Don't exceed the limitation. Re-try!"));
    AbsolutePosY();
  }
  stepper2.moveTo(targetPosY);

  if (stepper2.currentPosition() > targetPosY) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  }
  Serial.println(F(""));
  targetVelY = (directionState * stepperSpeed);
  AbsolutePos();
}
void AbsolutePosZ() {
  Serial.print(F("   By the calibration, the maximum distance for Z-Axis is ["));
  Serial.print(maxDisZ / stepsForMM);
  Serial.println(F(" (mm)]"));
  Serial.print(F("   Z-Axis (mm): "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  targetPosZ = (Serial.parseInt() * stepsForMM);
  Serial.println(targetPosZ / stepsForMM);
  if (targetPosZ >= maxDisZ) {
    Serial.println(F("   [Warning] Don't exceed the limitation. Re-try!"));
    AbsolutePosZ();
  }
  stepper3.moveTo(targetPosZ);

  if (stepper3.currentPosition() > targetPosZ) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  }
  Serial.println(F(""));
  targetVelZ = (-directionState * stepperSpeedZ);
  AbsolutePos();
}

void RelativePos() {
  Serial.println(F("※ [Setting State]"));
  Serial.println(F("   After setting all X, Y, and Z, please press '[4] Run' at the end"));
  Serial.println(F("   If you have any axis you don't want to change, then just leave it"));
  Serial.println(F("   [1] X-Axis"));
  Serial.println(F("   [2] Y-Axis"));
  Serial.println(F("   [3] Z-Axis"));
  Serial.println(F("   [4] Run"));
  Serial.println(F("   [5] Back to menu"));
  Serial.println(F(""));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  settingState = Serial.parseInt();

  if (settingState == 1 || settingState == 2 || settingState == 3 || settingState == 4 || settingState == 5) {
    switch (settingState) {
      case STATE3_X_AXIS:
        RelativePosX();
        break;
      case STATE3_Y_AXIS:
        RelativePosY();
        break;
      case STATE3_Z_AXIS:
        RelativePosZ();
        break;
      case STATE3_RUNALL:
        RunAll();
        break;
      case STATE3_BACKTOMENU:
        return;
        break;
    }
  } else {
    Serial.println(F("   [Warning] Please type the proper number. Re-try!"));
    Serial.println(F(""));
    RelativePos();
  }
}
void RelativePosX() {
  Serial.println(F("※ [Direction State] X-Axis"));
  Serial.println(F("   [1] CLOCKWISE         'Come close to the origin'"));
  Serial.println(F("   [2] COUNTER-CLOCKWISE 'Go far away from the origin'"));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  directionState = Serial.parseInt();

  if (directionState == direction_CW) {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  } else if (directionState == direction_CCW) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    Serial.println(F("   [Warning] Please type the proper number. Re-try!"));
    RelativePosX();
  }
  targetVelX = (directionState * stepperSpeed);

  Serial.print(F("   How much distance to move from the current position (mm): "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  targetDisX = (Serial.parseInt() * stepsForMM);
  Serial.println(targetDisX);
  Serial.println(F(""));

  targetPosX = (stepper1.currentPosition() + (directionState * targetDisX));
  if (abs(targetPosX) > abs(maxDisX)) {
    Serial.println(F("   [Warning] Please don't exceed the limitation. Re-try!"));
    RelativePosX();
  }
  stepper1.moveTo(targetPosX);
  RelativePos();
}
void RelativePosY() {
  Serial.println(F("※ [Direction State] Y-Axis"));
  Serial.println(F("   [1] CLOCKWISE         'Come close to the origin'"));
  Serial.println(F("   [2] COUNTER-CLOCKWISE 'Go far away from the origin'"));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  directionState = Serial.parseInt();

  if (directionState == direction_CW) {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  } else if (directionState == direction_CCW) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    Serial.println(F("   [Warning] Please type the proper number. Re-try!"));
    RelativePosY();
  }
  targetVelY = (directionState * stepperSpeed);
  Serial.print(F("   How much distance to move from the current position: "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  targetDisY = (Serial.parseInt() * stepsForMM);
  Serial.println(targetDisY);
  Serial.println(F(""));

  targetPosY = (stepper2.currentPosition() + (directionState * targetDisY));
  if (abs(targetPosY) > abs(maxDisY)) {
    Serial.println(F("   [Warning] Please don't exceed the limitation. Re-try!"));
    RelativePosY();
  }
  stepper2.moveTo(targetPosY);
  RelativePos();
}
void RelativePosZ() {
  Serial.println(F("※ [Direction State] Z-Axis"));
  Serial.println(F("   [1] CLOCKWISE         'Go far away from the origin'"));
  Serial.println(F("   [2] COUNTER-CLOCKWISE 'Come close to the origin'"));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  directionState = Serial.parseInt();

  if (directionState == direction_CW) {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  } else if (directionState == direction_CCW) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    Serial.println(F("   [Warning] Please type the proper number. Re-try!"));
    RelativePosZ();
  }
  targetVelZ = (-directionState * stepperSpeedZ);
  Serial.print(F("   How much distance to move from the current position: "));

  Serial.parseInt(); // Clear the buffer
  while (Serial.available() == 0) {}
  targetDisZ = (Serial.parseInt() * stepsForMM);
  Serial.println(targetDisZ);
  Serial.println(F(""));

  targetPosZ = (stepper3.currentPosition() + (directionState * targetDisZ));
  if (abs(targetPosZ) > abs(maxDisZ)) {
    Serial.println(F("   [Warning] Please don't exceed the limitation. Re-try!"));
    RelativePosZ();
  }
  stepper3.moveTo(targetPosZ);
  RelativePos();
}

void RunAll() {
  if (((targetPosX == prevPosX) || (targetPosX == 0)) && ((targetPosY == prevPosY) || (targetPosY == 0))) {
    goto JumpToZ;
  } else {
    // X-Axis and Y-Axis
    while ((stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(targetVelX);
      stepper2.setSpeed(targetVelY);
      stepper1.run();
      stepper2.run();
    }

    if ((stepper1.currentPosition() == targetPosX) || (targetPosX == 0)) {     // the second limit-switch detected is X-axis one
      Serial.print(F("   X-Axis Current Position: "));  //If this is in while loop, motor speed will be decreased
      Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisX) / stepsForMM);

      if ((targetPosY == prevPosY) || (targetPosY == 0)) {
        goto JumpToZ;
      } else {
        while (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) {
          stepper2.setSpeed(targetVelY);
          stepper2.run();
        }
        Serial.print(F("   Y-Axis Current Position: "));  //If this is in while loop, motor speed will be decreased
        Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
        Serial.print(F("/"));
        Serial.println(abs(maxDisY) / stepsForMM);
      }
    } else if (stepper2.currentPosition() == targetPosY || (targetPosY == 0)) {  // the second limit-switch detected is Y-axis one
      Serial.print(F("   Y-Axis Current Position: "));      //If this is in while loop, motor speed will be decreased
      Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisY) / stepsForMM);

      if ((targetPosX == prevPosX) || (targetPosX == 0)) {
        goto JumpToZ;
      } else {
        while (stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) {
          stepper1.setSpeed(targetVelX);
          stepper1.run();
        }
        Serial.print(F("   X-Axis Current Position: "));  //If this is in while loop, motor speed will be decreased
        Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
        Serial.print(F("/"));
        Serial.println(abs(maxDisX) / stepsForMM);
      }
    }
  }

  // Z-Axis
  JumpToZ:
  if ((targetPosZ == prevPosZ) || (targetPosZ == 0)) {
    return;
  } else {
    while (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0) {
      stepper3.setSpeed(targetVelZ);
      stepper3.run();
    }
    Serial.print(F("   Z-Axis Current Position: "));  //If this is in while loop, motor speed will be decreased
    Serial.print(abs(stepper3.currentPosition()) / stepsForMM);
    Serial.print(F("/"));
    Serial.println(maxDisZ / stepsForMM);
    Serial.println(F(""));
  }
  Serial.print(F(""));  
}

void ManualSet() {  // The reason why it needs to go back to the origin is to re-calculate the distancea as similiar as calibration function.
  // If 3-axis CNC machine has the encoder, it doesn't need this process. It will only need to move from the current possition with the shorter time.
  Serial.println(F("   [Manual Setting]"));
  Serial.println(F("※ Please move the machine to where you want it to be. Then, press 'R' once it's ready."));

  while (Ready != 'R') {
    Ready = Serial.read();
  }
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  digitalWrite(enablePin, LOW);  // Enable the motor pin

  // Move to the origin to measure the manual positions.
  while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper2.setSpeed(stepperSpeed);
    stepper3.setSpeed(stepperSpeedZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  if (digitalRead(limitSwitchX) == 0) {
    tempPosX = stepper1.currentPosition();
    stepper1.setCurrentPosition(0);
    while (stepper1.currentPosition() != safetyDis) {
      stepper1.setSpeed(-stepperSpeed);
      stepper1.run();
    }
    while (digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper2.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedZ);
      stepper2.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchY) == 0) {
      tempPosY = stepper2.currentPosition();
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedZ);
        stepper3.run();
      }
      tempPosZ = stepper3.currentPosition();
      stepper3.setCurrentPosition(0);
      while (stepper3.currentPosition() != safetyDisZ) {
        stepper3.setSpeed(-stepperSpeed);
        stepper3.run();
      }      
    } else {
      tempPosZ = stepper3.currentPosition();
      stepper3.setCurrentPosition(0);
      while (stepper3.currentPosition() != safetyDisZ) {
        stepper3.setSpeed(-stepperSpeed);
        stepper3.run();
      }      
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeed);
        stepper2.run();
      }
      tempPosY = stepper2.currentPosition();
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
    }
  } else if (digitalRead(limitSwitchY) == 0) {
    tempPosY = stepper2.currentPosition();
    stepper2.setCurrentPosition(0);
    while (stepper2.currentPosition() != safetyDis) {
      stepper2.setSpeed(-stepperSpeed);
      stepper2.run();
    }
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedZ);
      stepper1.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchX) == 0) {
      tempPosX = stepper1.currentPosition();
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedZ);
        stepper3.run();
      }
      tempPosZ = stepper3.currentPosition();
      stepper3.setCurrentPosition(0);
      while (stepper3.currentPosition() != safetyDisZ) {
        stepper3.setSpeed(-stepperSpeed);
        stepper3.run();
      }      
    } else {
      tempPosZ = stepper3.currentPosition();
      stepper3.setCurrentPosition(0);
      while (stepper3.currentPosition() != safetyDisZ) {
        stepper3.setSpeed(-stepperSpeed);
        stepper3.run();
      }      
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      tempPosX = stepper1.currentPosition();
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
    }
  } else {
    tempPosZ = stepper3.currentPosition();
    stepper3.setCurrentPosition(0);
    while (stepper3.currentPosition() != safetyDisZ) {
      stepper3.setSpeed(-stepperSpeedZ);
      stepper3.run();
    }
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper2.setSpeed(stepperSpeed);
      stepper1.run();
      stepper2.run();
    }
    if (digitalRead(limitSwitchX) == 0) {
      tempPosX = stepper1.currentPosition();
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeed);
        stepper2.run();
      }
      tempPosY = stepper2.currentPosition();
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
    } else {
      tempPosY = stepper2.currentPosition();
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      tempPosX = stepper1.currentPosition();
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
    }
  }

  // Move back to the manual positions after measurements.
  while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
    stepper1.setSpeed(-stepperSpeed);
    stepper2.setSpeed(-stepperSpeed);
    stepper3.setSpeed(-stepperSpeedZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  if (stepper1.currentPosition() == -tempPosX) {
    Serial.print(F("   X-Axis Manual Position: "));
    Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
    Serial.print(F("/"));
    Serial.println(abs(maxDisX) / stepsForMM);
    while ((stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper2.setSpeed(-stepperSpeed);
      stepper3.setSpeed(-stepperSpeedZ);
      stepper2.run();
      stepper3.run();
    }
    if (stepper2.currentPosition() == -tempPosY) {
      Serial.print(F("   Y-Axis Manual Position: "));
      Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisY) / stepsForMM);
      while (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedZ);
        stepper3.run();
      }
      Serial.print(F("   Z-Axis Manual Position: "));
      Serial.print(abs(stepper3.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisZ) / stepsForMM);
    } else {
      Serial.print(F("   Z-Axis Manual Position: "));
      Serial.print(abs(stepper3.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisZ) / stepsForMM);
      while (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
      Serial.print(F("   Y-Axis Manual Position: "));
      Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisY) / stepsForMM);
    }
  } else if (stepper2.currentPosition() == -tempPosY) {
    Serial.print(F("   Y-Axis Manual Position: "));
    Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
    Serial.print(F("/"));
    Serial.println(abs(maxDisY) / stepsForMM);
    while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper3.setSpeed(-stepperSpeedZ);
      stepper1.run();
      stepper3.run();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      Serial.print(F("   X-Axis Manual Position: "));
      Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisX) / stepsForMM);
      while (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedZ);
        stepper3.run();
      }
      Serial.print(F("   Z-Axis Manual Position: "));
      Serial.print(abs(stepper3.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisZ) / stepsForMM);
    } else {
      Serial.print(F("   Z-Axis Manual Position: "));
      Serial.print(abs(stepper3.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisZ) / stepsForMM);
      while (stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      Serial.print(F("   X-Axis Manual Position: "));
      Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisX) / stepsForMM);
    }
  } else {  // (stepper3.currentPosition() == -tempPosZ)
    Serial.print(F("   Z-Axis Manual Position: "));
    Serial.print(abs(stepper3.currentPosition()) / stepsForMM);
    Serial.print(F("/"));
    Serial.println(abs(maxDisZ) / stepsForMM);
    while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper2.setSpeed(-stepperSpeed);
      stepper1.run();
      stepper2.run();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      Serial.print(F("   X-Axis Manual Position: "));  //If this is in while loop, motor speed will be decreased
      Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisX) / stepsForMM);
      while (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
      Serial.print(F("   Y-Axis Manual Position: "));  //If this is in while loop, motor speed will be decreased
      Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisY) / stepsForMM);
    } else {
      Serial.print(F("   Y-Axis Manual Position: "));  //If this is in while loop, motor speed will be decreased
      Serial.print(abs(stepper2.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisY) / stepsForMM);
      while (stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      Serial.print(F("   X-Axis Manual Position: "));  //If this is in while loop, motor speed will be decreased
      Serial.print(abs(stepper1.currentPosition()) / stepsForMM);
      Serial.print(F("/"));
      Serial.println(abs(maxDisX) / stepsForMM);
    }
  }
}

void Rewinding() {  // The reason why it needs to go back to the origin is to reset and know where this motor was actually
  // since 'stepper1.currentPosition()' function cannot work itself to find its current position once the motor position is changed by the external elements.
  Serial.println(F("   [Rewinding]"));
  Serial.print(F("※ Previous Position ["));
  Serial.print(abs(prevPosX) / stepsForMM);
  Serial.print(F(", "));
  Serial.print(abs(prevPosY) / stepsForMM);
  Serial.print(F(", "));
  Serial.print(abs(prevPosZ) / stepsForMM);
  Serial.println(F("]"));
  Serial.println(F("   [Notice] Rewinding in process"));

  // Move to the origin to reset the origin
  while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper2.setSpeed(stepperSpeed);
    stepper3.setSpeed(stepperSpeedZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  if (digitalRead(limitSwitchX) == 0) {
    stepper1.setCurrentPosition(0);
    while (stepper1.currentPosition() != safetyDis) {
      stepper1.setSpeed(-stepperSpeed);
      stepper1.run();
    }
    while (digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper2.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedZ);
      stepper2.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchY) == 0) {
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedZ);
        stepper3.run();
      }
      stepper3.setCurrentPosition(0);
    } else {
      stepper3.setCurrentPosition(0);
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeed);
        stepper2.run();
      }
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
    }
  } else if (digitalRead(limitSwitchY) == 0) {
    stepper2.setCurrentPosition(0);
    while (stepper2.currentPosition() != safetyDis) {
      stepper2.setSpeed(-stepperSpeed);
      stepper2.run();
    }
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedZ);
      stepper1.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchX) == 0) {
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedZ);
        stepper3.run();
      }
      stepper3.setCurrentPosition(0);
    } else {
      stepper3.setCurrentPosition(0);
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
    }
  } else {
    stepper3.setCurrentPosition(0);
    while (stepper3.currentPosition() != safetyDisZ) {
      stepper3.setSpeed(-stepperSpeedZ);
      stepper3.run();
    }
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper2.setSpeed(stepperSpeed);
      stepper1.run();
      stepper2.run();
    }
    if (digitalRead(limitSwitchX) == 0) {
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeed);
        stepper2.run();
      }
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
    } else {
      stepper2.setCurrentPosition(0);
      while (stepper2.currentPosition() != safetyDis) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      stepper1.setCurrentPosition(0);
      while (stepper1.currentPosition() != safetyDis) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
    }
  }

  // Move back to the previous positions
  while ((stepper1.currentPosition() != prevPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != prevPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != prevPosZ && digitalRead(limitSwitchZ) != 0)) {
    stepper1.setSpeed(-stepperSpeed);
    stepper2.setSpeed(-stepperSpeed);
    stepper3.setSpeed(-stepperSpeedZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  if (stepper1.currentPosition() == prevPosX) {
    while ((stepper2.currentPosition() != prevPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != prevPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper2.setSpeed(-stepperSpeed);
      stepper3.setSpeed(-stepperSpeedZ);
      stepper2.run();
      stepper3.run();
    }
    if (stepper2.currentPosition() == prevPosY) {
      while (stepper3.currentPosition() != prevPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedZ);
        stepper3.run();
      }
    } else {
      while (stepper2.currentPosition() != prevPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
    }
  } else if (stepper2.currentPosition() == prevPosY) {
    while ((stepper1.currentPosition() != prevPosX && digitalRead(limitSwitchX) != 0) && (stepper3.currentPosition() != prevPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper3.setSpeed(-stepperSpeedZ);
      stepper1.run();
      stepper3.run();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      while (stepper3.currentPosition() != prevPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedZ);
        stepper3.run();
      }
    } else {
      while (stepper1.currentPosition() != prevPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
    }
  } else {  // (stepper3.currentPosition() == -tempPosZ)
    while ((stepper1.currentPosition() != prevPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != prevPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper2.setSpeed(-stepperSpeed);
      stepper1.run();
      stepper2.run();
    }
    if (stepper1.currentPosition() == -prevPosX) {
      while (stepper2.currentPosition() != prevPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeed);
        stepper2.run();
      }
    } else {
      while (stepper1.currentPosition() != prevPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
    }
  }
}