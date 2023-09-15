#include <AccelStepper.h>
//----------------------------------------------------------------------------------------------------------------------------------------------
// [Pre-Setting]
#define baudRate 115200
#define stepAng 1.8  //(degree) Step angle of the stepper motor
#define pitchLgth 4  //(mm) Pitch length of the lead screw
#define maxBuf 64    // Maximum number of buffer
#define timeInr 300  // (ms) Time interval sending the current position to Unity
#define arcInr 1     // (mm) Interval when generating straight lines to draw a arc motion
// Before uploading this code, please double-check few changable variables below here

// [Advanced-Setting]
long maxPosX = 14385, maxPosY = 17003, maxPosZ = 8375;  // Maximum distances on each axies from the offest measured in steps. This can change by the microstep.
// If you know it, no need to do calibration. Otherwise, let them to be 0 and then do calibration
//----------------------------------------------------------------------------------------------------------------------------------------------

// [Codes]
// 0. G-code
#define STATE1_RAPID_POSITIONING 0
#define STATE1_LINEAR_DRAWING 1
#define STATE1_CLOCKWISE_ARC 2
#define STATE1_COUNTER_CLOCKWISE_ARC 3
#define STATE1_INCH_UNITS 20
#define STATE1_MILLIMETER_UNITS 21
#define STATE1_AUTO_HOME 28
#define STATE1_AUTO_CALIBRATION 33
#define STATE1_SAVE_CURRENT_POSITION 60
#define STATE1_RETURN_TO_SAVED_POSITION 61
#define STATE1_ABSOLUTE_POSITINING 90
#define STATE1_RELATIVE_POSITINING 91

#define STATE2_ABSOLUTE_POSITINING 0  // Absolute positioning is the default
#define STATE2_RELATIVE_POSITINING 1

// 1. M-code
#define STATE1_ENABLE_STEPPERS 17
#define STATE1_DISABLE_STEPPERS 18
#define STATE1_GET_CURRENT_POSITION 114
#define STATE1_CHECK_MAXIMUM_DISTANCE 115
#define STATE1_SET_ADVANCED_SETTINGS 205

// [Globals]
// 0. Allocate the pins we will use
AccelStepper stepper1(1, 3, 6);  // (Type:driver, STEP, DIR) // X-axis
AccelStepper stepper2(1, 2, 5);  // (Type: Driver, Step, DIR) // Y-axis
AccelStepper stepper3(1, 4, 7);  // (Type:driver, STEP, DIR) // Z-axis
const int enablePin = 8;
const int limitSwitchX = 10, limitSwitchY = 9, limitSwitchZ = 11;

// 1. Set the variables for the operation
long targetPosX = 0, targetPosY = 0, targetPosZ = 0;
long targetVelX = 0, targetVelY = 0, targetVelZ = 0;
long maxPos1X = 0, maxPos1Y = 0, maxPos1Z = 0;  // 1st trial
long maxPos2X = 0, maxPos2Y = 0, maxPos2Z = 0;  // 2nd trial
double maxDisX = 0, maxDisY = 0, maxDisZ = 0;   // abs(MaxPos) / stesPerMM
long currPosX = 0, currPosY = 0, currPosZ = 0;
double currDisX = 0, currDisY = 0, currDisZ = 0;
long prevPosX = 0, prevPosY = 0, prevPosZ = 0;
long tempPosX = 0, tempPosY = 0, tempPosZ = 0;
char ready = 0;
long newMaxSpeed = 0, newSpeed = 0, newAcceleration = 0;
long stepperMaxSpeed = 0;                  // stepperSpeed = steps / second
long stepperSpeed = 0, stepperSpeedR = 0;  // Because of how the stepper motor is set. Need to put the minus in front of the value of 'stepperSpeed'
long stepperAcceleration = 0;              // stepperAcceleration = steps / (second)^2
unsigned int stepperMinPulseWidth = 20;
long safetyDis = 0, safetyDisR = 0;  // Distance from the limtiswitch

// 2. Changes depending on the measurement unit
double stepsPerRev_Motor = 0;  // Only consider the stepper motor specification
double stepsPerRev_Micro = 0;  // Consideration all together with the microstep setting
// 2-1. Convert the measurement unit from steps(pulses) to millimeter(mm)
double revPerMM = 0;
double stepsPerMM = 0;  // (M92) Steps per Millimeter - leadscrew driven systems
// 2-2. Convert the measurement unit from steps(pulses) to inch(in)
double revPerIN = 0;
double stepsPerIN = 0;

// 3. Interprete the incoming Gcode
// Refers to (https://github.com/MarginallyClever/GcodeCNCDemo)
String string;               // Use String class to use serial.readString()
int curBuf = 0;              // Current number of buffer
char array[maxBuf] = { 0 };  // Re-assmble each character of the received data in the array
char command = 0;            // Only number information after deleting alphabet on the code
int posMode = 0;             // Position mode: either absolute positioning or either relative positioning
double retVal = 0;           // Variable reffering to the return value
int stoAxn = -1;             // Storage to save the previous action was either G00 or G01 so that after G00 or G01 is inserted, Arduino knows immediatly whenever when \only the information about X, Y, and Z,
const int noLetr = -1;       // This is always -1 to check whether the letter is found or not by using the function 'Finding()'
int retMed = 1;

// 4. Generate the Gcode of the current position to address it to Unity
char M114[5] = "M114", M115[5] = "M115";
char *space = " ";
char *strDis = malloc(sizeof(char) * maxBuf);
char *X = "X", *Y = "Y", *Z = "Z";  // Append the letters to the array
char str[maxBuf] = { 0 };           // To store the value of current positions

// 5. TimerPos to execute specific code every (=timeInr) seconds and run everything else on loop
// Refers to (https://forum.arduino.cc/t/timer-to-execute-specific-code-every-2-seconds-and-run-everything-else-on-loop/315935)
unsigned long prevMS = 0;

// 6. G02 && G03 Arc Motion: 1st, Find the center of a circle given two points and a radius
// Refers to (https://math.stackexchange.com/questions/1781438/finding-the-center-of-a-circle-given-two-points-and-a-radius-algebraically)
int initPosX = 0, initPosY = 0, initPosYR = 0, initPosZ = 0, endPosX = 0, endPosY = 0, endPosYR = 0, endPosZ = 0;  // x_1, y_1, -y_1, z_1, x_2, y_2, -y_2, z_2
double offsetX = 0, offsetY = 0, radius = 0;                                                                       // I, J, R
double diffPosX = 0, diffPosY = 0, simpValC = 0, simpValCy = 0;                                                    // x_3, y_3, C, Cy (simplified variable of Cy)
double coefQdaA = 0, coefLinB = 0, coefCstC = 0;                                                                   // A: the coefficient of the quadratic term, B: the coefficient of the linear term, C: the constant term
double ctrPosX1 = 0, ctrPosY1 = 0, ctrPosX2 = 0, ctrPosY2 = 0;
double halfLenX = 0, halfLenY = 0;
double arcPosX = 0, arcPosY = 0;  // Variabels to make the for loop by adding number 1 every loop

// 7. Bresenham's line algorithm
// Refers to (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)

void setup() {  // Set initial seed values for the stepper
  pinMode(enablePin, OUTPUT);
  pinMode(limitSwitchX, INPUT_PULLUP);
  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);

  Serial.begin(baudRate);

  SettingDef();

  stepper1.setMaxSpeed(stepperMaxSpeed);
  stepper1.setSpeed(stepperSpeed);
  stepper1.setAcceleration(stepperAcceleration);
  stepper1.setMinPulseWidth(stepperMinPulseWidth);  // Prevent more steps drift due to the pulse width by TB6600 driver

  stepper2.setMaxSpeed(stepperMaxSpeed);
  stepper2.setSpeed(stepperSpeedR);
  stepper2.setAcceleration(stepperAcceleration);
  stepper2.setMinPulseWidth(stepperMinPulseWidth);

  stepper3.setMaxSpeed(stepperMaxSpeed);
  stepper3.setSpeed(stepperSpeedR);
  stepper3.setAcceleration(stepperAcceleration);
  stepper3.setMinPulseWidth(stepperMinPulseWidth);

  digitalWrite(enablePin, LOW);  // Enable
}

void loop() {  // Main interface
  prevPosX = stepper1.currentPosition();
  prevPosY = stepper2.currentPosition();
  prevPosZ = stepper3.currentPosition();

  if (Serial.available() > 0) {
    String received = Serial.readString();
    received.toCharArray(array, maxBuf);
    curBuf = strlen(array);
    if (array[curBuf - 1] == '\n') {  // If the entire message is received, then
      Decoding();
    }
  }
}

void SettingDef() {  // Pre-set the default microstep setting
  int microstepState = 0;
  Serial.parseInt();
  while (Serial.available() == 0) {}
  microstepState = Serial.parseInt();

  if (microstepState == 1 || microstepState == 2 || microstepState == 4 || microstepState == 8 || microstepState == 16 || microstepState == 32) {
    revPerMM = ((double)1 / pitchLgth);
    stepsPerRev_Motor = (360 / stepAng);
    stepsPerRev_Micro = (stepsPerRev_Motor * microstepState);
    stepsPerMM = (stepsPerRev_Micro * revPerMM);
    Serial.print(stepsPerMM);

    switch (microstepState) {
      case 1:
        stepperMaxSpeed = 1000;
        stepperSpeed = 600;  // 600 steps per second
        stepperAcceleration = 600;
        break;

      case 2:
        stepperMaxSpeed = 2000;
        stepperSpeed = 1200;
        stepperAcceleration = 1200;
        break;

      case 4:
        stepperMaxSpeed = 4000;
        stepperSpeed = 2400;
        stepperAcceleration = 2400;
        break;

      case 8:
        stepperMaxSpeed = 8000;
        stepperSpeed = 4800;
        stepperAcceleration = 4800;
        break;

      case 16:
        stepperMaxSpeed = 16000;
        stepperSpeed = 9600;
        stepperAcceleration = 9600;
        break;

      case 32:
        stepperMaxSpeed = 32000;
        stepperSpeed = 19200;
        stepperAcceleration = 19200;
        break;
    }
    safetyDis = (-1 * stepperSpeed / 5);
    stepperSpeedR = (-1 * stepperSpeed);
    safetyDisR = (-1 * safetyDis);
  } else {
    SettingDef();
  }
}

void Decoding() {  // Understand the command and seperate the word 'G', 'M', 'F', and 'S'
  // 0. G-code
  if ((Finding('G', noLetr) == noLetr) && (Finding('M', noLetr) == noLetr) && (stoAxn == 0 || stoAxn == 1 || stoAxn == 2 || stoAxn == 3)) {  // No G and M alphabet in the sentence but, if stoAxn == 0 or 1, then goto dcd_Gcode
    command = stoAxn;
    if (digitalRead(enablePin) == 0) {
      if (maxPosX == 0 || maxPosY == 0 || maxPosZ == 0) {
        Serial.println(F("[Warning] Calibration"));
      } else {
        switch (command) {
          case STATE1_RAPID_POSITIONING:
            switch (posMode) {
              case STATE2_ABSOLUTE_POSITINING:
                AbsLinPos();
                Positioning();
                break;
              case STATE2_RELATIVE_POSITINING:
                RelLinPos();
                Positioning();
                break;
            }
            break;
          case STATE1_LINEAR_DRAWING:
            switch (posMode) {
              case STATE2_ABSOLUTE_POSITINING:
                AbsLinPos();
                plotLine(initPosX, initPosY, endPosX, endPosY);
                break;
              case STATE2_RELATIVE_POSITINING:
                RelLinPos();
                plotLine(initPosX, initPosY, endPosX, endPosY);
                break;
            }
            break;
          case STATE1_CLOCKWISE_ARC:
            switch (posMode) {
              case STATE2_ABSOLUTE_POSITINING:
                AbsArcPosCW();
                break;
              case STATE2_RELATIVE_POSITINING:
                RelArcPosCW();
                break;
            }
            break;
          case STATE1_COUNTER_CLOCKWISE_ARC:
            switch (posMode) {
              case STATE2_ABSOLUTE_POSITINING:
                AbsArcPosCCW();
                break;
              case STATE2_RELATIVE_POSITINING:
                RelArcPosCCW();
                break;
            }
            break;
        }
      }
    } else {
      Serial.println(F("[Warning] Disabled"));
    }
  }

  if ((command = Finding('G', noLetr)) && (Finding('M', noLetr) == noLetr) && ((Finding('G', noLetr) != 0) || (Finding('G', noLetr) != 1))) {  // After finding 'G', the return value will be stored in 'command'.
    switch (command) {
      case STATE1_INCH_UNITS:
        Serial.println("inch");
        break;
      case STATE1_MILLIMETER_UNITS:
        Serial.println("mm");
        break;
      case STATE1_AUTO_HOME:
        if (digitalRead(enablePin) == 0) {
          Homing();
        } else {
          Serial.println(F("[Error] Disabled"));
          return;
        }
        break;
      case STATE1_AUTO_CALIBRATION:
        if (digitalRead(enablePin) == 0) {
          Calibrating();
        } else {
          Serial.println(F("[Error] Disabled"));
          return;
        }
        break;
      case STATE1_SAVE_CURRENT_POSITION:
        ManualSet();
        break;
      case STATE1_RETURN_TO_SAVED_POSITION:
        Rewinding();
        break;
      case STATE1_ABSOLUTE_POSITINING:
        posMode = STATE2_ABSOLUTE_POSITINING;
        break;
      case STATE1_RELATIVE_POSITINING:
        posMode = STATE2_RELATIVE_POSITINING;
        break;
    }
  }

  // 1. M-code
  else if (command = Finding('M', noLetr)) {
    switch (command) {
      case STATE1_ENABLE_STEPPERS:
        digitalWrite(enablePin, LOW);
        break;
      case STATE1_DISABLE_STEPPERS:
        digitalWrite(enablePin, HIGH);
        break;
      case STATE1_GET_CURRENT_POSITION:  // To communicate with Unity. This is little different feature from Marlin Firmware
        TimerPos();
        break;
      case STATE1_CHECK_MAXIMUM_DISTANCE:  // To communicate with Unity. This is little different feature from Marlin Firmware
        Ranging();
        break;
      case STATE1_SET_ADVANCED_SETTINGS:  // M203 Set Max Feedrate, M204 Set Starting Acceleration, M205 Set Advanced Settings
        Updating();
        break;
    }
  }
  curBuf = 0;
}

double Finding(char letter, double medium) {
  char *pointer = array;
  retMed = 1;  // Reset the variable 'retMed' to be 0
  while ((long)pointer > 1 && (*pointer) && (long)pointer < ((long)array + curBuf)) {
    if (*pointer == letter) {  // Check whether Arduino found one of characters such as X, Y, or Z. If yes, changes ret(characters) to be 1
      retVal = atof(pointer + 1);

      if (letter == 'G' && (retVal == 0 || retVal == 1 || retVal == 2 || retVal == 3)) {  // Return zero succesfully to 'Decoding();' function when G00 or G0 came
        stoAxn = retVal;
      } else {
        if ((letter == 'X' || letter == 'Y' || letter == 'Z') && retVal == 0) {
          return 'e';
        } else {
          return retVal;  // If the letter we are looking for is found, then return the double number only
        }
      }
    }
    pointer = (strchr(pointer, ' ') + 1);  // By using the white space as the seperator, it can know and read the position of the letter till the end of the array
  }
  retMed = medium + 1;
  return medium;  // Nothing found on the sentences
}

void Updating() {  // Updating the constant speed and the acceleration for once
  Serial.println(F("   [Updating]"));
  Serial.print(F("   Default maximum speed:"));
  Serial.println(stepperMaxSpeed);
  Serial.print(F("   Present maximum speed:"));
  if (newMaxSpeed == 0) {
    newMaxSpeed = stepperMaxSpeed;
  }
  Serial.println(newMaxSpeed);
  Serial.print("   New maximum speed value will be set as: ");  //confirm update by message

  Serial.parseFloat();
  while (Serial.available() == 0) {}
  newMaxSpeed = Serial.parseFloat();
  stepper1.setMaxSpeed(newMaxSpeed);  //update the value of the variable
  Serial.println(newMaxSpeed);
  Serial.println(F(""));

  Serial.print(F("   Default constant speed:"));
  Serial.println(stepperSpeed);
  Serial.print(F("   Present constant speed:"));
  if (newSpeed == 0) {
    newSpeed = stepperSpeed;
  }
  Serial.println(newSpeed);
  Serial.print("   New constant speed value will be set as: ");  //confirm update by message
  Serial.parseFloat();
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
  Serial.parseFloat();
  while (Serial.available() == 0) {}
  newAcceleration = Serial.parseFloat();      //receive the acceleration from serial
  stepper1.setAcceleration(newAcceleration);  //update the value of the variable
  Serial.println(newAcceleration);
  Serial.println(F(""));
}

void Homing() {                              // Move the slider to the initial position - homing
  if (Finding('X', noLetr) == noLetr) {      // No X letter in the sentence
    if (Finding('Y', noLetr) == noLetr) {    // No Y letter in the sentence
      if (Finding('Z', noLetr) == noLetr) {  // No Z letter in the sentence
        goto hom_XYZ;
      } else {
        goto hom_Z;
      }
    } else {
      if (Finding('Z', noLetr) == noLetr) {
        goto hom_Y;
      } else {
        goto hom_YZ;
      }
    }
  } else {
    if (Finding('Y', noLetr) == noLetr) {
      if (Finding('Z', noLetr) == noLetr) {
        goto hom_X;
      } else {
        goto hom_XZ;
      }
    } else {
      goto hom_XY;
    }
  }


hom_XYZ:
  // Run till the first limit-switch is detected
  while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper2.setSpeed(stepperSpeedR);
    stepper3.setSpeed(stepperSpeedR);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  // Run till the second limit-switch is detected
  if (digitalRead(limitSwitchX) == 0) {  // 1st case: the first limit-switch detected is X-axis one
    MoveBackX(safetyDis, -stepperSpeed);
    Generating();
hom_YZ:
    while (digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper2.setSpeed(stepperSpeedR);
      stepper3.setSpeed(stepperSpeedR);
      stepper2.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchY) == 0) {  // the second limit-switch detected is Y-axis one
      MoveBackY(safetyDisR, -stepperSpeedR);
      Generating();
hom_Z:
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedR);
        stepper3.run();
      }
      MoveBackZ(safetyDisR, -stepperSpeedR);
      Generating();
    } else {  // the second limit-switch detected is Z-axis one
      MoveBackZ(safetyDisR, -stepperSpeedR);
      Generating();
hom_Y:
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeedR);
        stepper2.run();
      }
      MoveBackY(safetyDisR, -stepperSpeedR);
      Generating();
    }
  } else if (digitalRead(limitSwitchY) == 0) {  // 2nd case: the first limit-switch detected is Y-axis one
    MoveBackY(safetyDisR, -stepperSpeedR);
    Generating();
hom_XZ:
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedR);
      stepper1.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchX) == 0) {  // the second limit-switch detected is X-axis one
      MoveBackX(safetyDis, -stepperSpeed);
      Generating();
      while (digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(stepperSpeedR);
        stepper3.run();
      }
      MoveBackZ(safetyDisR, -stepperSpeedR);
      Generating();
    } else {  // the second limit-switch detected is Z-axis one
      MoveBackZ(safetyDisR, -stepperSpeedR);
      Generating();
hom_X:
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      MoveBackX(safetyDis, -stepperSpeed);
      Generating();
    }
  } else {  // 3rd case: the first limit-switch detected is Z-axis one
    MoveBackZ(safetyDisR, -stepperSpeedR);
    Generating();
hom_XY:
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper2.setSpeed(stepperSpeedR);
      stepper1.run();
      stepper2.run();
    }
    if (digitalRead(limitSwitchX) == 0) {  // the second limit-switch detected is X-axis one
      MoveBackX(safetyDis, -stepperSpeed);
      Generating();
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeedR);
        stepper2.run();
      }
      MoveBackY(safetyDisR, -stepperSpeedR);
      Generating();
    } else {  // the second limit-switch detected is Y-axis one
      MoveBackY(safetyDisR, -stepperSpeedR);
      Generating();
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      MoveBackX(safetyDis, -stepperSpeed);
      Generating();
    }
  }
}
void Calibrating() {
  if (stepper1.currentPosition() != 0 || stepper2.currentPosition() != 0 || stepper3.currentPosition() != 0) {
    Serial.println(F("[Warning] Homing"));
    return;
  }

  // X-Axis
  // Move the slider to the right-hand position
  while (digitalRead(limitSwitchX) != 0) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
  maxPos1X = stepper1.currentPosition();
  MoveBackX(-safetyDis, stepperSpeed);
  // Move the slider to the left-hand position
  while (digitalRead(limitSwitchX) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }
  maxPos2X = stepper1.currentPosition();
  MoveBackX(safetyDis, -stepperSpeed);
  maxPosX = -1 * ((abs(maxPos1X) + abs(maxPos2X)) / 2);
  maxDisX = (abs(maxPosX) - 2 * safetyDis) / stepsPerMM;

  // Y-Axis
  while (digitalRead(limitSwitchY) != 0) {
    stepper2.setSpeed(-stepperSpeedR);
    stepper2.run();
  }
  maxPos1Y = stepper2.currentPosition();
  MoveBackY(-safetyDisR, stepperSpeedR);
  while (digitalRead(limitSwitchY) != 0) {
    stepper2.setSpeed(stepperSpeedR);
    stepper2.run();
  }
  maxPos2Y = stepper2.currentPosition();
  MoveBackY(safetyDisR, -stepperSpeedR);
  maxPosY = ((abs(maxPos1Y) + abs(maxPos2Y)) / 2);
  maxDisY = (abs(maxPosY) - 2 * safetyDisR) / stepsPerMM;

  // Z-Axis
  while (digitalRead(limitSwitchZ) != 0) {
    stepper3.setSpeed(-stepperSpeedR);
    stepper3.run();
  }
  maxPos1Z = stepper3.currentPosition();
  MoveBackZ(-safetyDisR, stepperSpeedR);
  // Move the slider to the left-hand position
  while (digitalRead(limitSwitchZ) != 0) {
    stepper3.setSpeed(stepperSpeedR);
    stepper3.run();
  }
  maxPos2Z = stepper3.currentPosition();
  MoveBackZ(safetyDisR, -stepperSpeedR);
  maxPosZ = ((abs(maxPos1Z) + abs(maxPos2Z)) / 2);
  maxDisZ = (abs(maxPosZ) - 2 * safetyDisR) / stepsPerMM;

  Ranging();
}
void MoveBackX(long distance, double speed) {
  stepper1.setCurrentPosition(0);  // When the limit switch is pressed, set the position to 0 steps
  while (stepper1.currentPosition() != distance) {
    stepper1.setSpeed(speed);
    stepper1.run();
  }
  stepper1.setCurrentPosition(0);
}
void MoveBackY(long distance, double speed) {
  stepper2.setCurrentPosition(0);
  while (stepper2.currentPosition() != distance) {
    stepper2.setSpeed(speed);
    stepper2.run();
  }
  stepper2.setCurrentPosition(0);
}
void MoveBackZ(long distance, double speed) {
  stepper3.setCurrentPosition(0);
  while (stepper3.currentPosition() != distance) {
    stepper3.setSpeed(speed);
    stepper3.run();
  }
  stepper3.setCurrentPosition(0);
}

void AbsLinPos() {
  int directionState = 0;
  const int direction_CW = 1, direction_CCW = -1;
  // X-Axis
  initPosX = stepper1.currentPosition();
  if (Finding('X', noLetr) == noLetr) {
    endPosX = prevPosX;
  } else if (Finding('X', noLetr) == 'e') {
    endPosX = 0;
  } else {
    endPosX = (-1) * Finding('X', noLetr) * stepsPerMM;
  }
  if (abs(endPosX) > abs(maxPosX)) {
    Serial.println(F("[Warning] X"));
    return;
  }
  stepper1.moveTo(endPosX);
  if (initPosX > endPosX) {
    directionState = direction_CCW;
  } else {
    directionState = direction_CW;
  }
  targetVelX = (directionState * stepperSpeed);

  // Y-Axis
  initPosY = stepper2.currentPosition();
  if (Finding('Y', noLetr) == noLetr) {
    endPosY = prevPosY;
  } else if (Finding('Y', noLetr) == 'e') {
    endPosY = 0;
  } else {
    endPosY = Finding('Y', noLetr) * stepsPerMM;
  }
  if (abs(endPosY) > abs(maxPosY)) {
    Serial.println(F("[Warning] Y"));
    return;
  }
  stepper2.moveTo(endPosY);
  if (initPosY > endPosY) {
    directionState = direction_CCW;
  } else {
    directionState = direction_CW;
  }
  targetVelY = (-directionState * stepperSpeedR);

  // Z-Axis
  initPosZ = stepper3.currentPosition();
  if (Finding('Z', noLetr) == noLetr) {
    endPosZ = prevPosZ;
  } else if (Finding('Z', noLetr) == 'e') {
    endPosZ = 0;
  } else {
    endPosZ = (Finding('Z', noLetr) * stepsPerMM);
  }
  if (endPosZ > maxPosZ) {
    Serial.println(F("[Warning] Z"));
    return;
  }
  stepper3.moveTo(endPosZ);
  if (initPosZ > endPosZ) {
    directionState = direction_CCW;
  } else {
    directionState = direction_CW;
  }
  targetVelZ = (-directionState * stepperSpeedR);
}
void RelLinPos() {
  int directionState = 0;
  const int direction_CW = 1, direction_CCW = -1;
  // X-Axis
  initPosX = stepper1.currentPosition();
  if (Finding('X', noLetr) <= 0) {
    directionState = direction_CW;
  } else {
    directionState = direction_CCW;
  }
  targetVelX = (directionState * stepperSpeed);
  if (Finding('X', noLetr) && retMed == 0) {
    endPosX = prevPosX;
  } else {
    endPosX = (initPosX + (-Finding('X', noLetr) * stepsPerMM));
  }
  Serial.println(endPosX);
  if (abs(endPosX) > abs(maxPosX)) {
    Serial.println(F("[Warning] X"));
    return;
  }
  stepper1.moveTo(endPosX);

  // Y-Axis
  initPosY = stepper2.currentPosition();
  if (Finding('Y', noLetr) >= 0) {
    directionState = direction_CW;
  } else {
    directionState = direction_CCW;
  }
  targetVelY = (-directionState * stepperSpeedR);
  if (Finding('Y', noLetr) && retMed == 0) {
    endPosY = prevPosY;
  } else {
    endPosY = (initPosY + (Finding('Y', noLetr) * stepsPerMM));
  }
  Serial.println(endPosY);
  if (abs(endPosY) > abs(maxPosY)) {
    Serial.println(F("[Warning] Y"));
    return;
  }
  stepper2.moveTo(endPosY);

  // Z-Axis
  initPosZ = stepper3.currentPosition();
  if (Finding('Z', noLetr) >= 0) {
    directionState = direction_CW;
  } else {
    directionState = direction_CCW;
  }
  targetVelZ = (-directionState * stepperSpeedR);
  if (Finding('Z', noLetr) && retMed == 0) {
    endPosZ = prevPosZ;
  } else {
    endPosZ = (initPosZ + (Finding('Z', noLetr) * stepsPerMM));
  }
  Serial.println(endPosZ);
  if (abs(endPosZ) > abs(maxPosZ)) {
    Serial.println(F("[Warning] Z"));
    return;
  }
  stepper3.moveTo(endPosZ);
}

void Positioning() {  // Run X, Y, and Z all together
  while ((stepper1.currentPosition() != endPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != endPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != endPosZ && digitalRead(limitSwitchZ) != 0)) {
    stepper1.setSpeed(targetVelX);
    stepper2.setSpeed(targetVelY);
    stepper3.setSpeed(targetVelZ);
    stepper1.run();
    stepper2.run();
    stepper3.run();
    TimerPos();
  }

  if (stepper1.currentPosition() == endPosX) {  // 1st case
    Generating();
    while ((stepper2.currentPosition() != endPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != endPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper2.setSpeed(targetVelY);
      stepper3.setSpeed(targetVelZ);
      stepper2.run();
      stepper3.run();
      TimerPos();
    }
    if (stepper2.currentPosition() == endPosY) {
      Generating();
      while (stepper3.currentPosition() != endPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(targetVelZ);
        stepper3.run();
        TimerPos();
      }
      Generating();
    } else {
      Generating();
      while (stepper2.currentPosition() != endPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(targetVelY);
        stepper2.run();
        TimerPos();
      }
      Generating();
    }
  } else if (stepper2.currentPosition() == endPosY) {  // 2nd case
    Generating();
    while ((stepper1.currentPosition() != endPosX && digitalRead(limitSwitchX) != 0) && (stepper3.currentPosition() != endPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper1.setSpeed(targetVelX);
      stepper3.setSpeed(targetVelZ);
      stepper1.run();
      stepper3.run();
      TimerPos();
    }
    if (stepper1.currentPosition() == endPosX) {
      Generating();
      while (stepper3.currentPosition() != endPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(targetVelZ);
        stepper3.run();
        TimerPos();
      }
      Generating();
    } else {
      Generating();
      while (stepper1.currentPosition() != endPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(targetVelX);
        stepper1.run();
        TimerPos();
      }
      Generating();
    }
  } else {  // 3rd case
    Generating();
    while ((stepper1.currentPosition() != endPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != endPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(targetVelX);
      stepper2.setSpeed(targetVelY);
      stepper1.run();
      stepper2.run();
      TimerPos();
    }
    if (stepper1.currentPosition() == endPosX) {
      Generating();
      while (stepper2.currentPosition() != endPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(targetVelY);
        stepper2.run();
        TimerPos();
      }
      Generating();
    } else {
      Generating();
      while (stepper1.currentPosition() != endPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(targetVelX);
        stepper1.run();
        TimerPos();
      }
      Generating();
    }
  }
}

void plotLine(int iPosX, int iPosY, int ePosX, int ePosY) {
  if (abs(ePosY - iPosY) < abs(ePosX - iPosX)) {  // slope(= dX/dY) is less than 1 but, larger than 0
    if (-iPosX > -ePosX) {
      plotLineLeft(-ePosX, ePosY, -iPosX, iPosY);  // Right to Left
    } else {
      plotLineRight(-iPosX, iPosY, -ePosX, ePosY);  // Left to Right
    }
  } else {  // slope is equal to 1 or larger than 1
    if (iPosY > ePosY) {
      plotLineBottom(-ePosX, ePosY, -iPosX, iPosY);  // Top to Bottom
    } else {
      plotLineTop(-iPosX, iPosY, -ePosX, ePosY);  // Bottom to Top
    }
  }
}
void plotLineLeft(int i_X, int i_Y, int e_X, int e_Y) {
  int dX = e_X - i_X, dY = e_Y - i_Y, Yi = 1;
  if (dY < 0) {  // Top to bottom
    Yi = -1, dY = -dY;
  }  // else: Bottom to Top
  int D = (2 * dY) - dX, X = e_X, Y = e_Y;
  for (X; X >= i_X; X = X - 1) {
    OperatingXY(-X, Y);
    TimerPos();
    if (D > 0) {
      Y = Y - Yi;
      D = D + (2 * (dY - dX));
    } else {
      D = D + 2 * dY;
    }
  }
  Generating();
  OperatingZ(endPosZ);
}
void plotLineRight(int i_X, int i_Y, int e_X, int e_Y) {
  int dX = e_X - i_X, dY = e_Y - i_Y, Yi = 1;
  if (dY < 0) {  // Top to bottom
    Yi = -1, dY = -dY;
  }  // else: Bottom to Top
  int D = (2 * dY) - dX, X = i_X, Y = i_Y;
  for (X; X <= e_X; X = X + 1) {
    OperatingXY(-X, Y);
    TimerPos();
    if (D > 0) {
      Y = Y + Yi;
      D = D + (2 * (dY - dX));
    } else {
      D = D + 2 * dY;
    }
  }
  Generating();
  OperatingZ(endPosZ);
}
void plotLineBottom(int i_X, int i_Y, int e_X, int e_Y) {
  int dX = e_X - i_X, dY = e_Y - i_Y, Xi = 1;
  if (dX < 0) {  // Right to Left
    Xi = -1, dX = -dX;
  }  // else: Left to Right
  int D = (2 * dX) - dY, X = e_X, Y = e_Y;
  for (Y; Y >= i_Y; Y = Y - 1) {
    OperatingXY(-X, Y);
    TimerPos();
    if (D > 0) {
      X = X - Xi;
      D = D + (2 * (dX - dY));
    } else {
      D = D + 2 * dX;
    }
  }
  Generating();
  OperatingZ(endPosZ);
}
void plotLineTop(int i_X, int i_Y, int e_X, int e_Y) {
  int dX = e_X - i_X, dY = e_Y - i_Y, Xi = 1;
  if (dX < 0) {  // Right to Left
    Xi = -1, dX = -dX;
  }  // else: Left to Right
  int D = (2 * dX) - dY, X = i_X, Y = i_Y;
  for (Y; Y <= e_Y; Y = Y + 1) {
    OperatingXY(-X, Y);
    TimerPos();
    if (D > 0) {
      X = X + Xi;
      D = D + (2 * (dX - dY));
    } else {
      D = D + 2 * dX;
    }
  }
  Generating();
  OperatingZ(endPosZ);
}
void OperatingXY(int targetPosX, int targetPosY) {  // Run X and Y first and then, Z
  stepper1.moveTo(targetPosX);
  stepper2.moveTo(targetPosY);

  // X-Axis and Y-Axis
  while ((stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0)) {
    stepper1.setSpeed(targetVelX);
    stepper2.setSpeed(targetVelY);
    stepper1.run();
    stepper2.run();
  }

  if ((stepper1.currentPosition() == targetPosX) || (targetPosX == 0)) {
    while (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) {
      stepper2.setSpeed(targetVelY);
      stepper2.run();
    }
  } else if (stepper2.currentPosition() == targetPosY || (targetPosY == 0)) {
    while (stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) {
      stepper1.setSpeed(targetVelX);
      stepper1.run();
    }
  }
}
void OperatingZ(int targetPosZ) {
  if ((targetPosZ == prevPosZ) || (targetPosZ == 0)) {
    return;
  } else {
    while (stepper3.currentPosition() != targetPosZ && digitalRead(limitSwitchZ) != 0) {
      stepper3.setSpeed(targetVelZ);
      stepper3.run();
      TimerPos();
    }
    Generating();
  }
}

void AbsArcPosCW() {
  initPosX = 0, initPosY = 0;
  CenterPos();
}
void AbsArcPosCCW() {
  initPosX = 0, initPosY = 0;
  CenterPos();
}
void RelArcPosCW() {
  initPosX = (stepper1.currentPosition() / stepsPerMM), initPosY = (stepper2.currentPosition() / stepsPerMM), initPosYR = (-1) * initPosY;
  CenterPos();

  if (initPosX < endPosX) {  // 1. left-side startPosX1 is less than right-side startPosX2
    if (initPosYR < endPosYR) {
      for (arcPosX = initPosX; arcPosX <= endPosX; arcPosX = arcPosX + arcInr) {
        arcPosY = sqrt(pow(radius, 2) - pow((arcPosX - ctrPosX1), 2)) + ctrPosY1;
        ArcMotionXY(arcPosX, -arcPosY);
      }
    } else if (initPosYR > endPosY) {
      for (arcPosX = initPosX; arcPosX <= endPosX; arcPosX = arcPosX + arcInr) {
        arcPosY = sqrt(pow(radius, 2) - pow((arcPosX - ctrPosX2), 2)) + ctrPosY2;
        ArcMotionXY(arcPosX, -arcPosY);
      }
    } else {  // initPosY == initPosY.
      ctrPosX1 = ctrPosX2 = (initPosX + endPosX) / 2;
      ctrPosY1 = initPosYR - ((halfLenY / halfLenX) * (abs(diffPosX) / 2));
      ctrPosY2 = initPosYR + ((halfLenY / halfLenX) * (abs(diffPosX) / 2));
      for (arcPosX = initPosX; arcPosX <= endPosX; arcPosX = arcPosX + 1) {
        arcPosY = sqrt(pow(radius, 2) - pow((arcPosX - ctrPosX1), 2)) + ctrPosY1;
        ArcMotionXY(arcPosX, -arcPosY);
      }
    }
  } else if (initPosX > endPosX) {  // 2. When the left-side startPosX1 is larger than the right-side startPosX2
    if (initPosYR < endPosYR) {
      for (arcPosX = initPosX; arcPosX >= endPosX; arcPosX = arcPosX - 1) {
        arcPosY = -sqrt(pow(radius, 2) - pow((arcPosX - ctrPosX1), 2)) + ctrPosY1;
        ArcMotionXY(arcPosX, -arcPosY);
      }
    } else if (initPosYR > endPosY) {
      for (arcPosX = initPosX; arcPosX >= endPosX; arcPosX = arcPosX - 1) {
        arcPosY = -sqrt(pow(radius, 2) - pow((arcPosX - ctrPosX2), 2)) + ctrPosY2;
        ArcMotionXY(arcPosX, -arcPosY);
      }
    } else {  // initPosY == initPosY
      ctrPosX1 = ctrPosX2 = (initPosX + endPosX) / 2;
      ctrPosY1 = initPosYR - ((halfLenY / halfLenX) * (abs(diffPosX) / 2));
      ctrPosY2 = initPosYR + ((halfLenY / halfLenX) * (abs(diffPosX) / 2));
      for (arcPosX = initPosX; arcPosX >= endPosX; arcPosX = arcPosX - 1) {
        arcPosY = -sqrt(pow(radius, 2) - pow((arcPosX - ctrPosX2), 2)) + ctrPosY2;
        ArcMotionXY(arcPosX, -arcPosY);
      }
    }
  } else {  // 3. When initPosY == initPosY
    if (initPosYR < endPosYR) {
    } else if (initPosYR > endPosYR) {
    } else {  // initPosY == endPosY
      Serial.println("[Warning] Becomes dot");
    }
  }
}
void RelArcPosCCW() {
}

void CenterPos() {
  if (Finding('X', noLetr) == noLetr || Finding('Y', noLetr) == noLetr) {
    Serial.println("[Warning] No 'X' or 'Y'");
    return;
  } else {
    endPosX = Finding('X', noLetr), endPosY = Finding('Y', noLetr), endPosYR = (-1) * endPosY;

    if (Finding('R', noLetr) == noLetr) {
      offsetX = Finding('I', noLetr);
      offsetY = Finding('J', noLetr);
      radius = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
    } else if (Finding('I', noLetr) == noLetr && Finding('J', noLetr) == noLetr) {
      radius = Finding('R', noLetr);
    } else {
      Serial.println("[Warning] No 'I', 'J', or 'R'");
      return;
    }

    diffPosX = initPosX - endPosX, diffPosY = initPosYR - endPosYR;
    simpValC = pow(initPosX, 2) + pow(initPosYR, 2) - pow(endPosX, 2) - pow(endPosYR, 2);
    simpValCy = (simpValC / (2 * diffPosY)) - initPosYR;
    coefQdaA = (pow(diffPosX, 2) + pow(diffPosY, 2)) / pow(diffPosY, 2);
    coefLinB = -(2 * initPosX) - (2 * simpValCy * (diffPosX / diffPosY));
    coefCstC = pow(initPosX, 2) + pow(simpValCy, 2) - pow(radius, 2);

    ctrPosX1 = (-coefLinB + sqrt(pow(coefLinB, 2) - 4 * coefQdaA * coefCstC)) / (2 * coefQdaA);  // ctrPosX1 > ctrPosX2. In other words, ctrPosX1 is always located in right-side
    ctrPosX2 = (-coefLinB - sqrt(pow(coefLinB, 2) - 4 * coefQdaA * coefCstC)) / (2 * coefQdaA);
    ctrPosY1 = -(diffPosX / diffPosY) * ctrPosX1 + (simpValC / (2 * diffPosY));
    ctrPosY2 = -(diffPosX / diffPosY) * ctrPosX2 + (simpValC / (2 * diffPosY));  // ctrPosY1 < ctrPosY2.

    halfLenX = sqrt(pow((abs(diffPosX) / 2), 2) + pow((abs(diffPosY) / 2), 2));
    halfLenY = sqrt(pow(radius, 2) - pow(halfLenX, 2));
  }
}
void ArcMotionXY(double a_PosX, double a_PosY) {
  // Set the direction either left or right
  int directionState = 0;
  const int direction_CW = 1, direction_CCW = -1;
  if (endPosX <= 0) {
    directionState = direction_CW;
  } else {
    directionState = direction_CCW;
  }
  targetPosX = (a_PosX * stepsPerMM);
  targetVelX = (directionState * stepperSpeed);
  stepper1.moveTo(targetPosX);

  if (endPosYR <= 0) {
    directionState = direction_CW;
  } else {
    directionState = direction_CCW;
  }
  targetPosY = (a_PosY * stepsPerMM);
  targetVelY = (directionState * stepperSpeedR);
  stepper2.moveTo(targetPosY);

  // Actually move X and Y axies
  while ((stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0)) {
    stepper1.setSpeed(targetVelX);
    stepper2.setSpeed(targetVelY);
    stepper1.run();
    stepper2.run();
    TimerPos();
  }
  if ((stepper1.currentPosition() == targetPosX) || (targetPosX == 0)) {
    Generating();
    while (stepper2.currentPosition() != targetPosY && digitalRead(limitSwitchY) != 0) {
      stepper2.setSpeed(targetVelY);
      stepper2.run();
      TimerPos();
    }
    Generating();
  } else {
    Generating();
    while (stepper1.currentPosition() != targetPosX && digitalRead(limitSwitchX) != 0) {
      stepper1.setSpeed(targetVelX);
      stepper1.run();
      TimerPos();
    }
    Generating();
  }
}

void ManualSet() {  // The reason why it needs to go back to the origin is to re-calculate the distancea as similiar as calibration function.
  // If 3-axis CNC machine has the encoder, it doesn't need this process. It will only need to move from the current possition with the shorter time.
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  digitalWrite(enablePin, LOW);  // Enable the motor pin

  // Move to the origin to measure the manual positions.
  while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper2.setSpeed(stepperSpeedR);
    stepper3.setSpeed(stepperSpeedR);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  if (digitalRead(limitSwitchX) == 0) {  // 1st: X
    tempPosX = stepper1.currentPosition();
    MoveBackX(safetyDis, -stepperSpeed);
    while (digitalRead(limitSwitchY) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper2.setSpeed(stepperSpeedR);
      stepper3.setSpeed(stepperSpeedR);
      stepper2.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchY) == 0) {  // 2nd: Y
      tempPosY = stepper2.currentPosition();
      MoveBackY(safetyDisR, -stepperSpeedR);
      while (digitalRead(limitSwitchZ) != 0) {  // 3rd: Z
        stepper3.setSpeed(stepperSpeedR);
        stepper3.run();
      }
      tempPosZ = stepper3.currentPosition();
      MoveBackZ(safetyDisR, -stepperSpeedR);
    } else {  // 2nd: Z
      tempPosZ = stepper3.currentPosition();
      MoveBackZ(safetyDisR, -stepperSpeedR);
      while (digitalRead(limitSwitchY) != 0) {  // 3rd: Y
        stepper2.setSpeed(stepperSpeedR);
        stepper2.run();
      }
      tempPosY = stepper2.currentPosition();
      MoveBackY(safetyDisR, -stepperSpeedR);
    }
  } else if (digitalRead(limitSwitchY) == 0) {  // 1st: Y
    tempPosY = stepper2.currentPosition();
    MoveBackY(safetyDisR, -stepperSpeedR);
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchZ) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper3.setSpeed(stepperSpeedR);
      stepper1.run();
      stepper3.run();
    }
    if (digitalRead(limitSwitchX) == 0) {  // 2nd: X
      tempPosX = stepper1.currentPosition();
      MoveBackX(safetyDis, -stepperSpeed);
      while (digitalRead(limitSwitchZ) != 0) {  // 3rd: Z
        stepper3.setSpeed(stepperSpeedR);
        stepper3.run();
      }
      tempPosZ = stepper3.currentPosition();
      MoveBackZ(safetyDisR, -stepperSpeedR);
    } else {  // 2nd: Z
      tempPosZ = stepper3.currentPosition();
      MoveBackZ(safetyDisR, -stepperSpeedR);
      while (digitalRead(limitSwitchX) != 0) {  // 3rd: X
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      tempPosX = stepper1.currentPosition();
      MoveBackX(safetyDis, -stepperSpeed);
    }
  } else {
    tempPosZ = stepper3.currentPosition();
    MoveBackZ(safetyDisR, -stepperSpeedR);
    while (digitalRead(limitSwitchX) != 0 && digitalRead(limitSwitchY) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper2.setSpeed(stepperSpeedR);
      stepper1.run();
      stepper2.run();
    }
    if (digitalRead(limitSwitchX) == 0) {
      tempPosX = stepper1.currentPosition();
      MoveBackX(safetyDis, -stepperSpeed);
      while (digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(stepperSpeedR);
        stepper2.run();
      }
      tempPosY = stepper2.currentPosition();
      MoveBackY(safetyDisR, -stepperSpeedR);
    } else {
      tempPosY = stepper2.currentPosition();
      MoveBackY(safetyDisR, -stepperSpeedR);
      while (digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(stepperSpeed);
        stepper1.run();
      }
      tempPosX = stepper1.currentPosition();
      MoveBackX(safetyDis, -stepperSpeed);
    }
  }

  // Move back to the manual positions after measurements.
  while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
    stepper1.setSpeed(-stepperSpeed);
    stepper2.setSpeed(-stepperSpeedR);
    stepper3.setSpeed(-stepperSpeedR);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  if (stepper1.currentPosition() == -tempPosX) {
    Generating();
    while ((stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper2.setSpeed(-stepperSpeedR);
      stepper3.setSpeed(-stepperSpeedR);
      stepper2.run();
      stepper3.run();
    }
    if (stepper2.currentPosition() == -tempPosY) {
      Generating();
      while (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedR);
        stepper3.run();
      }
      Generating();
    } else {
      Generating();
      while (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeedR);
        stepper2.run();
      }
      Generating();
    }
  } else if (stepper2.currentPosition() == -tempPosY) {
    Generating();
    while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper3.setSpeed(-stepperSpeedR);
      stepper1.run();
      stepper3.run();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      Generating();
      while (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedR);
        stepper3.run();
      }
      Generating();
    } else {
      Generating();
      while (stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      Generating();
    }
  } else {  // (stepper3.currentPosition() == -tempPosZ)
    Generating();
    while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper2.setSpeed(-stepperSpeedR);
      stepper1.run();
      stepper2.run();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      Generating();
      while (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeedR);
        stepper2.run();
      }
      Generating();
    } else {
      Generating();
      while (stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
      }
      Generating();
    }
  }
}

void Rewinding() {  // The reason why it needs to go back to the origin is to reset and know the actual position of motors
  // since 'stepper1.currentPosition()' function cannot work itself to find its current position once the motor position is changed by the external elements.
  Homing();

  // Move back to the previous positions
  while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
    stepper1.setSpeed(-stepperSpeed);
    stepper2.setSpeed(-stepperSpeedR);
    stepper3.setSpeed(-stepperSpeedR);
    stepper1.run();
    stepper2.run();
    stepper3.run();
    TimerPos();
  }
  if (stepper1.currentPosition() == -tempPosX) {
    Generating();
    while ((stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper2.setSpeed(-stepperSpeedR);
      stepper3.setSpeed(-stepperSpeedR);
      stepper2.run();
      stepper3.run();
      TimerPos();
    }
    if (stepper2.currentPosition() == -tempPosY) {
      Generating();
      while (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedR);
        stepper3.run();
        TimerPos();
      }
    } else {
      Generating();
      while (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeedR);
        stepper2.run();
        TimerPos();
      }
    }
  } else if (stepper2.currentPosition() == -tempPosY) {
    Generating();
    while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper3.setSpeed(-stepperSpeedR);
      stepper1.run();
      stepper3.run();
      TimerPos();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      Generating();
      while (stepper3.currentPosition() != -tempPosZ && digitalRead(limitSwitchZ) != 0) {
        stepper3.setSpeed(-stepperSpeedR);
        stepper3.run();
        TimerPos();
      }
    } else {
      Generating();
      while (stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
        TimerPos();
      }
    }
  } else {  // (stepper3.currentPosition() == -tempPosZ)
    Generating();
    while ((stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) && (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0)) {
      stepper1.setSpeed(-stepperSpeed);
      stepper2.setSpeed(-stepperSpeedR);
      stepper1.run();
      stepper2.run();
      TimerPos();
    }
    if (stepper1.currentPosition() == -tempPosX) {
      Generating();
      while (stepper2.currentPosition() != -tempPosY && digitalRead(limitSwitchY) != 0) {
        stepper2.setSpeed(-stepperSpeedR);
        stepper2.run();
        TimerPos();
      }
    } else {
      Generating();
      while (stepper1.currentPosition() != -tempPosX && digitalRead(limitSwitchX) != 0) {
        stepper1.setSpeed(-stepperSpeed);
        stepper1.run();
        TimerPos();
      }
    }
  }
}

void TimerPos() {
  unsigned long currMS = millis();
  if ((currMS - prevMS) > timeInr) {  // Every (= timeInr) seconds, this will be executed
    prevMS = currMS;
    Generating();
  }
}
void Generating() {
  currPosX = 0, currPosY = 0, currPosZ = 0;

  strcpy(strDis, M114);

  currPosX = -1 * stepper1.currentPosition();
  currPosY = stepper2.currentPosition();
  currPosZ = stepper3.currentPosition();
  currDisX = (currPosX / stepsPerMM);
  currDisY = (currPosY / stepsPerMM);
  currDisZ = (currPosZ / stepsPerMM);

  // If there is no new incoming number, then don't make the information about X, Y, and Z
  ReceivePosX(currDisX);
  ReceivePosY(currDisY);
  ReceivePosZ(currDisZ);

  Serial.println(strDis);
}
void Ranging() {
  strcpy(strDis, M115);

  ReceivePosX(maxDisX);
  ReceivePosY(maxDisY);
  ReceivePosZ(maxDisZ);

  Serial.println(strDis);
}
void ReceivePosX(double infoPosX) {
  strcat(strDis, space);
  strcat(strDis, X);
  dtostrf(infoPosX, 4, 2, str);
  strcat(strDis, str);
}
void ReceivePosY(double infoPosY) {
  strcat(strDis, space);
  strcat(strDis, Y);
  dtostrf(infoPosY, 4, 2, str);
  strcat(strDis, str);
}
void ReceivePosZ(double infoPosZ) {
  strcat(strDis, space);
  strcat(strDis, Z);
  dtostrf(infoPosZ, 4, 2, str);
  strcat(strDis, str);
}