#include <AccelStepper.h>

#define STATE1_UPDATING 1
#define STATE1_ENABLE 2
#define STATE1_HOMING 3
#define STATE1_CALIBRATING 4
#define STATE1_TESTDRIVING 5
#define STATE1_MANUALSETTING 6
#define STATE1_REWINDING 7
#define STATE2_ABSOULUTE 1
#define STATE2_RELATIVE 2

// Define the stepper motor and the pins we will use
AccelStepper stepper1(1, 4, 7);  // (Type: Driver, Step, DIR)
const int enablePin = 10;
const int limitSwitch = 11;

// Set the variables to repeatedly use
int stepperState = 0;
int testState = 0;
float directionState = 0;
int microstepState = 0;
const float direction_CW = 1;   // Clockwise
const float direction_CCW = 2;  // Counter-Clockwise
long targetDis = 0;
long targetPos = 0;
float targetVel = 0;
long maxDis1 = 0;
long maxDis2 = 0;
long maxDis = 0;
long prevPos = 0;
long tempPos = 0;
char Ready = 0;
float newMaximumSpeed = 0;
float newSpeed = 0;
float newAcceleration = 0;

float stepperMaxSpeed = 0;
float stepperSpeed = 0;         // stepperSpeed = steps / second
float stepperAcceleration = 0;  // stepperAcceleration = steps / (second)^2
unsigned int stepperMinPulseWidth = 20;
long safetyDis = 0;  // Distance from the limtiswitch

// This encoder has 600 pulse per revolution
volatile long temp = 0;
volatile long counter = 0;  //This variable will increase or decrease depending on the rotation of encoder 

void setup() {               // Set initial seed values for the stepper
  pinMode(2, INPUT_PULLUP);  // green (OUT B) interrupter pin of arduino
  pinMode(3, INPUT_PULLUP);  // white (OUT A) interrupter pin of arduino
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(limitSwitch, INPUT_PULLUP);

  Serial.begin(9600);
  attachInterrupt(0, EncoderAct0, RISING);  //A rising pulse from encodenren activated EncoderAct0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(1, EncoderAct1, RISING);  //B rising pulse from encodenren activated EncoderAct1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.

  SettingDef();

  stepper1.setMaxSpeed(stepperMaxSpeed);
  stepper1.setSpeed(stepperSpeed);
  stepper1.setAcceleration(stepperAcceleration);
  stepper1.setMinPulseWidth(stepperMinPulseWidth);  // Prevent more steps drift due to the pulse width by TB6600 driver

  digitalWrite(enablePin, HIGH);  // Disable the motor pin for the safety reason    
}

void loop() {                     // Main interface
  if (digitalRead(enablePin) == 0) {
    Serial.println(F("※ Present Status: Enabled"));    
  }
  else {
    Serial.println(F("※ Present Status: Disabled"));   
  }
  prevPos = stepper1.currentPosition();

  Serial.println(F("※ [Controller State]"));
  Serial.println(F("   [1] Updating"));
  Serial.println(F("   [2] Enable / Disable"));   
  Serial.println(F("   [3] Homing"));
  Serial.println(F("   [4] Step-Calibration"));
  Serial.println(F("   [5] Test Driving"));
  Serial.println(F("   [6] Manual Setting"));  
  Serial.println(F("   [7] Rewinding"));

  Serial.parseInt();
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
      Homing();
      break;

    case STATE1_CALIBRATING:
      Calibrating();
      break;

    case STATE1_TESTDRIVING:
      Serial.println(F("※ [Test State]"));
      Serial.println(F("   [1] Move to the certain location"));
      Serial.println(F("   [2] Move from the current position"));

      Serial.parseInt();
      while (Serial.available() == 0) {}
      testState = Serial.parseInt();

      Serial.print(F("   Selected ["));
      Serial.print(testState);
      Serial.println(F("]"));
      Serial.println(F(""));

      switch (testState) {
        case STATE2_ABSOULUTE:
          SettingPos();
          break;
        case STATE2_RELATIVE:
          SettingVel();
          break;
      }
      break;

    case STATE1_MANUALSETTING:
      ManualSet();
      break;

    case STATE1_REWINDING:
      Rewinding();
      break;
  }
}

void EncoderAct0() { // EncoderAct0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  if (digitalRead(3) == LOW) {  // Check pin 3 to determine the direction
    counter++;
  } else {
    counter--;
  }
}

void EncoderAct1() { // EncoderAct1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  if (digitalRead(2) == LOW) { // Check with pin 2 to determine the direction
    counter--;
  } else {
    counter++;
  }
}

void EncoderRead() {
  if (counter != temp) {
    Serial.print("Encoder Position: ");
    Serial.println(counter);
    temp = counter;
  }
}

void SettingDef() {  // Setting the default based on the microstep
  Serial.println(F("   [Notice]"));
  Serial.println(F("※ Once the operation is completed, CNC machine is automatically disconnected to the supply for the safety reason"));
  Serial.println(F(""));

  Serial.println(F("※ [Microstep state]"));
  Serial.println(F("   Microstep: 1,  Pulse/rev: 200"));
  Serial.println(F("   Microstep: 2,  Pulse/rev: 400"));
  Serial.println(F("   Microstep: 4,  Pulse/rev: 800"));
  Serial.println(F("   Microstep: 8,  Pulse/rev: 1600"));
  Serial.println(F("   Microstep: 16, Pulse/rev: 3200"));
  Serial.println(F("   Microstep: 32, Pulse/rev: 6400"));
  Serial.println(F(""));

  Serial.println(F("   Moderate maximum speed, constant speed, and acceleration for each microstep will be assigned"));
  Serial.print(F("※ Type the number of the microstep on your motor driver: "));

  Serial.parseInt();
  while (Serial.available() == 0) {}
  microstepState = Serial.parseInt();
  Serial.println(microstepState);
  Serial.println(F(""));

  if (microstepState == 1 || microstepState == 2 || microstepState == 4 || microstepState == 8 || microstepState == 16 || microstepState == 32) {
    switch (microstepState) {
      case 1:
        Serial.println(F("※ Default constant speed: 700 steps/s, default acceleration: 900 steps/s^2."));
        Serial.println(F("   One revolution for 1cm linear motion needs 1000 steps/s"));
        Serial.println(F(""));
        stepperMaxSpeed = 1000;
        stepperSpeed = 600;
        stepperAcceleration = 600;
        break;
      case 2:
        Serial.println(F("※ Default constant speed: 1400 steps/s, default acceleration: 1800 steps/s^2."));
        Serial.println(F("   One revolution for 1cm linear motion needs 2000 steps/s"));
        Serial.println(F(""));
        stepperMaxSpeed = 2000;
        stepperSpeed = 1200;
        stepperAcceleration = 1200;
        break;
      case 4:
        Serial.println(F("※ Default constant speed: 2800 steps/s, default acceleration: 3200 steps/s^2."));
        Serial.println(F("   One revolution for 1cm linear motion needs 4000 steps/s"));
        Serial.println(F(""));
        stepperMaxSpeed = 4000;
        stepperSpeed = 2400;
        stepperAcceleration = 2400;
        break;
      case 8:
        Serial.println(F("※ Default constant speed: 5600 steps/s, default acceleration: 6400 steps/s^2."));
        Serial.println(F("   One revolution for 1cm linear motion needs 8000 steps/s"));
        Serial.println(F(""));
        stepperMaxSpeed = 8000;
        stepperSpeed = 4800;
        stepperAcceleration = 4800;
        break;
      case 16:
        Serial.println(F("※ Default constant speed: 11100 steps/s, default acceleration: 12900 steps/s^2."));
        Serial.println(F("   One revolution for 1cm linear motion needs 16000 steps/s"));
        Serial.println(F(""));
        stepperMaxSpeed = 16000;
        stepperSpeed = 9600;
        stepperAcceleration = 9600;
        break;
      case 32:
        Serial.println(F("※ Default constant speed: 22200 steps/s, default acceleration: 25400 steps/s^2."));
        Serial.println(F("   One revolution for 1cm linear motion needs 32000 steps/s"));
        Serial.println(F(""));
        stepperMaxSpeed = 32000;
        stepperSpeed = 22200;
        stepperAcceleration = 25400;
        break;
    }
    safetyDis = (-1 * stepperSpeed / 5);
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
  Serial.parseInt();
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
  Serial.print("   New maximum speed value will be set as: ");  //confirm update by message
  Serial.parseInt();
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
  Serial.parseInt();
  while (Serial.available() == 0) {}
  newAcceleration = Serial.parseFloat();      //receive the acceleration from serial
  stepper1.setAcceleration(newAcceleration);  //update the value of the variable
  Serial.println(newAcceleration);
  Serial.println(F(""));
}

void Enable() { // Change the status of Enable/Disable 
  if (digitalRead(enablePin) == 0) {
      digitalWrite(enablePin, HIGH); 
  }
  else {
      digitalWrite(enablePin, LOW);
  }
}

void Homing() {  // Move the slider to the initial position - homing
  while (digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }

  Serial.println(F("   [Homing]"));
  Serial.println(F("   Limit switch is detected"));
  stepper1.setCurrentPosition(0);  // When the limit switch is pressed, set the position to 0 steps
  Serial.print(F("※ Reset Position: "));
  Serial.println(abs(stepper1.currentPosition()));

  // Move 320 steps back from the limit switch to stray from the boundray of limit swtich
  while (stepper1.currentPosition() != safetyDis) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();  
  }

  counter = 0;    
  Serial.print(F("   Current Position: "));
  Serial.println(abs(stepper1.currentPosition()));
  Serial.println(F(""));
}

void Calibrating() {
  if (stepper1.currentPosition() != safetyDis) {
    Serial.println(F("   [Warning] Please do homing before calibration!"));
    Serial.println(F(""));
    return;  // Cannot use loop(); since it will keep coming to "[Position calibration in process]"
  }
  Serial.println(F("   [Position calibration in process]"));

  counter = 0;  
  // Move the slider to the right-hand position
  while (digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
  EncoderRead();  

  maxDis1 = stepper1.currentPosition();
  stepper1.setCurrentPosition(0);

  while (stepper1.currentPosition() != (abs(safetyDis))) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }
  counter = 0; 
  delay(100);

  // Move the slider to the left-hand position
  while (digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }
  EncoderRead();  

  maxDis2 = stepper1.currentPosition();
  stepper1.setCurrentPosition(0);

  while (stepper1.currentPosition() != safetyDis) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
  counter = 0; 

  Serial.print(F("   Two measured maximum distances: "));
  Serial.print(abs(maxDis1));
  Serial.print(F(", "));
  Serial.println(abs(maxDis2));
  Serial.print(F("   The difference between two calibrations: "));
  Serial.println(maxDis1 + maxDis2);
  Serial.print(F("※ The average maximum distance: "));
  maxDis = -1 * ((abs(maxDis1) + abs(maxDis2)) / 2);
  Serial.println(abs(maxDis));
  Serial.println(F(""));
}

void SettingPos() {  // Do not change the offset
  Serial.print(F("   By the calibration, maximum distance is measured as "));
  Serial.println(abs(maxDis));

  if (maxDis == 0) {
    Serial.println(F("   [Warning] Please do calibration before moveTo!"));
    return;  // Cannot use loop(); since it will keep coming to "Specify the certain point: "
  }

  Serial.print(F("※ Specify the certain point: "));
  Serial.parseInt();
  while (Serial.available() == 0) {}
  targetPos = (-1 * Serial.parseInt());
  Serial.println(abs(targetPos));

  if (targetPos <= maxDis) {
    Serial.println(F("   [Warning] Don't exceed the limitation. Re-try!"));
    SettingPos();
  }

  stepper1.moveTo(targetPos);

  if (stepper1.currentPosition() > targetPos) {
    directionState = (-1 * direction_CW);
    Serial.println(F("   Selected [CCW]"));
  } else {
    directionState = direction_CW;
    Serial.println(F("   Selected [CW]"));
  }
  Serial.println(F(""));
  targetVel = (directionState * stepperSpeed);

  while (stepper1.currentPosition() != targetPos && digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(targetVel);
    stepper1.run();
  }

  Serial.print(F("※ Current Position: "));  //If this is in while loop, motor speed will be decreased
  Serial.print(abs(stepper1.currentPosition()));
  Serial.print(F("/"));
  Serial.println(abs(maxDis));
}

void SettingVel() {
  Serial.println(F("※ [Direction State]"));
  Serial.println(F("   [1] CLOCKWISE (<-)"));          // Go left
  Serial.println(F("   [2] COUNTER-CLOCKWISE (->)"));  // Go right

  Serial.parseInt();
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
    SettingVel();
  }
  targetVel = (directionState * stepperSpeed);
  Serial.print(F("※ How much distance to move from the current position: "));

  Serial.parseInt();
  while (Serial.available() == 0) {}
  targetDis = Serial.parseInt();
  Serial.println(targetDis);
  Serial.println(F(""));

  targetPos = (stepper1.currentPosition() + (directionState * targetDis));
  stepper1.moveTo(targetPos);

  while (stepper1.currentPosition() != targetPos && digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(targetVel);
    stepper1.run();
  }

  Serial.print(F("※ Current Position: "));  //If this is in while loop, motor speed will be decreased
  Serial.print(abs(stepper1.currentPosition()));
  Serial.print(F("/"));
  Serial.println(abs(maxDis));
}

void ManualSet() {
  if (digitalRead(enablePin) == 0) {
    Serial.println(F("   [Manual Setting]"));
    Serial.println(F("※ Please move the machine to where you want it to be. Then, press 'R' once it's ready."));

    while (Ready != 'R') {
      Ready = Serial.read();
    }

    stepper1.setCurrentPosition(0);
    digitalWrite(enablePin, LOW);  // Enable the motor pin
    while (digitalRead(limitSwitch) != 0) {
      stepper1.setSpeed(stepperSpeed);
      stepper1.run();
    }

    tempPos = stepper1.currentPosition();
    stepper1.setCurrentPosition(0);

    while (stepper1.currentPosition() != safetyDis) {
      stepper1.setSpeed(-stepperSpeed);
      stepper1.run();
    }

    Serial.print(F("   Manual Position: "));
    Serial.println(abs(tempPos));
    Serial.println(F(""));

    while (stepper1.currentPosition() != -tempPos && digitalRead(limitSwitch) != 0) {
      stepper1.setSpeed(-stepperSpeed);
      stepper1.run();
    }
  } else {
    Serial.println(F("   [Warning] Please disable the motor first. Then, continue this step"));
    return;
  }
}

void Rewinding() {
  while (digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(stepperSpeed);
    stepper1.run();
  }

  stepper1.setCurrentPosition(0);

  while (stepper1.currentPosition() != safetyDis) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }

  Serial.println(F("   [Rewinding]"));
  Serial.print(F("※ Previous Position: "));
  Serial.println(abs(prevPos));
  Serial.println(F(""));

  while (stepper1.currentPosition() != prevPos && digitalRead(limitSwitch) != 0) {
    stepper1.setSpeed(-stepperSpeed);
    stepper1.run();
  }
}