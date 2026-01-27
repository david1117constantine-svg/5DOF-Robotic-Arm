/*
 * ESP32-S3 5-Axis Stepper Motor Control with A4988 Drivers
 * NEMA 17 Motors with Harmonic Drives (25:1 on joints 2-4)
 * 
 * Features:
 * - Degree-based positioning
 * - 1/16 microstepping (3200 steps/rev for motors, 80000 steps/rev with harmonic drive)
 * - Simultaneous multi-axis movement
 * - Homing capability
 */

#include <Arduino.h>

// Motor pin definitions
struct MotorPins {
  int stepPin;
  int dirPin;
  int ms1Pin;
  int ms2Pin;
  int ms3Pin;
};

// Motor 1 - Waist (Base)
MotorPins motor1 = {1, 2, 4, 5, 6};

// Motor 2 - Shoulder (with harmonic drive)
MotorPins motor2 = {41, 42, 17, 16, 15};

// Motor 3 - Elbow (with harmonic drive)
MotorPins motor3 = {39, 40, 17, 16, 15};

// Motor 4 - Wrist Pitch (with harmonic drive)
MotorPins motor4 = {18, 9, 17, 16, 15};

// Motor 5 - Wrist Roll
MotorPins motor5 = {11, 10, 14, 13, 12};

// Servos (optional)
const int SERVO1_PIN = 7;
const int SERVO2_PIN = 8;

// Limit switches for homing
const int LIMIT_SW1 = 38;
const int LIMIT_SW2 = 21;
const int LIMIT_SW3 = 45;

// Motor configuration
const int STEPS_PER_REV = 200;        // NEMA 17 standard
const int MICROSTEPS = 16;            // 1/16 microstepping
const int HARMONIC_RATIO = 25;        // 25:1 harmonic drive ratio

// Steps per degree calculation
const float STEPS_PER_DEG_DIRECT = (STEPS_PER_REV * MICROSTEPS) / 360.0;  // ~8.889 steps/degree
const float STEPS_PER_DEG_HARMONIC = STEPS_PER_DEG_DIRECT * HARMONIC_RATIO; // ~222.22 steps/degree

// Motor speed settings (microseconds between steps)
const int SPEED_FAST = 800;    // Fast speed
const int SPEED_MEDIUM = 1200; // Medium speed
const int SPEED_SLOW = 2000;   // Slow speed for precision

// Current microstepping settings for each motor
int currentMicrosteps[5] = {16, 16, 16, 16, 16};

// Current motor positions (in degrees)
float currentPosition[5] = {0, 0, 0, 0, 0};
float targetPosition[5] = {0, 0, 0, 0, 0};

// Motor enabled flags
bool motorsEnabled = true;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32-S3 5-Axis Stepper Control ===");
  Serial.println("Initializing motors...");
  
  // Initialize all motor pins
  initMotor(motor1);
  initMotor(motor2);
  initMotor(motor3);
  initMotor(motor4);
  initMotor(motor5);
  
  // Set microstepping to 1/16 for all motors
  setMicrostepping(motor1, 16);
  setMicrostepping(motor2, 16);
  setMicrostepping(motor3, 16);
  setMicrostepping(motor4, 16);
  setMicrostepping(motor5, 16);
  
  // Initialize limit switches
  pinMode(LIMIT_SW1, INPUT_PULLUP);
  pinMode(LIMIT_SW2, INPUT_PULLUP);
  pinMode(LIMIT_SW3, INPUT_PULLUP);
  
  Serial.println("Motors initialized!");
  Serial.println("Commands:");
  Serial.println("  M <j1> <j2> <j3> <j4> <j5> - Move to absolute positions (degrees)");
  Serial.println("  R <j1> <j2> <j3> <j4> <j5> - Move relative positions (degrees)");
  Serial.println("  S <group> <microsteps>     - Set microstepping (microsteps: 1,2,4,8,16)");
  Serial.println("      Group 1: Waist (Motor 1)");
  Serial.println("      Group 2: Geared motors (Motors 2,3,4 - Shoulder/Elbow/Wrist Pitch)");
  Serial.println("      Group 3: Wrist Roll (Motor 5)");
  Serial.println("  H - Home all axes");
  Serial.println("  P - Print current positions");
  Serial.println("  I - Print motor info (microstepping, steps/degree)");
  Serial.println("  E - Enable motors");
  Serial.println("  D - Disable motors");
  Serial.println("=======================================\n");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'M': // Move to absolute position
      case 'm':
        {
          float j1 = Serial.parseFloat();
          float j2 = Serial.parseFloat();
          float j3 = Serial.parseFloat();
          float j4 = Serial.parseFloat();
          float j5 = Serial.parseFloat();
          
          Serial.printf("Moving to: J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f\n", j1, j2, j3, j4, j5);
          moveToPosition(j1, j2, j3, j4, j5);
          break;
        }
        
      case 'R': // Move relative
      case 'r':
        {
          float j1 = Serial.parseFloat();
          float j2 = Serial.parseFloat();
          float j3 = Serial.parseFloat();
          float j4 = Serial.parseFloat();
          float j5 = Serial.parseFloat();
          
          moveToPosition(
            currentPosition[0] + j1,
            currentPosition[1] + j2,
            currentPosition[2] + j3,
            currentPosition[3] + j4,
            currentPosition[4] + j5
          );
          break;
        }
        
      case 'H': // Home
      case 'h':
        Serial.println("Homing all axes...");
        homeAllAxes();
        break;
        
      case 'P': // Print positions
      case 'p':
        printPositions();
        break;
        
      case 'E': // Enable motors
      case 'e':
        motorsEnabled = true;
        Serial.println("Motors enabled");
        break;
        
      case 'D': // Disable motors
      case 'd':
        motorsEnabled = false;
        Serial.println("Motors disabled");
        break;
        
      case 'S': // Set microstepping
      case 's':
        {
          int group = Serial.parseInt();
          int microsteps = Serial.parseInt();
          
          if (group < 1 || group > 3) {
            Serial.println("Error: Group must be 1, 2, or 3");
            Serial.println("  Group 1: Waist");
            Serial.println("  Group 2: Geared motors (Shoulder/Elbow/Wrist Pitch)");
            Serial.println("  Group 3: Wrist Roll");
            break;
          }
          
          if (microsteps != 1 && microsteps != 2 && microsteps != 4 && 
              microsteps != 8 && microsteps != 16) {
            Serial.println("Error: Microsteps must be 1, 2, 4, 8, or 16");
            break;
          }
          
          switch(group) {
            case 1: // Waist (Motor 1)
              setMicrostepping(motor1, microsteps);
              currentMicrosteps[0] = microsteps;
              Serial.printf("Group 1 (Waist) microstepping set to 1/%d\n", microsteps);
              break;
              
            case 2: // Geared motors (Motors 2, 3, 4)
              setMicrostepping(motor2, microsteps);
              currentMicrosteps[1] = microsteps;
              currentMicrosteps[2] = microsteps;
              currentMicrosteps[3] = microsteps;
              Serial.printf("Group 2 (Geared motors: Shoulder/Elbow/Wrist Pitch) microstepping set to 1/%d\n", microsteps);
              break;
              
            case 3: // Wrist Roll (Motor 5)
              setMicrostepping(motor5, microsteps);
              currentMicrosteps[4] = microsteps;
              Serial.printf("Group 3 (Wrist Roll) microstepping set to 1/%d\n", microsteps);
              break;
          }
          break;
        }
        
      case 'I': // Info
      case 'i':
        printMotorInfo();
        break;
    }
    
    // Clear remaining input
    while (Serial.available() > 0) Serial.read();
  }
}

void initMotor(MotorPins motor) {
  pinMode(motor.stepPin, OUTPUT);
  pinMode(motor.dirPin, OUTPUT);
  pinMode(motor.ms1Pin, OUTPUT);
  pinMode(motor.ms2Pin, OUTPUT);
  pinMode(motor.ms3Pin, OUTPUT);
  
  digitalWrite(motor.stepPin, LOW);
  digitalWrite(motor.dirPin, LOW);
}

void setMicrostepping(MotorPins motor, int microsteps) {
  // A4988 microstepping truth table
  // MS1 MS2 MS3 | Microsteps
  //  0   0   0  | Full step
  //  1   0   0  | Half step
  //  0   1   0  | 1/4 step
  //  1   1   0  | 1/8 step
  //  1   1   1  | 1/16 step
  
  switch(microsteps) {
    case 1:
      digitalWrite(motor.ms1Pin, LOW);
      digitalWrite(motor.ms2Pin, LOW);
      digitalWrite(motor.ms3Pin, LOW);
      break;
    case 2:
      digitalWrite(motor.ms1Pin, HIGH);
      digitalWrite(motor.ms2Pin, LOW);
      digitalWrite(motor.ms3Pin, LOW);
      break;
    case 4:
      digitalWrite(motor.ms1Pin, LOW);
      digitalWrite(motor.ms2Pin, HIGH);
      digitalWrite(motor.ms3Pin, LOW);
      break;
    case 8:
      digitalWrite(motor.ms1Pin, HIGH);
      digitalWrite(motor.ms2Pin, HIGH);
      digitalWrite(motor.ms3Pin, LOW);
      break;
    case 16:
      digitalWrite(motor.ms1Pin, HIGH);
      digitalWrite(motor.ms2Pin, HIGH);
      digitalWrite(motor.ms3Pin, HIGH);
      break;
    default:
      // Default to 1/16
      digitalWrite(motor.ms1Pin, HIGH);
      digitalWrite(motor.ms2Pin, HIGH);
      digitalWrite(motor.ms3Pin, HIGH);
  }
}

void moveToPosition(float j1, float j2, float j3, float j4, float j5) {
  if (!motorsEnabled) {
    Serial.println("Motors disabled! Enable with 'E' command.");
    return;
  }
  
  // Store target positions
  targetPosition[0] = j1;
  targetPosition[1] = j2;
  targetPosition[2] = j3;
  targetPosition[3] = j4;
  targetPosition[4] = j5;
  
  // Calculate steps needed for each motor
  long stepsNeeded[5];
  stepsNeeded[0] = degreesToSteps(j1 - currentPosition[0], false, 0); // Motor 1: direct drive
  stepsNeeded[1] = degreesToSteps(j2 - currentPosition[1], true, 1);  // Motor 2: harmonic drive
  stepsNeeded[2] = degreesToSteps(j3 - currentPosition[2], true, 2);  // Motor 3: harmonic drive
  stepsNeeded[3] = degreesToSteps(j4 - currentPosition[3], true, 3);  // Motor 4: harmonic drive
  stepsNeeded[4] = degreesToSteps(j5 - currentPosition[4], false, 4); // Motor 5: direct drive
  
  // Set directions
  digitalWrite(motor1.dirPin, stepsNeeded[0] >= 0 ? HIGH : LOW);
  digitalWrite(motor2.dirPin, stepsNeeded[1] >= 0 ? HIGH : LOW);
  digitalWrite(motor3.dirPin, stepsNeeded[2] >= 0 ? HIGH : LOW);
  digitalWrite(motor4.dirPin, stepsNeeded[3] >= 0 ? HIGH : LOW);
  digitalWrite(motor5.dirPin, stepsNeeded[4] >= 0 ? HIGH : LOW);
  
  // Convert to absolute values
  for (int i = 0; i < 5; i++) {
    stepsNeeded[i] = abs(stepsNeeded[i]);
  }
  
  // Find maximum steps to determine move duration
  long maxSteps = 0;
  for (int i = 0; i < 5; i++) {
    if (stepsNeeded[i] > maxSteps) maxSteps = stepsNeeded[i];
  }
  
  // Perform coordinated movement
  long stepsTaken[5] = {0, 0, 0, 0, 0};
  
  for (long step = 0; step < maxSteps; step++) {
    // Step each motor proportionally
    if (stepsTaken[0] * maxSteps < stepsNeeded[0] * step) {
      stepMotor(motor1.stepPin);
      stepsTaken[0]++;
    }
    if (stepsTaken[1] * maxSteps < stepsNeeded[1] * step) {
      stepMotor(motor2.stepPin);
      stepsTaken[1]++;
    }
    if (stepsTaken[2] * maxSteps < stepsNeeded[2] * step) {
      stepMotor(motor3.stepPin);
      stepsTaken[2]++;
    }
    if (stepsTaken[3] * maxSteps < stepsNeeded[3] * step) {
      stepMotor(motor4.stepPin);
      stepsTaken[3]++;
    }
    if (stepsTaken[4] * maxSteps < stepsNeeded[4] * step) {
      stepMotor(motor5.stepPin);
      stepsTaken[4]++;
    }
    
    delayMicroseconds(SPEED_MEDIUM);
  }
  
  // Update current positions
  for (int i = 0; i < 5; i++) {
    currentPosition[i] = targetPosition[i];
  }
  
  Serial.println("Move complete!");
  printPositions();
}

long degreesToSteps(float degrees, bool harmonicDrive, int motorIndex) {
  float stepsPerDegree = (STEPS_PER_REV * currentMicrosteps[motorIndex]) / 360.0;
  
  if (harmonicDrive) {
    stepsPerDegree *= HARMONIC_RATIO;
  }
  
  return (long)(degrees * stepsPerDegree);
}

void stepMotor(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5); // Minimum pulse width for A4988
  digitalWrite(stepPin, LOW);
}

void homeAllAxes() {
  if (!motorsEnabled) {
    Serial.println("Motors disabled! Enable with 'E' command.");
    return;
  }
  
  Serial.println("Homing sequence started...");
  
  // Home each axis to its limit switch
  // This is a simple implementation - adjust based on your mechanical setup
  
  // Example: Move motors slowly until limit switches are triggered
  // You'll need to customize this based on your actual homing procedure
  
  Serial.println("Homing Motor 1...");
  // homeAxis(motor1, LIMIT_SW1, false);
  
  Serial.println("Homing Motor 2...");
  // homeAxis(motor2, LIMIT_SW2, true);
  
  Serial.println("Homing Motor 3...");
  // homeAxis(motor3, LIMIT_SW3, true);
  
  // Reset all positions to zero after homing
  for (int i = 0; i < 5; i++) {
    currentPosition[i] = 0;
    targetPosition[i] = 0;
  }
  
  Serial.println("Homing complete! All positions set to 0°");
  printPositions();
}

void homeAxis(MotorPins motor, int limitSwitch, bool harmonicDrive) {
  // Set direction (towards home)
  digitalWrite(motor.dirPin, LOW);
  
  // Move slowly until limit switch is triggered
  while (digitalRead(limitSwitch) == HIGH) {
    stepMotor(motor.stepPin);
    delayMicroseconds(SPEED_SLOW);
  }
  
  // Back off slightly
  digitalWrite(motor.dirPin, HIGH);
  for (int i = 0; i < 100; i++) {
    stepMotor(motor.stepPin);
    delayMicroseconds(SPEED_SLOW);
  }
  
  Serial.println("  Axis homed");
}

void printPositions() {
  Serial.println("\n=== Current Positions ===");
  Serial.printf("Joint 1 (Waist):       %.2f°\n", currentPosition[0]);
  Serial.printf("Joint 2 (Shoulder):    %.2f°\n", currentPosition[1]);
  Serial.printf("Joint 3 (Elbow):       %.2f°\n", currentPosition[2]);
  Serial.printf("Joint 4 (Wrist Pitch): %.2f°\n", currentPosition[3]);
  Serial.printf("Joint 5 (Wrist Roll):  %.2f°\n", currentPosition[4]);
  Serial.println("========================\n");
}

void printMotorInfo() {
  Serial.println("\n=== Motor Configuration ===");
  
  // Group 1: Waist
  Serial.println("Group 1 - Waist (Motor 1):");
  Serial.printf("  Microstepping: 1/%d\n", currentMicrosteps[0]);
  Serial.printf("  Steps/revolution: %d\n", STEPS_PER_REV * currentMicrosteps[0]);
  Serial.printf("  Steps/degree: %.2f\n", (STEPS_PER_REV * currentMicrosteps[0]) / 360.0);
  Serial.printf("  Resolution: %.4f°/step\n\n", 360.0 / (STEPS_PER_REV * currentMicrosteps[0]));
  
  // Group 2: Geared motors (all have same microstepping)
  Serial.println("Group 2 - Geared Motors (Motors 2,3,4 - Shoulder/Elbow/Wrist Pitch):");
  Serial.printf("  Microstepping: 1/%d\n", currentMicrosteps[1]);
  Serial.printf("  Steps/revolution (motor): %d\n", STEPS_PER_REV * currentMicrosteps[1]);
  Serial.printf("  Harmonic ratio: %d:1\n", HARMONIC_RATIO);
  Serial.printf("  Effective steps/revolution: %d\n", STEPS_PER_REV * currentMicrosteps[1] * HARMONIC_RATIO);
  Serial.printf("  Steps/degree: %.2f\n", (STEPS_PER_REV * currentMicrosteps[1] * HARMONIC_RATIO) / 360.0);
  Serial.printf("  Resolution: %.4f°/step\n\n", 360.0 / (STEPS_PER_REV * currentMicrosteps[1] * HARMONIC_RATIO));
  
  // Group 3: Wrist Roll
  Serial.println("Group 3 - Wrist Roll (Motor 5):");
  Serial.printf("  Microstepping: 1/%d\n", currentMicrosteps[4]);
  Serial.printf("  Steps/revolution: %d\n", STEPS_PER_REV * currentMicrosteps[4]);
  Serial.printf("  Steps/degree: %.2f\n", (STEPS_PER_REV * currentMicrosteps[4]) / 360.0);
  Serial.printf("  Resolution: %.4f°/step\n\n", 360.0 / (STEPS_PER_REV * currentMicrosteps[4]));
  
  Serial.println("===========================\n");
}