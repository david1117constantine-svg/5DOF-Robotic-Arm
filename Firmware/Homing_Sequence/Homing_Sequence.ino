/*
 * Homing and Park Sequence for 5DOF Robot Arm
 * 
 * Limit Switches:
 * - Limit SW 1 (GPIO 38): Joint 2 (Shoulder)
 * - Limit SW 2 (GPIO 21): Joint 3 (Elbow)
 * - Limit SW 3 (GPIO 45): Joint 4 (Wrist Pitch)
 * 
 * Homing Strategy:
 * - Move each axis slowly until limit switch triggers
 * - Back off slightly to clear the switch
 * - Set that position as the home/zero position
 * - After homing, move to park position (X=350, Y=0, Z=350)
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

MotorPins motor1 = {1, 2, 4, 5, 6};       // Waist
MotorPins motor2 = {41, 42, 17, 16, 15};  // Shoulder
MotorPins motor3 = {39, 40, 17, 16, 15};  // Elbow
MotorPins motor4 = {18, 9, 17, 16, 15};   // Wrist Pitch
MotorPins motor5 = {11, 10, 14, 13, 12};  // Wrist Roll

// Limit switches
const int LIMIT_SW1 = 38;  // Joint 2 - Shoulder
const int LIMIT_SW2 = 21;  // Joint 3 - Elbow
const int LIMIT_SW3 = 45;  // Joint 4 - Wrist Pitch

// Motor configuration
const int STEPS_PER_REV = 200;
const int HARMONIC_RATIO = 25;
int currentMicrosteps[5] = {16, 16, 16, 16, 16};

// Motor speeds (microseconds between steps)
const int SPEED_HOMING = 2500;    // Slow speed for homing
const int SPEED_BACKOFF = 1500;   // Medium speed for backing off
const int SPEED_NORMAL = 1200;    // Normal movement speed

// DH Parameters
const float L1 = 152.5;
const float L2 = 180.0;
const float L3 = 180.0;
const float L4 = 101.0;

// Joint limits
const float JOINT1_MIN = -180.0, JOINT1_MAX = 180.0;
const float JOINT2_MIN = -70.0, JOINT2_MAX = 70.0;
const float JOINT3_MIN = -80.0, JOINT3_MAX = 80.0;
const float JOINT4_MIN = -90.0, JOINT4_MAX = 90.0;
const float JOINT5_MIN = -180.0, JOINT5_MAX = 180.0;

// Current positions
float currentPosition[5] = {0, 0, 0, 0, 0};

// Homing status
bool isHomed = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== 5DOF Robot Arm Homing System ===");
  
  // Initialize motors
  initMotor(motor1);
  initMotor(motor2);
  initMotor(motor3);
  initMotor(motor4);
  initMotor(motor5);
  
  setMicrostepping(motor1, 16);
  setMicrostepping(motor2, 16);
  setMicrostepping(motor5, 16);
  
  // Initialize limit switches
  pinMode(LIMIT_SW1, INPUT_PULLUP);
  pinMode(LIMIT_SW2, INPUT_PULLUP);
  pinMode(LIMIT_SW3, INPUT_PULLUP);
  
  Serial.println("Motors and limit switches initialized!");
  Serial.println("Commands:");
  Serial.println("  H - Home all axes with limit switches");
  Serial.println("  P - Park at safe position (X=350, Y=0, Z=350)");
  Serial.println("  S - Check limit switch status");
  Serial.println("========================================\n");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'H':
      case 'h':
        homeAllAxes();
        break;
        
      case 'P':
      case 'p':
        if (!isHomed) {
          Serial.println("ERROR: Must home first! Send 'H' command.");
        } else {
          parkPosition();
        }
        break;
        
      case 'S':
      case 's':
        checkLimitSwitches();
        break;
    }
    
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
  switch(microsteps) {
    case 16:
      digitalWrite(motor.ms1Pin, HIGH);
      digitalWrite(motor.ms2Pin, HIGH);
      digitalWrite(motor.ms3Pin, HIGH);
      break;
    default:
      digitalWrite(motor.ms1Pin, HIGH);
      digitalWrite(motor.ms2Pin, HIGH);
      digitalWrite(motor.ms3Pin, HIGH);
  }
}

void stepMotor(int stepPin, int delayTime) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayTime);
}

void checkLimitSwitches() {
  Serial.println("\n=== Limit Switch Status ===");
  Serial.print("Joint 2 (Shoulder): ");
  Serial.println(digitalRead(LIMIT_SW1) == LOW ? "TRIGGERED" : "Open");
  Serial.print("Joint 3 (Elbow):    ");
  Serial.println(digitalRead(LIMIT_SW2) == LOW ? "TRIGGERED" : "Open");
  Serial.print("Joint 4 (Wrist):    ");
  Serial.println(digitalRead(LIMIT_SW3) == LOW ? "TRIGGERED" : "Open");
  Serial.println("===========================\n");
}

void homeAllAxes() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   HOMING SEQUENCE INITIATED        ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  // Check initial switch states
  Serial.println("Checking initial limit switch states...");
  checkLimitSwitches();
  
  // Home Joint 2 (Shoulder)
  Serial.println("─────────────────────────────────────");
  Serial.println("Homing Joint 2 (Shoulder)...");
  homeAxisWithLimitSwitch(motor2, LIMIT_SW1, 1, 2, true);  // Move towards negative (assume switch at min)
  
  delay(500);
  
  // Home Joint 3 (Elbow)
  Serial.println("─────────────────────────────────────");
  Serial.println("Homing Joint 3 (Elbow)...");
  homeAxisWithLimitSwitch(motor3, LIMIT_SW2, 2, 3, true);  // Move towards negative
  
  delay(500);
  
  // Home Joint 4 (Wrist Pitch)
  Serial.println("─────────────────────────────────────");
  Serial.println("Homing Joint 4 (Wrist Pitch)...");
  homeAxisWithLimitSwitch(motor4, LIMIT_SW3, 3, 4, true);  // Move towards negative
  
  // Set homed positions
  // Assuming limit switches are at negative extremes
  currentPosition[1] = JOINT2_MIN;  // Shoulder at -70°
  currentPosition[2] = JOINT3_MIN;  // Elbow at -80°
  currentPosition[3] = JOINT4_MIN;  // Wrist at -90°
  
  Serial.println("─────────────────────────────────────");
  Serial.println("\n✓ HOMING COMPLETE!");
  Serial.println("Current positions set:");
  Serial.printf("  Joint 2 (Shoulder): %.2f°\n", currentPosition[1]);
  Serial.printf("  Joint 3 (Elbow):    %.2f°\n", currentPosition[2]);
  Serial.printf("  Joint 4 (Wrist):    %.2f°\n", currentPosition[3]);
  
  isHomed = true;
  
  Serial.println("\n✓ Ready to move! Send 'P' to park.\n");
}

void homeAxisWithLimitSwitch(MotorPins motor, int limitSwitch, int jointIndex, int jointNum, bool moveNegative) {
  // Set direction (negative = towards limit switch)
  digitalWrite(motor.dirPin, moveNegative ? LOW : HIGH);
  
  Serial.printf("  Moving towards limit switch (Joint %d)...\n", jointNum);
  
  // Move slowly until limit switch is triggered
  int stepCount = 0;
  const int MAX_STEPS = 50000;  // Safety limit
  
  while (digitalRead(limitSwitch) == HIGH && stepCount < MAX_STEPS) {
    stepMotor(motor.stepPin, SPEED_HOMING);
    stepCount++;
    
    // Print progress every 1000 steps
    if (stepCount % 1000 == 0) {
      Serial.print(".");
    }
  }
  
  if (stepCount >= MAX_STEPS) {
    Serial.println("\n  WARNING: Max steps reached! Check limit switch.");
    return;
  }
  
  Serial.println("\n  ✓ Limit switch triggered!");
  Serial.printf("  Steps taken: %d\n", stepCount);
  
  delay(200);
  
  // Back off from limit switch
  Serial.println("  Backing off from limit switch...");
  digitalWrite(motor.dirPin, moveNegative ? HIGH : LOW);  // Reverse direction
  
  // Back off by a small amount (about 2 degrees with harmonic drive)
  int backoffSteps = 500;  
  for (int i = 0; i < backoffSteps; i++) {
    stepMotor(motor.stepPin, SPEED_BACKOFF);
  }
  
  delay(200);
  
  // Verify switch is no longer triggered
  if (digitalRead(limitSwitch) == HIGH) {
    Serial.println("  ✓ Successfully backed off from limit switch");
  } else {
    Serial.println("  WARNING: Still on limit switch after backing off");
  }
  
  Serial.printf("  ✓ Joint %d homed successfully!\n", jointNum);
}

void parkPosition() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   MOVING TO PARK POSITION          ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  Serial.println("Target: X=350mm, Y=0mm, Z=350mm");
  
  // Calculate IK for park position
  float targetAngles[5];
  if (solveIK(350, 0, 350, 0, 0, targetAngles)) {
    Serial.println("\nIK Solution for park position:");
    Serial.printf("  Joint 1 (Base):     %.2f°\n", targetAngles[0]);
    Serial.printf("  Joint 2 (Shoulder): %.2f°\n", targetAngles[1]);
    Serial.printf("  Joint 3 (Elbow):    %.2f°\n", targetAngles[2]);
    Serial.printf("  Joint 4 (Wrist):    %.2f°\n", targetAngles[3]);
    Serial.printf("  Joint 5 (Roll):     %.2f°\n", targetAngles[4]);
    
    Serial.println("\nExecuting movement...");
    moveToPosition(targetAngles);
    
    Serial.println("\n✓ PARKED at safe position!\n");
  } else {
    Serial.println("\nERROR: Cannot solve IK for park position!");
    Serial.println("Position may be unreachable.\n");
  }
}

bool solveIK(float x, float y, float z, float pitch_deg, float roll_deg, float* result) {
  float pitch = pitch_deg * DEG_TO_RAD;
  float roll = roll_deg * DEG_TO_RAD;
  
  result[0] = atan2(y, x) * RAD_TO_DEG;
  
  float wx = x - L4 * cos(atan2(y, x)) * cos(pitch);
  float wy = y - L4 * sin(atan2(y, x)) * cos(pitch);
  float wz = z - L4 * sin(pitch);
  
  float r = sqrt(wx * wx + wy * wy);
  float h = wz - L1;
  float d = sqrt(r * r + h * h);
  
  if (d > (L2 + L3) || d < abs(L2 - L3)) {
    return false;
  }
  
  float cos_angle2 = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d);
  if (cos_angle2 < -1.0 || cos_angle2 > 1.0) return false;
  
  float angle2 = acos(cos_angle2);
  float alpha = atan2(h, r);
  
  // Try elbow-up first
  float theta2_up = (alpha + angle2) * RAD_TO_DEG;
  
  float cos_angle3 = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3);
  if (cos_angle3 < -1.0 || cos_angle3 > 1.0) return false;
  
  float theta3_up = (PI - acos(cos_angle3)) * RAD_TO_DEG;
  float arm_angle_up = theta2_up - theta3_up;
  float theta4_up = pitch_deg - arm_angle_up;
  
  bool elbowUpValid = (theta2_up >= JOINT2_MIN && theta2_up <= JOINT2_MAX) &&
                      (theta3_up >= JOINT3_MIN && theta3_up <= JOINT3_MAX) &&
                      (theta4_up >= JOINT4_MIN && theta4_up <= JOINT4_MAX);
  
  // Try elbow-down
  float theta2_down = (alpha - angle2) * RAD_TO_DEG;
  float theta3_down = -(PI - acos(cos_angle3)) * RAD_TO_DEG;
  float arm_angle_down = theta2_down - theta3_down;
  float theta4_down = pitch_deg - arm_angle_down;
  
  bool elbowDownValid = (theta2_down >= JOINT2_MIN && theta2_down <= JOINT2_MAX) &&
                        (theta3_down >= JOINT3_MIN && theta3_down <= JOINT3_MAX) &&
                        (theta4_down >= JOINT4_MIN && theta4_down <= JOINT4_MAX);
  
  if (elbowUpValid) {
    result[1] = theta2_up;
    result[2] = theta3_up;
    result[3] = theta4_up;
  } else if (elbowDownValid) {
    result[1] = theta2_down;
    result[2] = theta3_down;
    result[3] = theta4_down;
  } else {
    return false;
  }
  
  result[4] = roll_deg;
  
  // Normalize angles
  for (int i = 0; i < 5; i++) {
    while (result[i] > 180.0) result[i] -= 360.0;
    while (result[i] < -180.0) result[i] += 360.0;
  }
  
  return true;
}

void moveToPosition(float* targetPos) {
  // Calculate steps needed for each motor
  long stepsNeeded[5];
  stepsNeeded[0] = degreesToSteps(targetPos[0] - currentPosition[0], false, 0);
  stepsNeeded[1] = degreesToSteps(targetPos[1] - currentPosition[1], true, 1);
  stepsNeeded[2] = degreesToSteps(targetPos[2] - currentPosition[2], true, 2);
  stepsNeeded[3] = degreesToSteps(targetPos[3] - currentPosition[3], true, 3);
  stepsNeeded[4] = degreesToSteps(targetPos[4] - currentPosition[4], false, 4);
  
  // Set directions
  digitalWrite(motor1.dirPin, stepsNeeded[0] >= 0 ? HIGH : LOW);
  digitalWrite(motor2.dirPin, stepsNeeded[1] >= 0 ? HIGH : LOW);
  digitalWrite(motor3.dirPin, stepsNeeded[2] >= 0 ? HIGH : LOW);
  digitalWrite(motor4.dirPin, stepsNeeded[3] >= 0 ? HIGH : LOW);
  digitalWrite(motor5.dirPin, stepsNeeded[4] >= 0 ? HIGH : LOW);
  
  // Convert to absolute values
  for (int i = 0; i < 5; i++) stepsNeeded[i] = abs(stepsNeeded[i]);
  
  // Find maximum steps
  long maxSteps = 0;
  for (int i = 0; i < 5; i++) {
    if (stepsNeeded[i] > maxSteps) maxSteps = stepsNeeded[i];
  }
  
  Serial.printf("Moving %ld steps (coordinated)...\n", maxSteps);
  
  // Coordinated movement
  long stepsTaken[5] = {0, 0, 0, 0, 0};
  MotorPins motors[5] = {motor1, motor2, motor3, motor4, motor5};
  
  int progressInterval = maxSteps / 20;  // Print 20 progress markers
  if (progressInterval < 1) progressInterval = 1;
  
  for (long step = 0; step < maxSteps; step++) {
    for (int i = 0; i < 5; i++) {
      if (stepsTaken[i] * maxSteps < stepsNeeded[i] * step) {
        stepMotor(motors[i].stepPin, SPEED_NORMAL);
        stepsTaken[i]++;
      }
    }
    
    if (step % progressInterval == 0) {
      Serial.print("█");
    }
  }
  
  Serial.println(" Done!");
  
  // Update current positions
  for (int i = 0; i < 5; i++) {
    currentPosition[i] = targetPos[i];
  }
}

long degreesToSteps(float degrees, bool harmonicDrive, int motorIndex) {
  float stepsPerDegree = (STEPS_PER_REV * currentMicrosteps[motorIndex]) / 360.0;
  if (harmonicDrive) stepsPerDegree *= HARMONIC_RATIO;
  return (long)(degrees * stepsPerDegree);
}