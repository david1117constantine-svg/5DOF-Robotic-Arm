/*
 * Complete 5DOF Robotic Arm Control System
 * ESP32-S3 with IK Solver, Web Interface, and Stepper Control
 * 
 * Features:
 * - Web-based IK solver with visualization
 * - Manual joint control
 * - Stepper motor control with A4988 drivers
 * - Serial terminal interface
 * - Movement confirmation system
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// WiFi credentials - UPDATE THESE
const char* ssid = "SSID";
const char* password = "PASSWORD";

WebServer server(80);

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

// Motor configuration
const int STEPS_PER_REV = 200;
const int HARMONIC_RATIO = 25;
int currentMicrosteps[5] = {16, 16, 16, 16, 16};

// Motor speed
const int SPEED_MEDIUM = 1200;

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

// Joint angles
float theta[5] = {0, 0, 0, 0, 0};
float currentPosition[5] = {0, 0, 0, 0, 0};
float pendingMove[5] = {0, 0, 0, 0, 0};
bool hasPendingMove = false;

// IK solver state
String lastSolveStatus = "Ready";
String lastSolveError = "";
float lastX = 0, lastY = 0, lastZ = 0, lastPitch = 0, lastRoll = 0;

// Serial buffer
String serialBuffer = "";
const int MAX_SERIAL_BUFFER = 5000;

// Motors enabled
bool motorsEnabled = true;

// Forward declarations
bool solveIK(float x, float y, float z, float pitch_deg, float roll_deg);
void handleRoot();
void handleSolve();
void handleStatus();
void handleSerial();
void handleSerialSend();
void handleManualMove();
void handleExecuteMove();
void moveToPosition(float j1, float j2, float j3, float j4, float j5);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== 5DOF Robot Arm Control System ===");
  
  // Initialize motors
  initMotor(motor1);
  initMotor(motor2);
  initMotor(motor3);
  initMotor(motor4);
  initMotor(motor5);
  
  setMicrostepping(motor1, 16);
  setMicrostepping(motor2, 16);
  setMicrostepping(motor5, 16);
  
  Serial.println("Motors initialized!");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/solve", handleSolve);
  server.on("/status", handleStatus);
  server.on("/serial", handleSerial);
  server.on("/serialSend", HTTP_POST, handleSerialSend);
  server.on("/manualMove", HTTP_POST, handleManualMove);
  server.on("/executeMove", HTTP_POST, handleExecuteMove);
  
  server.begin();
  Serial.println("Web server started!");
  Serial.println("======================================\n");
}

void loop() {
  server.handleClient();
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
  }
}

long degreesToSteps(float degrees, bool harmonicDrive, int motorIndex) {
  float stepsPerDegree = (STEPS_PER_REV * currentMicrosteps[motorIndex]) / 360.0;
  if (harmonicDrive) stepsPerDegree *= HARMONIC_RATIO;
  return (long)(degrees * stepsPerDegree);
}

void stepMotor(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(stepPin, LOW);
}

void moveToPosition(float j1, float j2, float j3, float j4, float j5) {
  if (!motorsEnabled) {
    Serial.println("Motors disabled!");
    return;
  }
  
  float targetPos[5] = {j1, j2, j3, j4, j5};
  
  // Calculate steps needed
  long stepsNeeded[5];
  stepsNeeded[0] = degreesToSteps(j1 - currentPosition[0], false, 0);
  stepsNeeded[1] = degreesToSteps(j2 - currentPosition[1], true, 1);
  stepsNeeded[2] = degreesToSteps(j3 - currentPosition[2], true, 2);
  stepsNeeded[3] = degreesToSteps(j4 - currentPosition[3], true, 3);
  stepsNeeded[4] = degreesToSteps(j5 - currentPosition[4], false, 4);
  
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
  
  // Coordinated movement
  long stepsTaken[5] = {0, 0, 0, 0, 0};
  MotorPins motors[5] = {motor1, motor2, motor3, motor4, motor5};
  
  for (long step = 0; step < maxSteps; step++) {
    for (int i = 0; i < 5; i++) {
      if (stepsTaken[i] * maxSteps < stepsNeeded[i] * step) {
        stepMotor(motors[i].stepPin);
        stepsTaken[i]++;
      }
    }
    delayMicroseconds(SPEED_MEDIUM);
  }
  
  // Update current positions
  for (int i = 0; i < 5; i++) {
    currentPosition[i] = targetPos[i];
  }
  
  Serial.println("Move complete!");
}

bool solveIK(float x, float y, float z, float pitch_deg, float roll_deg) {
  float pitch = pitch_deg * DEG_TO_RAD;
  float roll = roll_deg * DEG_TO_RAD;
  
  theta[0] = atan2(y, x) * RAD_TO_DEG;
  
  float wx = x - L4 * cos(atan2(y, x)) * cos(pitch);
  float wy = y - L4 * sin(atan2(y, x)) * cos(pitch);
  float wz = z - L4 * sin(pitch);
  
  float r = sqrt(wx * wx + wy * wy);
  float h = wz - L1;
  float d = sqrt(r * r + h * h);
  
  if (d > (L2 + L3) || d < abs(L2 - L3)) {
    lastSolveError = "Target out of reach (distance)";
    return false;
  }
  
  float cos_angle2 = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d);
  if (cos_angle2 < -1.0 || cos_angle2 > 1.0) {
    lastSolveError = "Invalid geometry (shoulder)";
    return false;
  }
  
  float angle2 = acos(cos_angle2);
  float alpha = atan2(h, r);
  
  // Try elbow-up first
  float theta2_up = (alpha + angle2) * RAD_TO_DEG;
  
  float cos_angle3 = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3);
  if (cos_angle3 < -1.0 || cos_angle3 > 1.0) {
    lastSolveError = "Invalid geometry (elbow)";
    return false;
  }
  
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
    theta[1] = theta2_up;
    theta[2] = theta3_up;
    theta[3] = theta4_up;
  } else if (elbowDownValid) {
    theta[1] = theta2_down;
    theta[2] = theta3_down;
    theta[3] = theta4_down;
  } else {
    lastSolveError = "Both elbow configurations exceed joint limits";
    return false;
  }
  
  theta[4] = roll_deg;
  
  for (int i = 0; i < 5; i++) {
    while (theta[i] > 180.0) theta[i] -= 360.0;
    while (theta[i] < -180.0) theta[i] += 360.0;
  }
  
  lastSolveError = "";
  return true;
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>5DOF Robot Arm Control</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Roboto', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
      background: #121212;
      color: #e0e0e0;
      min-height: 100vh;
      padding: 20px;
    }
    .header {
      background: #1e1e1e;
      border-bottom: 1px solid #2d2d2d;
      padding: 16px 24px;
      margin: -20px -20px 20px -20px;
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .header h1 {
      font-size: 20px;
      font-weight: 500;
      color: #ffffff;
      margin: 0;
    }
    .header-icon {
      width: 32px;
      height: 32px;
      background: #ef5350;
      border-radius: 4px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 18px;
    }
    .main-container {
      max-width: 1200px;
      margin: 0 auto;
    }
    .card {
      background: #2d2d2d;
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      margin-bottom: 16px;
      max-width: 600px;
      overflow: hidden;
    }
    .card-header {
      padding: 16px 20px;
      cursor: pointer;
      display: flex;
      justify-content: space-between;
      align-items: center;
      user-select: none;
      transition: background 0.2s;
    }
    .card-header:hover {
      background: #333333;
    }
    .card-title {
      font-size: 14px;
      font-weight: 500;
      color: #ffffff;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      margin: 0;
    }
    .card-icon {
      font-size: 18px;
      color: #9e9e9e;
      transition: transform 0.3s;
    }
    .card-icon.collapsed {
      transform: rotate(-90deg);
    }
    .card-content {
      padding: 0 20px 20px 20px;
      max-height: 2000px;
      overflow: hidden;
      transition: max-height 0.3s ease-out, padding 0.3s;
    }
    .card-content.collapsed {
      max-height: 0;
      padding: 0 20px;
    }
    .input-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
      gap: 16px;
      margin-bottom: 20px;
    }
    .input-group {
      margin-bottom: 16px;
    }
    label {
      display: block;
      font-size: 11px;
      color: #9e9e9e;
      font-weight: 500;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      margin-bottom: 6px;
    }
    .label-with-range {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 6px;
    }
    .range-info {
      font-size: 10px;
      color: #666;
      font-weight: 400;
      text-transform: none;
    }
    input[type="number"] {
      width: 100%;
      background: #1e1e1e;
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      padding: 8px 10px;
      color: #e0e0e0;
      font-size: 14px;
      transition: all 0.2s;
    }
    input:focus {
      outline: none;
      border-color: #ef5350;
      background: #252525;
    }
    .button-group {
      display: flex;
      gap: 10px;
      margin-top: 16px;
    }
    button {
      padding: 10px 16px;
      border: none;
      border-radius: 4px;
      font-size: 13px;
      font-weight: 500;
      cursor: pointer;
      transition: all 0.2s;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .btn-primary {
      background: #ef5350;
      color: white;
      flex: 1;
    }
    .btn-primary:hover {
      background: #e53935;
    }
    .btn-secondary {
      background: #3d3d3d;
      color: #e0e0e0;
      border: 1px solid #4d4d4d;
    }
    .btn-secondary:hover {
      background: #4d4d4d;
    }
    .btn-execute {
      background: #66bb6a;
      color: white;
      flex: 1;
    }
    .btn-execute:hover {
      background: #57a05a;
    }
    .btn-execute:disabled {
      background: #3d3d3d;
      color: #666;
      cursor: not-allowed;
    }
    .card.disabled {
      opacity: 0.5;
      pointer-events: none;
    }
    .joint-grid {
      display: grid;
      gap: 10px;
    }
    .joint-item {
      background: #1e1e1e;
      border: 1px solid #3d3d3d;
      border-left: 3px solid #ef5350;
      border-radius: 4px;
      padding: 12px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .joint-label {
      font-size: 11px;
      color: #9e9e9e;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .joint-value {
      font-size: 18px;
      font-weight: 500;
      color: #ef5350;
    }
    .joint-control {
      display: grid;
      grid-template-columns: 80px 1fr 80px;
      gap: 8px;
      align-items: center;
      background: #1e1e1e;
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      padding: 12px;
      margin-bottom: 8px;
    }
    .joint-control-label {
      font-size: 11px;
      color: #9e9e9e;
      text-transform: uppercase;
    }
    .joint-control-input {
      display: flex;
      gap: 8px;
      align-items: center;
    }
    .joint-control-input input {
      flex: 1;
      padding: 6px;
      font-size: 13px;
    }
    .joint-control-btn {
      padding: 6px 12px;
      font-size: 12px;
      min-width: 60px;
    }
    .status-bar {
      background: #2d2d2d;
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      padding: 10px 14px;
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 12px;
    }
    .status-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #4caf50;
      animation: pulse 2s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    .status-dot.error {
      background: #ef5350;
    }
    .results-card {
      display: none;
    }
    .results-card.show {
      display: block;
    }
    .error-card {
      background: #2d1515;
      border: 1px solid #5d2020;
      border-left: 3px solid #ef5350;
      border-radius: 4px;
      padding: 14px;
      color: #ef9a9a;
      font-size: 13px;
    }
    .pending-move {
      background: #2d2515;
      border: 1px solid #5d5020;
      border-left: 3px solid #ffa726;
      border-radius: 4px;
      padding: 14px;
      color: #ffb74d;
      font-size: 13px;
      margin-bottom: 16px;
    }
    .math-section {
      margin-bottom: 20px;
    }
    .math-section:last-child {
      margin-bottom: 0;
    }
    .math-title {
      color: #ef5350;
      font-size: 13px;
      font-weight: 600;
      margin-bottom: 8px;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .math-line {
      font-family: 'Courier New', monospace;
      color: #b0b0b0;
      font-size: 13px;
      line-height: 1.8;
      padding: 4px 0;
    }
    .math-var {
      color: #66bb6a;
    }
    .math-result {
      color: #ef5350;
      font-weight: 600;
    }
    .math-comment {
      color: #666;
      font-style: italic;
      margin-left: 10px;
    }
    .math-error {
      background: #2d1515;
      border-left: 3px solid #ef5350;
      padding: 12px;
      margin: 8px 0;
      color: #ef9a9a;
      border-radius: 4px;
    }
    .math-warning {
      background: #2d2515;
      border-left: 3px solid #ffa726;
      padding: 12px;
      margin: 8px 0;
      color: #ffb74d;
      border-radius: 4px;
    }
    .terminal {
      background: #0d0d0d;
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      padding: 12px;
      font-family: 'Courier New', monospace;
      font-size: 13px;
      color: #00ff00;
      height: 300px;
      overflow-y: auto;
      margin-bottom: 12px;
      white-space: pre-wrap;
      word-wrap: break-word;
    }
    .terminal-input-group {
      display: flex;
      gap: 8px;
    }
    .terminal-input {
      flex: 1;
      background: #1e1e1e;
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      padding: 10px;
      color: #00ff00;
      font-family: 'Courier New', monospace;
      font-size: 13px;
    }
    .terminal-input:focus {
      outline: none;
      border-color: #ef5350;
    }
    .btn-terminal {
      background: #2d2d2d;
      color: #00ff00;
      border: 1px solid #3d3d3d;
      padding: 10px 20px;
      border-radius: 4px;
      cursor: pointer;
      font-family: 'Courier New', monospace;
      font-size: 13px;
      transition: background 0.2s;
    }
    .btn-terminal:hover {
      background: #3d3d3d;
    }
  </style>
</head>
<body>
  <div class="header">
    <div class="header-icon">🦾</div>
    <h1>Robot Arm Control System</h1>
  </div>
  
  <div class="main-container">
    <div class="card">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">Current Position</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content">
        <div class="joint-grid">
          <div class="joint-item">
            <div class="joint-label">Joint 1 (Base)</div>
            <div class="joint-value" id="currentJ1">0.00°</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Joint 2 (Shoulder)</div>
            <div class="joint-value" id="currentJ2">0.00°</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Joint 3 (Elbow)</div>
            <div class="joint-value" id="currentJ3">0.00°</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Joint 4 (Wrist Pitch)</div>
            <div class="joint-value" id="currentJ4">0.00°</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Joint 5 (Wrist Roll)</div>
            <div class="joint-value" id="currentJ5">0.00°</div>
          </div>
        </div>
      </div>
    </div>
    
    <div class="card">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">Manual Joint Control</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content">
        <div class="joint-control">
          <div class="joint-control-label">Joint 1</div>
          <div class="joint-control-input">
            <input type="number" id="manual1" value="0" step="1">
            <span style="color:#666">°</span>
          </div>
          <button class="btn-secondary joint-control-btn" onclick="moveJoint(0)">Move</button>
        </div>
        <div class="joint-control">
          <div class="joint-control-label">Joint 2</div>
          <div class="joint-control-input">
            <input type="number" id="manual2" value="0" step="1">
            <span style="color:#666">°</span>
          </div>
          <button class="btn-secondary joint-control-btn" onclick="moveJoint(1)">Move</button>
        </div>
        <div class="joint-control">
          <div class="joint-control-label">Joint 3</div>
          <div class="joint-control-input">
            <input type="number" id="manual3" value="0" step="1">
            <span style="color:#666">°</span>
          </div>
          <button class="btn-secondary joint-control-btn" onclick="moveJoint(2)">Move</button>
        </div>
        <div class="joint-control">
          <div class="joint-control-label">Joint 4</div>
          <div class="joint-control-input">
            <input type="number" id="manual4" value="0" step="1">
            <span style="color:#666">°</span>
          </div>
          <button class="btn-secondary joint-control-btn" onclick="moveJoint(3)">Move</button>
        </div>
        <div class="joint-control">
          <div class="joint-control-label">Joint 5</div>
          <div class="joint-control-input">
            <input type="number" id="manual5" value="0" step="1">
            <span style="color:#666">°</span>
          </div>
          <button class="btn-secondary joint-control-btn" onclick="moveJoint(4)">Move</button>
        </div>
        <div class="button-group">
          <button class="btn-secondary" onclick="homeAll()">Home All</button>
          <button class="btn-secondary" onclick="updateManualFromCurrent()">Sync Values</button>
        </div>
      </div>
    </div>
    
    <div class="card">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">IK Solver - Target Position</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content">
        <div class="input-grid">
          <div class="input-group">
            <div class="label-with-range">
              <label>X Position (mm)</label>
              <span class="range-info">Max reach ~360 mm</span>
            </div>
            <input type="number" id="x" step="0.01" value="200">
          </div>
          
          <div class="input-group">
            <div class="label-with-range">
              <label>Y Position (mm)</label>
              <span class="range-info">Max reach ~360 mm</span>
            </div>
            <input type="number" id="y" step="0.01" value="100">
          </div>
          
          <div class="input-group">
            <div class="label-with-range">
              <label>Z Position (mm)</label>
              <span class="range-info">Approx -152 to 513 mm</span>
            </div>
            <input type="number" id="z" step="0.01" value="200">
          </div>
          
          <div class="input-group">
            <div class="label-with-range">
              <label>Pitch (degrees)</label>
              <span class="range-info">Any angle</span>
            </div>
            <input type="number" id="pitch" step="0.01" value="0">
          </div>
          
          <div class="input-group">
            <div class="label-with-range">
              <label>Roll (degrees)</label>
              <span class="range-info">Any angle</span>
            </div>
            <input type="number" id="roll" step="0.01" value="0">
          </div>
        </div>
        
        <div class="button-group">
          <button class="btn-primary" onclick="solveIK()">Calculate IK</button>
          <button class="btn-secondary" onclick="resetForm()">Reset</button>
        </div>
      </div>
    </div>
    
    <div id="executeCard" class="card disabled">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">Execute Movement</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content">
        <div class="pending-move" id="pendingMoveInfo">No movement pending. Calculate IK first.</div>
        <div class="button-group">
          <button class="btn-execute" id="executeBtn" onclick="executeMove()" disabled>Execute Move</button>
          <button class="btn-secondary" onclick="cancelMove()">Cancel</button>
        </div>
      </div>
    </div>
    
    <div id="mathCard" class="card disabled">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">IK Calculations</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content" id="mathContent">
        <p style="color:#666;">Calculate IK to see detailed mathematics.</p>
      </div>
    </div>
    
    <div class="card">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">Serial Terminal</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content">
        <div class="terminal" id="terminal">ESP32 Serial Monitor
Ready...
</div>
        <div class="terminal-input-group">
          <input type="text" class="terminal-input" id="terminalInput" placeholder="Enter command (e.g., 200 100 150 0 0)">
          <button class="btn-terminal" onclick="sendSerial()">Send</button>
          <button class="btn-terminal" onclick="clearTerminal()">Clear</button>
        </div>
      </div>
    </div>
    
    <div id="results" class="card results-card"></div>
    
    <div class="status-bar">
      <div class="status-dot" id="statusDot"></div>
      <span id="status">Ready</span>
    </div>
  </div>

  <script>
    let currentPos = [0, 0, 0, 0, 0];
    let pendingAngles = null;
    
    function toggleCard(header) {
      const content = header.nextElementSibling;
      const icon = header.querySelector('.card-icon');
      content.classList.toggle('collapsed');
      icon.classList.toggle('collapsed');
    }
    
    function updateCurrentDisplay() {
      document.getElementById('currentJ1').textContent = currentPos[0].toFixed(2) + '°';
      document.getElementById('currentJ2').textContent = currentPos[1].toFixed(2) + '°';
      document.getElementById('currentJ3').textContent = currentPos[2].toFixed(2) + '°';
      document.getElementById('currentJ4').textContent = currentPos[3].toFixed(2) + '°';
      document.getElementById('currentJ5').textContent = currentPos[4].toFixed(2) + '°';
    }
    
    function updateManualFromCurrent() {
      for (let i = 0; i < 5; i++) {
        document.getElementById('manual' + (i + 1)).value = currentPos[i].toFixed(2);
      }
    }
    
    function enableExecuteCard() {
      document.getElementById('executeCard').classList.remove('disabled');
      document.getElementById('executeBtn').disabled = false;
    }
    
    function disableExecuteCard() {
      document.getElementById('executeCard').classList.add('disabled');
      document.getElementById('executeBtn').disabled = true;
      document.getElementById('pendingMoveInfo').innerHTML = 'No movement pending. Calculate IK first.';
    }
    
    function enableMathCard() {
      document.getElementById('mathCard').classList.remove('disabled');
    }
    
    function disableMathCard() {
      document.getElementById('mathCard').classList.add('disabled');
      document.getElementById('mathContent').innerHTML = '<p style="color:#666;">Calculate IK to see detailed mathematics.</p>';
    }
    
    function displayMathCalculations(x, y, z, pitch, roll, success = true, errorMsg = '') {
      const L1 = 152.5, L2 = 180, L3 = 180, L4 = 101;
      const pitchRad = pitch * Math.PI / 180;
      
      const theta1 = Math.atan2(y, x) * 180 / Math.PI;
      const baseAngle = Math.atan2(y, x);
      
      const wx = x - L4 * Math.cos(baseAngle) * Math.cos(pitchRad);
      const wy = y - L4 * Math.sin(baseAngle) * Math.cos(pitchRad);
      const wz = z - L4 * Math.sin(pitchRad);
      
      const r = Math.sqrt(wx * wx + wy * wy);
      const h = wz - L1;
      const d = Math.sqrt(r * r + h * h);
      
      const maxReach = L2 + L3;
      const minReach = Math.abs(L2 - L3);
      const reachable = d <= maxReach && d >= minReach;
      
      let cos_angle2 = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d);
      let cos_angle3 = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3);
      
      let angle2 = null, alpha = null, theta2 = null, theta3 = null, theta4 = null;
      let angle2Valid = false, angle3Valid = false;
      
      if (cos_angle2 >= -1.0 && cos_angle2 <= 1.0) {
        angle2Valid = true;
        angle2 = Math.acos(cos_angle2);
        alpha = Math.atan2(h, r);
        theta2 = (alpha + angle2) * 180 / Math.PI;
      }
      
      if (cos_angle3 >= -1.0 && cos_angle3 <= 1.0) {
        angle3Valid = true;
        theta3 = (Math.PI - Math.acos(cos_angle3)) * 180 / Math.PI;
      }
      
      if (theta2 !== null && theta3 !== null) {
        const arm_angle = theta2 - theta3;
        theta4 = pitch - arm_angle;
      }
      
      const j2InRange = theta2 !== null && theta2 >= -70 && theta2 <= 70;
      const j3InRange = theta3 !== null && theta3 >= -80 && theta3 <= 80;
      const j4InRange = theta4 !== null && theta4 >= -90 && theta4 <= 90;
      
      const mathHTML = `
        <div class="math-section">
          <div class="math-title">Step 1: Base Rotation (Joint 1)</div>
          <div class="math-line"><span class="math-var">θ₁</span> = atan2(<span class="math-var">y</span>, <span class="math-var">x</span>)</div>
          <div class="math-line"><span class="math-var">θ₁</span> = atan2(${y.toFixed(2)}, ${x.toFixed(2)})</div>
          <div class="math-line"><span class="math-result">θ₁ = ${theta1.toFixed(2)}°</span></div>
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 2: Wrist Center Position</div>
          <div class="math-line"><span class="math-var">w<sub>x</sub></span> = ${x.toFixed(2)} - ${L4} × ${Math.cos(baseAngle).toFixed(3)} × ${Math.cos(pitchRad).toFixed(3)}</div>
          <div class="math-line"><span class="math-result">w<sub>x</sub> = ${wx.toFixed(2)} mm</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-result">w<sub>y</sub> = ${wy.toFixed(2)} mm</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-var">w<sub>z</sub></span> = ${z.toFixed(2)} - ${L4} × ${Math.sin(pitchRad).toFixed(3)}</div>
          <div class="math-line"><span class="math-result">w<sub>z</sub> = ${wz.toFixed(2)} mm</span></div>
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 3: Arm Geometry</div>
          <div class="math-line"><span class="math-var">r</span> = √(${wx.toFixed(2)}² + ${wy.toFixed(2)}²)</div>
          <div class="math-line"><span class="math-result">r = ${r.toFixed(2)} mm</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-var">h</span> = ${wz.toFixed(2)} - ${L1}</div>
          <div class="math-line"><span class="math-result">h = ${h.toFixed(2)} mm</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-var">d</span> = √(${r.toFixed(2)}² + ${h.toFixed(2)}²)</div>
          <div class="math-line"><span class="math-result">d = ${d.toFixed(2)} mm</span></div>
          ${!reachable ? `<div class="math-error">❌ Distance check failed! d = ${d.toFixed(2)} mm must be between ${minReach.toFixed(2)} mm and ${maxReach.toFixed(2)} mm</div>` : `<div class="math-warning">✓ Distance OK: ${minReach.toFixed(2)} ≤ ${d.toFixed(2)} ≤ ${maxReach.toFixed(2)}</div>`}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 4: Joint 2 (Shoulder)</div>
          <div class="math-line">cos(<span class="math-var">angle₂</span>) = (${L2}² + ${d.toFixed(2)}² - ${L3}²) / (2 × ${L2} × ${d.toFixed(2)})</div>
          <div class="math-line">cos(<span class="math-var">angle₂</span>) = ${cos_angle2.toFixed(4)}</div>
          ${!angle2Valid ? `<div class="math-error">❌ Invalid cosine value!</div>` : `
          <div class="math-line"><span class="math-var">angle₂</span> = ${(angle2 * 180 / Math.PI).toFixed(2)}°</div>
          <div class="math-line"><span class="math-var">α</span> = atan2(${h.toFixed(2)}, ${r.toFixed(2)}) = ${(alpha * 180 / Math.PI).toFixed(2)}°</div>
          <div class="math-line"><span class="math-result">θ₂ = ${theta2.toFixed(2)}°</span></div>
          ${!j2InRange ? `<div class="math-error">❌ Joint 2 limit exceeded! (±70°)</div>` : `<div class="math-warning">✓ Joint 2 OK</div>`}
          `}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 5: Joint 3 (Elbow)</div>
          <div class="math-line">cos(<span class="math-var">angle₃</span>) = ${cos_angle3.toFixed(4)}</div>
          ${!angle3Valid ? `<div class="math-error">❌ Invalid cosine value!</div>` : `
          <div class="math-line"><span class="math-result">θ₃ = ${theta3.toFixed(2)}°</span></div>
          ${!j3InRange ? `<div class="math-error">❌ Joint 3 limit exceeded! (±80°)</div>` : `<div class="math-warning">✓ Joint 3 OK</div>`}
          `}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 6: Joint 4 (Wrist Pitch)</div>
          ${theta2 !== null && theta3 !== null ? `
          <div class="math-line"><span class="math-result">θ₄ = ${theta4.toFixed(2)}°</span></div>
          ${!j4InRange ? `<div class="math-error">❌ Joint 4 limit exceeded! (±90°)</div>` : `<div class="math-warning">✓ Joint 4 OK</div>`}
          ` : '<div class="math-error">Cannot calculate</div>'}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 7: Joint 5 (Wrist Roll)</div>
          <div class="math-line"><span class="math-result">θ₅ = ${roll}°</span> <span class="math-comment">// Direct passthrough</span></div>
        </div>
      `;
      
      document.getElementById('mathContent').innerHTML = mathHTML;
      enableMathCard();
    }
    
    function moveJoint(index) {
      const value = parseFloat(document.getElementById('manual' + (index + 1)).value);
      const angles = [...currentPos];
      angles[index] = value;
      
      fetch('/manualMove', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({angles: angles})
      })
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          currentPos = angles;
          updateCurrentDisplay();
          document.getElementById('status').textContent = 'Joint ' + (index + 1) + ' moved';
        }
      });
    }
    
    function homeAll() {
      const angles = [0, 0, 0, 0, 0];
      fetch('/manualMove', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({angles: angles})
      })
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          currentPos = angles;
          updateCurrentDisplay();
          updateManualFromCurrent();
          document.getElementById('status').textContent = 'Homed to zero';
        }
      });
    }
    
    function solveIK() {
      const x = document.getElementById('x').value;
      const y = document.getElementById('y').value;
      const z = document.getElementById('z').value;
      const pitch = document.getElementById('pitch').value;
      const roll = document.getElementById('roll').value;
      
      const statusDot = document.getElementById('statusDot');
      const statusText = document.getElementById('status');
      
      statusText.textContent = 'Calculating...';
      statusDot.className = 'status-dot';
      
      fetch(`/solve?x=${x}&y=${y}&z=${z}&pitch=${pitch}&roll=${roll}`)
        .then(response => response.json())
        .then(data => {
          const resultsDiv = document.getElementById('results');
          
          if (data.success) {
            pendingAngles = data.angles;
            
            displayMathCalculations(parseFloat(x), parseFloat(y), parseFloat(z), parseFloat(pitch), parseFloat(roll), true, '');
            
            resultsDiv.innerHTML = `
              <div class="card-header" onclick="toggleCard(this)">
                <div class="card-title">Calculated Joint Angles</div>
                <div class="card-icon">▼</div>
              </div>
              <div class="card-content">
                <div class="joint-grid">
                  ${data.angles.map((angle, i) => `
                    <div class="joint-item">
                      <div class="joint-label">Joint ${i + 1}</div>
                      <div class="joint-value">${angle}°</div>
                    </div>
                  `).join('')}
                </div>
              </div>
            `;
            resultsDiv.className = 'card results-card show';
            
            enableExecuteCard();
            document.getElementById('pendingMoveInfo').innerHTML = `
              ⚠️ Movement calculated and ready to execute.<br>
              Target: X=${x} Y=${y} Z=${z} Pitch=${pitch}° Roll=${roll}°<br>
              <strong>Click "Execute Move" to send to motors.</strong>
            `;
            
            statusText.textContent = 'Solution ready - awaiting confirmation';
            statusDot.className = 'status-dot';
          } else {
            pendingAngles = null;
            disableExecuteCard();
            disableMathCard();
            
            const errorMsg = data.error || 'Target position unreachable';
            resultsDiv.innerHTML = `
              <div class="card-header" onclick="toggleCard(this)">
                <div class="card-title">Error</div>
                <div class="card-icon">▼</div>
              </div>
              <div class="card-content">
                <div class="error-card">${errorMsg}</div>
              </div>
            `;
            resultsDiv.className = 'card results-card show';
            statusText.textContent = 'Error';
            statusDot.className = 'status-dot error';
          }
        })
        .catch(error => {
          pendingAngles = null;
          disableExecuteCard();
          disableMathCard();
          statusText.textContent = 'Connection error';
          statusDot.className = 'status-dot error';
        });
    }
    
    function executeMove() {
      if (!pendingAngles) return;
      
      document.getElementById('status').textContent = 'Executing movement...';
      
      fetch('/executeMove', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({angles: pendingAngles})
      })
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          currentPos = pendingAngles;
          updateCurrentDisplay();
          updateManualFromCurrent();
          pendingAngles = null;
          disableExecuteCard();
          document.getElementById('status').textContent = 'Movement complete!';
        } else {
          document.getElementById('status').textContent = 'Movement failed';
        }
      });
    }
    
    function cancelMove() {
      pendingAngles = null;
      disableExecuteCard();
      disableMathCard();
      document.getElementById('status').textContent = 'Movement cancelled';
    }
    
    function resetForm() {
      document.getElementById('x').value = '200';
      document.getElementById('y').value = '100';
      document.getElementById('z').value = '200';
      document.getElementById('pitch').value = '0';
      document.getElementById('roll').value = '0';
      document.getElementById('results').className = 'card results-card';
      pendingAngles = null;
      disableExecuteCard();
      disableMathCard();
      document.getElementById('status').textContent = 'Ready';
      document.getElementById('statusDot').className = 'status-dot';
    }
    
    updateCurrentDisplay();
    
    // Start polling serial output
    setInterval(updateTerminal, 500);
    
    // Allow Enter key to send in terminal
    document.getElementById('terminalInput').addEventListener('keypress', function(e) {
      if (e.key === 'Enter') {
        sendSerial();
      }
    });
    
    function updateTerminal() {
      fetch('/serial')
        .then(response => response.text())
        .then(data => {
          const terminal = document.getElementById('terminal');
          if (data && data !== terminal.textContent) {
            terminal.textContent = data;
            terminal.scrollTop = terminal.scrollHeight;
          }
        })
        .catch(err => console.error('Terminal update error:', err));
    }
    
    function sendSerial() {
      const input = document.getElementById('terminalInput');
      const command = input.value;
      
      if (!command.trim()) return;
      
      const terminal = document.getElementById('terminal');
      terminal.textContent += '> ' + command + '\n';
      terminal.scrollTop = terminal.scrollHeight;
      
      fetch('/serialSend', {
        method: 'POST',
        headers: {'Content-Type': 'text/plain'},
        body: command
      })
      .then(response => response.text())
      .then(data => {
        if (data) {
          terminal.textContent += data;
          terminal.scrollTop = terminal.scrollHeight;
        }
      })
      .catch(err => {
        terminal.textContent += 'Error: ' + err + '\n';
        terminal.scrollTop = terminal.scrollHeight;
      });
      
      input.value = '';
    }
    
    function clearTerminal() {
      document.getElementById('terminal').textContent = 'ESP32 Serial Monitor\nReady...\n';
    }
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

void handleSolve() {
  if (server.hasArg("x") && server.hasArg("y") && server.hasArg("z") && 
      server.hasArg("pitch") && server.hasArg("roll")) {
    
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    float z = server.arg("z").toFloat();
    float pitch = server.arg("pitch").toFloat();
    float roll = server.arg("roll").toFloat();
    
    lastX = x;
    lastY = y;
    lastZ = z;
    lastPitch = pitch;
    lastRoll = roll;
    
    bool success = solveIK(x, y, z, pitch, roll);
    
    String json = "{";
    json += "\"success\":" + String(success ? "true" : "false") + ",";
    if (!success && lastSolveError.length() > 0) {
      json += "\"error\":\"" + lastSolveError + "\",";
    }
    json += "\"angles\":[";
    for (int i = 0; i < 5; i++) {
      json += String(theta[i], 2);
      if (i < 4) json += ",";
    }
    json += "]}";
    
    if (success) {
      // Store as pending move
      for (int i = 0; i < 5; i++) pendingMove[i] = theta[i];
      hasPendingMove = true;
      lastSolveStatus = "Pending";
    } else {
      hasPendingMove = false;
      lastSolveStatus = "Failed";
    }
    
    Serial.println("\n=== IK Solve ===");
    Serial.printf("Target: X=%.2f Y=%.2f Z=%.2f Pitch=%.2f Roll=%.2f\n", x, y, z, pitch, roll);
    if (success) {
      Serial.println("Solution found - awaiting execution");
    } else {
      Serial.print("Failed: ");
      Serial.println(lastSolveError);
    }
    Serial.println("================\n");
    
    server.send(200, "application/json", json);
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"status\":\"" + lastSolveStatus + "\",";
  json += "\"currentPosition\":[";
  for (int i = 0; i < 5; i++) {
    json += String(currentPosition[i], 2);
    if (i < 4) json += ",";
  }
  json += "]}";
  
  server.send(200, "application/json", json);
}

void handleSerial() {
  server.send(200, "text/plain", serialBuffer);
}

void handleSerialSend() {
  if (server.hasArg("plain")) {
    String command = server.arg("plain");
    Serial.println(command);
    serialBuffer += "> " + command + "\n";
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "No command");
  }
}

void handleManualMove() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    // Parse JSON manually (simple approach)
    int idx = body.indexOf("[");
    int endIdx = body.indexOf("]");
    String anglesStr = body.substring(idx + 1, endIdx);
    
    float angles[5];
    int lastComma = -1;
    for (int i = 0; i < 5; i++) {
      int nextComma = anglesStr.indexOf(',', lastComma + 1);
      if (nextComma == -1) nextComma = anglesStr.length();
      angles[i] = anglesStr.substring(lastComma + 1, nextComma).toFloat();
      lastComma = nextComma;
    }
    
    Serial.println("\n=== Manual Move ===");
    for (int i = 0; i < 5; i++) {
      Serial.printf("Joint %d: %.2f°\n", i + 1, angles[i]);
    }
    
    moveToPosition(angles[0], angles[1], angles[2], angles[3], angles[4]);
    
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
  }
}

void handleExecuteMove() {
  if (!hasPendingMove) {
    server.send(400, "application/json", "{\"error\":\"No pending move\"}");
    return;
  }
  
  Serial.println("\n=== Executing IK Solution ===");
  for (int i = 0; i < 5; i++) {
    Serial.printf("Joint %d: %.2f°\n", i + 1, pendingMove[i]);
  }
  
  moveToPosition(pendingMove[0], pendingMove[1], pendingMove[2], pendingMove[3], pendingMove[4]);
  
  hasPendingMove = false;
  lastSolveStatus = "Complete";
  
  server.send(200, "application/json", "{\"success\":true}");
}
