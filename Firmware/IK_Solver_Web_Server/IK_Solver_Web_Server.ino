/*
 * 5DOF Robotic Arm Inverse Kinematics for ESP32-S3
 * With WiFi Web Interface and 3D Visualizer
 * 
 * Setup: Update WiFi credentials below
 * Access: Connect to ESP32's IP address shown in Serial Monitor
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// WiFi credentials - UPDATE THESE
const char* ssid = "SSID";
const char* password = "PASSWORD";

WebServer server(80);

// Modified DH Parameters (in mm)
struct DHParam {
  float alpha;
  float a;
  float theta;
  float d;
};

DHParam dh[5] = {
  {0.0,   0.0,   0.0, 152.5},
  {90.0,  0.0,   90.0, 0.0},
  {0.0,   180.0, 0.0, 0.0},
  {0.0,   180.0, 0.0, 90.0},
  {90.0,  0.0,   101.0, 0.0}
};

// Link lengths
const float L1 = 152.5;
const float L2 = 180.0;
const float L3 = 180.0;
const float L4 = 101.0;

// Joint limits (in degrees, from vertical parked position)
const float JOINT1_MIN = -180.0;
const float JOINT1_MAX = 180.0;
const float JOINT2_MIN = -70.0;
const float JOINT2_MAX = 70.0;
const float JOINT3_MIN = -80.0;
const float JOINT3_MAX = 80.0;
const float JOINT4_MIN = -90.0;
const float JOINT4_MAX = 90.0;
const float JOINT5_MIN = -180.0;
const float JOINT5_MAX = 180.0;

// Joint angles (in degrees)
float theta[5] = {0, 0, 0, 0, 0};

// Last solve status
String lastSolveStatus = "Ready";
String lastSolveError = "";
float lastX = 0, lastY = 0, lastZ = 0, lastPitch = 0, lastRoll = 0;

// Forward declarations
bool solveIK(float x, float y, float z, float pitch_deg, float roll_deg);
void handleRoot();
void handleSolve();
void handleStatus();
void handleSerial();
void handleSerialSend();

// Serial buffer for web terminal
String serialBuffer = "";
const int MAX_SERIAL_BUFFER = 5000;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== 5DOF Robot Arm IK Solver with WiFi ===");
  
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
  Serial.print("Access the web interface at: http://");
  Serial.println(WiFi.localIP());
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/solve", handleSolve);
  server.on("/status", handleStatus);
  server.on("/serial", handleSerial);
  server.on("/serialSend", HTTP_POST, handleSerialSend);
  
  server.begin();
  Serial.println("Web server started!");
  Serial.println("==========================================\n");
}

void loop() {
  server.handleClient();
  
  // Also handle serial input
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    serialBuffer += line + "\n";
    
    // Keep buffer size manageable
    if (serialBuffer.length() > MAX_SERIAL_BUFFER) {
      serialBuffer = serialBuffer.substring(serialBuffer.length() - MAX_SERIAL_BUFFER);
    }
    
    float x = line.toFloat();
    // Parse command if it's coordinates
    int spaceCount = 0;
    for (char c : line) if (c == ' ') spaceCount++;
    
    if (spaceCount >= 4) {
      int idx = 0;
      float vals[5];
      String temp = line;
      for (int i = 0; i < 5; i++) {
        idx = temp.indexOf(' ');
        if (idx == -1 && i < 4) break;
        vals[i] = temp.substring(0, idx).toFloat();
        temp = temp.substring(idx + 1);
      }
      
      if (spaceCount >= 4) {
        if (solveIK(vals[0], vals[1], vals[2], vals[3], vals[4])) {
          Serial.println("\n=== Joint Angles (degrees) ===");
          for (int i = 0; i < 5; i++) {
            Serial.print("Joint ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println(theta[i], 2);
            serialBuffer += "Joint " + String(i + 1) + ": " + String(theta[i], 2) + "°\n";
          }
          Serial.println("==============================\n");
          serialBuffer += "==============================\n\n";
        } else {
          Serial.println("ERROR: Target unreachable!\n");
          serialBuffer += "ERROR: Target unreachable!\n\n";
        }
      }
    }
  }
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
  
  // ELBOW UP/DOWN CONFIGURATION
  // We have two solutions: elbow up and elbow down
  // Elbow up: shoulder angle is alpha + angle2 (elbow above wrist)
  // Elbow down: shoulder angle is alpha - angle2 (elbow below wrist)
  
  // Try elbow-up configuration first (preferred)
  float theta2_up = (alpha + angle2) * RAD_TO_DEG;
  
  float cos_angle3 = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3);
  if (cos_angle3 < -1.0 || cos_angle3 > 1.0) {
    lastSolveError = "Invalid geometry (elbow)";
    return false;
  }
  
  float theta3_up = (PI - acos(cos_angle3)) * RAD_TO_DEG;
  
  float arm_angle_up = theta2_up - theta3_up;
  float theta4_up = pitch_deg - arm_angle_up;
  
  // Check if elbow-up configuration is within limits
  bool elbowUpValid = (theta2_up >= JOINT2_MIN && theta2_up <= JOINT2_MAX) &&
                      (theta3_up >= JOINT3_MIN && theta3_up <= JOINT3_MAX) &&
                      (theta4_up >= JOINT4_MIN && theta4_up <= JOINT4_MAX);
  
  // Try elbow-down configuration (fallback)
  float theta2_down = (alpha - angle2) * RAD_TO_DEG;
  float theta3_down = -(PI - acos(cos_angle3)) * RAD_TO_DEG;
  float arm_angle_down = theta2_down - theta3_down;
  float theta4_down = pitch_deg - arm_angle_down;
  
  bool elbowDownValid = (theta2_down >= JOINT2_MIN && theta2_down <= JOINT2_MAX) &&
                        (theta3_down >= JOINT3_MIN && theta3_down <= JOINT3_MAX) &&
                        (theta4_down >= JOINT4_MIN && theta4_down <= JOINT4_MAX);
  
  // Prefer elbow-up, use elbow-down only if elbow-up fails
  if (elbowUpValid) {
    theta[1] = theta2_up;
    theta[2] = theta3_up;
    theta[3] = theta4_up;
  } else if (elbowDownValid) {
    theta[1] = theta2_down;
    theta[2] = theta3_down;
    theta[3] = theta4_down;
  } else {
    // Report which configuration was closest to working
    if (!elbowUpValid && !elbowDownValid) {
      lastSolveError = "Both elbow-up and elbow-down exceed joint limits";
    } else {
      lastSolveError = "Joint limits exceeded";
    }
    return false;
  }
  
  theta[4] = roll_deg;
  
  // Normalize angles to reasonable ranges
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
      padding: 20px;
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
    input {
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
    input:hover {
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
    .results-card {
      display: none;
    }
    .results-card.show {
      display: block;
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
    .error-card {
      background: #2d1515;
      border: 1px solid #5d2020;
      border-left: 3px solid #ef5350;
      border-radius: 4px;
      padding: 14px;
      color: #ef9a9a;
      font-size: 13px;
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
    canvas {
      border: 1px solid #3d3d3d;
      border-radius: 4px;
      margin: 16px 0;
      background: #1a1a1a;
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
    <h1>Robot Arm Control</h1>
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
            <div class="joint-label">X Position</div>
            <div class="joint-value" id="currentX">0.00 mm</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Y Position</div>
            <div class="joint-value" id="currentY">0.00 mm</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Z Position</div>
            <div class="joint-value" id="currentZ">0.00 mm</div>
          </div>
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
            <div class="joint-label">Joint 4 (Wrist 1)</div>
            <div class="joint-value" id="currentJ4">0.00°</div>
          </div>
          <div class="joint-item">
            <div class="joint-label">Joint 5 (Wrist 2)</div>
            <div class="joint-value" id="currentJ5">0.00°</div>
          </div>
        </div>
      </div>
    </div>
    
    <div class="card">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">Target Position</div>
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
          <button class="btn-primary" onclick="solveIK()">Calculate</button>
          <button class="btn-secondary" onclick="resetForm()">Reset</button>
        </div>
      </div>
    </div>
    
    <div id="mathCard" class="card" style="display:none;">
      <div class="card-header" onclick="toggleCard(this)">
        <div class="card-title">IK Calculations</div>
        <div class="card-icon">▼</div>
      </div>
      <div class="card-content" id="mathContent">
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
    // Current position state
    let currentPosition = {
      x: 0, y: 0, z: 0,
      angles: [0, 0, 0, 0, 0]
    };
    
    // IK calculation data for display
    let ikData = {};
    
    function toggleCard(header) {
      const content = header.nextElementSibling;
      const icon = header.querySelector('.card-icon');
      
      content.classList.toggle('collapsed');
      icon.classList.toggle('collapsed');
    }
    
    function updateCurrentDisplay() {
      document.getElementById('currentX').textContent = currentPosition.x.toFixed(2) + ' mm';
      document.getElementById('currentY').textContent = currentPosition.y.toFixed(2) + ' mm';
      document.getElementById('currentZ').textContent = currentPosition.z.toFixed(2) + ' mm';
      document.getElementById('currentJ1').textContent = currentPosition.angles[0].toFixed(2) + '°';
      document.getElementById('currentJ2').textContent = currentPosition.angles[1].toFixed(2) + '°';
      document.getElementById('currentJ3').textContent = currentPosition.angles[2].toFixed(2) + '°';
      document.getElementById('currentJ4').textContent = currentPosition.angles[3].toFixed(2) + '°';
      document.getElementById('currentJ5').textContent = currentPosition.angles[4].toFixed(2) + '°';
    }
    
    function displayMathCalculations(x, y, z, pitch, roll, success = true, errorMsg = '') {
      const L1 = 152.5, L2 = 180, L3 = 180, L4 = 101;
      const pitchRad = pitch * Math.PI / 180;
      
      // Calculate all intermediate values
      const theta1 = Math.atan2(y, x) * 180 / Math.PI;
      const baseAngle = Math.atan2(y, x);
      
      const wx = x - L4 * Math.cos(baseAngle) * Math.cos(pitchRad);
      const wy = y - L4 * Math.sin(baseAngle) * Math.cos(pitchRad);
      const wz = z - L4 * Math.sin(pitchRad);
      
      const r = Math.sqrt(wx * wx + wy * wy);
      const h = wz - L1;
      const d = Math.sqrt(r * r + h * h);
      
      // Check reachability
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
      
      // Check joint limits
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
          <div class="math-line"><span class="math-comment">// Subtract end-effector offset based on pitch</span></div>
          <div class="math-line"><span class="math-var">w<sub>x</sub></span> = ${x.toFixed(2)} - ${L4} × ${Math.cos(baseAngle).toFixed(3)} × ${Math.cos(pitchRad).toFixed(3)}</div>
          <div class="math-line"><span class="math-result">w<sub>x</sub> = ${wx.toFixed(2)} mm</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-result">w<sub>y</sub> = ${wy.toFixed(2)} mm</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-var">w<sub>z</sub></span> = ${z.toFixed(2)} - ${L4} × ${Math.sin(pitchRad).toFixed(3)}</div>
          <div class="math-line"><span class="math-result">w<sub>z</sub> = ${wz.toFixed(2)} mm</span></div>
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 3: Arm Geometry</div>
          <div class="math-line"><span class="math-var">r</span> = √(${wx.toFixed(2)}² + ${wy.toFixed(2)}²)</div>
          <div class="math-line"><span class="math-result">r = ${r.toFixed(2)} mm</span> <span class="math-comment">// Horizontal distance</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-var">h</span> = ${wz.toFixed(2)} - ${L1}</div>
          <div class="math-line"><span class="math-result">h = ${h.toFixed(2)} mm</span> <span class="math-comment">// Vertical distance from shoulder</span></div>
          <div class="math-line" style="margin-top:8px;"><span class="math-var">d</span> = √(${r.toFixed(2)}² + ${h.toFixed(2)}²)</div>
          <div class="math-line"><span class="math-result">d = ${d.toFixed(2)} mm</span> <span class="math-comment">// Distance from shoulder to wrist</span></div>
          ${!reachable ? `<div class="math-error">❌ Distance check failed!<br>d = ${d.toFixed(2)} mm must be between ${minReach.toFixed(2)} mm and ${maxReach.toFixed(2)} mm<br>Target is ${d > maxReach ? 'too far' : 'too close'}!</div>` : `<div class="math-warning">✓ Distance OK: ${minReach.toFixed(2)} ≤ ${d.toFixed(2)} ≤ ${maxReach.toFixed(2)}</div>`}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 4: Joint 2 (Shoulder) - Law of Cosines</div>
          <div class="math-line"><span class="math-comment">// Using law of cosines: c² = a² + b² - 2ab×cos(C)</span></div>
          <div class="math-line">cos(<span class="math-var">angle₂</span>) = (${L2}² + ${d.toFixed(2)}² - ${L3}²) / (2 × ${L2} × ${d.toFixed(2)})</div>
          <div class="math-line">cos(<span class="math-var">angle₂</span>) = ${cos_angle2.toFixed(4)}</div>
          ${!angle2Valid ? `<div class="math-error">❌ Invalid cosine value! Must be between -1 and 1.<br>This indicates the geometry is impossible.</div>` : `
          <div class="math-line"><span class="math-var">angle₂</span> = acos(${cos_angle2.toFixed(4)}) = ${(angle2 * 180 / Math.PI).toFixed(2)}°</div>
          <div class="math-line"><span class="math-var">α</span> = atan2(${h.toFixed(2)}, ${r.toFixed(2)}) = ${(alpha * 180 / Math.PI).toFixed(2)}°</div>
          <div class="math-line"><span class="math-result">θ₂ = ${(alpha * 180 / Math.PI).toFixed(2)}° + ${(angle2 * 180 / Math.PI).toFixed(2)}° = ${theta2.toFixed(2)}°</span></div>
          ${!j2InRange ? `<div class="math-error">❌ Joint 2 limit exceeded!<br>θ₂ = ${theta2.toFixed(2)}° is outside the range [-70°, 70°]</div>` : `<div class="math-warning">✓ Joint 2 OK: -70° ≤ ${theta2.toFixed(2)}° ≤ 70°</div>`}
          `}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 5: Joint 3 (Elbow) - Law of Cosines</div>
          <div class="math-line">cos(<span class="math-var">angle₃</span>) = (${L2}² + ${L3}² - ${d.toFixed(2)}²) / (2 × ${L2} × ${L3})</div>
          <div class="math-line">cos(<span class="math-var">angle₃</span>) = ${cos_angle3.toFixed(4)}</div>
          ${!angle3Valid ? `<div class="math-error">❌ Invalid cosine value! Must be between -1 and 1.</div>` : `
          <div class="math-line"><span class="math-result">θ₃ = 180° - acos(${cos_angle3.toFixed(4)}) = ${theta3.toFixed(2)}°</span></div>
          ${!j3InRange ? `<div class="math-error">❌ Joint 3 limit exceeded!<br>θ₃ = ${theta3.toFixed(2)}° is outside the range [-80°, 80°]</div>` : `<div class="math-warning">✓ Joint 3 OK: -80° ≤ ${theta3.toFixed(2)}° ≤ 80°</div>`}
          `}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 6: Joint 4 (Wrist Pitch)</div>
          ${theta2 !== null && theta3 !== null ? `
          <div class="math-line"><span class="math-var">arm_angle</span> = ${theta2.toFixed(2)}° - ${theta3.toFixed(2)}° = ${(theta2 - theta3).toFixed(2)}°</div>
          <div class="math-line"><span class="math-result">θ₄ = ${pitch}° - ${(theta2 - theta3).toFixed(2)}° = ${theta4.toFixed(2)}°</span></div>
          ${!j4InRange ? `<div class="math-error">❌ Joint 4 limit exceeded!<br>θ₄ = ${theta4.toFixed(2)}° is outside the range [-90°, 90°]</div>` : `<div class="math-warning">✓ Joint 4 OK: -90° ≤ ${theta4.toFixed(2)}° ≤ 90°</div>`}
          ` : '<div class="math-error">Cannot calculate - previous steps failed</div>'}
        </div>
        
        <div class="math-section">
          <div class="math-title">Step 7: Joint 5 (Wrist Roll)</div>
          <div class="math-line"><span class="math-result">θ₅ = ${roll}°</span> <span class="math-comment">// Direct passthrough</span></div>
        </div>
        
        <canvas id="armCanvas" width="600" height="400"></canvas>
      `;
      
      document.getElementById('mathContent').innerHTML = mathHTML;
      document.getElementById('mathCard').style.display = 'block';
      
      // Draw the arm visualization
      setTimeout(() => drawArmDiagram(x, y, z, theta1, theta2, theta3, theta4, L1, L2, L3, L4), 100);
    }
    
    function drawArmDiagram(x, y, z, theta1, theta2, theta3, theta4, L1, L2, L3, L4) {
      const canvas = document.getElementById('armCanvas');
      if (!canvas) return;
      
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      // Scale and offset for visualization (side view)
      const scale = 0.7;
      const offsetX = 100;
      const offsetY = 350;
      
      // Calculate positions (side view - looking from the side)
      const r = Math.sqrt(x*x + y*y);
      
      // Base
      ctx.fillStyle = '#424242';
      ctx.fillRect(offsetX - 30, offsetY, 60, 20);
      
      // Joint positions
      const j0 = {x: offsetX, y: offsetY};
      const j1 = {x: offsetX, y: offsetY - L1 * scale};
      
      // Calculate joint 2 position
      const theta2Rad = theta2 * Math.PI / 180;
      const j2 = {
        x: j1.x + L2 * scale * Math.cos(theta2Rad - Math.PI/2),
        y: j1.y - L2 * scale * Math.sin(theta2Rad - Math.PI/2)
      };
      
      // Calculate joint 3 position
      const theta3Rad = theta3 * Math.PI / 180;
      const cumulativeAngle2 = theta2Rad + (Math.PI - theta3Rad * Math.PI / 180);
      const j3 = {
        x: j2.x + L3 * scale * Math.cos(theta2Rad + theta3Rad * Math.PI / 180 - Math.PI/2),
        y: j2.y - L3 * scale * Math.sin(theta2Rad + theta3Rad * Math.PI / 180 - Math.PI/2)
      };
      
      // Calculate end effector position
      const theta4Rad = theta4 * Math.PI / 180;
      const totalAngle = theta2Rad + theta3Rad * Math.PI / 180 + theta4Rad;
      const j4 = {
        x: j3.x + L4 * scale * Math.cos(totalAngle - Math.PI/2),
        y: j3.y - L4 * scale * Math.sin(totalAngle - Math.PI/2)
      };
      
      // Draw target point
      ctx.fillStyle = '#ef5350';
      ctx.globalAlpha = 0.5;
      ctx.beginPath();
      ctx.arc(offsetX + r * scale, offsetY - z * scale, 8, 0, 2 * Math.PI);
      ctx.fill();
      ctx.globalAlpha = 1.0;
      
      // Draw dashed line to target
      ctx.strokeStyle = '#ef5350';
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(j4.x, j4.y);
      ctx.lineTo(offsetX + r * scale, offsetY - z * scale);
      ctx.stroke();
      ctx.setLineDash([]);
      
      // Draw links
      ctx.lineWidth = 8;
      ctx.lineCap = 'round';
      
      // Link 1 (base to shoulder)
      ctx.strokeStyle = '#757575';
      ctx.beginPath();
      ctx.moveTo(j0.x, j0.y);
      ctx.lineTo(j1.x, j1.y);
      ctx.stroke();
      
      // Link 2 (shoulder to elbow)
      ctx.strokeStyle = '#ef5350';
      ctx.beginPath();
      ctx.moveTo(j1.x, j1.y);
      ctx.lineTo(j2.x, j2.y);
      ctx.stroke();
      
      // Link 3 (elbow to wrist)
      ctx.strokeStyle = '#66bb6a';
      ctx.beginPath();
      ctx.moveTo(j2.x, j2.y);
      ctx.lineTo(j3.x, j3.y);
      ctx.stroke();
      
      // Link 4 (wrist to end effector)
      ctx.strokeStyle = '#42a5f5';
      ctx.beginPath();
      ctx.moveTo(j3.x, j3.y);
      ctx.lineTo(j4.x, j4.y);
      ctx.stroke();
      
      // Draw joints
      ctx.fillStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#9e9e9e';
      [j0, j1, j2, j3, j4].forEach(joint => {
        ctx.beginPath();
        ctx.arc(joint.x, joint.y, 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
      });
      
      // Draw angle arcs and labels
      ctx.lineWidth = 1.5;
      ctx.font = '12px monospace';
      
      // Angle at shoulder (theta2)
      if (theta2 !== null) {
        ctx.strokeStyle = '#ef5350';
        ctx.fillStyle = '#ef5350';
        ctx.beginPath();
        ctx.arc(j1.x, j1.y, 30, -Math.PI/2, theta2Rad - Math.PI/2, theta2 < 0);
        ctx.stroke();
        ctx.fillText(`θ₂=${theta2.toFixed(1)}°`, j1.x + 35, j1.y - 20);
      }
      
      // Angle at elbow (theta3)
      if (theta3 !== null) {
        ctx.strokeStyle = '#66bb6a';
        ctx.fillStyle = '#66bb6a';
        ctx.beginPath();
        const elbowStartAngle = theta2Rad - Math.PI/2;
        const elbowEndAngle = theta2Rad + theta3Rad * Math.PI / 180 - Math.PI/2;
        ctx.arc(j2.x, j2.y, 25, elbowStartAngle, elbowEndAngle, false);
        ctx.stroke();
        ctx.fillText(`θ₃=${theta3.toFixed(1)}°`, j2.x + 30, j2.y);
      }
      
      // Draw coordinate system
      ctx.strokeStyle = '#666';
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 2]);
      
      // Vertical reference
      ctx.beginPath();
      ctx.moveTo(offsetX, offsetY);
      ctx.lineTo(offsetX, offsetY - 400);
      ctx.stroke();
      
      // Horizontal reference
      ctx.beginPath();
      ctx.moveTo(offsetX, offsetY);
      ctx.lineTo(offsetX + 500, offsetY);
      ctx.stroke();
      
      ctx.setLineDash([]);
      
      // Labels
      ctx.fillStyle = '#9e9e9e';
      ctx.font = '11px sans-serif';
      ctx.fillText('Base', offsetX - 15, offsetY + 35);
      ctx.fillText('Ground (Z=0)', offsetX + 10, offsetY + 15);
      ctx.fillText(`Target (r=${r.toFixed(0)}, z=${z.toFixed(0)})`, offsetX + r * scale + 10, offsetY - z * scale);
      ctx.fillStyle = '#66bb6a';
      ctx.fillText('End Effector', j4.x + 10, j4.y);
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
            displayMathCalculations(parseFloat(x), parseFloat(y), parseFloat(z), parseFloat(pitch), parseFloat(roll), true, '');
            
            resultsDiv.innerHTML = `
              <div class="card-header" onclick="toggleCard(this)">
                <div class="card-title">Joint Angles</div>
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
            statusText.textContent = 'Solution found';
            statusDot.className = 'status-dot';
            
            // Update current position
            currentPosition.x = parseFloat(x);
            currentPosition.y = parseFloat(y);
            currentPosition.z = parseFloat(z);
            currentPosition.angles = data.angles;
            updateCurrentDisplay();
          } else {
            displayMathCalculations(parseFloat(x), parseFloat(y), parseFloat(z), parseFloat(pitch), parseFloat(roll), false, data.error);
            
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
          displayMathCalculations(parseFloat(x), parseFloat(y), parseFloat(z), parseFloat(pitch), parseFloat(roll), false, 'Connection error');
          
          const resultsDiv = document.getElementById('results');
          resultsDiv.innerHTML = `
            <div class="card-header" onclick="toggleCard(this)">
              <div class="card-title">Error</div>
              <div class="card-icon">▼</div>
            </div>
            <div class="card-content">
              <div class="error-card">Connection error</div>
            </div>
          `;
          resultsDiv.className = 'card results-card show';
          statusText.textContent = 'Connection error';
          statusDot.className = 'status-dot error';
        });
    }
    
    function resetForm() {
      document.getElementById('x').value = '200';
      document.getElementById('y').value = '100';
      document.getElementById('z').value = '200';
      document.getElementById('pitch').value = '0';
      document.getElementById('roll').value = '0';
      document.getElementById('results').className = 'card results-card';
      document.getElementById('mathCard').style.display = 'none';
      document.getElementById('status').textContent = 'Ready';
      document.getElementById('statusDot').className = 'status-dot';
    }
    
    // Initialize display on load
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
    
    lastSolveStatus = success ? "Success" : "Failed";
    
    // Print to serial
    Serial.println("\n=== Web Request ===");
    Serial.printf("Target: X=%.2f Y=%.2f Z=%.2f Pitch=%.2f Roll=%.2f\n", x, y, z, pitch, roll);
    if (success) {
      Serial.println("Joint Angles:");
      for (int i = 0; i < 5; i++) {
        Serial.printf("  Joint %d: %.2f°\n", i + 1, theta[i]);
      }
    } else {
      Serial.print("Status: Failed - ");
      Serial.println(lastSolveError);
    }
    Serial.println("==================\n");
    
    server.send(200, "application/json", json);
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"status\":\"" + lastSolveStatus + "\",";
  json += "\"lastX\":" + String(lastX, 2) + ",";
  json += "\"lastY\":" + String(lastY, 2) + ",";
  json += "\"lastZ\":" + String(lastZ, 2) + ",";
  json += "\"lastPitch\":" + String(lastPitch, 2) + ",";
  json += "\"lastRoll\":" + String(lastRoll, 2) + ",";
  json += "\"angles\":[";
  for (int i = 0; i < 5; i++) {
    json += String(theta[i], 2);
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
    
    // Parse if it's coordinates (X Y Z Pitch Roll)
    int spaceCount = 0;
    for (char c : command) if (c == ' ') spaceCount++;
    
    if (spaceCount >= 4) {
      float vals[5];
      int lastIdx = 0;
      for (int i = 0; i < 5; i++) {
        int nextIdx = command.indexOf(' ', lastIdx);
        if (nextIdx == -1) {
          vals[i] = command.substring(lastIdx).toFloat();
        } else {
          vals[i] = command.substring(lastIdx, nextIdx).toFloat();
          lastIdx = nextIdx + 1;
        }
      }
      
      if (solveIK(vals[0], vals[1], vals[2], vals[3], vals[4])) {
        String response = "\n=== Joint Angles ===\n";
        for (int i = 0; i < 5; i++) {
          response += "Joint " + String(i + 1) + ": " + String(theta[i], 2) + "°\n";
        }
        response += "====================\n";
        Serial.print(response);
        serialBuffer += response;
        server.send(200, "text/plain", response);
      } else {
        String error = "ERROR: " + lastSolveError + "\n";
        Serial.print(error);
        serialBuffer += error;
        server.send(200, "text/plain", error);
      }
    } else {
      server.send(200, "text/plain", "Command sent to serial\n");
    }
  } else {
    server.send(400, "text/plain", "No command provided");
  }
}
