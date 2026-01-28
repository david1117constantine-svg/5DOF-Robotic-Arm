/*
 * 5DOF Robotic Arm Inverse Kinematics for ESP32-S3
 * Modified DH Parameters (in millimeters)
 * 
 * Usage: Enter Cartesian coordinates via Serial Monitor
 * Format: X Y Z Pitch Roll
 * Example: 200 100 150 45 0
 */

#include <Arduino.h>
#include <math.h>

// Modified DH Parameters (in mm)
struct DHParam {
  float alpha;  // ai-1
  float a;      // di
  float theta;  // θi (will be calculated)
  float d;      // offset
};

// DH table from your image
DHParam dh[5] = {
  {0.0,   0.0,   0.0, 152.5},  // Link 1
  {90.0,  0.0,   90.0, 0.0},   // Link 2
  {0.0,   180.0, 0.0, 0.0},    // Link 3
  {0.0,   180.0, 0.0, 90.0},   // Link 4
  {90.0,  0.0,   101.0, 0.0}   // Link 5
};

// Link lengths
const float L1 = 152.5;  // Base height
const float L2 = 180.0;  // Upper arm
const float L3 = 180.0;  // Forearm
const float L4 = 101.0;  // Wrist to end-effector

// Joint angles (in degrees)
float theta[5] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=== 5DOF Robot Arm IK Solver ===");
  Serial.println("Enter target position:");
  Serial.println("Format: X Y Z Pitch Roll");
  Serial.println("Example: 200 100 150 45 0");
  Serial.println("Units: mm for position, degrees for orientation");
  Serial.println("================================\n");
}

void loop() {
  if (Serial.available() > 0) {
    // Read input coordinates
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();
    float pitch = Serial.parseFloat();
    float roll = Serial.parseFloat();
    
    // Clear remaining input
    while (Serial.available() > 0) {
      Serial.read();
    }
    
    Serial.print("\nTarget Position: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Z=");
    Serial.print(z);
    Serial.print(" Pitch=");
    Serial.print(pitch);
    Serial.print(" Roll=");
    Serial.println(roll);
    
    // Solve IK
    if (solveIK(x, y, z, pitch, roll)) {
      Serial.println("\n=== Joint Angles (degrees) ===");
      Serial.print("Joint 1 (Base):     ");
      Serial.println(theta[0], 2);
      Serial.print("Joint 2 (Shoulder): ");
      Serial.println(theta[1], 2);
      Serial.print("Joint 3 (Elbow):    ");
      Serial.println(theta[2], 2);
      Serial.print("Joint 4 (Wrist 1):  ");
      Serial.println(theta[3], 2);
      Serial.print("Joint 5 (Wrist 2):  ");
      Serial.println(theta[4], 2);
      Serial.println("==============================\n");
    } else {
      Serial.println("ERROR: Target position unreachable!\n");
    }
    
    Serial.println("Ready for next input...\n");
  }
}

bool solveIK(float x, float y, float z, float pitch_deg, float roll_deg) {
  // Convert angles to radians
  float pitch = pitch_deg * DEG_TO_RAD;
  float roll = roll_deg * DEG_TO_RAD;
  
  // Joint 1: Base rotation (around Z-axis)
  theta[0] = atan2(y, x) * RAD_TO_DEG;
  
  // Calculate wrist center position
  // Subtract the end-effector offset based on pitch
  float wx = x - L4 * cos(atan2(y, x)) * cos(pitch);
  float wy = y - L4 * sin(atan2(y, x)) * cos(pitch);
  float wz = z - L4 * sin(pitch);
  
  // Distance from base to wrist in XY plane
  float r = sqrt(wx * wx + wy * wy);
  
  // Height from base
  float h = wz - L1;
  
  // Distance from shoulder joint to wrist
  float d = sqrt(r * r + h * h);
  
  // Check if target is reachable
  if (d > (L2 + L3) || d < abs(L2 - L3)) {
    return false;
  }
  
  // Joint 2: Shoulder angle using law of cosines
  float cos_angle2 = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d);
  if (cos_angle2 < -1.0 || cos_angle2 > 1.0) {
    return false;
  }
  
  float angle2 = acos(cos_angle2);
  float alpha = atan2(h, r);
  theta[1] = (alpha + angle2) * RAD_TO_DEG;
  
  // Joint 3: Elbow angle using law of cosines
  float cos_angle3 = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3);
  if (cos_angle3 < -1.0 || cos_angle3 > 1.0) {
    return false;
  }
  
  theta[2] = (PI - acos(cos_angle3)) * RAD_TO_DEG;
  
  // Joint 4: Wrist pitch
  // The pitch of the end-effector relative to horizontal
  float arm_angle = theta[1] - theta[2];
  theta[3] = pitch_deg - arm_angle;
  
  // Joint 5: Wrist roll
  theta[4] = roll_deg;
  
  // Normalize angles to reasonable ranges
  for (int i = 0; i < 5; i++) {
    while (theta[i] > 180.0) theta[i] -= 360.0;
    while (theta[i] < -180.0) theta[i] += 360.0;
  }
  
  return true;
}
