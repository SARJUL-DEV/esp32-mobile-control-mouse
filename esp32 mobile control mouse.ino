#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32 Smart Mouse", "Espressif", 100);

const int BUTTON_PIN = 14; // Button on GPIO 14

// Kalman Filter State Variables
float x_angle = 0, y_angle = 0; 
float x_bias = 0, y_bias = 0;   
float P[2][2] = { { 1, 0 }, { 0, 1 } };  

// Precise tuning variables
float q_angle = 0.005;   
float q_bias = 0.003;    
float r_measure = 0.01;  

// Calibration offsets
float accelX_offset = 0;
float accelY_offset = 0;
float gyroX_offset = 0;
float gyroY_offset = 0;

bool lastButtonState = HIGH; 
unsigned long buttonPressTime = 0;

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) {
  float rate = newRate - bias;  
  angle += dt * rate;  
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += q_bias * dt;
  float S = P[0][0] + r_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
  return angle;  
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  
  if (!mpu.begin()) {
    while (1) delay(1);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);   
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);      
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);   

  // Automatic sensor calibration
  delay(500);
  sensors_event_t a, g, temp;
  int samples = 100;
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    accelX_offset += a.acceleration.x;
    accelY_offset += a.acceleration.y;
    gyroX_offset += g.gyro.x;
    gyroY_offset += g.gyro.y;
    delay(10);
  }
  accelX_offset /= samples;
  accelY_offset /= samples;
  gyroX_offset /= samples;
  gyroY_offset /= samples;
  
  bleMouse.begin(); 
  Serial.println("Calibrated & Ready!");
}

void loop() {
  if (bleMouse.isConnected()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    static unsigned long prevTime = millis();
    float dt = (millis() - prevTime) / 1000.0;
    if (dt <= 0.0) dt = 0.001; 
    prevTime = millis();

    // Apply offsets
    float corrected_ax = a.acceleration.x - accelX_offset;
    float corrected_ay = a.acceleration.y - accelY_offset;
    float corrected_gx = g.gyro.x - gyroX_offset;
    float corrected_gy = g.gyro.y - gyroY_offset;

    // Filter values
    float filteredX = kalmanFilter(corrected_ax, corrected_gx, dt, x_angle, x_bias);
    float filteredY = kalmanFilter(corrected_ay, corrected_gy, dt, y_angle, y_bias);

    // FIXED: Inverted directions fixed by modifying the minus signs (-).
    // FIXED: Sensitivity reduced from 3 to 1.8 for optimal control.
    int moveX = (int)(filteredX * 1.8);  // Removed minus to fix Left/Right inversion
    int moveY = (int)(-filteredY * 1.8); // Added minus to fix Forward/Backward inversion

    // Deadzone check
    if (abs(moveX) < 3) moveX = 0;
    if (abs(moveY) < 3) moveY = 0;

    bool currentButtonState = digitalRead(BUTTON_PIN);

    // 1. Drag & Drop Logic
    if (currentButtonState == LOW && !bleMouse.isPressed(MOUSE_LEFT)) {
      bleMouse.press(MOUSE_LEFT); 
    }

    // 2. Click Release Logic
    if (currentButtonState == HIGH && lastButtonState == LOW) {
      bleMouse.release(MOUSE_LEFT); 
      if (millis() - buttonPressTime < 300) { 
        bleMouse.click(MOUSE_LEFT); 
      }
    }

    if (currentButtonState == LOW && lastButtonState == HIGH) {
      buttonPressTime = millis(); 
    }

    lastButtonState = currentButtonState;

    // 3. Move Pointer Vector
    if (moveX != 0 || moveY != 0) {
      bleMouse.move(moveX, moveY);
    }
  }
  delay(10); 
}
