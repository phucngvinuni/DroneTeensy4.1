/*
 * TEENSY 4.1 FLIGHT CONTROLLER - STABILIZE MODE
 * VERSION: TUNING + RC NOISE FILTER
 * ------------------------------------------------
 * Pinout: M1(Pin5-FR) | M2(Pin2-BR) | M3(Pin3-BL) | M4(Pin4-FL)
 * RX: Pin 14 (PulsePosition)
 */

#include <Wire.h>
#include <PulsePosition.h>

// ================= 1. CẤU HÌNH PHẦN CỨNG =================
const int MOTOR_1_PIN = 5; // FR - Trước Phải (CCW)
const int MOTOR_2_PIN = 2; // BR - Sau Phải (CW)
const int MOTOR_3_PIN = 3; // BL - Sau Trái (CCW)
const int MOTOR_4_PIN = 4; // FL - Trước Trái (CW)
const int RX_PIN = 14;

// ================= 2. PID TUNING (SETUP) =================
// GIỮ NGUYÊN THÔNG SỐ CỦA BẠN
float Kp_roll = 1.0;   float Ki_roll = 0.5;   float Kd_roll = 0.3;

// PITCH & YAW: Tạm để 0 để tập trung Tune Roll trước
float Kp_pitch = 1.0;  float Ki_pitch = 0.5;  float Kd_pitch = 0.3;
float Kp_yaw = 0;    float Ki_yaw = 0;    float Kd_yaw = 0; 

// Cấu hình cảm giác lái
float rc_expo = 0.6;      
float max_angle = 30.0;     
float max_yaw_rate = 160.0; 

// ================= 3. BIẾN HỆ THỐNG =================
#define MPU_addr 0x68
float dt;
unsigned long current_time, prev_time;
unsigned long loop_timer;

// IMU Variables
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float roll_IMU, pitch_IMU, yaw_IMU;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float B_madgwick = 0.04;

// Control Variables
float thro_des, roll_des, pitch_des, yaw_des;
float error_roll, integral_roll, roll_PID;
float error_pitch, integral_pitch, pitch_PID;
float error_yaw, integral_yaw, yaw_PID;
// (Đã bỏ last_gyro vì dùng công thức D mới)
float i_limit = 150.0; 

// RC & Motor
PulsePositionInput ReceiverInput(RISING);
float channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm;
float m1_out, m2_out, m3_out, m4_out;

// Helper Math
float invSqrt(float x) { if(x<=0) return 0; return 1.0/sqrtf(x); }
float applyExpo(float input, float expo) {
  return input * input * input * expo + input * (1.0 - expo);
}

// ================= SETUP =================
void setup() {
  Serial.begin(500000); // Baudrate cao cho Serial Plotter
  Wire.begin(); Wire.setClock(400000);

  analogWriteFrequency(MOTOR_1_PIN, 250); analogWriteResolution(12);
  analogWriteFrequency(MOTOR_2_PIN, 250); analogWriteResolution(12);
  analogWriteFrequency(MOTOR_3_PIN, 250); analogWriteResolution(12);
  analogWriteFrequency(MOTOR_4_PIN, 250); analogWriteResolution(12);
  pinMode(13, OUTPUT); 

  // MPU6050 Config
  Wire.beginTransmission(MPU_addr); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_addr); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(MPU_addr); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(MPU_addr); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(); // LPF 10Hz

  ReceiverInput.begin(RX_PIN);
  delay(500);
  calculate_IMU_error();
  loop_timer = micros();
}

// ================= LOOP =================
void loop() {
  // Loop 400Hz (2500us)
  if (micros() - loop_timer > 2500) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loop_timer = current_time;

    // --- A. ĐỌC RC (CÓ LỌC NHIỄU) ---
    // Update mới: Chỉ nhận giá trị trong khoảng 900-2100us
    int num = ReceiverInput.available();
    if (num > 0) {
      float raw_1 = ReceiverInput.read(3); // Throttle
      float raw_2 = ReceiverInput.read(1); // Roll
      float raw_3 = ReceiverInput.read(2); // Pitch
      float raw_4 = ReceiverInput.read(4); // Yaw

      // Bộ lọc Sanity Check: Nếu tín hiệu bị nhiễu (về 0 hoặc quá lớn), giữ nguyên giá trị cũ
      if (raw_1 > 900 && raw_1 < 2100) channel_1_pwm = raw_1;
      if (raw_2 > 900 && raw_2 < 2100) channel_2_pwm = raw_2;
      if (raw_3 > 900 && raw_3 < 2100) channel_3_pwm = raw_3;
      if (raw_4 > 900 && raw_4 < 2100) channel_4_pwm = raw_4;
    }
    
    // Safety check cho Throttle
    if (channel_1_pwm < 800) channel_1_pwm = 1000; 

    // --- B. ĐỌC CẢM BIẾN ---
    read_mpu_raw();
    Madgwick(GyroX*0.01745, -GyroY*0.01745, -GyroZ*0.01745, -AccX, AccY, AccZ, dt);

    // --- C. LOGIC ĐIỀU KHIỂN ---
    if (channel_1_pwm > 1060) {
      digitalWrite(13, HIGH); // LED báo Arm
      thro_des = constrain(channel_1_pwm - 1000.0, 0, 1000);

      // Tính toán góc mong muốn (Setpoint)
      float r_raw = channel_2_pwm - 1500.0; if(abs(r_raw)<8) r_raw=0;
      float p_raw = channel_3_pwm - 1500.0; if(abs(p_raw)<8) p_raw=0;
      float y_raw = channel_4_pwm - 1500.0; if(abs(y_raw)<8) y_raw=0;

      roll_des = applyExpo(r_raw/500.0, rc_expo) * max_angle;
      pitch_des = applyExpo(p_raw/500.0, rc_expo) * max_angle;
      yaw_des = applyExpo(y_raw/500.0, rc_expo) * max_yaw_rate;

      // ---------------- PID CONTROLLER ----------------
      // ROLL
      error_roll = roll_des - roll_IMU;
      integral_roll = constrain(integral_roll + error_roll * dt * Ki_roll, -i_limit, i_limit);
      roll_PID = (Kp_roll * error_roll) + integral_roll - (Kd_roll * GyroX); 

      // PITCH
      error_pitch = pitch_des - pitch_IMU;
      integral_pitch = constrain(integral_pitch + error_pitch * dt * Ki_pitch, -i_limit, i_limit);
      pitch_PID = (Kp_pitch * error_pitch) + integral_pitch - (Kd_pitch * GyroY); 

      // YAW
      error_yaw = yaw_des - GyroZ;
      integral_yaw = constrain(integral_yaw + error_yaw * dt * Ki_yaw, -i_limit, i_limit);
      yaw_PID = (Kp_yaw * error_yaw) + integral_yaw; 

      // ---------------- MIXER (ĐÃ SỬA DẤU) ----------------
      m1_out = thro_des + roll_PID + pitch_PID + yaw_PID; // FR
      m2_out = thro_des + roll_PID - pitch_PID - yaw_PID; // BR
      m3_out = thro_des - roll_PID - pitch_PID + yaw_PID; // BL
      m4_out = thro_des - roll_PID + pitch_PID - yaw_PID; // FL

      // Idle speed & Limit
      m1_out = constrain(m1_out + 1000, 1150, 2000);
      m2_out = constrain(m2_out + 1000, 1150, 2000);
      m3_out = constrain(m3_out + 1000, 1150, 2000);
      m4_out = constrain(m4_out + 1000, 1150, 2000);

    } else {
      // DISARMED
      digitalWrite(13, LOW);
      m1_out = 1000; m2_out = 1000; m3_out = 1000; m4_out = 1000;
      integral_roll = 0; integral_pitch = 0; integral_yaw = 0;
      roll_PID = 0; pitch_PID = 0; yaw_PID = 0;
    }

    // ================= SERIAL PLOTTER =================
    static uint8_t plot_counter = 0;
    plot_counter++;
    if (plot_counter >= 10) { 
      plot_counter = 0;
      // Chế độ Tune Roll
      Serial.print("Roll_Des:"); Serial.print(roll_des); Serial.print(",");
      Serial.print("Roll_IMU:"); Serial.print(roll_IMU);
      Serial.println();
      Serial.print("Pitch_Des:"); Serial.print(pitch_des); Serial.print(",");
      Serial.print("Pitch_IMU:"); Serial.print(pitch_IMU);
      Serial.println();
      Serial.print("Yaw_Des:"); Serial.print(yaw_des); Serial.print(",");
      Serial.print("Yaw_IMU:"); Serial.print(yaw_IMU);
      Serial.println();
    }

    writeMotorsRaw(m1_out, m2_out, m3_out, m4_out);
  }
}

// Helpers
void writeMotorsRaw(float m1, float m2, float m3, float m4) { 
  analogWrite(MOTOR_1_PIN, m1*1.024); 
  analogWrite(MOTOR_2_PIN, m2*1.024); 
  analogWrite(MOTOR_3_PIN, m3*1.024); 
  analogWrite(MOTOR_4_PIN, m4*1.024); 
}

void read_mpu_raw() { 
  Wire.beginTransmission(MPU_addr); Wire.write(0x3B); Wire.endTransmission(false); 
  Wire.requestFrom((uint8_t)MPU_addr, (uint8_t)14, (uint8_t)1); 
  int16_t AcX=Wire.read()<<8|Wire.read(); int16_t AcY=Wire.read()<<8|Wire.read(); int16_t AcZ=Wire.read()<<8|Wire.read(); 
  Wire.read(); Wire.read(); 
  int16_t GyX=Wire.read()<<8|Wire.read(); int16_t GyY=Wire.read()<<8|Wire.read(); int16_t GyZ=Wire.read()<<8|Wire.read(); 
  AccX=AcX/4096.0-AccErrorX; AccY=AcY/4096.0-AccErrorY; AccZ=AcZ/4096.0-AccErrorZ; 
  GyroX=GyX/65.5-GyroErrorX; GyroY=GyY/65.5-GyroErrorY; GyroZ=GyZ/65.5-GyroErrorZ; 
}

void calculate_IMU_error() { 
  float sAX=0, sAY=0, sAZ=0, sGX=0, sGY=0, sGZ=0; 
  for(int i=0; i<2000; i++){ read_mpu_raw(); sAX+=AccX; sAY+=AccY; sAZ+=AccZ; sGX+=GyroX; sGY+=GyroY; sGZ+=GyroZ; delay(1); } 
  AccErrorX=sAX/2000.0; AccErrorY=sAY/2000.0; AccErrorZ=sAZ/2000.0-1.0; 
  GyroErrorX=sGX/2000.0; GyroErrorY=sGY/2000.0; GyroErrorZ=sGZ/2000.0; 
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) { 
  float recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4; 
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3; 
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz); qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy); 
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx); qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx); 
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) { 
    recipNorm = invSqrt(ax * ax + ay * ay + az * az); 
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm; 
    _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3; 
    _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _8q1 = 8.0f * q1; _8q2 = 8.0f * q2; 
    q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3; 
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay; 
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az; 
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az; 
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay; 
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm; 
    qDot1 -= B_madgwick * s0; qDot2 -= B_madgwick * s1; qDot3 -= B_madgwick * s2; qDot4 -= B_madgwick * s3; 
  } 
  q0 += qDot1 * invSampleFreq; q1 += qDot2 * invSampleFreq; q2 += qDot3 * invSampleFreq; q3 += qDot4 * invSampleFreq; 
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); 
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm; 
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; 
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; 
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; 
}