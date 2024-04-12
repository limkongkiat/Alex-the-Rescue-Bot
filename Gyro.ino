#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float driftRate = 0;
float speed = 16.36; // speed in cm/ sec, can be calculated using claibrate_speed()
float original_left_speed = -1 * 40/100.0*255;
float original_right_speed = 43/100.0*255;
float obs_angle;
float obs_stop_dist = 8;
float obs_size = 12; //assuming obstcle is a square, where each side is this set size

float x_bottom;
float y_bottom;
float x_top;
float y_top;

int obs_case = 9; //cases of where obticle is in the map, 1=left, 2=top, 3=right, 4=bottom, 5= middle

float a; //  forward acceleration value in cm/s^2

float calibration(){
  delay(2000);
  float drift = 0;
  float driftedAngle = 0;
  float elapsed = 0;
  float originalAngle = check_angle();
  float StartTime = millis();

  while (elapsed<2000){
    driftedAngle=check_angle();
    elapsed = millis()-StartTime;
  }

  driftRate = (driftedAngle-originalAngle)/2;
  return;
}

float check_angle() { // returns the current orientation of the bot in degrees

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  yaw =  yaw + GyroZ * elapsedTime + (-driftRate*elapsedTime);

  return (yaw);
}

void setupGyro() {
  //Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  calibration(); // calculates drift rate of gyro
}
/*
void loop(){
  Serial.println(check_angle());
}*/
