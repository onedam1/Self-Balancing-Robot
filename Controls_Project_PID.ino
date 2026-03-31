#include <Wire.h>
#include <math.h>

// MPU6050 I2C address
#define MPU_ADDR 0x68

// Motor pins
#define AIN1 25
#define AIN2 27
#define BIN1 26
#define BIN2 13

// PID parameters
double Kp = 35.0;  // proportional
double Ki = 120;   // integral
double Kd = 1.2;   // derivative

// Complementary/Kalman filter constants
double Q_angle = 0.001;   // process noise variance for the accelerometer
double Q_gyro = 0.003;    // process noise variance for the gyro bias
double R_angle = 0.03;    // measurement noise variance

// Timing
unsigned long lastTime = 0;
float dt = 0;

// MPU6050 offsets (averaged)
const int16_t ax_offset = 328;
const int16_t ay_offset = 872;
const int16_t az_offset = -121;
const int16_t gx_offset = -24;
const int16_t gy_offset = 482;
const int16_t gz_offset = 133;

// Max tilt before full motor output
const float maxAngle = 25.0;

// Kalman filter variables
double angle = 0;    // filtered angle
double bias = 0;     // gyro bias
double P[2][2] = {{0,0},{0,0}}; // error covariance matrix

// PID variables
double pidOutput = 0;
double errorSum = 0;
double lastError = 0;

// Raw MPU6050 readings
int16_t ax, ay, az, gx, gy, gz;

// Read 16-bit MPU6050 register
int16_t readMPU16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  while(Wire.available() < 2);
  return Wire.read() << 8 | Wire.read();
}

// Wake up MPU6050
void mpuInit(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); // wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // gyro ±250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // accel ±2g
}

// Read raw accel + gyro
void readMPU(){
  ax = readMPU16(0x3B) - ax_offset;
  ay = readMPU16(0x3D) - ay_offset;
  az = readMPU16(0x3F) - az_offset;
  gx = readMPU16(0x43) - gx_offset;
  gy = readMPU16(0x45) - gy_offset;
  gz = readMPU16(0x47) - gz_offset;
}

// ----------- DEAD BAND FUNCTION (55–255) ------------
int applyDeadband(int x) {
  if (x == 0) return 0;
  int sign = (x > 0) ? 1 : -1;
  int mag = abs(x);

  if (mag < 55) mag = 55;   // minimum usable PWM
  if (mag > 255) mag = 255; // clamp max

  return sign * mag;
}
// ----------------------------------------------------

// Kalman filter update
double kalmanUpdate(double newAngle, double newRate, double dt){
  // Predict
  double rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;

  // Update
  double S = P[0][0] + R_angle;
  double K[2];
  K[0] = P[0][0]/S;
  K[1] = P[1][0]/S;

  double y = newAngle - angle;
  angle += K[0]*y;
  bias += K[1]*y;

  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K[0]*P00_temp;
  P[0][1] -= K[0]*P01_temp;
  P[1][0] -= K[1]*P00_temp;
  P[1][1] -= K[1]*P01_temp;

  return angle;
}

// Motor control
void setMotor(int pwmA, int pwmB){
  if(pwmA >= 0){ analogWrite(AIN1,pwmA); analogWrite(AIN2,0); }
  else{ analogWrite(AIN1,0); analogWrite(AIN2,-pwmA); }

  if(pwmB >= 0){ analogWrite(BIN1,pwmB); analogWrite(BIN2,0); }
  else{ analogWrite(BIN1,0); analogWrite(BIN2,-pwmB); }
}

void setup(){
  Serial.begin(115200);
  Wire.begin(21,22); // SDA/SCL
  delay(500);
  Serial.println("Initializing MPU6050...");
  mpuInit();
  delay(100);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  lastTime = millis();
  Serial.println("Setup complete. Start balancing...");
}

void loop(){
  unsigned long now = millis();
  dt = (now - lastTime)/1000.0;
  lastTime = now;

  readMPU();

  // Accelerometer angle
  double accelAngle = atan2(ay, az)*180.0/PI;

  // Gyro rate in deg/s
  double gyroRate = gx/131.0;

  // Kalman filter
  double pitch = kalmanUpdate(accelAngle, gyroRate, dt);

  // PID
  double error = 0.0 - pitch;
  errorSum += error*dt;
  double dError = (error - lastError)/dt;
  pidOutput = Kp*error + Ki*errorSum + Kd*dError;
  lastError = error;

  // Max tilt safety
  if(pitch >= maxAngle) pidOutput = -255;
  else if(pitch <= -maxAngle) pidOutput = 255;

  // Clamp PID
  if(pidOutput > 255) pidOutput = 255;
  if(pidOutput < -255) pidOutput = -255;

  // Apply deadband **here**
  int pwmA = applyDeadband((int)pidOutput);
  int pwmB = applyDeadband(-(int)pidOutput);

  // Motors towards fall direction
  setMotor(pwmA, pwmB);

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" | PID: "); Serial.println(pidOutput);

  delay(5);
}
