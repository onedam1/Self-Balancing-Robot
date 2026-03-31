#include <Wire.h>

#define MPU_ADDR 0x68

int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

int16_t ax, ay, az, gx, gy, gz;

const int samples = 2000;

// Read 16-bit register
int16_t readMPU16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  while(Wire.available() < 2);
  return Wire.read() << 8 | Wire.read();
}

// Initialize MPU6050
void mpuInit(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); // wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // gyro ±250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // accel ±2g
}

// Read raw values
void readMPU(){
  ax = readMPU16(0x3B);
  ay = readMPU16(0x3D);
  az = readMPU16(0x3F);
  gx = readMPU16(0x43);
  gy = readMPU16(0x45);
  gz = readMPU16(0x47);
}

void setup(){
  Serial.begin(115200);
  Wire.begin(21,22);

  delay(1000);
  Serial.println("Keep MPU6050 COMPLETELY STILL...");
  delay(3000);

  mpuInit();
  delay(100);

  // Collect samples
  for(int i = 0; i < samples; i++){
    readMPU();

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(2); // ~500Hz sampling
  }

  // Calculate averages
  int16_t ax_offset = ax_sum / samples;
  int16_t ay_offset = ay_sum / samples;
  int16_t az_offset = az_sum / samples;
  int16_t gx_offset = gx_sum / samples;
  int16_t gy_offset = gy_sum / samples;
  int16_t gz_offset = gz_sum / samples;

  // IMPORTANT: keep gravity on Z axis (~16384)
  Serial.println("\n=== CALIBRATION RESULTS ===");
  Serial.print("ax_offset = "); Serial.println(ax_offset);
  Serial.print("ay_offset = "); Serial.println(ay_offset);
  Serial.print("az_offset = "); Serial.println(az_offset);
  Serial.print("gx_offset = "); Serial.println(gx_offset);
  Serial.print("gy_offset = "); Serial.println(gy_offset);
  Serial.print("gz_offset = "); Serial.println(gz_offset);

  Serial.println("\nCopy these into your main code.");
}

void loop(){}