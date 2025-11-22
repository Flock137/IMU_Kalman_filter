#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Calibration offsets
float AccX_offset = 0, AccY_offset = 0, AccZ_offset = 0;

// Kalman filter variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

void kalman_1d(float KalmanState, float KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement)
{
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting...");
  Serial.println("Initializing MPU6050...");

  // Initialize MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      Serial.println("Stuck in error loop");
      delay(1000);
    }
  }

  Serial.println("MPU6050 found!");

  // Configure accelerometer range to ±8g
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("Accel range set");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("Gyro range set");

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Filter set");

  Serial.println("MPU6050 initialized!");
  delay(100);
}

void loop()
{
  // Read sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert to g's and apply calibration
  float AccX = (a.acceleration.x / 9.81) - AccX_offset;
  float AccY = (a.acceleration.y / 9.81) - AccY_offset;
  float AccZ = (a.acceleration.z / 9.81) - AccZ_offset;

  // Calculate angles from accelerometer
  float AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  float AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);

  // Gyro rates (convert from rad/s to deg/s)
  float RateRoll = g.gyro.x * (180.0 / PI);
  float RatePitch = g.gyro.y * (180.0 / PI);

  // Apply Kalman filter for Roll
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // Apply Kalman filter for Pitch
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Print for Serial Plotter: Raw Roll, Filtered Roll, Raw Pitch, Filtered Pitch
  // Print for Serial Plotter with labels
// Print in Teleplot format
  Serial.print(">RawRoll:");
  Serial.println(AngleRoll);
  
  Serial.print(">FilteredRoll:");
  Serial.println(KalmanAngleRoll);
  
  Serial.print(">RawPitch:");
  Serial.println(AnglePitch);
  
  Serial.print(">FilteredPitch:");
  Serial.println(KalmanAnglePitch);
  
  delay(4);
}