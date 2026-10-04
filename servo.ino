#include <Servo.h>
#include <Wire.h>

Servo Servo1;
Servo Servo2; 

int servo1Pin = 9;
int servo2Pin = 10;

int i = 0;
int count = 0;

int16_t temp_values[128];

const uint16_t MPU_address = 0x68;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Servo1.attach(servo1Pin);
  Servo1.write(90);

  Servo2.attach(servo2Pin);
  Servo2.write(90);

  Wire.beginTransmission(MPU_address); //address of device
  Wire.write(0x6B); //sleep
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_address);
  Wire.write(0x1C);
  Wire.write(0x00); //config sensitivity for accelerometer
  Wire.endTransmission();
}

void loop() {

  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B); //accounts for X,Y,Z since registers come back to back
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)MPU_address, (uint8_t)8); //Wire.requestFrom(address, quantity, stop)

  if(Wire.available() >= 8){
    uint8_t ACCEL_X_H = Wire.read();
    uint8_t ACCEL_X_L = Wire.read();
    int16_t ACCEL_X_F = ((uint16_t)(ACCEL_X_H << 8) | (uint16_t)ACCEL_X_L); //_F for final value

    float accel_x_f = ACCEL_X_F / 16384.0f; //16384 raw counts for every 1m/s^2 or 1g

    uint8_t ACCEL_Y_H = Wire.read();
    uint8_t ACCEL_Y_L = Wire.read();
    int16_t ACCEL_Y_F = (((uint16_t)ACCEL_Y_H << 8) | (uint16_t)ACCEL_Y_L);

    float accel_y_f = ACCEL_Y_F / 16384.0f;

    uint8_t ACCEL_Z_H = Wire.read();
    uint8_t ACCEL_Z_L = Wire.read();
    int16_t ACCEL_Z_F = (((uint16_t)ACCEL_Z_H << 8) | (uint16_t)ACCEL_Z_L);

    float accel_z_f = ACCEL_Z_F / 16384.0f;

    uint8_t TEMP_H = Wire.read(); //Temp_H = 0x41, Temp_L = 0x42
    uint8_t TEMP_L = Wire.read();
    int16_t TEMP_F = (((uint16_t)TEMP_H << 8) | (uint16_t)TEMP_L);

    Serial.print("X: ");
    Serial.print(accel_x_f);
    Serial.print(" | Y: ");
    Serial.print(accel_y_f);
    Serial.print(" | Z: ");
    Serial.println(accel_z_f);

    float s1motion = atan2(-accel_x_f, sqrt(accel_y_f * accel_y_f + accel_z_f * accel_z_f)) * (180.0f/PI); 
    float s2motion = atan2(accel_y_f, accel_z_f) * (180.0f/PI);

    s1motion = constrain(s1motion, -45.0f, 45.0f);
    s2motion = constrain(s2motion, -45.0f, 45.0f);

    int servoAngle1 = (int)round(90.0f + s1motion);
    int servoAngle2 = (int)round(90.0f + s2motion);

    Servo1.write(servoAngle1);
    Servo2.write(servoAngle2); 
    
    temp_values[i] = TEMP_F;
    i = (i + 1) % 128;

    if(count < 128){
      count++;
    }

    if(count == 128){
      long sum = 0;

      for(int j = 0; j < 128; j++){ //add all array values
        sum += temp_values[j];
      }
      double mean = (double)sum / 128;

      double hold = 0;

      for(int j = 0; j < 128; j++){
        double diff = temp_values[j] - mean;
        hold += diff * diff;
      }

      double stddev = sqrt(hold / 128);

      Serial.print("Temperature raw mean: ");
      Serial.print(mean, 3);
      Serial.print(" | Temperature raw standard deviation: ");
      Serial.println(stddev, 3);
      }
    }
  delay(500);
}
