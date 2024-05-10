#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define speed 220
#define motorL 6
#define motorR 5
#define pwm_pin A0

Adafruit_MPU6050 mpu;

float P, I, D, Error, Last_error, dif, Position;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  Last_error = 0;
  analogWrite(5, 240);
  analogWrite(6, 240);
  delay(2000);
  analogWrite(5, 0);
  analogWrite(6, 0);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float kp = mapfloat(analogRead(pwm_pin), 0, 1023, 0.00, 100.00);

  Position = a.acceleration.y;
  Error = Position - 0;
  P = Error;
  D = Error - Last_error;
  dif = (P*kp) + (D*0.2);
  Last_error = Error;

  /* Print out the values acceleration*/
  Serial.print("Y: ");
  Serial.print(String(a.acceleration.y) + "\t");
  Serial.print("Error: " + String(Error) + "\t");
  Serial.print("dif: " + String(dif) + "\t");
  /* Print out the values Kp*/
  Serial.println(kp);
  if (dif > speed) {
    analogWrite(motorR, speed);
    analogWrite(motorL, 0);
  }
  else if (dif < -speed) {
    analogWrite(motorR, 0);
    analogWrite(motorL, speed);
  }
  else if (dif > 0) {
    analogWrite(motorR, speed);
    analogWrite(motorL, speed-dif);
  }
  else if (dif < 0) {
    analogWrite(motorR, 100);
    analogWrite(motorL, speed);
  }
  else {
    analogWrite(motorR, speed);
    analogWrite(motorL, speed);
    Serial.println("AAAAAA");
  }
  delay(100);


}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}