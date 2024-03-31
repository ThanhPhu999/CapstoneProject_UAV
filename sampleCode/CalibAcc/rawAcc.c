#include <Wire.h>
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
void gyro_signals(void)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

    // AccX = (float)AccXLSB / 4096 - 0.07;
    // AccY = (float)AccYLSB / 4096 - 0.00;
    // AccZ = (float)AccZLSB / 4096 - 0.06;
    // AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (PI / 180);
    // AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (PI / 180);
}
void setup()
{
    Serial.begin(57600);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}
void loop()
{
    // gyro_signals();
    // Serial.print("Roll angle [°]= ");
    // Serial.print(AngleRoll);
    // Serial.print(" Pitch angle [°]= ");
    // Serial.println(AnglePitch);
    // Serial.print("Acceleration X [g]= ");
    // Serial.print(AccX);
    // Serial.print(" Acceleration Y [g]= ");
    // Serial.print(AccY);
    // Serial.print(" Acceleration Z [g]= ");
    // Serial.println(AccZ);

    // delay(50);
    LoopTimer = micros();
    gyro_signals();
    Serial.print(AccX, 6);
    Serial.print(",");
    Serial.print(AccY, 6);
    Serial.print(",");
    Serial.println(AccZ, 6);
    while(micros()-LoopTimer < 4000){

    }
}