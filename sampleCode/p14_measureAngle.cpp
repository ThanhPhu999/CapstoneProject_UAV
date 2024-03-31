// #include <Wire.h>
// float RateRoll, RatePitch, RateYaw;
// float AccXYZ[3][1]={{0}, {0}, {0}};
// // float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch;
// float LoopTimer;

// float matrixA[3][3]={{0.993879, -0.007421, 0.000172},
//                     {-0.007421, 0.999112, 0.001981},
//                     {0.000172, 0.001981, 0.984192}};
// float vectorB[3][1]={{0.064609}, {0.005212}, {0.041257}};
// void calibAcc(){
//     float temp[3][3]={{},{},{}};
//     for(int i = 0; i < 3; i++){
//         temp[i][0] = AccXYZ[i][0] - vectorB[i][0];
//     }

//     for(int i = 0; i < 3; i++){
//       AccXYZ[i][0] = 0;
//       for(int j = 0; j < 3; j++){
//           AccXYZ[i][0] += matrixA[i][j]*temp[j][0];
//       }
//     }
// }
// void gyro_signals(void)
// {
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1A);
//     Wire.write(0x05);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1C);
//     Wire.write(0x10);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x3B);
//     Wire.endTransmission();
//     Wire.requestFrom(0x68, 6);
//     int16_t AccXLSB = Wire.read() << 8 | Wire.read();
//     int16_t AccYLSB = Wire.read() << 8 | Wire.read();
//     int16_t AccZLSB = Wire.read() << 8 | Wire.read();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1B);
//     Wire.write(0x8);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x43);
//     Wire.endTransmission();
//     Wire.requestFrom(0x68, 6);
//     int16_t GyroX = Wire.read() << 8 | Wire.read();
//     int16_t GyroY = Wire.read() << 8 | Wire.read();
//     int16_t GyroZ = Wire.read() << 8 | Wire.read();
//     RateRoll = (float)GyroX / 65.5;
//     RatePitch = (float)GyroY / 65.5;
//     RateYaw = (float)GyroZ / 65.5;
//     AccXYZ[0][0] = (float)AccXLSB / 4096;
//     AccXYZ[1][0] = (float)AccYLSB / 4096;
//     AccXYZ[2][0] = (float)AccZLSB / 4096;
// }
// void setup()
// {
//     Serial.begin(57600);
//     pinMode(13, OUTPUT);
//     digitalWrite(13, HIGH);
//     Wire.setClock(400000);
//     Wire.begin();
//     delay(250);
//     Wire.beginTransmission(0x68);
//     Wire.write(0x6B);
//     Wire.write(0x00);
//     Wire.endTransmission();
// }
// void loop()
// {
//     LoopTimer = micros();
//     gyro_signals();
//     calibAcc();
//     // AccXYZ[1][0] -= 0.02;
//     // AccXYZ[0][0] += 0.01;
//     AngleRoll = atan(AccXYZ[1][0] / sqrt(AccXYZ[0][0] * AccXYZ[0][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
//     AnglePitch = -atan(AccXYZ[0][0] / sqrt(AccXYZ[1][0] * AccXYZ[1][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
//     Serial.print("Roll angle [°]= ");
//     Serial.print(AngleRoll);
//     Serial.print(" Pitch angle [°]= ");
//     Serial.println(AnglePitch);
//     Serial.print("Acceleration X [g]= ");
//     Serial.print(AccXYZ[0][0]);
//     Serial.print(" Acceleration Y [g]= ");
//     Serial.print(AccXYZ[1][0]);
//     Serial.print(" Acceleration Z [g]= ");
//     Serial.println(AccXYZ[2][0]);
//     Serial.println("---------------------------------------");
//     while(micros()-LoopTimer < 4000){

//     }
// }

//=============================================================================================
#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccXYZ[3][1] = {{0}, {0}, {0}};
int16_t AccXLSB, AccYLSB, AccZLSB;
float angle_roll, angle_pitch;
float angle_roll_acc;
float angle_pitch_acc;
float LoopTimer;
bool first_angle = false;

void gyro_signals(void)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    AccXLSB = Wire.read() << 8 | Wire.read();
    AccYLSB = Wire.read() << 8 | Wire.read();
    AccZLSB = Wire.read() << 8 | Wire.read();
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

    RatePitch *= -1;
    RateYaw *= -1;
    // AccXYZ[0][0] = (float)AccXLSB / 4096;
    // AccXYZ[1][0] = (float)AccYLSB / 4096;
    // AccXYZ[2][0] = (float)AccZLSB / 4096;
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

    for (RateCalibrationNumber = 0;
         RateCalibrationNumber < 2000;
         RateCalibrationNumber++)
    {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
}
void loop()
{
    LoopTimer = micros();
    gyro_signals();

    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    angle_pitch += RatePitch * 0.0000611;
    angle_roll += RateRoll * 0.0000611;

    // If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_pitch -= angle_roll * sin(RateYaw * 0.000001066);
    angle_roll += angle_pitch * sin(RateYaw * 0.000001066);

    // Accelerometer angle calculations
    if (AccXLSB > 4096)
        AccXLSB = 4096;
    if (AccXLSB < -4096)
        AccXLSB = -4096;
    if (AccYLSB > 4096)
        AccYLSB = 4096;
    if (AccYLSB < -4096)
        AccYLSB = -4096;

    // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)AccXLSB / 4096) * 57.296; // Calculate the pitch angle.
    angle_roll_acc = asin((float)AccYLSB / 4096) * 57.296;  // Calculate the roll angle.
    angle_pitch_acc -= 2.3;
    angle_roll_acc -= 0.9;

    if (!first_angle)
    {
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        first_angle = true;
    }
    else
    {
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }

    Serial.print("Roll angle [°]= ");
    Serial.print(angle_roll);
    Serial.print("   Pitch angle [°]= ");
    Serial.println(angle_pitch);
    // Serial.print("Acceleration X [g]= ");
    // Serial.print(AccXYZ[0][0]);
    // Serial.print(" Acceleration Y [g]= ");
    // Serial.print(AccXYZ[1][0]);
    // Serial.print(" Acceleration Z [g]= ");
    // Serial.println(AccXYZ[2][0]);
    Serial.println("---------------------------------------");
    while (micros() - LoopTimer < 4000)
    {
    }
}

angle_pitch += gyro_x * 0.0000611;
angle_roll += gyro_y * 0.0000611;

// 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
angle_pitch -= angle_roll * sin(gyro_z * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
angle_roll += angle_pitch * sin(gyro_z * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

// Accelerometer angle calculations
if (acc_axis[1] > 4096)
    acc_axis[1] = 4096;
if (acc_axis[1] < -4096)
    acc_axis[1] = -4096;
if (acc_axis[2] > 4096)
    acc_axis[2] = 4096;
if (acc_axis[2] < -4096)
    acc_axis[2] = -4096;

acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
angle_roll_acc = asin((float)acc_x / acc_total_vector) * (-57.296);

angle_pitch_acc -= 0.5;
angle_roll_acc -= 0.4;

if (!first_angle)
{                                  // When this is the first time.
    angle_pitch = angle_pitch_acc; // Set the pitch angle to the accelerometer angle.
    angle_roll = angle_roll_acc;   // Set the roll angle to the accelerometer angle.
    first_angle = true;
}
else
{                                                                  // When this is not the first time.
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.
}