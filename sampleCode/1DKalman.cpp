#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccZInertial;
float VelocityVertical;

float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

float AccXYZ[3][1]={{0}, {0}, {0}};
float matrixA[3][3]={{0.993879, -0.007421, 0.000172},
                    {-0.007421, 0.999112, 0.001981},
                    {0.000172, 0.001981, 0.984192}};
float vectorB[3][1]={{0.064609}, {0.005212}, {0.041257}};
void calibAcc(){
    float temp[3][3]={{},{},{}};
    for(int i = 0; i < 3; i++){
        temp[i][0] = AccXYZ[i][0] - vectorB[i][0];
    }

    for(int i = 0; i < 3; i++){
      AccXYZ[i][0] = 0;
      for(int j = 0; j < 3; j++){
          AccXYZ[i][0] += matrixA[i][j]*temp[j][0];
      }
    }
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}
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
    AccXYZ[0][0] = (float)AccXLSB / 4096;
    AccXYZ[1][0] = (float)AccYLSB / 4096;
    AccXYZ[2][0] = (float)AccZLSB / 4096;
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
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
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
    LoopTimer = micros();
}
void loop()
{
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    calibAcc();
    // AccXYZ -= 0.02;
    AngleRoll = atan(AccXYZ[1][0] / sqrt(AccXYZ[0][0] * AccXYZ[0][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
    AnglePitch = -atan(AccXYZ[0][0] / sqrt(AccXYZ[1][0] * AccXYZ[1][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    Serial.print("Roll Angle = ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Pitch Angle = ");
    Serial.println(KalmanAnglePitch);

    // AccZInertial = -sin(KalmanAnglePitch * (3.142 / 180)) * AccX + cos(KalmanAnglePitch * (3.142 / 180)) * sin(KalmanAngleRoll * (3.142 / 180)) * AccY + cos(KalmanAnglePitch * (3.142 / 180)) * cos(KalmanAngleRoll * (3.142 / 180)) * AccZ;
    // AccZInertial = (AccZInertial - 1) * 9.81 * 100;
    // VelocityVertical = VelocityVertical + AccZInertial * 0.004;
    // Serial.print("Vertical velocity [cm/s]: ");
    // Serial.println(VelocityVertical);
    while (micros() - LoopTimer < 4000)
        ;
    LoopTimer = micros();
}