#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccZInertial;
float VelocityVertical = 0.00;
float velocityTemp = 0.00;
uint8_t count = 0;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

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
    // AccX = (float)AccXLSB / 4096 - 0.05;
    // AccY = (float)AccYLSB / 4096 + 0.02;
    // AccZ = (float)AccZLSB / 4096 + 0.19;
    AccX = (float)AccXLSB / 4096 - 0.08;
    AccY = (float)AccYLSB / 4096 + 0.01;
    AccZ = (float)AccZLSB / 4096 + 0.19;
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (PI / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (PI / 180);
}

//================================================================

#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;
float InputThrottle;
void read_receiver(void)
{
    ChannelNumber = ReceiverInput.available();
    if (ChannelNumber > 0)
    {
        for (int i = 1; i <= ChannelNumber; i++)
        {
            ReceiverValue[i - 1] = ReceiverInput.read(i);
        }
    }
}
void setup()
{
    Serial.begin(57600);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    ReceiverInput.begin(14);
    analogWriteFrequency(9, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12);
    delay(250);
    while (ReceiverValue[2] < 1020 ||
           ReceiverValue[2] > 1050)
    {
        read_receiver();
        delay(4);
    }
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
    read_receiver();
    InputThrottle = ReceiverValue[2];
    analogWrite(9, 1.024 * InputThrottle);
    analogWrite(2, 1.024 * InputThrottle);
    analogWrite(3, 1.024 * InputThrottle);
    analogWrite(4, 1.024 * InputThrottle);

    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    AccZInertial = -sin(KalmanAnglePitch * (PI / 180)) * AccX + cos(KalmanAnglePitch * (PI / 180)) * sin(KalmanAngleRoll * (PI / 180)) * AccY + cos(KalmanAnglePitch * (PI / 180)) * cos(KalmanAngleRoll * (PI / 180)) * AccZ;
    // Serial.println(AccZInertial);
    if ((AccZInertial - 1.00) >= -0.01 && (AccZInertial - 1.00) <= 0.01)
    {
        count++;
        if (count == 15)
        {
            count = 0;
            VelocityVertical = 0.00;
        }
    }

    if (AccZInertial > 1.00)
    {
        AccZInertial = (AccZInertial - 1.00) * 9.81 * 100;
        if (AccZInertial > 9.81)
        {
            count = 0;
            VelocityVertical = VelocityVertical + AccZInertial * 0.004;
        }
    }
    else if (AccZInertial < 1.00)
    {
        AccZInertial = (AccZInertial - 1.00) * 9.81 * 100;
        if (AccZInertial < -9.81)
        {
            count = 0;
            VelocityVertical = VelocityVertical - AccZInertial * 0.004;
        }
    }

    Serial.println(VelocityVertical);
    while (micros() - LoopTimer < 4000)
        ;
    LoopTimer = micros();
}