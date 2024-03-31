#include <Wire.h>
/*-----------------------Variable for rate mode---------------------------*/
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
/*---------------------------------------------------------------------*/

/*----------------------------hardware variable-----------------------*/
#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 1300;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
/*----------------------------------------------------------------*/

uint32_t LoopTimer;
float PIDReturn[] = {0, 0, 0};
float InputRoll, InputThrottle, InputPitch, InputYaw;
float KeepThrottle;

/*--------------------------------PID rate mode------------------------*/
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PRateRoll = 0.8;
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = IRateRoll;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = DRateRoll;
float DRateYaw = 0;
/*----------------------------------------------------------------*/

/*-------------Kalman 1D vs PID angle mode------------------------*/
float AccXYZ[3][1] = {{0}, {0}, {0}};
float matrixA[3][3] = {{0.993879, -0.007421, 0.000172},
                       {-0.007421, 0.999112, 0.001981},
                       {0.000172, 0.001981, 0.984192}};
float vectorB[3][1] = {{0.064609}, {0.005212}, {0.041257}};
void calibAcc()
{
    float temp[3][3] = {{}, {}, {}};
    for (int i = 0; i < 3; i++)
    {
        temp[i][0] = AccXYZ[i][0] - vectorB[i][0];
    }

    for (int i = 0; i < 3; i++)
    {
        AccXYZ[i][0] = 0;
        for (int j = 0; j < 3; j++)
        {
            AccXYZ[i][0] += matrixA[i][j] * temp[j][0];
        }
    }
}

// kalman filter and PID for stable mode
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2;
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0;
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0;
float DAnglePitch = DAngleRoll;
/*----------------------------------------------------------------*/

/*-------------------------PID hole altitude----------------------*/
#define MS5611_address 0x77 // The I2C address of the MS5611 barometer is 0x77 in
uint8_t start;
uint32_t stable_time = 0;

// Pressure variables.
// float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[20], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
// float ground_pressure, altutude_hold_pressure;
int32_t dT;
// int32_t dT_C5;

int32_t pressure_rotating_mem[60], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;

bool hold_altitude_flag = false;
bool keep_altitude_flag = true;
float desired_pressure = 0.00;
float error_pressure = 0.00;
float prevError_pressure = 0.00;
float prev_iterm_pressure = 0.00;
float P_pressure = 1.2;
float I_pressure = 0.002;
float D_pressure = 1.5;

void read_barometer(void)
{
    barometer_counter++;
    stable_time++;
    // is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.
    if (barometer_counter == 1)
    { // When the barometer_counter variable is 1.
        if (temperature_counter == 0)
        { // And the temperature counter is 0.
            // Get temperature data from MS-5611
            Wire.beginTransmission(MS5611_address); // Open a connection with the MS5611
            Wire.write(0x00);                       // Send a 0 to indicate that we want to poll the requested data.
            Wire.endTransmission();                 // End the transmission with the MS5611.
            Wire.requestFrom(MS5611_address, 3);    // Poll 3 data bytes from the MS5611.
            // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
            raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
            raw_temperature_rotating_memory[average_temperature_mem_location] = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
            raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
            average_temperature_mem_location++;
            if (average_temperature_mem_location == 5)
                average_temperature_mem_location = 0;
            raw_temperature = raw_average_temperature_total / 5; // Calculate the avarage temperature of the last 5 measurements.
        }
        else
        {
            // Get pressure data from MS-5611
            Wire.beginTransmission(MS5611_address);                            // Open a connection with the MS5611.
            Wire.write(0x00);                                                  // Send a 0 to indicate that we want to poll the requested data.
            Wire.endTransmission();                                            // End the transmission with the MS5611.
            Wire.requestFrom(MS5611_address, 3);                               // Poll 3 data bytes from the MS5611.
            raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read(); // Shift the individual bytes in the correct position and add them to the raw_pressure variable.
        }

        temperature_counter++; // Increase the temperature_counter variable.
        if (temperature_counter == 10)
        {                            // When the temperature counter equals 20.
            temperature_counter = 0; // Reset the temperature_counter variable.
            // Request temperature data
            Wire.beginTransmission(MS5611_address); // Open a connection with the MS5611.
            Wire.write(0x58);                       // Send a 0x58 to indicate that we want to request the temperature data.
            Wire.endTransmission();                 // End the transmission with the MS5611.
        }
        else
        { // If the temperature_counter variable does not equal 20.
            // Request pressure data
            Wire.beginTransmission(MS5611_address); // Open a connection with the MS5611
            Wire.write(0x48);                       // Send a 0x48 to indicate that we want to request the pressure data.
            Wire.endTransmission();                 // End the transmission with the MS5611.
        }
    }
    if (barometer_counter == 4)
    { // If the barometer_counter variable equals 4.
        // Calculate pressure as explained in the datasheet of the MS-5611.
        barometer_counter = 0;
        dT = C[5];
        dT <<= 8;
        dT *= -1;
        dT += raw_temperature;
        OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
        SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
        P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
        // To get a smoother pressure value we will use a 20 location rotating memory.
        pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location]; // Subtract the current memory position to make room for the new value.
        pressure_rotating_mem[pressure_rotating_mem_location] = P;                       // Calculate the new change between the actual pressure and the previous measurement.
        pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location]; // Add the new value to the long term avarage value.
        pressure_rotating_mem_location++;                                                // Increase the rotating memory location.
        if (pressure_rotating_mem_location == 60)
            pressure_rotating_mem_location = 0;                      // Start at 0 when the memory location 20 is reached.
        actual_pressure_fast = (float)pressure_total_avarage / 60.0; // Calculate the average pressure of the last 20 pressure readings.

        // To get better results we will use a complementary fillter that can be adjusted by the fast average.
        if (stable_time < 2500)
        {
            actual_pressure_slow = actual_pressure_slow * (float)0.9 + actual_pressure_fast * (float)0.1;
        }
        else
        {
            actual_pressure_slow = actual_pressure_slow * (float)0.995 + actual_pressure_fast * (float)0.005;
            stable_time--;
        }

        actual_pressure_diff = actual_pressure_slow - actual_pressure_fast; // Calculate the difference between the fast and the slow avarage value.
        if (actual_pressure_diff > 8)
            actual_pressure_diff = 8; // If the difference is larger then 8 limit the difference to 8.
        if (actual_pressure_diff < -8)
            actual_pressure_diff = -8; // If the difference is smaller then -8 limit the difference to -8.
        // If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
        if (actual_pressure_diff > 1 || actual_pressure_diff < -1.0)
            actual_pressure_slow -= actual_pressure_diff / 3.0;
        actual_pressure = actual_pressure_slow; // The actual_pressure is used in the program for altitude calculations.
    }
}

/*----------------------------------------------------------------*/

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
void battery_voltage(void)
{
    Voltage = (float)analogRead(15) / 62;
    Current = (float)analogRead(21) * 0.089;
}
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
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
    if (Iterm > 400)
        Iterm = 400;
    else if (Iterm < -400)
        Iterm = -400;
    float Dterm = D * (Error - PrevError) / 0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)
        PIDOutput = 400;
    else if (PIDOutput < -400)
        PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}
void pid_equation_pressure(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.032 / 2;
    if (Iterm > 400)
        Iterm = 400;
    else if (Iterm < -400)
        Iterm = -400;
    float Dterm = D * (Error - PrevError) / 0.032;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)
        PIDOutput = 400;
    else if (PIDOutput < -400)
        PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}
void reset_pid(void)
{
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;

    prevError_pressure = 0;
    prev_iterm_pressure = 0;
}
void setup()
{
    Serial.begin(57600);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    // gyro and accel
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

    // barometer
    Wire.beginTransmission(MS5611_address); // Start communication with the MS5611.
    Wire.endTransmission();                 // End the transmission and register the exit status.
    // For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    // These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++)
    {
        Wire.beginTransmission(MS5611_address); // Start communication with the MPU-6050.
        Wire.write(0xA0 + start * 2);           // Send the address that we want to read.
        Wire.endTransmission();                 // End the transmission.

        Wire.requestFrom(MS5611_address, 2);       // Request 2 bytes from the MS5611.
        C[start] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the C[x] calibration variable.
    }
    OFF_C2 = C[2] * pow(2, 16);
    SENS_C1 = C[1] * pow(2, 15);

    for (start = 0; start < 100; start++)
    {                     // This loop runs 100 times.
        read_barometer(); // Read and calculate the barometer data.
        delay(4);         // The main program loop also runs 250Hz (4ms per loop).
    }
    actual_pressure = 0; // Reset the pressure calculations.

    analogWriteFrequency(9, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12);
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);
    battery_voltage();
    if (Voltage > 8.3)
    {
        digitalWrite(5, LOW);
        BatteryAtStart = BatteryDefault;
    }
    else if (Voltage < 7.5)
    {
        BatteryAtStart = 30 / 100 * BatteryDefault;
    }
    else
    {
        digitalWrite(5, LOW);
        BatteryAtStart = (82 * Voltage - 580) / 100 * BatteryDefault;
    }
    ReceiverInput.begin(14);
    while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050)
    {
        read_receiver();
        delay(4);
    }
    LoopTimer = micros();
}
void loop()
{
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    calibAcc();
    AngleRoll = atan(AccXYZ[1][0] / sqrt(AccXYZ[0][0] * AccXYZ[0][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
    AnglePitch = -atan(AccXYZ[0][0] / sqrt(AccXYZ[1][0] * AccXYZ[1][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
    AngleRoll -= 2.15;
    AnglePitch -= 1.5;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    // AccZInertial = -sin(AnglePitch * (3.142 / 180)) * AccX + cos(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY + cos(AnglePitch * (3.142 / 180)) * cos(AngleRoll * (3.142 / 180)) * AccZ;
    // AccZInertial = (AccZInertial - 1) * 9.81 * 100;
    // barometer_signals();
    // AltitudeBarometer -= AltitudeBarometerStartUp;
    // kalman_2d();
    read_barometer();
    if (hold_altitude_flag)
    {
        if (keep_altitude_flag)
        {
            desired_pressure = actual_pressure;
            keep_altitude_flag = false;
        }
        error_pressure = desired_pressure - actual_pressure;
        pid_equation_pressure(error_pressure, P_pressure, I_pressure, D_pressure, prevError_pressure, prev_iterm_pressure);
        InputThrottle = 1500 + PIDReturn[0];
        prevError_pressure = PIDReturn[1];
        prev_iterm_pressure = PIDReturn[2];
    }
    else
    {
        keep_altitude_flag = true;
        prevError_pressure = 0;
        prev_iterm_pressure = 0;
    }

    read_receiver();
    DesiredAngleRoll = 0.10 * (ReceiverValue[0] - 1529.95);
    DesiredAnglePitch = 0.10 * (ReceiverValue[1] - 1508.95);
    InputThrottle = ReceiverValue[2];
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1499.95);

    if (ReceiverValue[4] > 1500)
    {
        hold_altitude_flag = true;
        KeepThrottle = InputThrottle;
    }
    else
    {
        InputThrottle = ReceiverValue[2];
        hold_altitude_flag = false;
    }

    // DesiredVelocityVertical = 0.3 * (ReceiverValue[2] - 1500);
    // ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
    // pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical);
    // InputThrottle = 1500 + PIDReturn[0];
    // PrevErrorVelocityVertical = PIDReturn[1];
    // PrevItermVelocityVertical = PIDReturn[2];

    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];
    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    if (InputThrottle > 1800)
        InputThrottle = 1800;
    MotorInput1 = 1.024 * (InputThrottle - InputPitch - InputRoll - InputYaw);
    MotorInput2 = 1.024 * (InputThrottle + InputPitch - InputRoll + InputYaw);
    MotorInput3 = 1.024 * (InputThrottle + InputPitch + InputRoll - InputYaw);
    MotorInput4 = 1.024 * (InputThrottle - InputPitch + InputRoll + InputYaw);
    if (MotorInput1 > 2000)
        MotorInput1 = 1999;
    if (MotorInput2 > 2000)
        MotorInput2 = 1999;
    if (MotorInput3 > 2000)
        MotorInput3 = 1999;
    if (MotorInput4 > 2000)
        MotorInput4 = 1999;
    int ThrottleIdle = 1180;
    if (MotorInput1 < ThrottleIdle)
        MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle)
        MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle)
        MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle)
        MotorInput4 = ThrottleIdle;
    int ThrottleCutOff = 1000;
    if (ReceiverValue[2] < 1050)
    {
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;
        reset_pid();
    }
    analogWrite(9, MotorInput1);
    analogWrite(2, MotorInput2);
    analogWrite(3, MotorInput3);
    analogWrite(4, MotorInput4);
    battery_voltage();
    CurrentConsumed = Current * 1000 * 0.004 / 3600 + CurrentConsumed;
    BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100;
    if (BatteryRemaining <= 30)
        digitalWrite(5, HIGH);
    else
        digitalWrite(5, LOW);
    while (micros() - LoopTimer < 4000)
        ;
    LoopTimer = micros();
}