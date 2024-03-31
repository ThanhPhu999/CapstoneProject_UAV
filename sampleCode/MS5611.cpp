#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

#define MS5611_address 0x77 // The I2C address of the MS5611 barometer is 0x77 in
uint32_t loop_timer;
uint8_t start;
// uint8_t error = 0;
// uint32_t stable_time = 0;

// Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[20], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT;
// int32_t dT_C5;

// Altitude PID variables
// float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
// uint8_t parachute_rotating_mem_location;
// int32_t parachute_buffer[35], parachute_throttle;
// float pressure_parachute_previous;
int32_t pressure_rotating_mem[90], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

void read_barometer(void)
{
    barometer_counter++;
    // stable_time++;
    // Every time this function is called the barometer_counter variable is incremented. This way a specific action
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
        if (temperature_counter == 20)
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
    if (barometer_counter == 2)
    { // If the barometer_counter variable equals 2.
        // Calculate pressure as explained in the datasheet of the MS-5611.
        // barometer_counter = 0;
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
        if (pressure_rotating_mem_location == 80)
            pressure_rotating_mem_location = 0;                      // Start at 0 when the memory location 20 is reached.
        actual_pressure_fast = (float)pressure_total_avarage / 80.0; // Calculate the average pressure of the last 20 pressure readings.

        // To get better results we will use a complementary fillter that can be adjusted by the fast average.
        // if (stable_time < 1250)
        // {
        //     actual_pressure_slow = actual_pressure_slow * (float)0.9 + actual_pressure_fast * (float)0.1;
        // }
        // else
        // {
        //     actual_pressure_slow = actual_pressure_slow * (float)0.995 + actual_pressure_fast * (float)0.005;
        //     stable_time--;
        // }
        actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;

        actual_pressure_diff = actual_pressure_slow - actual_pressure_fast; // Calculate the difference between the fast and the slow avarage value.
        if (actual_pressure_diff > 8)
            actual_pressure_diff = 8; // If the difference is larger then 8 limit the difference to 8.
        if (actual_pressure_diff < -8)
            actual_pressure_diff = -8; // If the difference is smaller then -8 limit the difference to -8.
        // If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
        if (actual_pressure_diff > 1 || actual_pressure_diff < -1.0)
            actual_pressure_slow -= actual_pressure_diff / 6.0;
        actual_pressure = actual_pressure_slow; // The actual_pressure is used in the program for altitude calculations.
    }
    if (barometer_counter == 3)
    {
        barometer_counter = 0;
    }
}

void setup()
{
    Serial.begin(57600);
    Wire.setClock(400000);
    Wire.begin();
    // Check if the MS5611 barometer is responding.
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

    loop_timer = micros(); // Set the timer for the first loop.
}

void loop()
{
    read_barometer(); // Read and calculate the barometer data
    Serial.println(actual_pressure);
    while (micros() - loop_timer < 4000)
        ; // We wait until 4000us are passed.
    loop_timer = micros();
}
