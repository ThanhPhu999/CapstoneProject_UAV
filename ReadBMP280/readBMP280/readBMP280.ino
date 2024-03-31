/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
uint32_t LoopTimer;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

float actual_pressure_fast = 0.00;
float actual_pressure_slow = 0.00;
float pressure_total_avarage = 0.00;
uint8_t pressure_rotating_mem_location = 0;
float pressure_rotating_mem[200] = {};
float actual_pressure = 0.00;
float actual_pressure_diff = 0.00;

#include <iostream>

class KalmanFilter
{
public:
    KalmanFilter(double processVariance, double measurementVariance, double initialEstimate = 0, double initialEstimateError = 1)
        : estimate(initialEstimate), estimateError(initialEstimateError), processVariance(processVariance), measurementVariance(measurementVariance) {}

    double update(double measurement)
    {
        // Prediction
        double prediction = estimate;
        double predictionError = estimateError + processVariance * 0.004 * 0.004;

        // Update
        double kalmanGain = predictionError / (predictionError + measurementVariance);
        estimate = prediction + kalmanGain * (measurement - prediction);
        estimateError = (1 - kalmanGain) * predictionError;

        return estimate;
    }

private:
    double estimate;
    double estimateError;
    double processVariance;
    double measurementVariance;
};
// double processVariance = 0.01;
double processVariance = 4;
double measurementVariance = 0.01;
KalmanFilter kalman(processVariance, measurementVariance);
// int main()
// {
//     // Example usage with BMP280 pressure values
//     double processVariance = 0.01;  // Adjust this based on your system and noise characteristics
//     double measurementVariance = 1; // Adjust this based on your BMP280 datasheet and noise characteristics

//     // Initialize the Kalman filter
//     KalmanFilter kalman(processVariance, measurementVariance);

//     // Simulate pressure measurements from BMP280
//     // Replace this with actual pressure readings from your BMP280 sensor
//     double pressureMeasurements[] = {1000, 1001, 999, 1002, 998, 1003};
//     int numMeasurements = sizeof(pressureMeasurements) / sizeof(pressureMeasurements[0]);

//     // Filter the pressure values using the Kalman filter
//     for (int i = 0; i < numMeasurements; ++i)
//     {
//         double filteredPressure = kalman.update(pressureMeasurements[i]);
//         std::cout << "Measurement: " << pressureMeasurements[i] << ", Filtered: " << filteredPressure << std::endl;
//     }

//     return 0;
// }

void setup()
{
    Serial.begin(9600);
    unsigned status;
    status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    bmp_temp->printSensorDetails();
    LoopTimer = micros();
}
void loop()
{
    sensors_event_t pressure_event;
    bmp_pressure->getEvent(&pressure_event);

    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem[pressure_rotating_mem_location] = pressure_event.pressure * 100;
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem_location++;
    if (pressure_rotating_mem_location == 200)
        pressure_rotating_mem_location = 0;
    actual_pressure_fast = (float)pressure_total_avarage / 200.0;
    actual_pressure_slow = actual_pressure_slow * (float)0.995 + actual_pressure_fast * (float)0.005;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;

    // Serial.println(actual_pressure_diff);
    // if (actual_pressure_diff > 8)
    //   actual_pressure_diff = 8;
    // if (actual_pressure_diff < -8)
    //   actual_pressure_diff = -8;
    // if (actual_pressure_diff > 1 || actual_pressure_diff < -1)
    //   actual_pressure_slow -= actual_pressure_diff / 6.0;

    // actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;
    actual_pressure = kalman.update(actual_pressure);
    // Serial.print(F("Pressure = "));
    Serial.println(actual_pressure);
    // Serial.println(" Pa");
    //   delay(30);
    while (micros() - LoopTimer < 4000)
        ;
    LoopTimer = micros();
}
