#include <Wire.h>

#define GY_US42V2_ADDRESS 0x70

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  // Start measurement
  Wire.beginTransmission(GY_US42V2_ADDRESS);
  Wire.write(0x51); // Command to start measurement
  Wire.endTransmission();

  delay(100); // Wait for measurement to complete (adjust delay as needed, max is 65ms)

  // Read result
  Wire.requestFrom(GY_US42V2_ADDRESS, 2); // Request 2 bytes from sensor
  if (Wire.available() >= 2)
  {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    unsigned int distance = (highByte << 8) | lowByte; // Combine high and low bytes
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  else
  {
    Serial.println("Error: No data available from sensor");
  }

  delay(1000); // Delay before next measurement
}

//==============================================================================
