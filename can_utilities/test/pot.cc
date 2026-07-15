#include <Arduino.h>
// Define the pin connected to the potentiometer wiper
const int potPin = GPIO_NUM_34; // Use an appropriate GPIO pin for your ESP32 board

// Variable to store the read value
int potValue = 0;

void setup() {
  // Start Serial communication at 115200 baud
  Serial.begin(115200);
}

void loop() {
  // Read the analog value from the potentiometer
  potValue = analogRead(potPin);

  // Print the value to the Serial Monitor
  Serial.print("Potentiometer Value: ");
  Serial.println(potValue);

  // Delay for a short period to make the output readable
  delay(100);
}