// #include "HX711.h"

// // Define the pins connected to the HX711
// #define DT_PIN  21  // Data pin
// #define SCK_PIN 22  // Clock pin

// HX711 scale;

// void setup() {
//   Serial.begin(115200);  // Initialize serial communication
//   scale.begin(DT_PIN, SCK_PIN);  // Initialize the scale with the pins

//   // Optional: Set the scale's calibration factor
//   scale.set_scale();  // If you have a calibration factor, set it with scale.set_scale(calibration_factor);
  
//   // Optional: Tare the scale (remove any weight before this step)
//   scale.tare();  // Reset the scale to 0

//   Serial.println("HX711 calibration example");
// }

// void loop() {
//   if (scale.is_ready()) {
//     long weight = scale.get_units(10);  // Read the weight in grams (or the units of your choice)
//     Serial.print("Weight: ");
//     Serial.print(weight);
//     Serial.println(" g");
//   } else {
//     Serial.println("HX711 not detected.");
//   }
//   delay(1000);  // Delay between readings
// }
