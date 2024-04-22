// // #include <Arduino.h>
// // #include <TinyGPSPlus.h>
// // // ================================================================
// // // Variable declaration
// // // ================================================================
// // #define RXD2 17
// // #define TXD2 16
// // TinyGPSPlus gps;
// // // ================================================================
// // // Function Declaration
// // // ================================================================
// // void Init_Serial(); // Init the serial monitor
// // void Get_GPSData(); // Get the GPS data
// // void displayInfo(); // Display info from the GPS
// // // ================================================================
// // // Setup function
// // // ================================================================
// // void setup()
// // {
// //   Init_Serial();
// //   // Serial2.begin(baud-rate, protocol, RX pin, TX pin);
// //   Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
// //   Serial.println("Serial Txd is on pin: "+String(TX));
// //   Serial.println("Serial Rxd is on pin: "+String(RX));
// // }
// // // ================================================================
// // // Loop function
// // // ================================================================
// // void loop()
// // {
// //   Get_GPSData();
// // }
// // // ================================================================
// // // Function Definition
// // // ================================================================
// // void Init_Serial()
// // {
// //   Serial.begin(115200);
// //   while (!Serial)
// //     ;
// // }
// // // ================================================================
// // void Get_GPSData()
// // {
// //   Serial.print("Already get in Get_GPSDATA "); 
// //   Serial.print(Serial2.available());
// //   Serial.print("\n");
// //   while (Serial2.available() > 0)
// //     if (gps.encode(Serial2.read()))
// //       displayInfo();
// //     else{
// //       Serial.print("Cannot get data");
// //     }
// // }
// // // ================================================================
// // void displayInfo()
// // {
// //   Serial.print(F("Location: "));
// //   if (gps.location.isValid())
// //   {
// //     Serial.print(gps.location.lat(), 6);
// //     Serial.print(F(","));
// //     Serial.print(gps.location.lng(), 6);
// //   }
// //   else
// //   {
// //     Serial.print(F("INVALID"));
// //   }

// //   Serial.print(F("  Altitude: "));
// //   if (gps.altitude.isValid())
// //   {
// //     Serial.print(gps.altitude.meters());
// //     Serial.print(F("m"));
// //   }
// //   else
// //   {
// //     Serial.print(F("INVALID"));
// //   }

// //   Serial.print(F("  Date/Time: "));
// //   if (gps.date.isValid())
// //   {
// //     Serial.print(gps.date.month());
// //     Serial.print(F("/"));
// //     Serial.print(gps.date.day());
// //     Serial.print(F("/"));
// //     Serial.print(gps.date.year());
// //   }
// //   else
// //   {
// //     Serial.print(F("INVALID"));
// //   }

// //   Serial.print(F(" "));
// //   if (gps.time.isValid())
// //   {
// //     if (gps.time.hour() < 10)
// //       Serial.print(F("0"));
// //     Serial.print(gps.time.hour());
// //     Serial.print(F(":"));
// //     if (gps.time.minute() < 10)
// //       Serial.print(F("0"));
// //     Serial.print(gps.time.minute());
// //     Serial.print(F(":"));
// //     if (gps.time.second() < 10)
// //       Serial.print(F("0"));
// //     Serial.print(gps.time.second());
// //     Serial.print(F("."));
// //     if (gps.time.centisecond() < 10)
// //       Serial.print(F("0"));
// //     Serial.print(gps.time.centisecond());
// //   }
// //   else
// //   {
// //     Serial.print(F("INVALID"));
// //   }
// //   Serial.println();
// // }

// #include <Arduino.h>
// #include <TinyGPSPlus.h>
// // ================================================================
// // Variable declaration
// // ================================================================
// #define RXD2 16
// #define TXD2 17
// TinyGPSPlus gps;
// // ================================================================
// // Function Declaration
// // ================================================================
// void Init_Serial(); // Init the serial monitor
// void Get_GPSData(); // Get the GPS data
// void displayInfo(); // Display info from the GPS
// // ================================================================
// // Setup function
// // ================================================================
// void setup()
// {
//   Init_Serial();
//   // Serial2.begin(baud-rate, protocol, RX pin, TX pin);
//   Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
// }
// // ================================================================
// // Loop function
// // ================================================================
// void loop()
// {
//   Get_GPSData();
// }
// // ================================================================
// // Function Definition
// // ================================================================
// void Init_Serial()
// {
//   Serial.begin(115200);
//   while (!Serial)
//     ;
// }
// // ================================================================
// void Get_GPSData()
// {
//   while (Serial2.available() > 0)
//     if (gps.encode(Serial2.read()))
//       displayInfo();
// }
// // ================================================================
// void displayInfo()
// {
//   Serial.print(F("Location: "));
//   if (gps.location.isValid())
//   {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Altitude: "));
//   if (gps.altitude.isValid())
//   {
//     Serial.print(gps.altitude.meters());
//     Serial.print(F("m"));
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10)
//       Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10)
//       Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10)
//       Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10)
//       Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }
//   Serial.println();
// }