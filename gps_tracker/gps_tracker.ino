#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define RX 20
#define TX 21

TinyGPSPlus gps;
HardwareSerial GPS(1); // Use UART1
void setup() {
  Serial.begin(115200); //START UART0 PC
  GPS.begin(115200, SERIAL_8N1, RX, TX); // Start UART1 GPS
  Serial.println("Hello from ESP32-C3!");
}

void loop() {
  // Do nothing
   while (GPS.available() > 0)
    if (gps.encode(GPS.read()))
      showdata();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

void showdata()
{
  Serial.println("No data to show for now");
}

