#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
//#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

//Generic warning LED
#define LED_PIN 0

// I2C PINS for communicating with OLED
#define I2C_SCL 1
#define I2C_SDA 2

// OLED display size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

//GPS PINS

#define TX 21
#define RX 20

HardwareSerial gpss(1);

// Create display object (I2C address 0x3C is default)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// ##################### PROTOTYPES ###################################
bool check_switch();                // TODO finish this on circuit first here we check the
                                    // slide switch if it is open we proceed if not the program
                                    // stalls there until the switch is on
void stop_w_err(const String &msg); /*this method will be used to halt any operation with a message in case of error */
void init_lcd();
void init_sd();
void init_gps();
void create_csv_sd();    // since we init the gps we can get a timestamp and use
                         // that to name the csv with current date
void read_values_gps();  // Lat, long, alt, timestp, satellites numb
String format_data();    // will take the raw data and format it so that the write
                         // can send it to the sd
void write_values_gps_sd_lcd(float lat, float lon, float alt, int sats, int hdop);
//create a gps object
TinyGPSPlus gps;
// Serial for GPS
//SoftwareSerial ss(RX, TX);

void setup() {


  Serial.begin(9600);  // START UART0 PC
  delay(500);
  gpss.begin(9600, SERIAL_8N1, RX, TX);  // Initialize GPS on gpss
  pinMode(LED_PIN, OUTPUT);
  // GPS.begin(115200, SERIAL_8N1, RX, TX); // Start UART1 GPS
  delay(1000);
  Serial.println("Hello from ESP32-C3!!!");
  Wire.begin(I2C_SDA, I2C_SCL);
  init_lcd();
  init_sd();
  init_gps();
  create_csv_sd();
  delay(1000);
  gps.encode(gpss.read());
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Lat = ");
  display.println(gps.location.lat(), 6);
  display.print("Lon = ");
  display.println(gps.location.lng(), 6);
  display.print("Sat = ");
  display.println(gps.satellites.value());
  display.display();
}

void loop() {

  if (gps.location.isUpdated() || gps.location.isValid()) {
    Serial.print("Latitude= ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude= ");
    Serial.println(gps.location.lng(), 6);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Lat = ");
    display.println(gps.location.lat(), 6);
    display.print("Lon = ");
    display.println(gps.location.lng(), 6);
    display.print("SatL = ");
    display.print(gps.satellites.value());
    display.print(" UPT=");
    display.print(gps.location.isUpdated());
    display.print(" VAL=");
    display.print(gps.location.isValid());
    display.display();
    write_values_gps_sd_lcd(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.satellites.value(), gps.hdop.value() / 100.0);
  }
  delay(2000);
}

// TODO add lcd  cases in case the lcd is connected
void stop_w_err(const String &msg) {
  // show message if not empty
  if (msg.length() > 0) {
    display.clearDisplay();               // Clear buffer
    display.setTextSize(2);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 0);              // Start at top-left corner
    display.println(msg);
    display.display();

  } else {
    pinMode(LED_PIN, OUTPUT);
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
}

void init_lcd() {

  int i2c = 0b0000000;
  // Detect if the display is connected first
  for (int i = 0; i < 127; i++) {
    Serial.print("Search at [");
    Serial.print(i2c, HEX);
    Serial.print("]: ");
    Wire.beginTransmission(i2c);
    delay(10);
    byte busStatus = Wire.endTransmission();
    if (busStatus == 0) {
      Serial.println("OLED FOUND!");
      // Initialize display
      if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
          ;  // Don't proceed, loop forever
      }

      display.clearDisplay();               // Clear buffer
      display.setTextSize(1);               // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);  // Draw white text
      display.setCursor(0, 0);              // Start at top-left corner
      display.println("OLED V");            // Next line
      display.display();
      return;
    } else {
      Serial.println("OLED NOT FOUND");
      if (i == 126) {
        stop_w_err("");
      }
    }
    i2c++;
  }
}

void init_sd() {
  if (!SD.begin()) {
    stop_w_err("No SD Card");
    return;
  } else {
    display.println("SD Card V");
    display.display();
  }
}

void init_gps() {
  display.println("GPS fix...");
  display.display();

  bool gps_fixed = false;

  while (!gps_fixed) {
    while (gpss.available() > 0) {
      gps.encode(gpss.read());

      if (gps.location.isUpdated() && gps.location.isValid()) {
        display.println("GPS Fix V");
        display.display();
        gps_fixed = true;

        Serial.print("Lat: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Lng: ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("HDOP: ");
        Serial.println(gps.hdop.value());
      } else {
        Serial.println("No fix yet");
      }
    }
    delay(1000);
  }
}

void create_csv_sd() {
  String filename = "Test1";
  // Check if file already exists
  if (SD.exists(filename)) {
    Serial.println("CSV file already exists. Nothing new created.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("CSV V");
    return;
  }

  // Create new file and write header
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.println("Time,Latitude,Longitude,Altitude,Satellites,HDOP");  // Example header
    file.close();
    Serial.println("CSV file created with header.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("CSV V");
  } else {
    Serial.println("Error creating CSV file!");
  }
}

void write_values_gps_sd_lcd(float lat, float lon, float alt, int sats, int hdop) {
  String filename = "Test1";
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.print(millis());
    file.print(",");
    file.print(lat, 6);
    file.print(",");
    file.print(lon, 6);
    file.print(",");
    file.print(alt);
    file.print(",");
    file.print(sats);
    file.print(",");
    file.println(hdop);
    file.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}
