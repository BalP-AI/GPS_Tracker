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
void write_values_gps_sd_lcd(String data);
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
}

void loop() {

  gps.encode(gpss.read());
  Serial.println(gps.location.isUpdated());
  if (gps.location.isUpdated()) {
    
    Serial.print("Latitude= ");
    Serial.println(gps.location.lat(), 6);
    Serial.print(" Longitude= ");
    Serial.println(gps.location.lng(), 6);
          display.setCursor(0, 0);              // Start at top-left corner

    display.clearDisplay();
    display.print("Lat = ");
    display.println(gps.location.lat(), 6);
    display.print("Lon = ");
    display.println(gps.location.lng(), 6);
    display.print("Sat = ");
    display.println(gps.satellites.value());
    display.display();
  }
  delay(5000);
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
    delay(500);
gps.encode(gpss.read());

Serial.print("\n Satelites = ");
Serial.println(gps.satellites.value());
display.clearDisplay();
display.setCursor(0, 0); 
display.print("Sat =") ;
display.print(gps.satellites.value());
display.display();
          display.display();    if (gpss.available()) {
      String line = gpss.readStringUntil('\n');
      line.trim();
      //TODO change this check with the TIny gps gps.satellite.value() and check that way
      if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
        if (line.indexOf(",1,") >= 0 || line.indexOf(",2,") >= 0) {
          display.println("GPS Fix V");
          display.display();
          gps_fixed = true;
        } else {
          Serial.println("No fix yet");
        }
      } else {
        Serial.println(line);
      }
    } else {
      // no data right now; don't call stop_w_err repeatedly
      // simply wait and loop again
    }
  }
}