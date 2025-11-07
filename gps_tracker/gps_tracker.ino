#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

#define LED_PIN 3

#define RX 20
#define TX 21

//iquidCrystal_I2C lcd(0x27, 16, 2);
// ##################### PROTOTYPES ###################################
bool check_switch();  // TODO finish this on circuit first here we check the
                      // slide switch if it is open we proceed if not the program
                      // stalls there until the switch is on
void stop_w_err(
  const String &msg); /*this method will be used to halt any operation with a
                           message in case of error */
void init_lcd();
void init_sd();
void init_gps();
void create_csv_sd();    // since we init the gps we can get a timestamp and use
                         // that to name the csv with current date
void read_values_gps();  // Lat, long, alt, timestp, satellites numb
String format_data();    // will take the raw data and format it so that the write
                         // can send it to the sd
void write_values_gps_sd_lcd(String data);

TinyGPSPlus gps;
HardwareSerial GPS(1);  // Use UART1

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);  // START UART0 PC
  //GPS.begin(115200, SERIAL_8N1, RX, TX); // Start UART1 GPS
  Serial.println("Hello from ESP32-C3!!!");
  init_lcd();
}

void loop() {
  Serial.print("LOOP");
}
// TODO add lcd in case the lcd is connected
void stop_w_err(const String &msg) {
  // show message if not empty
  if (msg.length() > 0) {
    //lcd.clear();
    //lcd.print(msg);
    Serial.print(msg);
  }
  pinMode(LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

void init_lcd() {
  Wire.begin();
  int i2c = 0b0000000;
  for (int i = 0; i < 127; i++) {
    Serial.print("Search at [");
    Serial.print(i2c, HEX);
    Serial.print("]: ");
    Wire.beginTransmission(i2c);
    byte busStatus = Wire.endTransmission();
    if (busStatus == 0) {
      Serial.println("FOUND!");
      break;
    } else {
      Serial.println("not found");
      stop_w_err("");
    }
    i2c++;
  }
}
