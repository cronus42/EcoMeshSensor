#define OLED_I2C_ADDRESS 0x3C /// default for HELTEC OLED
#define BME_I2C_ADDRESS 0x76 // default for the BME/BMP280
#define i2c_sda_pin 4 // sharing i2c with the HALTEC_ESP32_OLED
#define i2c_scl_pin 15 // same
#define serial1_rx_pin 27 //12 will interfere with flash
#define serial1_tx_pin 14
#define wire_Frequency 300000   /// could be in the range of 100000 to 400000
#define rst_pin 16  /// <== delete this line if your OLED does not have/support OLED reset pin
#define HARDWARE_LED 25 /// HELTEC LED/// <== delete this line if your OLED does not have/support OLED reset pin
#define TIMEZONE_OFFSET -8
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 30 /* Time ESP32 will go to sleep (in seconds) */

// GPS ubx packets
const unsigned char UBX_CFG_RXM_PWR_SAVE[] = { 0x06, 0x11, 0x02, 0x00, 0x00, 0x01 };
const unsigned char UBX_GNSS_DISABLE[] = { 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x00 };
const unsigned char UBX_CFG_PM2_PSMCT____[] = { 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x02, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00 };
const unsigned char UBX_CFG_RATE_1_SEC[] = { 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00 };
const unsigned char UBX_CFG_RATE_3_SEC[] = { 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00 };
const unsigned char UBX_CFG_RATE_5_SEC[] = { 0x06, 0x08, 0x06, 0x00, 0x88, 0x13, 0x01, 0x00, 0x01, 0x00 };
const unsigned char UBX_CFG_RATE_10_SEC[] = { 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00 };
const unsigned char UBX_CFG_PRT_DISABLE[] = { 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const unsigned char UBX_CFG_PRT_9600[] = { 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const unsigned char UBX_CFG_MSG_GET[] = { 0x06, 0x01, 0x02, 0x00, 0x01, 0x07};
const unsigned char UBX_CFG_MSG_NAV_PVT[] = { 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x02};
const unsigned char UBX_CFG_MSG_ESF_STS[] = { 0x06, 0x01, 0x03, 0x00, 0x10, 0x10, 0xB4};
const unsigned char UBX_CFG_MSG_NAV_PVT_DIS[] = { 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x00};
const unsigned char UBX_CFG_MSG_ESF_STS_DIS[] = { 0x06, 0x01, 0x03, 0x00, 0x10, 0x10, 0x00};
const unsigned char UBX_CFG_NAV5_SET[] = { 0x06, 0x24, 0x24, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/// #include <Wire.h> /// not necessary as it is included by command #include "SSD1306AsciiWire.h" bellow
#include <config.h>
#include <Arduino.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "NTPClient.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>  // For OTA
#include <WiFiUdp.h>  // For OTA
#include <ArduinoOTA.h>  // For OTA
#include <Adafruit_BME280.h> // Pressure and temp sensor
#include <AWS_IOT.h>
#include <time.h>
#include <TinyGPS++.h>
#include <Adafruit_VEML6070.h>


// ESP32 + OLED PRINTING LIKE Serial.print but in OLED ///
SSD1306AsciiWire oled;

int chPos = 0;
// Define a custom console for OLED debug output
class CustomConsole
  : public Print // This automatically adds functions like .println()
{
  public:
    CustomConsole() {}
    virtual size_t write(uint8_t ch) {  // Grabs character from print
      if (chPos >= 20 || char(ch) == '\n') { /// auto line splitting if line is bigger than 20 characters, font depended ...
        chPos = 0;
        oled.print('\n');
      }
      if ( char(ch) != '\n') {
        oled.print(char(ch));
        Serial.print(char(ch));  // Repeat all character output to Serial monitor
        chPos++ ;
      }
      return 1; // Processed 1 character
    }
  private:
    // define variables used in myConsole
};

// initiate myConsole
CustomConsole myConsole;
char payload[512];

// Hardware serial RX/TX
HardwareSerial hws1(1);
// The TinyGPS++ object
TinyGPSPlus gps;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "us.pool.ntp.org", TIMEZONE_OFFSET * 3600);
Adafruit_BME280 bme;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
AWS_IOT AWS_CLIENT;

void AWSClientConnect() {
  if (AWS_CLIENT.connect(AWS_HOST_ADDRESS, AWS_CLIENT_ID) == 0)
  {
    myConsole.println("Connected to AWS");
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
  else
  {
    myConsole.println("AWS connection failed, Check the HOST Address");
    while (1);
  }
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void StartWifi() {
  myConsole.print("Connecting to ");
  myConsole.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass); /// provided you have put the correct one above ///
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { /// Stall until connected
    vTaskDelay(10000/portTICK_PERIOD_MS);
    myConsole.print(".");
    i++;
    if (i > 7) {
      ESP.restart();
    }
  }
  myConsole.println();
}

float UvaConvert(float uva_data) {
  //converting the digital data to uva intensity
  int i;
  float intensity_step = 25; //intesity step count in mW/m^2
  word uva_intensity_mapping_table[4] = {561, 1121, 1494, 2055}; //range of digital uva data from the sensor
  float intensity [5] = {1.5 * intensity_step, 4 * intensity_step, 7 * intensity_step, 9.5 * intensity_step, 11.5 * intensity_step};

  for (i = 0; i < 4; i++)
  {
    if (uva_data <= uva_intensity_mapping_table[i])
    {
      break;
    }
  }

  return intensity[i];
}

void TimeDateCheck() {
  String time = "Unk";
  if (gps.time.isValid()) {
    time = gps.time.value();
  } else {
    timeClient.update();
    time = timeClient.getFormattedTime();
  }
  myConsole.println(time);
}

void RSSICheck() {
  myConsole.print("IP: ");
  myConsole.println(WiFi.localIP());
  myConsole.print("Wifi Strength: ");
  myConsole.println(WiFi.RSSI());
}

void LED() {
  // digitalWrite(HARDWARE_LED, LOW);  // Turn ON
  // delay(300);
  digitalWrite(HARDWARE_LED, HIGH);  // Turn ON
}

void CheckSensors() {
  myConsole.print(F("Temperature: "));
  myConsole.print(bme.readTemperature());
  myConsole.println("C");

  myConsole.print(F("Atmos: "));
  myConsole.print(bme.readPressure() / 100.0F);
  myConsole.println("hPa");

  myConsole.print("Humidity: ");
  myConsole.print(bme.readHumidity());
  myConsole.println("%");

  myConsole.print("UV: ");
  myConsole.print(uv.readUV());
  myConsole.println("mW/m^2");
}

void PublishSensors() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = bme.readHumidity();
  // Read temperature as Celsius (the default)
  float t = bme.readTemperature();
  // Read Pressure
  float p = bme.readPressure() / 100.0F;
  // Read UV convert to mW/m^2
  int u = UvaConvert(uv.readUV());
  // read gps
  float lat = 0;
  float lng = 0;
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }

  // Get time and do a stupid dance with "strings"
  String StringTime = "Unknown";
  if (!gps.time.isValid()) {
    StringTime = timeClient.getFormattedTime();
  } else {
    StringTime = gps.time.value();
  }
    int strLength = StringTime.length() + 1;
    char time[strLength];
    StringTime.toCharArray(time, strLength);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(p) || isnan(u)) {
    myConsole.println("Failed to read from BME sensor!");
  }
  else
  {
    sprintf(payload, "{\"state\": { \"time\": \"%s\", \"pressure\": %f, \"humidity\": %f, \"temperature\": %f, \"ultraviolet\": %i, \"latitude\": %f, \"longitude\": %f }}", time, p, h, t, u, lat, lng); // Create the payload for publishing

    if (AWS_CLIENT.publish(AWS_TOPIC_NAME, payload) == 0) // Publish the message(Temp and humidity)
    {
      myConsole.println("Published Message");
    }
    else
    {
      myConsole.println("Publish failed");
      //myConsole.println(payload);
    }
    // publish the temp and humidity every 5 seconds.
    // vTaskDelay(30000 / portTICK_RATE_MS);
  }
}

void sendUBX(const byte *MSG, uint8_t len) {
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    bool    sent = false;
    hws1.flush();
    hws1.write(0xFF);
    delay(500);

    hws1.write(0xB5); // Header bytes
    hws1.write(0x62);

    for(uint8_t ubxi = 0; ubxi < len; ubxi++) {
        byte data = pgm_read_byte_near(MSG + ubxi);
        hws1.write(data);
        checksumA += data;
        checksumB += checksumA;
    }

    hws1.write(checksumA);
    hws1.write(checksumB);
    delay(500);
}

/* Set up UBX communications with the UBLOX because tinygps doesn't support it */

bool setupUBX() {
    bool initialized = true;
    //DISABLE OUTPUT PORTS
    sendUBX(UBX_CFG_PRT_DISABLE, sizeof(UBX_CFG_PRT_DISABLE));
    sendUBX(UBX_CFG_MSG_NAV_PVT_DIS, sizeof(UBX_CFG_MSG_NAV_PVT_DIS));
    sendUBX(UBX_CFG_MSG_ESF_STS_DIS, sizeof(UBX_CFG_MSG_ESF_STS_DIS));
    //ERASE BUFFER
    uint32_t    timeCount = millis();
    while(hws1.available() && ((millis() - timeCount) < 100)){
        hws1.read();
    }
    //ENABLE UBX MESSAGES
    sendUBX(UBX_CFG_RATE_1_SEC, sizeof(UBX_CFG_RATE_1_SEC));
    sendUBX(UBX_CFG_NAV5_SET, sizeof(UBX_CFG_NAV5_SET));//STATIC HOLD
    sendUBX(UBX_CFG_MSG_ESF_STS, sizeof(UBX_CFG_MSG_ESF_STS)); //enable esf
    sendUBX(UBX_CFG_MSG_NAV_PVT, sizeof(UBX_CFG_MSG_NAV_PVT));
    sendUBX(UBX_CFG_PRT_9600, sizeof(UBX_CFG_PRT_9600));

    return initialized;//initialized;
}

void CheckGPS() {
  if (gps.location.isValid()) {
    myConsole.print("Lat: ");
    myConsole.println(gps.location.lat(), 6);
    myConsole.print(" Long: ");
    myConsole.println(gps.location.lng(), 6);
  } else {
    myConsole.print("GPS Lost: ");
    myConsole.println(gps.satellites.value());
  }
}

void FeedGPS() {
  while (hws1.available() > 0) {
    gps.encode(hws1.read());
  }
}

void setup() {
  Serial.begin(115200);
  hws1.begin(9600, SERIAL_8N1, serial1_rx_pin, serial1_tx_pin);

  /// IT IS VERY IMPORTANT TO GIVE YOUR VALUES IN THE Wire.begin(...) bellow Wire.begin(sda_pin, scl_pin, wire_Frequency); form 100000 to 400000
  Wire.begin(i2c_sda_pin, i2c_scl_pin, wire_Frequency); /// wire_Frequency: form 100000 to 400000

#ifdef rst_pin
  pinMode(rst_pin, OUTPUT); /// THIS IS THE RESET PIN OF HELTEC OLED ... ///
  digitalWrite(rst_pin, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(rst_pin, HIGH); // while OLED is running, must set GPIO16 in high
#endif

  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS); /// in this demo sketch the selected resolution is 128x64
  ///oled.setFont(System5x7); /// you can change this font but always choose monospaced font for "defined" characters length per line...
  oled.setFont(Adafruit5x7);
  oled.setScrollMode(SCROLL_MODE_OFF);

  pinMode(HARDWARE_LED, OUTPUT);  // initialize onboard LED as output
  digitalWrite(HARDWARE_LED, LOW); // dim the LED
  myConsole.println("Setting up sensors.");
  if (!bme.begin(BME_I2C_ADDRESS)) {
    myConsole.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  uv.begin(VEML6070_1_T);

  if (!setupUBX()) {
    myConsole.println("Could not find GPS sensor!");
    while(1);
  }

  StartWifi();

  /*
  set ESP32 to wake up every TIME_TO_SLEEP seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  // Start NTP Client
  myConsole.println("Starting NTP");
  timeClient.begin();

  ArduinoOTA.setHostname("HALTEC_ESP32_OLED");  /// For OTA recognition put yours here ///
  ArduinoOTA.begin();  // For OTA

  AWSClientConnect();

} // end of setup

void loop() {
  while (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();  // For OTA
    oled.clear();
    LED();
    TimeDateCheck();
    // RSSICheck();
    CheckSensors();
    FeedGPS();
    CheckGPS();
    PublishSensors();
    vTaskDelay(5000/portTICK_PERIOD_MS);
    esp_deep_sleep_start();
  }
  StartWifi();
  AWSClientConnect();
} // end of loop
