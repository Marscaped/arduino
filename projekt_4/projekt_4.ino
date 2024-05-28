//#include <TimerOne.h>
#include <Arduino.h>
#include <DHT.h>
#include <Encoder.h>
#include <SI114X.h>
#include <SPI.h>
#include <Si115X.h>
#include <U8g2lib.h>
//#include <UnoWiFiDevEd.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <RCSwitch.h>
#include "Grove_Temperature_And_Humidity_Sensor.h"

Si115X si1151;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

char ssid[] = "Janniks Hotspot";  // your network SSID (name) // Janniks Hotspot
char pass[] =
    "12345678";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;  // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

//#define DHTTYPE DHT20  // DHT 22  (AM2302)
#define DHTTYPE DHT20  // DHT 22  (AM2302)
//#define DHTPIN A1
#define MoisturePin A1
#define MoistureValue 0
#define EncoderPin1 3
#define EncoderPin2 4
#define OneSecond 1000
#define DataUpdateInterval 20000
#define NoWaterTimeOut 3
#define DHTPIN 7
#define PUMPPIN 8
//DHT dht(DHTPIN, DHTTYPE);
DHT dht(DHTTYPE);

unsigned int uiWaterVolume = 0;
// unsigned char WaterflowFlag = 0;
unsigned int WaterflowRate = 0;
unsigned char EncoderFlag = 0;
unsigned long StartTime = 0;
const int BUTTON_PIN = 5;  // Arduino pin connected to button's pin
const int RELAY_PIN = 6;   // Arduino pin connected to relay's pin // ?????????? FAN PIN ????????????
unsigned long counter = 0;

// for water flow sensor
volatile int flow_frequency = 0;  // Measures flow sensor pulsesunsigned
int l_hour;                       // Calculated litres/hour
unsigned char flowsensor = 2;     // Sensor Input
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;
unsigned long cloopTime;

// SOLL DATEN FÜR AUTOMATISCHER ABFOLGE
float temperature_soll = 20.0;
float humidity_soll = 60.0;        
int moisture_soll = 1;  
int sunlight_soll = 200;

// IST DATEN FÜR AUTOMATISCHER ABFOLGE 
float temperature = 0.0;  // Читаем температуру // Temperatur Lesen
float humidity = 0.0;        // Читаем влажность // Luftfeuchtigkeit Lesen
int moisture = 0;  // Читаем влажность почвы // Bodenfeuchtigkeit Lesen
int sunlight = 0;  // Читаем сигнал от солнечного датчика // Helligkeit Lesen

void flow()  // Interrupt function
{
  flow_frequency++;
}

float SensorHumidity = 0;
float SensorTemperature = 0;
float SensorMoisture = 0;
float UVIndex = 0;
char buffer[30];

String output1 = "off";
String output2 = "off";
String header;

// für Temperature und Humidity
#if defined(ARDUINO_ARCH_AVR)
#define debug Serial

#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAM)
#define debug SerialUSB
#else
#define debug Serial
#endif

Encoder myEnc(3, 4);

RCSwitch rcSwitch = RCSwitch();

bool autoAblauf = true;

void setup() {
  // Initialisiere den Serial Monitor
  Serial.begin(9600);
  Wire.begin();
  u8g2.begin();  // initialize u8g2 oled display
  dht.begin();

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  // Print firmware version on the module
  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println(F("Please upgrade the firmware"));
  }

  connectToWiFi();
  
  printWifiStatus();

  // encoder test

  // initialize the button
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(RELAY_PIN, OUTPUT);         // set arduino pin to output mode
  // attachInterrupt(0, ButtonClick, FALLING);

  // initialize the encoder
  pinMode(EncoderPin1, INPUT);
  pinMode(EncoderPin2, INPUT);
  //  attachInterrupt(1, EncoderRotate, RISING);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, OUTPUT);
  // Setze den Pin für das Relay der Pumpe als Ausgang
  pinMode(PUMPPIN, OUTPUT);

  // Setze den Pin für das Relay des Lüfters als Ausgang
  // pinMode(5, OUTPUT);

  pinMode(8, INPUT);
  digitalWrite(8, HIGH);

  attachInterrupt(
      8, []() { flow_frequency++; }, RISING);

  // Water Flow Sensor
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);  // Optional Internal Pull-Up
  Serial.begin(9600);
  attachInterrupt(0, flow, RISING);  // Setup Interrupt
  sei();                             // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;

  // für SunlightSensor
  if (!si1151.Begin()) {
    Serial.println("Si1151 is not ready!");
    while (1) {
      delay(1000);
      Serial.print(".");
    };
  } else {
    Serial.println("SunlightSensor gestartet!");
  }

  // Funksteckdose
  rcSwitch.enableTransmit(4);

  WiFi.setHostname("GruppeA");
}

long oldPosition = -999;
int shownDisplay = 1;

void loop() {
  

  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();
  }

  Serial.println(status);
  Serial.println(WiFi.localIP());

  connectToWiFi();
  

  Serial.println("-= Durchlauf begonnen! =-");
    WiFiClient client = server.available();   // listen for incoming clients

    httpRequestHandler(client);

  temperature = dht.readTemperature();  // Читаем температуру // Temperatur Lesen
  humidity = dht.readHumidity();        // Читаем влажность // Luftfeuchtigkeit Lesen
  moisture = analogRead(MoisturePin);  // Читаем влажность почвы // Bodenfeuchtigkeit Lesen
  sunlight = si1151.ReadVisible();  // Читаем сигнал от солнечного датчика // Helligkeit Lesen

  int buttonState = digitalRead(BUTTON_PIN);

  if(shownDisplay == 1) {
    showSettingsDisplay();
  } else {
    showDataDisplay(temperature, humidity, moisture, sunlight);
  }

  if (buttonState == HIGH) {
    if(shownDisplay == 1) {
      shownDisplay = 0;
    } else {
      shownDisplay = 1;
    }
  }
  
  delay(1000);  // Задержка в миллисекундах между обновлениями значений

  // Temp & Humidity Sensor function
  float temp_hum_val[2] = {0};

  // Water Flow Sensor
  currentTime = millis();  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime >= 1000)) {
    cloopTime = currentTime;  // Updates cloopTime
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
    l_hour = (flow_frequency /
              7.5);  // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
    flow_frequency = 0;  // Reset Counter

    if (l_hour == 0) {
      digitalWrite(PUMPPIN, LOW);
      Serial.println("Pumpe aus, da keine Wasser verfügbar ist!");
    }
  }

  //  Button funсtion with Relay
   // read new state
  /*
  
  */
  // Encoder
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  
  //automatischer Ablauf
  if (autoAblauf) {
    automaticControl();
  }
}

void httpRequestHandler(WiFiClient client) {
  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character
          
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print(sunlight);
            client.print(";");
            client.print(temperature);
            client.print(";");
            client.print(moisture);
            client.print(";");
            client.print(humidity);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        
        //Serial.println(currentLine);
        // Check to see if the client request was "GET /H" or "GET /L":
        
        // Switch automatical Settings
        if (currentLine.endsWith("POST /setAutoOn")) {
          autoAblauf = true;
          Serial.println("Automatischer Ablauf an!");
        }
        if (currentLine.endsWith("POST /setAutoOff")) {
          autoAblauf = false;
          Serial.println("Automatischer Ablauf aus!");
        }

        // Switch Light
        if (currentLine.endsWith("POST /setLightOn")) {
          rcSwitch.switchOn("10101", "10000");
          Serial.println("Lampe an!");
        }
        if (currentLine.endsWith("POST /setLightOff")) {
          rcSwitch.switchOff("10101", "10000");
          Serial.println("Lampe aus!");
        }

        // Switch Heatmat
        if (currentLine.endsWith("POST /setHeatmatOn")) {
          rcSwitch.switchOn("10101", "01000");
          Serial.println("Heizmatte an!");
        }
        if (currentLine.endsWith("POST /setHeatmatOff")) {
          rcSwitch.switchOff("10101", "01000");
          Serial.println("Heizmatte aus!");
        }

        // Switch Fan
        if (currentLine.endsWith("POST /setFanOn")) {
          digitalWrite(RELAY_PIN, HIGH);
          Serial.println("Lüfter an!");
        }
        if (currentLine.endsWith("POST /setFanOff")) {
          digitalWrite(RELAY_PIN, LOW);
          Serial.println("Lüfter aus!");
        }

        // Switch Pump
        if (currentLine.endsWith("POST /setPumpOn")) {
          digitalWrite(PUMPPIN, HIGH);
          Serial.println("Pumpe an!");
          delay(3000);
          digitalWrite(PUMPPIN, LOW);
          Serial.println("Pumpe aus!");
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void automaticControl() {
  //auto data = SensorData();

  // HOWTO:
  // https://web.archive.org/web/20151007013852/https://code.google.com/p/rc-switch/wiki/HowTo_OperateLowCostOutlets


  // Outlet A -- Lamp
  Serial.println(sunlight);
  if (sunlight < sunlight_soll) {
    rcSwitch.switchOn("10101", "10000");
    Serial.println("Lampe an!");
  } else {
    rcSwitch.switchOff("10101", "10000");
    Serial.println("Lampe aus!");
  }
  delay(3000);

  // Outlet B -- Heatmat
  Serial.println(temperature);
  if (temperature < temperature_soll) {
    rcSwitch.switchOn("10101", "01000");
    Serial.println("Heizmatte an!");
  } else {
    rcSwitch.switchOff("10101", "01000");
    Serial.println("Heizmatte aus!");
  }
  delay(3000);

  Serial.println(humidity);
  if (humidity > humidity_soll) {
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Lüfter an!");
  } else {
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Lüfter aus!");
  }
  delay(3000);

  if (moisture < moisture_soll) {
    digitalWrite(PUMPPIN, HIGH);
    Serial.println("Pumpe an!");
    delay(3000);
    digitalWrite(PUMPPIN, LOW);
    Serial.println("Pumpe aus!");
  } else {
    digitalWrite(PUMPPIN, LOW);
    Serial.println("Pumpe aus!");
  }

  Serial.println("-= Durchlauf abgeschlossen! =-");
  delay(1000);
  //auto_control_counter = 0;
}

int serverStarted = 0;

void connectToWiFi() {
  int count = 1;

  while (status != WL_CONNECTED && count != 5) {
    Serial.print("Verbindungsversuch ");
    Serial.print(count);
    Serial.print(": ");
    Serial.println(ssid);  // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP
    // network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(3000);
    count++;

    serverStarted = 0;
  }

  if (status != WL_CONNECTED) {
    Serial.println("Automatischer Ablauf wird eingeschlatet da keine WiFi-Verbindung verfügbar ist.");
    autoAblauf = true;
  }

  if(serverStarted == 0) {
    Serial.println("Starting Webserver...");
    server.begin();  // start the web server on port 80
    serverStarted = 1;
  }
}

void showDataDisplay(float temp, float humid, int moist, int light) {
  // Daten auf Display vorbereiten
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_11_mr);
  u8g2.setCursor(0, 10);
  u8g2.print("Ausgelesende Daten:");

  u8g2.setCursor(0, 20);
  u8g2.print("Temperature: ");
  u8g2.print(temp);
  u8g2.print(" C");
  
  u8g2.setCursor(0, 30);
  u8g2.print("Humidity: ");
  u8g2.print(humid);
  u8g2.print(" %");
  
  u8g2.setCursor(0, 40);
  u8g2.print("Moisture: ");
  u8g2.print(moist);
 
  u8g2.setCursor(0, 50);
   u8g2.print("Sunlight: ");
  u8g2.print(light);
  
  u8g2.setCursor(0, 60);
  u8g2.print(l_hour);
  u8g2.print(" L/hour: ");
  // Daten auf Display ausgeben
  u8g2.sendBuffer();
}

void showSettingsDisplay() {
  // Daten auf Display vorbereiten
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_11_mr);
  u8g2.setCursor(0, 10);
  u8g2.print("WiFi Einstellungen:");

  u8g2.setCursor(0, 20);
  u8g2.print("SSID: ");
  u8g2.print(WiFi.SSID());
  
  u8g2.setCursor(0, 30);
  u8g2.print("IP: ");
  u8g2.print(WiFi.localIP());

  // Daten auf Display ausgeben
  u8g2.sendBuffer();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
