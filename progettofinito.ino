#include <dht_nonblocking.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ArduinoHttpClient.h>
#include <DHT.h>

// Declarations for the buzzer
#define BUZZER_PIN 8 // Replace with the actual pin number of the buzzer
#define SIGNAL_PAUSE 1000 // Pause between signals in milliseconds
#define SIGNAL_DURATION 2000 // Signal duration in milliseconds
#define SIGNAL_REPEAT_COUNT 3 // Number of times the signal is repeated

// Constants for Smoke Sensor
#define MQ2pin (0) //analogic pin
#define WARM_UP_TIME 2000 // Warm-up time in milliseconds
#define INTERVAL 200      // Reading interval in milliseconds
#define SEND_INTERVAL 10000 // Data send interval in milliseconds


// Declarations for Humidity and Temperature Sensor
#define DHT_SENSOR_TYPE DHT_TYPE_11
#define DHT_SENSOR_PIN 2

// Costant for Identifier Device
#define ID 2

// Global variables...
int signalCount = 0; // Counter for the number of received signals
unsigned long buzzerStartTime = 0; // Time when the buzzer started
bool signalReceived = false; // Replace this with your actual signal condition


// Function declaration for listening to the bridge
void listenForBridgeSignal();
void startBuzzer();
void stopBuzzer();

// Humidity and Temperature Sensor
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// Device ID
String deviceId = String(ID);

// Configure RX and TX pins for GPS module
SoftwareSerial gpsSerial(6, 7); // RX, TX for the GPS module
TinyGPS gps;

// Thresholds for triggering alerts based on sensor readings
const int smokeThreshold = 400, temperatureThreshold = 50, humidityThreshold = 90;

// Simulated sensor readings (to be replaced with actual sensor logic)
float smokeValue, temperatureValue, humidityValue;

// Function for efficient delay
static void smartDelay(unsigned long ms);

// Variables to keep track of time
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  float latitude, longitude;
  unsigned long age, date, time, chars = 0;
  int smokeCall = 0, tempCall = 0, humidityCall = 0;

  // Listen the bridge waiting for a signal
  listenForBridgeSignal();

  // Get sensor readings
  gps.f_get_position(&latitude, &longitude, &age); // Get GPS values

  if(latitude == 1000 && longitude == 1000){
    latitude = 0;
    longitude = 0;
  }

  measureEnvironment(&temperatureValue, &humidityValue); // Get temperature/humidity values from DHT11
  smokeValue = analogRead(MQ2pin); // Read analog input from pin 0 (smoke)

  // Check humidity
  if (humidityValue > humidityThreshold) {
    humidityCall = 1;
  }

  // Check smoke
  if (smokeValue > smokeThreshold) {
    // startBuzzer();
    // smartDelay(5000);
    // stopBuzzer();
    smokeCall = 1;
  }

  // Check temperature
  if (temperatureValue > temperatureThreshold) {
    tempCall = 1;
  }

  // Check if it's time to send data
  if (millis() - lastSendTime >= SEND_INTERVAL || smokeCall || humidityCall || tempCall) {
    // Send data to the server
    sendJsonData(latitude, longitude, smokeValue, humidityValue, temperatureValue, smokeCall, humidityCall, tempCall);
    lastSendTime = millis(); // Update the last send time
  }

  smartDelay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void listenForBridgeSignal() {
  // Check if there's data available on the serial port
  while (Serial.available() > 0) {
    // Read the incoming character
    char incomingChar = Serial.read();

    // Check for the start of the signal "$"
    if (incomingChar == '$') {
      // Signal received, trigger the buzzer
      if (!signalReceived) {
        startBuzzer();
        signalReceived = true;
        buzzerStartTime = millis();
      }
    }
  }

  // Check if it's time to stop the buzzer
  if (signalReceived && (millis() - buzzerStartTime >= SIGNAL_DURATION)) {
    stopBuzzer();
    signalReceived = false; // Reset the flag for the next signal
  }
}

void startBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void stopBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);
}

void sendJsonData(float latitude, float longitude, int smoke, int humidity, int temperature, int callSmoke, int callHumidity, int callTemperature) {
  // Build JSON packet
  String jsonString = "{\"id\":" + deviceId + ",\"latitude\":" + String(latitude) + ",\"longitude\":" + String(longitude) +
                      ",\"smoke\":" + String(smoke) + ",\"humidity\":" + String(humidity) + ",\"temperature\":" + String(temperature) +
                      ",\"s\":" + String(callSmoke) + ",\"u\":" + String(callHumidity) + ",\"t\":" + String(callTemperature) + "}";
  Serial.write('$');
  // Send JSON packet
  Serial.println(jsonString);
  Serial.write('!');
}

static bool measureEnvironment(float *temperature, float *humidity) {
  static unsigned long measurementTimestamp = millis();

  // Measure once every four seconds
  if (millis() - measurementTimestamp > 3000ul) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurementTimestamp = millis();
      return true;
    }
  }

  return false;
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}
