#define BLYNK_TEMPLATE_ID "TMPL37XDMcpgT"
#define BLYNK_TEMPLATE_NAME "VERTICAL FARMING SYSTEM"
#define BLYNK_AUTH_TOKEN "h3IGHG-2v7Ovl5Y3mSSBsIxg6IB51Nw1"

#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define WIFI_SSID "Honor 8X"
#define WIFI_PASSWORD "00000000"

#define TRIGGER_PIN 23
#define ECHO_PIN 22
#define MOISTURE_SENSOR_PIN 34
#define DHT_PIN 27
#define SERVO_PIN 18
#define TURBIDITY_SENSOR_PIN 32
#define PH_SENSOR_PIN 33
#define RELAY_PIN 19
#define FLOW_SENSOR_PIN 35

#define MOISTURE_MIN_THRESHOLD 40
#define MOISTURE_MAX_THRESHOLD 90
#define TURBIDITY_THRESHOLD 500

DHT dht(DHT_PIN, DHT11);
Servo myservo;

const char* ssid = "Honor 8X";
const char* password = "00000000";
char auth[] = "h3IGHG-2v7Ovl5Y3mSSBsIxg6IB51Nw1";

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, WIFI_SSID, WIFI_PASSWORD);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT);
  dht.begin();
  myservo.attach(SERVO_PIN);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
  } else {
    Serial.println("WiFi not connected");
  }
}

void loop() {
  Blynk.run();

  // Ultrasonic sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance_cm = duration * 0.034 / 2;
  float distance_percent = constrain((distance_cm / 400) * 100, 0, 100);

  Blynk.virtualWrite(V1, distance_percent);
  Serial.print("Distance: ");
  Serial.print(distance_percent);
  Serial.println(" %");

  // Moisture sensor
  int moistureValue = analogRead(MOISTURE_SENSOR_PIN);
  float moisture_percent = constrain(map(moistureValue, 0, 4095, 0, 100), 0, 100);

  Blynk.virtualWrite(V2, moisture_percent);
  Serial.print("Moisture Level: ");
  Serial.println(moisture_percent);

  // DHT11 sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float temperature_percent = constrain(map(temperature, -10, 50, 0, 100), 0, 100);

  Blynk.virtualWrite(V3, temperature_percent);
  Blynk.virtualWrite(V4, humidity);
  Serial.print("Temperature: ");
  Serial.print(temperature_percent);
  Serial.println(" %");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Turbidity sensor
  int turbidityValue = analogRead(TURBIDITY_SENSOR_PIN);
  float turbidity_percent = constrain(map(turbidityValue, 0, 4095, 0, 100), 0, 100);

  Blynk.virtualWrite(V5, turbidity_percent);
  Serial.print("Turbidity Level: ");
  Serial.println(turbidity_percent);

  // pH sensor
  int phRawValue = analogRead(PH_SENSOR_PIN);
  float phValue = map(phRawValue, 0, 1023, 0, 1400) / 100.0;
  phValue = constrain(phValue, 0, 14);

  Blynk.virtualWrite(V6, phValue);
  Serial.print("pH Level: ");
  Serial.println(phValue);

  // Flow sensor
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    int flowFrequency = pulseIn(FLOW_SENSOR_PIN, HIGH);
    float flowRate = flowFrequency / 7.5;

    Blynk.virtualWrite(V7, flowRate);
    Serial.print("Flow Rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
  }

  // Control servo motor
  if (moisture_percent < MOISTURE_MIN_THRESHOLD) {
    myservo.write(180); // Turn on servo motor
  } else if (moisture_percent > MOISTURE_MAX_THRESHOLD) {
    myservo.write(0); // Turn off servo motor
  }

  // Control water pump
  if (moisture_percent < MOISTURE_MIN_THRESHOLD || turbidity_percent >= TURBIDITY_THRESHOLD) {
    digitalWrite(RELAY_PIN, HIGH); // Turn on water pump
  } else {
    digitalWrite(RELAY_PIN, LOW); // Turn off water pump
  }

  delay(3000); // Wait before next measurement
}
