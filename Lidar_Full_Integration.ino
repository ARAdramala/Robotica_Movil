//----- ----- ----- Includes ----- ----- -----//
#include <ESP32Servo.h>
#include <stdint.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include "Adafruit_VL53L0X.h"
#include "WifiSecrets.h"
#include "MQTTSecrets.h"
#include "pinout.h"

//----- ----- ----- Defines ----- ----- -----//

#define NUMBER_OF_MEASUREMENTS 10 
#define DELAY_BETWEEN_MEASUREMENTS 500 

//----- ----- ----- Function declaration ----- ----- -----//

void startWifi(void);  // WiFi start function
void startMQTT(void);  // MQTT start function
void startLidar(void); // Lidar start function
void startServo(void); // Servo start function
void wifiCheck(void);  // WiFi check connection
void callback(char* topic, byte* payload, unsigned int length); // MQTT receive message callback
uint16_t getLidarMeasurement (Adafruit_VL53L0X *VL53L0X); // Get distance function
void sendMeasurementMQTT(uint16_t measurement); // Send data through MQTT

//----- ----- ----- Gloabal Variables ----- ----- -----//

WiFiClient espClient;
PubSubClient client(MQTT_SERVER, MQTT_PORT, callback, espClient);
Adafruit_VL53L0X lidar = Adafruit_VL53L0X();
Servo servoLidar;

uint16_t wifiCheckPeriod = 30000; // 30s wait for checking wifi connection
uint32_t wifiPreviousMillis =  0; // Wifi last check time
uint8_t lidarReadingPeriod = 100; // 0.1s wait between lidar measurement 
uint32_t lidarPreviousMillis = 0; // Lidar last measurement time
uint16_t lidarMeasurement_mm = 0; // Lidar measured distance
bool enabledDataSend = true;       // Allows to enter the communication loop

//----- ----- ----- Program code ----- ----- -----//
void setup() {
  Serial.begin(115200); // Configure serial communication
  startWifi();
  startMQTT();
  startLidar();
  startServo();
}

void loop() {
  wifiCheck();
  client.loop();

  if(enabledDataSend){
    char msg[6];
    sprintf(msg, "%d", NUMBER_OF_MEASUREMENTS);
    client.publish("ConfigCom", msg);
    for(uint8_t i = 0; i <= 180; i += 180/(NUMBER_OF_MEASUREMENTS - 1)){
      servoLidar.write(i);
      lidarMeasurement_mm = getLidarMeasurement(&lidar);
      sendMeasurementMQTT(lidarMeasurement_mm);
      delay(DELAY_BETWEEN_MEASUREMENTS);
    }
    enabledDataSend = false;
  }
}

//----- ----- ----- Function implementation ----- ----- -----//

void startWifi(void){
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID_PORTATIL, PASSWORD_PORTATIL);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

void startMQTT(void){
  if(client.connect("ESP32", MQTT_USER, MQTT_PASSWORD)){
    client.subscribe("ErrorCom");
    //client.subscribe("DataCom");
    //client.subscribe("ConfigCom");
  }else{
    Serial.println("MQTT NOT Connected");
    Serial.println(client.state());
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("MQTT MESSAGE:");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  String content = "";
  for (int i = 0; i < length; i++)
  {
    content.concat((char)payload[i]);
  }
  Serial.print(content);
  Serial.println();
  if(content == "new"){
    enabledDataSend = true;
  }
}

void wifiCheck(void){
  uint32_t currentMillis = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiPreviousMillis >= wifiCheckPeriod)) {
    Serial.print(currentMillis);
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    wifiPreviousMillis = currentMillis;
  }
}

void startLidar(void){
  while(!lidar.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(100);
  }
  Serial.println("VL53L0X is connected");
}

uint16_t getLidarMeasurement (Adafruit_VL53L0X *VL53L0X){
  VL53L0X_RangingMeasurementData_t measure;

  VL53L0X -> rangingTest(&measure, false);
  if (measure.RangeStatus != 4) { 
    Serial.print("Distance (mm): "); 
    Serial.println(measure.RangeMilliMeter);
    Serial.println();
    return measure.RangeMilliMeter;
  } else {
    Serial.println("(Lidar) measurement out of range");
  }
  return UINT16_MAX;
}

void sendMeasurementMQTT(uint16_t measurement){
  char msg[6];
  
  sprintf(msg, "%d", (int)lidarMeasurement_mm);
  client.publish("DataCom", msg);
}

void startServo(void){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoLidar.setPeriodHertz(50); // Standard 50 hz servo
  servoLidar.attach(SERVO_LIDAR_PIN, 100, 4000);
}
