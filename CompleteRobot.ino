//----- Includes -----//
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <stdint.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include "Adafruit_VL53L0X.h"
#include "WifiSecrets.h"
#include "MQTTSecrets.h"

//----- Pinout -----//
#define RIGHT_MOTOR_FORWARD_PIN 26
#define RIGHT_MOTOR_BACKWARD_PIN 25
#define LEFT_MOTOR_FORWARD_PIN 17
#define LEFT_MOTOR_BACKWARD_PIN 16

#define SERVO_LIDAR_PIN 27

#define IR_SENSOR_RIGHT_PIN 2

//----- APP Data Values -----//
#define MODE_REMOTE_CONTROL 0
#define MODE_LINE_FOLLOWER 1
#define BUTTON_UP 101
#define BUTTON_LEFT 102
#define BUTTON_STOP 103
#define BUTTON_RIGHT 104
#define BUTTON_DOWN 105
#define BUTTON_SCAN 106

//----- LIDAR -----//

#define NUMBER_OF_MEASUREMENTS 50 
#define DELAY_BETWEEN_MEASUREMENTS 250

void startWifi(void);  // WiFi start function
void startMQTT(void);  // MQTT start function
void startLidar(void); // Lidar start function
void startServo(void); // Servo start function
void wifiCheck(void);  // WiFi check connection
void callback(char* topic, byte* payload, unsigned int length); // MQTT receive message callback
uint16_t getLidarMeasurement (Adafruit_VL53L0X *VL53L0X); // Get distance function
void sendMeasurementMQTT(uint16_t measurement); // Send data through MQTT
void lidarFunction(void);

WiFiClient espClient;
PubSubClient client(MQTT_SERVER, MQTT_PORT, callback, espClient);
Adafruit_VL53L0X lidar = Adafruit_VL53L0X();
Servo servoLidar;

uint16_t wifiCheckPeriod = 30000; // 30s wait for checking wifi connection
uint32_t wifiPreviousMillis =  0; // Wifi last check time
uint8_t lidarReadingPeriod = 100; // 0.1s wait between lidar measurement 
uint32_t lidarPreviousMillis = 0; // Lidar last measurement time
uint16_t lidarMeasurement_mm = 0; // Lidar measured distance
bool enabledDataSend = true;      // Allows to enter the communication loop

//----- Bluetooth -----//
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
uint8_t bluetoothData; // Data for the received value

void getBluetoothData(void);

//----- State Machine -----//
uint8_t stateMachine_State = MODE_REMOTE_CONTROL;

void remoteControl(void);
void lineFollower(void);

void remoteActions(void);

//----- Begin Code -----//
void setup() {
  //----- Serial Port -----// 
  Serial.begin(115200); // Begin Serial (for debugging)

  //----- Bluetooth Configuration -----//
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  //----- Pins Configuration -----//
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);

  //----- LIDAR Configuration -----//
  startWifi();
  startMQTT();
  startLidar();
  startServo();
}

void loop() {
  switch(stateMachine_State){
    case MODE_REMOTE_CONTROL:
      remoteControl();
      break;
    case MODE_LINE_FOLLOWER:
      lineFollower();
      break;
    default:
      break;
  }
}

//----- Bluetooth -----//
void getBluetoothData(void){
  uint8_t temporaryDataVal;
  if(SerialBT.available()) {
     temporaryDataVal = SerialBT.read();
     Serial.println(temporaryDataVal);
  }
  else{
    return;
  }
  
  switch(temporaryDataVal){
    case MODE_REMOTE_CONTROL:
    case MODE_LINE_FOLLOWER:
      stateMachine_State = temporaryDataVal;
      break;
    default:
      bluetoothData = temporaryDataVal;
      if(bluetoothData == BUTTON_SCAN){
        enabledDataSend = true;  
      }
      break;
  }
}

//----- State Machine -----//
void remoteControl(void){
  // Configurations
  Serial.println("Entered Remote Control Mode");
  
  // Loop
  while(stateMachine_State == MODE_REMOTE_CONTROL){
    wifiCheck();
    client.loop();
    
    getBluetoothData();
    remoteActions();
  }
  
  // Leaving Actions
  bluetoothData = BUTTON_STOP;
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
}

void lineFollower(void){
  // Configurations
  Serial.println("Entered Line Follower Mode");
  // Loop
  while(stateMachine_State == MODE_LINE_FOLLOWER){
    wifiCheck();
    client.loop();
    
    getBluetoothData();

    if(digitalRead(IR_SENSOR_RIGHT_PIN)){
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 255);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 100);
    }
    else{
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 255);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 100);
    }
  }

  // Leaving Actions
}

void remoteActions(void){
  switch(bluetoothData){
    case BUTTON_UP:
      Serial.println("Going Forward");
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 200);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 200);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      break;
    case BUTTON_LEFT:
      Serial.println("Turning Left");
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 180);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 180);
      break;
    case BUTTON_STOP:
      Serial.println("Stop");
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      break;
    case BUTTON_RIGHT:
      Serial.println("Turning Right");
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 180);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 180);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      break;
    case BUTTON_DOWN:
      Serial.println("Going Backward");
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 200);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 200);
      break;
    case BUTTON_SCAN:
      Serial.println("Scanning");
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      lidarFunction();
      break;
    default:
      break;
  }
}

//----- LIDAR -----//

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
    Serial.println("MQTT Connected");
    //client.subscribe("DataCom");
    //client.subscribe("ConfigCom");
  }
  else{
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

void lidarFunction(void){
  if(enabledDataSend){
    char msg[6];
    sprintf(msg, "%d", NUMBER_OF_MEASUREMENTS);
    client.publish("ConfigCom", msg);
    servoLidar.write(0);
    delay(500);
    for(uint8_t i = 0; i <= 180; i += 180/(NUMBER_OF_MEASUREMENTS - 1)){
      servoLidar.write(i);
      lidarMeasurement_mm = getLidarMeasurement(&lidar);
      sendMeasurementMQTT(lidarMeasurement_mm);
      delay(DELAY_BETWEEN_MEASUREMENTS);
    }
    enabledDataSend = false;
  }
}
