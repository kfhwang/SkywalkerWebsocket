/*********
  Setup Wifi and websocket server for communication between Artisan and Skywalker Roaster
  Please use this program carefully and take risk by yourself.
*********/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
//display
#include <limits.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include "MAX6675.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
// SCL=>D22, SDA=>D21
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// const int dataPin   = 15; //SO hardware 19
// const int selectPin = 2;  //CS hardware 5
// const int clockPin  = 4; //SCK hardware 18

// MAX6675 thermoCouple(selectPin, dataPin, clockPin);
// MAX6675 thermoCouple(selectPin, &SPI); //hardware


//Communication wit Skywalker
const int txPin = 25;// 白
const int rxPin = 27;// 綠

const int preamble = 7000;
const int one_length = 1200;
const int roasterLength = 7;
const int controllerLength = 6;

uint8_t receiveBuffer[roasterLength];
uint8_t sendBuffer[controllerLength];

int ventByte = 0;
int drumByte = 3;
int coolByte = 2;
int filterByte = 1;
int heatByte = 4;
int checkByte = 5;

double BeanTemp = 55.0; //bean teamperature
double envTemp = 30.0; //environment temp

unsigned long lastEventTime = 0;
unsigned long lastEventTimeout = 10000000;
char CorF = 'C';

void setControlChecksum() {
  uint8_t sum = 0;
  for (int i = 0; i < (controllerLength - 1); i++) {
    sum += sendBuffer[i];
  }
  sendBuffer[controllerLength - 1] = sum;
}

bool setValue(uint8_t* bytePtr, uint8_t v) {
  *bytePtr = v;
  setControlChecksum();  // Always keep the checksum updated.
}

void shutdown() {  //Turn everything off!
  for (int i = 0; i < controllerLength; i++) {
    sendBuffer[i] = 0;
  }
}

void pulsePin(int pin, int duration) {
  //Assuming pin is HIGH when we get it
  digitalWrite(pin, LOW);
  delayMicroseconds(duration);
  digitalWrite(pin, HIGH);
  //we leave it high
}

void sendMessage() {
  //send Preamble
  pulsePin(txPin, 7500);
  delayMicroseconds(3800);

  //send Message
  for (int i = 0; i < controllerLength; i++) {
    for (int j = 0; j < 8; j++) {
      if (bitRead(sendBuffer[i], j) == 1) {
        pulsePin(txPin, 1500);  //delay for a 1
      } else {
        pulsePin(txPin, 650);  //delay for a 0
      }
      delayMicroseconds(750);  //delay between bits
    }
  }
}

double calculateTemp() {
  /* 
    I really hate this. 
    It seems to work.. but I feel like there must be a better way than 
    using a 4th degree polynomial to model this but I sure can't seem to figure it out. 
  */

  double x = ((receiveBuffer[0] << 8) + receiveBuffer[1]) / 1000.0;
  double y = ((receiveBuffer[2] << 8) + receiveBuffer[3]) / 1000.0;

#ifdef __DEBUG__
  Serial.print(x);
  Serial.print(',');
  Serial.println(y);
#endif

  double v = 583.1509258523457 + -714.0345395202813 * x + -196.071718077524 * y
             + 413.37964344228334 * x * x + 2238.149675349052 * x * y
             + -4099.91031297056 * y * y + 357.49007607425233 * x * x * x
             + -5001.419602972793 * x * x * y + 8242.08618555862 * x * y * y
             + 247.6124684730026 * y * y * y + -555.8643213534281 * x * x * x * x
             + 3879.431274654493 * x * x * x * y + -6885.682277959339 * x * x * y * y
             + 2868.4191998911865 * x * y * y * y + -1349.1588373011923 * y * y * y * y;

  if (CorF == 'C') v = (v - 32) * 5 / 9;

  return v;
}

void getMessage(int bytes, int pin) {
  unsigned long timeIntervals[roasterLength * 8];
  unsigned long pulseDuration = 0;
  int bits = bytes * 8;

  while (pulseDuration < preamble) {  //Wait for it or exut
    pulseDuration = pulseIn(pin, LOW);
  }

  for (int i = 0; i < bits; i++) {  //Read the proper number of bits..
    timeIntervals[i] = pulseIn(pin, LOW);
  }

  for (int i = 0; i < 7; i++) {  //zero that buffer
    receiveBuffer[i] = 0;
  }

  for (int i = 0; i < bits; i++) {  //Convert timings to bits
    //Bits are received in LSB order..
    if (timeIntervals[i] > one_length) {  // we received a 1
      receiveBuffer[i / 8] |= (1 << (i % 8));
    }
  }
}

bool calculateRoasterChecksum() {
  uint8_t sum = 0;
  for (int i = 0; i < (roasterLength - 1); i++) {
    sum += receiveBuffer[i];
  }

#ifdef __DEBUG__
  Serial.print("sum: ");
  Serial.print(sum, HEX);
  Serial.print(" Checksum Byte: ");
  Serial.println(receiveBuffer[roasterLength - 1], HEX);
#endif
  return sum == receiveBuffer[roasterLength - 1];
}

void printBuffer(int bytes) {
  for (int i = 0; i < bytes; i++) {
    Serial.print(sendBuffer[i], HEX);
    Serial.print(',');
  }
  Serial.print("\n");
}

void getRoasterMessage() {
#ifdef __DEBUG__
  Serial.print("R ");
#endif

  bool passedChecksum = false;
  int count = 0;

  while (!passedChecksum) {
    count += 1;
    getMessage(roasterLength, rxPin);
    passedChecksum = calculateRoasterChecksum();
  }
#ifdef __DEBUG__
  printBuffer(roasterLength);
#endif

#ifdef __WARN__
  if (count > 1) {
    Serial.print("[!] WARN: Took ");
    Serial.print(count);
    Serial.println(" tries to read roaster.");
  }
#endif

  BeanTemp = calculateTemp();
  lastEventTime = micros();
}
void handleHEAT(uint8_t value) {
  if (value <= 100) {
    setValue(&sendBuffer[heatByte], value);
  }
  lastEventTime = micros();
}

void handleVENT(uint8_t value) {
  if (value <= 100) {
    setValue(&sendBuffer[ventByte], value);
  }
  lastEventTime = micros();
}

void handleCOOL(uint8_t value) {
  if (value >= 0 && value <= 100) {
    setValue(&sendBuffer[coolByte], value);
  }
  lastEventTime = micros();
}

void handleFILTER(uint8_t value) {
  if (value <= 100) {
    setValue(&sendBuffer[filterByte], value);
  }
  lastEventTime = micros();
}

void handleDRUM(uint8_t value) {
  if (value > 0) {
   sendBuffer[3]=100;
  } else {
   sendBuffer[3]=0;
  }
  setControlChecksum();
  lastEventTime = micros();
}
 

bool itsbeentoolong() {
  unsigned long now = micros();
  unsigned long duration = now - lastEventTime;
  // if (duration < 0) {
  //   duration = (ULONG_MAX - time) + now;  //I think this is right.. right?
  // }
  if (duration > lastEventTimeout) {
    return true;  //We turn everything off
  }
  else return false;
}

void handleCHAN() {
  Serial.println("# Active channels set to 0200");
}

#define LOGO_HEIGHT   64
#define LOGO_WIDTH    64
static const unsigned char PROGMEM logo_bmp[] =
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x7f, 0x80, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x3f, 0x00, 0x00, 
	0x00, 0x03, 0xf0, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x03, 0xe0, 0x00, 
	0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 
	0x00, 0x3c, 0x00, 0x07, 0xe0, 0x00, 0x3c, 0x00, 0x00, 0x78, 0x00, 0x7f, 0xfe, 0x00, 0x1e, 0x00, 
	0x00, 0xf0, 0x03, 0xff, 0xff, 0xc0, 0x0f, 0x00, 0x01, 0xe0, 0x07, 0xf0, 0x7f, 0xe0, 0x07, 0x80, 
	0x03, 0xc0, 0x1f, 0x80, 0xf1, 0xf8, 0x03, 0xc0, 0x03, 0x80, 0x3e, 0x01, 0xe0, 0x7c, 0x01, 0xc0, 
	0x07, 0x00, 0x78, 0x03, 0xc0, 0x3e, 0x00, 0xe0, 0x07, 0x00, 0xf0, 0x03, 0xbf, 0xff, 0x00, 0xe0, 
	0x0e, 0x01, 0xe0, 0x07, 0xff, 0xff, 0x80, 0x70, 0x0e, 0x03, 0xc0, 0x07, 0xff, 0xdf, 0xc0, 0x70, 
	0x1c, 0x03, 0x80, 0x07, 0xc0, 0x3d, 0xc0, 0x38, 0x1c, 0x07, 0xe0, 0x07, 0x80, 0x78, 0xe0, 0x38, 
	0x1c, 0x0f, 0xf8, 0x07, 0x80, 0xf0, 0xf0, 0x18, 0x38, 0x0f, 0xfe, 0x03, 0x81, 0xe0, 0x70, 0x1c, 
	0x38, 0x0f, 0xff, 0x03, 0xff, 0xc0, 0x70, 0x1c, 0x38, 0x1c, 0xf7, 0x81, 0xff, 0x80, 0x38, 0x1c, 
	0x70, 0x1c, 0x73, 0xc0, 0x7e, 0x00, 0x38, 0x0e, 0x70, 0x1c, 0x39, 0xc0, 0x01, 0xf0, 0x38, 0x0e, 
	0x70, 0x1c, 0x38, 0xe0, 0x07, 0xf8, 0x38, 0x0e, 0x70, 0x18, 0x38, 0xe0, 0x1f, 0xfe, 0x18, 0x0e, 
	0x71, 0x18, 0x38, 0x70, 0x3f, 0xfe, 0x18, 0x8e, 0x73, 0xb8, 0x30, 0x70, 0x79, 0xff, 0x1d, 0xce, 
	0x73, 0xbc, 0x70, 0x70, 0xe3, 0x87, 0x1d, 0xce, 0x70, 0x1c, 0x70, 0x71, 0xe3, 0x87, 0x18, 0x0e, 
	0x70, 0x1e, 0x30, 0x73, 0xc7, 0x07, 0x18, 0x0e, 0x70, 0x1f, 0x38, 0x73, 0x87, 0x07, 0x38, 0x0e, 
	0x70, 0x1f, 0xbc, 0x77, 0x07, 0x07, 0x38, 0x0e, 0x30, 0x1f, 0xff, 0xe7, 0x07, 0x07, 0x38, 0x0c, 
	0x38, 0x1d, 0xff, 0xe7, 0x07, 0x0e, 0x38, 0x1c, 0x38, 0x0e, 0xff, 0xc7, 0x07, 0x0e, 0x70, 0x1c, 
	0x38, 0x0e, 0x1f, 0x07, 0x07, 0x1c, 0x70, 0x1c, 0x1c, 0x07, 0x00, 0x07, 0x0e, 0x3f, 0xe0, 0x38, 
	0x1c, 0x07, 0x80, 0x07, 0xfe, 0xff, 0xe0, 0x38, 0x1c, 0x03, 0x80, 0x03, 0xff, 0xf3, 0xc0, 0x38, 
	0x0e, 0x03, 0xc0, 0x0f, 0xff, 0xc3, 0xc0, 0x70, 0x0e, 0x01, 0xff, 0xff, 0xff, 0x07, 0x80, 0x70, 
	0x07, 0x00, 0xff, 0xfe, 0x3c, 0x0f, 0x00, 0xe0, 0x07, 0x80, 0x7f, 0xf0, 0x00, 0x3e, 0x01, 0xe0, 
	0x03, 0x80, 0x3f, 0x00, 0x00, 0x7c, 0x01, 0xc0, 0x01, 0xc0, 0x0f, 0x80, 0x01, 0xf0, 0x03, 0x80, 
	0x01, 0xe0, 0x07, 0xfe, 0x7f, 0xe0, 0x07, 0x80, 0x00, 0xf0, 0x01, 0xff, 0xff, 0x80, 0x0f, 0x00, 
	0x00, 0x78, 0x00, 0x3f, 0xfc, 0x00, 0x1e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 
	0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x01, 0xf0, 0x00, 
	0x00, 0x07, 0xc0, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x0f, 0x80, 0x00, 
	0x00, 0x00, 0xfe, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x07, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void drawLogo(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}
// Replace with your network credentials
const char* ssid = "AP Name"; 
const char* password = "12345678";  
IPAddress local_IP(192, 168, 31, 123); 
IPAddress gateway(192, 168, 31, 1);
IPAddress subnet(255, 255, 255, 0);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

 

// Initialize WiFi connect to external AP
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

//receive websocket message from Artisan and response
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    //data[len] = 0;
    String message = (char*)data;
    Serial.println(message);
    JSONVar artisanMessage = JSON.parse(message);
    if (JSON.typeof(artisanMessage) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
    }
    else{ 
      JSONVar json_res, res_data;
        
      if(strcmp((const char *) artisanMessage["command"],"getData") == 0){
        json_res["id"] = (int)artisanMessage["id"];
        res_data["BT"] = (long) BeanTemp;
        res_data["ET"] = (long) BeanTemp;
        //json_res["Data"] = (String) "{BT:100, ET:200}"; //please mapping to Ports Configuration of Artisan
        json_res["data"] = res_data;
        String resString=JSON.stringify(json_res);
        delay(200);
        ws.textAll(resString); 
        Serial.println(resString);
        lastEventTime = micros();
      }
      else if(strcmp((const char *) artisanMessage["command"], "setControlParams") == 0){
        if(artisanMessage.hasOwnProperty("fan")){
          Serial.printf("set FAN %d\n", (int)artisanMessage["fan"]); 
          handleVENT((uint8_t)artisanMessage["fan"]);
        }
        if(artisanMessage.hasOwnProperty("fire")){
          Serial.printf("set Fire %d\n", (int)artisanMessage["fire"]);
          handleHEAT((uint8_t)artisanMessage["fire"]);
        }
        if(artisanMessage.hasOwnProperty("cool")){
          Serial.printf("set Cool %d\n", (int)artisanMessage["cool"]);
          handleCOOL((uint8_t)artisanMessage["cool"]);
        }
        if(artisanMessage.hasOwnProperty("drum")){
          Serial.printf("set Drum %d\n", (int)artisanMessage["drum"]);
          handleDRUM((uint8_t)artisanMessage["drum"]);
        }
        if(artisanMessage.hasOwnProperty("OFF")){
          Serial.printf("set OFF %d\n", (uint8_t)artisanMessage["OFF"]);
          shutdown();
        }
        if(artisanMessage.hasOwnProperty("filter")){
          Serial.printf("set filter %d\n", (int)artisanMessage["filter"]);
          handleFILTER((uint8_t)artisanMessage["filter"]);
        }
      }
      
    }

  }
}

bool isConnected=false;

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      display.clearDisplay();
      display.setTextSize(2);             // Normal 2x pixel scale
      display.setTextColor(SSD1306_WHITE);     
      display.setCursor(0,0);
      display.println("Connected!");
      display.println("Artisan!");
      display.display();
      delay(1500);
      isConnected=true;
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      display.clearDisplay();
      display.setTextSize(2);             // Normal 2x pixel scale
      display.setTextColor(SSD1306_WHITE);     
      display.setCursor(0,0);
      display.println("Disconnect");
      display.display();
      delay(1500);
      isConnected=false;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); //LED
  digitalWrite(2, HIGH);
  delay(500);
  
  //For MAX6675, It's not workable
  // SPI.begin();
  // thermoCouple.begin();
  // thermoCouple.setSPIspeed(4000000);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //Serial.println(F("SSD1306 allocation failed"));
    digitalWrite(2, HIGH);
    delay(2000);
  }
  digitalWrite(2, LOW);
  delay(500);
  display.clearDisplay();
  drawLogo(); 
  
  //SERVER setup
  //WiFi.mode(WIFI_AP);
  //WiFi.softAPConfig(local_IP,gateway,subnet);
  //WiFi.softAP(ssid, password);
  //Serial.println(WiFi.softAPIP());
  //Serial.println("Server started.....");
  //Client Mode
  initWiFi();
  Serial.println("Configuring soft-AP...");
  
  Serial.print("AP IP address: ");
  Serial.println(WiFi.localIP());
  // Serial.print("softAP macAddress: ");
  // Serial.println(WiFi.softAPmacAddress());
  
  //
  display.clearDisplay();
  display.setTextSize(2);             // Normal 2x pixel scale
  display.setTextColor(SSD1306_WHITE);     
  display.setCursor(0,0);
  display.println("Skywalker"); 
  display.setTextSize(1);   
  display.println("Pwd: Skywalker");
  display.println("");
  display.println("WebSocket Address:");
  //display.print(WiFi.softAPIP());
  display.println(WiFi.localIP());
  display.println(":80");
  display.print("Wait connect...");
  //display.print(ssid);
  display.display();
  delay(500);

  //initWwebSocket;
  initWebSocket();

  // Start server
  server.begin();

  pinMode(txPin, OUTPUT);
  shutdown();
  handleDRUM(100);
}

void loop() {

  ws.cleanupClients();

  if(isConnected){
    display.clearDisplay();
    display.setTextSize(2);  // Normal 2x pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);  // Start at top-left corner
    display.println("Skywalker");         
    display.print("BT:");
    display.println(BeanTemp);
    // display.print("ET:");
    // display.println(envTemp);
    display.print("Heat:");
    display.println(sendBuffer[heatByte]);
    display.print("Fan:");
    display.print(sendBuffer[ventByte]);
    if(sendBuffer[coolByte]==100)
      display.print("Cool");
    display.display();
    delay(100);
    // if(itsbeentoolong()){
    //   digitalWrite(2, HIGH);
    //   sendMessage();
    // }else{
    //   digitalWrite(2, LOW);
    //}
    //roaster communication
    sendMessage();
    getRoasterMessage();

    // int status = thermoCouple.read();
    // if(status== STATUS_OK){
    //   double envNew = (double)thermoCouple.getTemperature()-3;
    //   envTemp = (envTemp+envNew)/2.0;
    //   if(CorF=='F')
    //     envTemp = envTemp * (9 / 5) + 32;
    // }
    
  }else{
    display.clearDisplay();
    display.setTextSize(2);             // Normal 2x pixel scale
    display.setTextColor(SSD1306_WHITE);     
    display.setCursor(0,0);
    display.println("Skywalker"); 
    display.setTextSize(1);   
    //display.println("Pwd: Skywalker");
    display.println("");
    display.println("WebSocket Address:");
    //display.print(WiFi.softAPIP());
    display.println(WiFi.localIP());
    display.println(":8080");
    display.print("Wait connect...");
    //display.print(ssid);
    display.display();
    shutdown();
  }
}
