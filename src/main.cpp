#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
// MY address: C4:DD:57:67:31:D0
// ^02 Amp Limit
// 0 = 30A
// 1 = 45A
// 2 = 60A
// 3 = 75A
// 4 = 90A
// 5 = 105A (default) 
// 6 = 120A


int incomingByte = 0;
uint64_t lastRXTimeStamp = 0;
uint32_t lastRXTimer = 0;
bool newRemoteDataAvailable = false;
bool lastInterlockStatus = false;

uint32_t lastBeaconEvent = 0;
uint32_t beaconStrobeSpeed = 500;



static uint8_t BTN_OB_PIN = 0;
Adafruit_NeoPixel beacon(7, 27, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel obled(1, 2, NEO_GRB + NEO_KHZ800);

typedef struct bcast_message {  
  float x;
  float y;
  float z;  
  bool b1;
  bool b2;
  bool b3;
  bool bj;  
} bcast_message;

typedef struct button_state {    
  bool state;
  bool didUpdate;  
} button_state;

typedef struct remote_button_state {    
  button_state b1;
  button_state b2;
  button_state b3;
  button_state bj;    
} remote_button_state;

enum BeaconMode {
  off,
  armed,
  standby,
  inMotion,
  interlock
};


int x;
int y;
int z;
bool b1;
bool b2;
bool b3;
bool bj;

bcast_message remoteData;
remote_button_state remoteButtonState;
bool beaconState = false;

BeaconMode beaconMode = BeaconMode::off;

void enterSerialMode();
void serialPassThru();
void checkWatchDog();
void setMotor(int channel, int speed);
void stopAll();
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void parseData();
void updateRunMode();
void updateMotion();
void updateRemoteButtonState();
void updateBeacon();
void beaconEvent();
void setBeacon(BeaconMode newMode);


String decToHex(int decimal);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_7E1, 19, 23);
  WiFi.mode(WIFI_STA);  

  pinMode(BTN_OB_PIN, INPUT);
  
  

  Serial.println("FrankyStain 0.1 [BOOT]]\n");
  Serial.println("MacAddress: " + WiFi.macAddress());

  delay(500);  
   
//  FastLED.addLeds<WS2812, 27, GRB>(leds, 7).setCorrection( TypicalLEDStrip );
  // FastLED.addLeds<SK6812, 2, GRB>(obled, 1);
  beacon.begin();   
  obled.begin(); 
  // FastLED.addLeds<WS2812B, 27, GRB>(beacon, getRGBWsize(7)).setCorrection( TypicalLEDStrip );
  





  if (esp_now_init() != ESP_OK) {
    Serial.println("Error: ESPNOW Failed");        
  }
 
  enterSerialMode();
  esp_now_register_recv_cb(OnDataRecv);  
  
}

void loop() {
  updateMotion();  
  updateRemoteButtonState();
  updateRunMode();
  updateBeacon();
}

void updateRemoteButtonState() {

if (remoteButtonState.b1.didUpdate == true) {
  if (remoteButtonState.b1.state == false) {
    Serial.println("RELEASED B1");
  } else {
    Serial.println("PRESSED B1");
  }
  remoteButtonState.b1.didUpdate = false;
}

if (remoteButtonState.b2.didUpdate == true) {
  if (remoteButtonState.b2.state == false) {
    Serial.println("RELEASED B2");
  } else {
    Serial.println("PRESSED B2");
  }
  remoteButtonState.b2.didUpdate = false;
}

if (remoteButtonState.b3.didUpdate == true) {
  if (remoteButtonState.b3.state == false) {
    Serial.println("RELEASED B3");
  } else {
    Serial.println("PRESSED B3");
  }
  remoteButtonState.b3.didUpdate = false;
}

}

void updateRunMode() {
  bool obbtn_state = digitalRead(BTN_OB_PIN);

  if (obbtn_state == LOW) {
    Serial.print("Serial PassThru Mode");
    while(true) {
      serialPassThru();
    }
  }
}


void updateMotion() {
  lastRXTimer = millis() - lastRXTimeStamp;
 
  if (lastRXTimer > 300) {
    stopAll();
    Serial.println("Remote Message Expired. Stop All: " + String(lastRXTimer));        
    // beaconMode = BeaconMode::interlock;   
    setBeacon(BeaconMode::interlock);
    return;
  }

  if (bj == false) {    
    stopAll();
    // beaconMode = BeaconMode::standby;
    setBeacon(BeaconMode::standby);
    lastInterlockStatus = true;
    return;
  }
  
  // int mB = 127 - z;
  int motorA = 127 - y;
  int motorB = 127 - z;
  

  if (lastInterlockStatus == true) {
    if (abs(motorA) > 20) {      
      // beaconMode = BeaconMode::interlock;
      setBeacon(BeaconMode::interlock);
      Serial.println("Motor Power Might JumpStart. Reduce command and try again");
      return;
    }    
  }

  // beaconMode = BeaconMode::armed;
  setBeacon(BeaconMode::armed);
  
  lastInterlockStatus = false; 

  Serial.print(y); //Debug Line
  Serial.print(",");
  Serial.println(z);

  if (motorA > 20 || motorA <  -20) {         
    setMotor(0, motorA);        
  } else {    
    setMotor(0, 0); //Stop All    
  }

  setMotor(1, motorB);  
}

String decToHex(int decimal) {
  char hexadecimalnum [5];
  sprintf(hexadecimalnum, "%02X", decimal);
  String hex(hexadecimalnum);
  return hex;
}


void stopAll() {
  setMotor(0,0);
  setMotor(1,0);
}

void setMotor(int channel, int speed) {

  if (speed > 127) { speed = 127; }
  if (speed < -127) { speed = -127; }

  String motorCmd = "!";  

  if (channel == 0) {
    if (speed >= 0) {
      motorCmd = motorCmd + "A";
    } else if (speed < 0) {
      motorCmd = motorCmd + "a";
    }
  }

  if (channel == 1) {
    if (speed >= 0) {
      motorCmd = motorCmd + "B";
    } else if (speed < 0) {
      motorCmd = motorCmd + "b";
    }
  }

  speed = abs(speed);

  motorCmd = motorCmd + decToHex(speed);
  Serial1.println(motorCmd);
}

void checkWatchDog() {
  if (Serial1.available()) {
    incomingByte = Serial1.read();    
    if ((char)incomingByte=='W') {
      Serial.println("Watchdog");
      Serial1.print((char)13);      
    }
  }    
}

void enterSerialMode() {
for (int i = 0; i <= 20; i++) {   
    Serial1.print((char)13);
    delay(30);        
  } 
}

void serialPassThru() {
   if (Serial.available()) {
    incomingByte = Serial.read();         
    Serial1.print((char)incomingByte);              
  }

 if (Serial1.available()) {
    incomingByte = Serial1.read();      
    Serial.print((char)incomingByte);
    digitalWrite(13, HIGH);    
  }
  
}

void parseData() {
  //(127/3.3) * 3.3
  float scaledX = 254 / 3.3 * remoteData.x;
  float scaledY = 254 / 3.3 * remoteData.y;
  float scaledZ = 254 / 3.3 * remoteData.z; 
  x = int(scaledX);
  y = int(scaledY);
  z = int(scaledZ);
  b1 = remoteData.b1;
  b2 = remoteData.b2;
  b3 = remoteData.b3;
  bj = remoteData.bj;   

  if (remoteButtonState.b1.state != b1) {
    remoteButtonState.b1.state = b1;
    remoteButtonState.b1.didUpdate = true;
  }

 if (remoteButtonState.b2.state != b2) {
    remoteButtonState.b2.state = b2;
    remoteButtonState.b2.didUpdate = true;
  }

  if (remoteButtonState.b3.state != b3) {
    remoteButtonState.b3.state = b3;
    remoteButtonState.b3.didUpdate = true;
  }



  newRemoteDataAvailable = true;       
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&remoteData, incomingData, sizeof(remoteData));
  lastRXTimeStamp = millis();  
  parseData();   
}


void beaconEvent() {  
  
  if (beaconState==true) {
    beaconState = false;
    beacon.setPixelColor(0, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(1, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(2, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(3, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(4, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(5, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(6, beacon.Color(0, 0, 0, 0));
    obled.setPixelColor(0, obled.Color(0,0,0));
    beacon.show();
    obled.show();
  } else {
    beaconState = true;
    switch (beaconMode)
    {
    case BeaconMode::off:
      beaconState = false;
    beacon.setPixelColor(0, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(1, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(2, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(3, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(4, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(5, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(6, beacon.Color(0, 0, 0, 0));
    obled.setPixelColor(0, obled.Color(0,0,0));
      beaconStrobeSpeed = 1000;  
      break;    
    case BeaconMode::standby:
    beacon.setPixelColor(0, beacon.Color(0, 255, 0, 0));
    beacon.setPixelColor(1, beacon.Color(0, 255, 0, 0));
    beacon.setPixelColor(2, beacon.Color(0, 255, 0, 0));
    beacon.setPixelColor(3, beacon.Color(0, 255, 0, 0));
    beacon.setPixelColor(4, beacon.Color(0, 255, 0, 0));
    beacon.setPixelColor(5, beacon.Color(0, 255, 0, 0));
    beacon.setPixelColor(6, beacon.Color(0, 255, 0, 0));  
    obled.setPixelColor(0, obled.Color(0,255,0));
      beaconStrobeSpeed = 1850;  
      break;    
    case BeaconMode::inMotion:
    beacon.setPixelColor(0, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(1, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(2, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(3, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(4, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(5, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(6, beacon.Color(255, 239, 0, 0));
    obled.setPixelColor(0, obled.Color(255,239,0));
      beaconStrobeSpeed = 500;  
      break;    
    case BeaconMode::armed:
    beacon.setPixelColor(0, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(1, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(2, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(3, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(4, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(5, beacon.Color(255, 239, 0, 0));
    beacon.setPixelColor(6, beacon.Color(255, 239, 0, 0));
    obled.setPixelColor(0, obled.Color(255,239,0));    
      beaconStrobeSpeed = 500;  
      break;    
    case BeaconMode::interlock:
    beacon.setPixelColor(0, beacon.Color(255, 0, 0, 0));
    beacon.setPixelColor(1, beacon.Color(255, 0, 0, 0));
    beacon.setPixelColor(2, beacon.Color(255, 0, 0, 0));
    beacon.setPixelColor(3, beacon.Color(255, 0, 0, 0));
    beacon.setPixelColor(4, beacon.Color(255, 0, 0, 0));
    beacon.setPixelColor(5, beacon.Color(255, 0, 0, 0));
    beacon.setPixelColor(6, beacon.Color(255, 0, 0, 0));  
    obled.setPixelColor(0, obled.Color(255,0,0));
      beaconStrobeSpeed = 2500;  
      break;    
    default:
    beacon.setPixelColor(0, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(1, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(2, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(3, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(4, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(5, beacon.Color(0, 0, 0, 0));
    beacon.setPixelColor(6, beacon.Color(0, 0, 0, 0));
    obled.setPixelColor(0, obled.Color(0,0,0));
      beaconStrobeSpeed = 1000;  
      break;
    }    
  }

  beacon.show();  
  obled.show();
}

void updateBeacon() {  
  if ((millis() >= lastBeaconEvent + (beaconStrobeSpeed/2)) && beaconState == false) {    
    beaconEvent();
    lastBeaconEvent = millis();
  } else if ((millis() >= lastBeaconEvent + beaconStrobeSpeed) && beaconState == true) {
    beaconEvent();
    lastBeaconEvent = millis();
  }
}

void setBeacon(BeaconMode newMode) {
      beaconMode = newMode;
      updateBeacon();
}
