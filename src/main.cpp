#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <FastLED_RGBW.h>
#include <LoRa.h>
#include <vector>
#include <FlexCAN_T4.h>
#include <isotp.h>

isotp<RX_BANKS_16, 512> tp; /* 16 slots for multi-ID support, at 512bytes buffer each payload rebuild */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;


#define NUM_LEDS 7
#define DATA_PIN 3

const long frequency = 915E6;
const int LORA_CS = 10;
const int LORA_RS = 6;
const int LORA_RXEN = 2;
const int LORA_TXEN = 9;
const int LORA_ISR = A7;

const uint16_t MIN_DISTANCE_STOP = 150;

int yCenter = 127;
int zCenter = 127;
float yOffset = 20; //ADC
float zOffset = 14; //ADC
float xOffset = 0; //ADC


int incomingByte = 0;
uint64_t lastRXTimeStamp = 0;
uint32_t lastRXTimer = 0;
bool newRemoteDataAvailable = false;
bool lastInterlockStatus = false;

String debug;

uint32_t lastBeaconEvent = 0;
uint32_t beaconStrobeSpeed = 500;


CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];

int x;
int y;
int z;
bool b1;
bool b2;
bool b3;
bool bj;

bool bj_last;

typedef struct bcast_message {  
  float x;
  float y;
  float z;  
  bool b1 = false;
  bool b2 = false;
  bool b3 = false;
  bool bj = false;
} bcast_message;

typedef struct button_state {    
  bool state = false;
  bool didUpdate = false;  
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

bcast_message remoteData;
remote_button_state remoteButtonState;
bool beaconState = false;

volatile BeaconMode beaconMode = BeaconMode::off;

void enterSerialMode();
void serialPassThru();
void checkWatchDog();
void setMotor(int channel, int speed);
void stopAll();
void parseData();
void updateMotion();
void updateRemoteButtonState();
void updateBeacon();
void beaconEvent();
void setBeacon(BeaconMode newMode);
void transmitStatus();
void canBusCallback(const ISOTP_data &config, const uint8_t *buf);
void checkTimeOut();

void onReceive(int packetSize);
String decToHex(int decimal);


void setup() {
  Serial.begin(115200);
  Serial4.begin(9600, SERIAL_7E1);  
  pinMode(LORA_ISR, INPUT);
  pinMode(LORA_TXEN, OUTPUT);
  pinMode(LORA_RXEN, OUTPUT);
  pinMode(3, OUTPUT);    
  Serial.println("FrankyStain 0.2 Teensy [BOOT]\n");  
  delay(500);  
  
  LoRa.setPins(LORA_CS, -2, LORA_ISR);

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("[SETUP] Radio Failed");
    while (true);                       // if failed, do nothing
  }


  //FastLED with RGBW
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(ledsRGB, getRGBWsize(NUM_LEDS));  

    //CAN
  Can1.begin();
  Can1.setClock(CLK_60MHz);
  Can1.setBaudRate(95238);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  tp.begin();
  tp.setWriteBus(&Can1); 
  tp.onReceive(canBusCallback);


  enterSerialMode();
  Serial.println("[Entering Loop]");  
  digitalWrite(LORA_RXEN, HIGH);
  lastRXTimeStamp = millis();
  setBeacon(BeaconMode::standby);
}





void loop() {
    
  onReceive(LoRa.parsePacket());  
  checkTimeOut();
  updateMotion();      
  updateBeacon(); 
  
}

std::vector<String> split(String str, String token) {
    std::vector<String> fields;

    String field;
    for (uint16_t i = 0; i < str.length()+1; i++) {
      String character = str.substring(i,i+1);      
      if (character == token || i == str.length()) {
        fields.push_back(field);
        field = "";
      } else {
        field = field + character;
      }
      
    }
    return fields;
}


void onReceive(int packetSize) {

  if (packetSize <=0 ) { return; }

  lastRXTimeStamp = millis(); 
  
  if (LoRa.available()) {
    String inString = "";
    while(LoRa.available()) {
       inString += (char)LoRa.read();
    }
    
    std::vector<String> lengthFields = split(inString,"-");
    uint16_t rxLength = lengthFields.back().toInt();
    std::vector<String> fields = split(lengthFields.front(),",");


    if (rxLength+2!=(uint16_t)inString.length()) {
      Serial.println("[onReceive] Invalid Data Received:");
      Serial.println(inString);
      return;
    }

    if (fields.size() != 4) {
      Serial.println("[onReceive] Invalid Data Received [Fields]:");
      Serial.println(inString);
      return;
    }
    
    remoteData.x = fields[0].toFloat();
    remoteData.y = fields[1].toFloat();
    remoteData.z = fields[2].toFloat();

    remoteData.b1 = (bool)fields[3].substring(0,1).toInt();
    remoteData.b2 = (bool)fields[3].substring(1,2).toInt();
    remoteData.b3 = (bool)fields[3].substring(2,3).toInt();
    remoteData.bj = (bool)fields[3].substring(3,4).toInt();

    debug = fields[3].substring(0,1);

    parseData();   
  } 

}

void parseData() {
  //(127/3.3) * 3.3
  float scaledX = (254.0 + xOffset) / 3.3 * remoteData.x;
  float scaledY = (254.0 + yOffset) / 3.3 * remoteData.y; //20
  float scaledZ = (254.0 + zOffset) / 3.3 * remoteData.z;  //14
  x = (uint16_t)scaledX;
  y = (uint16_t)scaledY;
  z = (uint16_t)scaledZ;
 


  if (remoteData.b1 != b1) {     
    remoteButtonState.b1.state = remoteData.b1;
    remoteButtonState.b1.didUpdate = true;
  } 

 if (remoteData.b2 != b2) {
    remoteButtonState.b2.state = b2;
    remoteButtonState.b2.didUpdate = true;    
  }

  if (remoteData.b3 != b3) {
    remoteButtonState.b3.state = b3;
    remoteButtonState.b3.didUpdate = true;
  }

  b1 = remoteData.b1;
  b2 = remoteData.b2;
  b3 = remoteData.b3;
  bj = remoteData.bj;  

  newRemoteDataAvailable = true; 
  updateRemoteButtonState();
  transmitStatus();
}

void transmitStatus() {  
  
  //Not Working
  // if (LoRa.available() > 0) { return; }

  if (LoRa.beginPacket() == true) {   
    digitalWrite(LORA_RXEN, LOW);
    digitalWrite(LORA_TXEN, HIGH);                      
    LoRa.write(beaconMode);        
    digitalWrite(LORA_RXEN, HIGH);
    digitalWrite(LORA_TXEN, LOW);         
    LoRa.endPacket(false);          
  }
}

void updateRemoteButtonState() {

if (remoteButtonState.b1.didUpdate == true) {
  if (remoteButtonState.b1.state == false) {
    Serial.println("RELEASED B1");
    setBeacon(BeaconMode::standby);
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



void checkTimeOut() {
  lastRXTimer = millis() - lastRXTimeStamp;

  if (lastRXTimer > 300) {
    stopAll();
    Serial.println("Remote Message Expired. Stop All: " + String(lastRXTimer));            
    setBeacon(BeaconMode::interlock);
    return;
  }
}

void updateMotion() {  
 
  if (beaconMode == BeaconMode::interlock) {
    return;
  }

  if (bj == true) {
    int motorA = yCenter - y;
      int motorB = zCenter - z;

    if (bj_last == false) {      
      if (abs(motorA) > 20) {     
        lastInterlockStatus = true;          
          Serial.println("Motor Power Might JumpStart. Reduce command and try again");
          return;
        }    
      bj_last = true;
      
      setBeacon(BeaconMode::armed);      
    }    
            

      bool shouldLog = false;
      if (motorA > 15 || motorA < -15) {         
        setMotor(0, motorA);  
        shouldLog = true;             
      } else {    
        setMotor(0, 0); //Stop Traction            
      }

      if (motorB > 3 || motorB < -3) {                 
        if (motorB > 127) { motorB = 127; }
        if (motorB < -127) { motorB = -127; }    

        int attenuatedMotorB = (int)map(motorB, (0 - zCenter), zCenter, -60, 60);    
        setMotor(1, attenuatedMotorB);               
      } else {    
        setMotor(1, 0); //Stop Steering    
      }

      if (z < 89 || z > 140) {
        shouldLog = true;
      }

      if (shouldLog == true) {
        // Serial.print(y); 
        // Serial.print(",");
        // Serial.print(z);
        // Serial.print(",");
        // Serial.println(lastRXTimer);
      }
  } else if (bj_last == true) {
    stopAll();    
    bj_last = false;
    setBeacon(BeaconMode::standby);
  }

  // if (bj == false) {    
  //   stopAll();    
  //   setBeacon(BeaconMode::standby);
  //   lastInterlockStatus = true; 
  //   return;   
  // }
  
  


  //Deadband
  
   
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
// // ^02 Amp Limit
// // 0 = 30A
// // 1 = 45A
// // 2 = 60A
// // 3 = 75A
// // 4 = 90A
// // 5 = 105A (default) 
// // 6 = 120A
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
  Serial4.println(motorCmd);
}

void checkWatchDog() {
  if (Serial4.available()) {
    incomingByte = Serial4.read();
    incomingByte = incomingByte & 0x7F;   
       
    if ((char)incomingByte=='W') {
      Serial.println("Watchdog");
      Serial4.print((char)13);      
    }
  }    
}

void enterSerialMode() {
for (int i = 0; i <= 20; i++) {   
    Serial4.print((char)13);
    delay(30);        
  } 
}

void serialPassThru() {
  int incomingByte;

  delay(200);
  // Serial4.print("!");  

   if (Serial.available()) {
    incomingByte = Serial.read(); 
    // incomingByte = incomingByte;
    // Serial.print((char)incomingBqwyte);  
    
    Serial4.print((char)incomingByte);  
    
  }

 if (Serial4.available()) {
    incomingByte = Serial4.read();     
    incomingByte = incomingByte & 0x7F;      
    Serial.print((char)incomingByte);       
  }
  
}



void beaconEvent() {    
  if (beaconState==true) {    
    beaconState = false;
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    leds[3] = CRGB::Black;
    leds[4] = CRGB::Black;
    leds[5] = CRGB::Black;
    leds[6] = CRGB::Black;    
    FastLED.show();
  } else {
    beaconState = true;    
    switch (beaconMode)
    {
    case BeaconMode::off:
      beaconState = false;
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    leds[3] = CRGB::Black;
    leds[4] = CRGB::Black;
    leds[5] = CRGB::Black;
    leds[6] = CRGB::Black;    
    beaconStrobeSpeed = 1000;  
      break;    
    case BeaconMode::standby:
    leds[0] = CRGB::Green;
    leds[1] = CRGB::Green;
    leds[2] = CRGB::Green;
    leds[3] = CRGB::Green;
    leds[4] = CRGB::Green;
    leds[5] = CRGB::Green;
    leds[6] = CRGB::Green;    
    beaconStrobeSpeed = 1850;  
      break;    
    case BeaconMode::inMotion:
    leds[0] = CRGB::Yellow;
    leds[1] = CRGB::Yellow;
    leds[2] = CRGB::Yellow;
    leds[3] = CRGB::Yellow;
    leds[4] = CRGB::Yellow;
    leds[5] = CRGB::Yellow;
    leds[6] = CRGB::Yellow;    
      beaconStrobeSpeed = 300;  
      break;    
    case BeaconMode::armed:
    leds[0] = CRGB::Yellow;
    leds[1] = CRGB::Yellow;
    leds[2] = CRGB::Yellow;
    leds[3] = CRGB::Yellow;
    leds[4] = CRGB::Yellow;
    leds[5] = CRGB::Yellow;
    leds[6] = CRGB::Yellow;    
      beaconStrobeSpeed = 800;  
      break;    
    case BeaconMode::interlock:
    leds[0] = CRGB::Red;
    leds[1] = CRGB::Red;
    leds[2] = CRGB::Red;
    leds[3] = CRGB::Red;
    leds[4] = CRGB::Red;
    leds[5] = CRGB::Red;
    leds[6] = CRGB::Red;    
      beaconStrobeSpeed = 2500;  
      break;    
    default:
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    leds[3] = CRGB::Black;
    leds[4] = CRGB::Black;
    leds[5] = CRGB::Black;
    leds[6] = CRGB::Black;    
      beaconStrobeSpeed = 1000;  
      break;
    }    
  }

  FastLED.show();  
  

  
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
  if (beaconMode != newMode) {    
      beaconMode = newMode;      
      beaconEvent();    
  }    
}


void canBusCallback(const ISOTP_data &config, const uint8_t *buf) {
 
  int len = config.len-1;
  String parsedString;
  for(int i=0; i<len; i++)
  {
    String byte = (char)buf[i];    
    parsedString += byte;
  }

  std::vector<String> fields = split(parsedString,",");
  
  if (fields[0] <= MIN_DISTANCE_STOP || fields[1] <= MIN_DISTANCE_STOP || fields[2] <= MIN_DISTANCE_STOP ) {        
    if (beaconMode != BeaconMode::interlock) {      
      stopAll();
      setBeacon(BeaconMode::interlock);                          
      Serial.println("[canBusCallback] Minimum Distance Reached");
    }
  }
  
  // Serial.println(newString);

}

// void canBusCallback(const ISOTP_data &config, const uint8_t *buf) {

//   Serial.println(buf[0]);
//   Serial.println(buf[1]);
//   Serial.println(buf[2]);


//   int len = config.len-1;
//   String newString;
//   for(int i=0; i<len; i++)
//   {
//     String byte = (char)buf[i];    
//     newString = newString + byte;
//   }
//   Serial.println(newString);

// }




// /////
// #include <FlexCAN_T4.h>
// #include <isotp.h>
// isotp<RX_BANKS_16, 512> tp; /* 16 slots for multi-ID support, at 512bytes buffer each payload rebuild */
// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// void myCallback(const ISOTP_data &config, const uint8_t *buf) {

// Serial.println(buf[0]);
// Serial.println(buf[1]);
// Serial.println(buf[2]);


// int len = config.len-1;
// String newString;
// for(int i=0; i<len; i++)
// {
//     String byte = (char)buf[i];    
//     newString = newString + byte;
// }
// Serial.println(newString);

// }


// void setup() {
//   Serial.begin(115200); delay(400);
//   Can1.begin();
//   Can1.setClock(CLK_60MHz);
//   Can1.setBaudRate(95238);
//   Can1.setMaxMB(16);
//   Can1.enableFIFO();
//   Can1.enableFIFOInterrupt();
//   tp.begin();
//   tp.setWriteBus(&Can1); /* we write to this bus */
//   tp.onReceive(myCallback); /* set callback */

//   pinMode(3, OUTPUT);
//   pinMode(4, OUTPUT);

// }

// void loop() {
//   static uint32_t sendTimer = millis();
//   if ( millis() - sendTimer > 1000 ) {
//     uint8_t buf[] = {100, 200, 255};
//     ISOTP_data config;
//     config.id = 0x666;
//     config.flags.extended = 0; /* standard frame */
//     config.separation_time = 10; /* time between back-to-back frames in millisec */
//     tp.write(config, buf, sizeof(buf));
//     // tp.write(config, b, sizeof(b));
//     sendTimer = millis();

    


//   }