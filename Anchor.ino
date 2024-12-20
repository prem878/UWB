#include <SoftwareSerial.h>

#define RX_PIN 2  // Connect to DWM1000 TX
#define TX_PIN 3  // Connect to DWM1000 RX

SoftwareSerial mySerial(RX_PIN, TX_PIN);

// #define MODE_TAG

void setup() {
  // Start hardware serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("Starting initialization...");
  
  // Start software serial for DWM1000
  mySerial.begin(115200);
  delay(2000);  // Give more time to initialize
  
  Serial.println("Serial ports initialized");
  
  #ifdef MODE_TAG
    Serial.println("Configuring as TAG...");
    // Tag configuration
    mySerial.println("AT+anchor_tag=0");
    delay(5);
    Serial.println("Sent anchor_tag command");
    
    mySerial.println("AT+RST");
    delay(5000);
    Serial.println("Sent reset command");
    
    mySerial.println("AT+interval=5");
    delay(5000);
    Serial.println("Sent interval command");
    
    mySerial.println("AT+switchdis=1");
    delay(5000);
    Serial.println("Sent switchdis command");
  #else
    // Anchor configuration
    Serial.println("Configuring as ANCHOR...");
    mySerial.println("AT+anchor_tag=1,1");    // First set as anchor
    delay(5000);
    mySerial.println("AT+RST");             // Reset to apply changes
    delay(5000);
  #endif
  
  Serial.println("Setup complete");
}
void loop() {
  // Forward data from module to serial monitor
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
