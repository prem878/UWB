#include <HardwareSerial.h>
#include <math.h>

// Use Hardware Serial2 for UART communication
HardwareSerial mySerial(2);

// Pin configuration
#define RX_PIN 16   // Connect to sensor's TX
#define TX_PIN 17   // Connect to sensor's RX

// Define anchor positions (x, y, z) in meters
struct Anchor {
    float x;
    float y;
    float z;
};

// Anchor positions in your local frame
Anchor anchors[4] = {
    {0.0, 0.0, 0.0},     // Anchor 1 position
    {1.70, 0.0, 0.0},     // Anchor 2 position
    {0.0, 1.20, 0.0},     // Anchor 3 position
    {0.0, 0.0, 0.78}      // Anchor 4 position
};

// Array to store distances from tag to anchors
float distances[4] = {0.0, 0.0, 0.0, 0.0};

// Structure to hold position
struct Position {
    float x;
    float y;
    float z;
};

void setup() {
    Serial.begin(115200);
    mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    // Tag configuration
    mySerial.println("AT+anchor_tag=0");
    delay(500);
    mySerial.println("AT+RST");
    delay(500);
    mySerial.println("AT+interval=5");
    delay(500);
    mySerial.println("AT+switchdis=1");
    delay(500);
}

// Parse the distance data from UWB module
bool parseDistanceData(String data) {
    // Format: "distanceX:Y.ZZm"
    static int currentDistance = 0;
    
    if(data.startsWith("distance")) {
        int colonIndex = data.indexOf(':');
        int mIndex = data.indexOf('m');
        
        if(colonIndex != -1 && mIndex != -1) {
            String distStr = data.substring(colonIndex + 1, mIndex);
            distances[currentDistance] = distStr.toFloat();
            
            currentDistance = (currentDistance + 1) % 4;
            return true;
        }
    }
    return false;
}

// Perform trilateration to find tag position
Position calculatePosition() {
    Position pos = {0.0, 0.0, 0.0};
    
    // Using linear least squares method for trilateration
    float A[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    float b[3] = {0.0, 0.0, 0.0};
    
    // Build matrices for least squares solution
    for(int i = 1; i < 4; i++) {
        A[i-1][0] = 2.0 * (anchors[i].x - anchors[0].x);
        A[i-1][1] = 2.0 * (anchors[i].y - anchors[0].y);
        A[i-1][2] = 2.0 * (anchors[i].z - anchors[0].z);
        
        b[i-1] = pow(distances[0], 2) - pow(distances[i], 2) - 
                 (pow(anchors[0].x, 2) + pow(anchors[0].y, 2) + pow(anchors[0].z, 2)) +
                 (pow(anchors[i].x, 2) + pow(anchors[i].y, 2) + pow(anchors[i].z, 2));
    }
    
    // Solve using Cramer's rule (for simplicity - could be improved with other methods)
    float det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if(abs(det) < 0.0001) {
        return pos; // Return zero position if singular
    }
    
    pos.x = ((b[0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
              A[0][1] * (b[1] * A[2][2] - A[1][2] * b[2]) +
              A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2])) / det);
              
    pos.y = ((A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2]) -
              b[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
              A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0])) / det);
              
    pos.z = ((A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1]) -
              A[0][1] * (A[1][0] * b[2] - b[1] * A[2][0]) +
              b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) / det);
    
    return pos;
}

void loop() {
    static String buffer = "";
    static unsigned long lastCalculation = 0;
    const unsigned long CALC_INTERVAL = 100; // Calculate position every 100ms
    
    while (mySerial.available()) {
        char c = mySerial.read();
        if (c == '\n') {
            if (parseDistanceData(buffer)) {
                // Check if it's time to calculate position
                unsigned long currentTime = millis();
                if (currentTime - lastCalculation >= CALC_INTERVAL) {
                    Position tagPosition = calculatePosition();
                    
                    // Print distances and calculated position
                    Serial.println("\nCurrent Measurements:");
                    for(int i = 0; i < 4; i++) {
                        Serial.print("Distance "); Serial.print(i+1);
                        Serial.print(": "); Serial.print(distances[i]);
                        Serial.println("m");
                    }
                    
                    Serial.println("\nCalculated Position:");
                    Serial.print("X: "); Serial.print(tagPosition.x);
                    Serial.println("m");
                    Serial.print("Y: "); Serial.print(tagPosition.y);
                    Serial.println("m");
                    Serial.print("Z: "); Serial.print(tagPosition.z);
                    Serial.println("m");
                    Serial.println("------------------------");
                    
                    lastCalculation = currentTime;
                }
            }
            buffer = "";
        } else {
            buffer += c;
        }
    }
}
