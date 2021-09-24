#include <probe_gps.h>
 
void setup() {
    Serial.begin(115200); 
    initGpsSerial();
}

void loop() {
    GpsData_t gpsData;
    bool probeSuccessful = probeGps(&gpsData, 10);

    if (probeSuccessful) {
        Serial.println("At:" + String(gpsData.timestamp, 3) + " => " + String(gpsData.latitude, 6) + " " + String(gpsData.longitude, 6) + " ; speed: " + gpsData.speedOverGround + " m/s ; course: " + gpsData.courseOverGround + " °");
    }
    
    delay(1000);
}