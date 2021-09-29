#include <probe_gps.h>
#include <Sampler.h>
#include <NaiveMultiDimensionalSampler.h>

void setup() {
    Serial.begin(115200); 
    initGpsSerial();
}

DoubleSampler latitudeSampler(3, 5);
NaiveMultiDimensionalSampler gpsSampler(3, 5, 2);


void loop() {
    

    for(int k = 0; k < 3; k++) {
        GpsData_t gpsData;
        bool probeSuccessful = probeGps(&gpsData, 3);

        if (probeSuccessful) {
            Serial.println("At:" + String(gpsData.timestamp, 3) + " => " + String(gpsData.latitude, 6) + " " + String(gpsData.longitude, 6) + " ; speed: " + gpsData.speedOverGround + " m/s ; course: " + gpsData.courseOverGround + " °");
            latitudeSampler.addSample(gpsData.latitude);
            double gpsSample[2] = {gpsData.latitude, gpsData.longitude};
            gpsSampler.addSample(gpsSample);
        }
    }

    if (latitudeSampler.isSignificant()) {
        Serial.println("Median latitude: " + String(latitudeSampler.median(), 6));
        Serial.println("Average latitude: " + String(latitudeSampler.average(), 6));
        Serial.println("Min latitude: " + String(latitudeSampler.min(), 6));
        Serial.println("Max latitude: " + String(latitudeSampler.max(), 6));
    }
    
    delay(1000);
}