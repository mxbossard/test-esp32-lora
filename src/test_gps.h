#include <SoftwareSerial.h>
 

#define GPS_RX 36
#define GPS_TX 39

#define KNOT_TO_METER_PER_SECOND 1.943844

SoftwareSerial GpsSerial(GPS_TX, GPS_RX);
 
void setup() {
  GpsSerial.begin(9600);
  Serial.begin(115200); 
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

double adaptGpsData(String data) {
    // 4754.0518 => 47.540518 ; 00154.8579 => 1.548579
    // String integer = getValue(data, '.', 0);
    // String decimal = getValue(data, '.', 1);
    // String rescaleInteger = integer.substring(0, integer.length() - 2);
    // String rescaleDecimal = integer.substring(integer.length() - 2) + decimal;
    // String rescaleData = rescaleInteger + "." + rescaleDecimal;
    return data.toDouble() / 100;
}

void loop() {
    char carac = 0;
    String message = "";
    // boucle qui attends un \n pour valider la trame et la décoder (/!\ Passer l'option en bas à droite du moniteur série en "Nouvelle ligne")
    while (carac != '\n') {
        // si un caractère est présent sur la liaison
        if(GpsSerial.available()) {
            // lecture d'un caractère
            carac = GpsSerial.read();
            // concaténation du caractère au message
            message = message + carac; 
        }
    }
    //Serial.print(message);

    int payloadStart = message.indexOf("$GPRMC");
    if(payloadStart > -1) {
        String payload = message.substring(payloadStart);
        Serial.print(payload);
        String utsPosition = getValue(payload, ',', 1);
        String status = getValue(payload, ',', 2);
        String latitude = getValue(payload, ',', 3);
        String nsIndicator = getValue(payload, ',', 4);
        String longitude = getValue(payload, ',', 5);
        String ewIndicator = getValue(payload, ',', 6);
        String speedOverGroundKnots = getValue(payload, ',', 7);
        String courseOverGroundDegrees = getValue(payload, ',', 8);
        String dateString = getValue(payload, ',', 9);

        if (latitude != "" && longitude != "") {
            // $GPRMC,192441.000,A,4754.0518,N,00154.8579,E,1.77,327.72,230921,,,D*64
            // At:192441.000 => 4754.0518N 00154.8579E
            Serial.println("At:" + utsPosition + " => " + latitude + nsIndicator + " " + longitude + ewIndicator);
            double floatLatitude = adaptGpsData(latitude);
            double floatLongitude = adaptGpsData(longitude);
            float speedOverGround = KNOT_TO_METER_PER_SECOND * speedOverGroundKnots.toDouble();
            float courseOverGround = courseOverGroundDegrees.toDouble();
            String date = dateString + " " + utsPosition;
            Serial.println("At:" + date + " => " + String(floatLatitude, 6) + " " + String(floatLongitude, 6) + " ; speed: " + speedOverGround + " m/s ; course: " + courseOverGround + " °");
        }
    }
}
 
/*
$GPRMC,165614.000,A,4754.0383,N,00154.8070,E,0.34,78.56,230921,,,A*5A
$GPGGA,165615.000,4754.0383,N,00154.8072,E,1,4,2.79,135.0,M,47.6,M,,*51






*/