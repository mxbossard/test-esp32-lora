
/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

 // References:
 // [feather] adafruit-feather-m0-radio-with-lora-module.pdf

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_BME280.h>
#include <CayenneLPP.h>

#define LPP_BME280_CHANNEL 1
#define PROBE_MINIMUM_SIGNIFICANT_VALUES 3
#define PROBE_MAXIMUM_PROBING_ITERATION 5
#define ADC_CORRECTION_RATIO ( 1 )

#define PROBE_SLEEP_DELAY_IN_SEC 300

#define D39 39 // BME 280 SDA
#define D34 34 // BME 280 SCL

// BME280 sensor on I2C
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// Lorawan credentials in another file
#include "privateCredentials.h"

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

CayenneLPP lpp(128);

// project-specific definitions in lmic_project_config.h file
// #define DISABLE_PING
// #define DISABLE_BEACONS
// #define LMIC_ENABLE_DeviceTimeReq 0
// #define DISABLE_JOIN
// #define DISABLE_MCMD_PingSlotChannelReq


// TTGO ESP32 Lora pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};


/**
 * Return [temperature, humidty, pressure]
 */ 
float * probeBme280() {
    //Serial.println(F("Probing BME280 ..."));
    bool error = false;
    float temp = 0;
    float humidity = 0;
    float pressure = 0;

    if (bme.begin(0x76, & Wire)) {
        sensors_event_t temp_event, pressure_event, humidity_event;
        bme_temp->getEvent(&temp_event);
        bme_pressure->getEvent(&pressure_event);
        bme_humidity->getEvent(&humidity_event);

        temp = temp_event.temperature;
        humidity = humidity_event.relative_humidity;
        pressure = pressure_event.pressure;
    } else {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        error = true;
    }

    static float array[4];
    array[0] = temp;
    array[1] = humidity;
    array[2] = pressure;
    array[3] = error;
    return array;
}

int sortFloatAsc(const void *cmp1, const void *cmp2) {
    // Need to cast the void * to float *
    float a = *((float *)cmp1);
    float b = *((float *)cmp2);

    // NAN values get pushed at the end of the array
    if (isnan(a)) return 1;
    if (isnan(b)) return -1;

    // The comparison
    //return a > b ? 1 : (a < b ? -1 : 0);
    // A simpler, probably faster way:
    return a - b;
}

/**
 * Return the median of a float array.
 * Exclude NAN values.
 * return a NAN value if less than PROBE_MINIMUM_SIGNIFICANT_VALUES non NAN values in the array.
 */
float medianOfArray(float array[]) {
    int arrayLength = sizeof(array) - 1;
    int arraySize = arrayLength * sizeof(array[0]);
    float buffer[arrayLength];
    memcpy(buffer, array, arraySize);
    
    // qsort - last parameter is a function pointer to the sort function
    qsort(buffer, arrayLength, sizeof(array[0]), sortFloatAsc);
    
    // Count values in array (not NAN values)
    byte valuesCount = 0;
    for (int k=0; k < arrayLength; k++) {
        if (!isnan(array[k])) valuesCount++;
    }

    // If too few significant values, return NAN.
    if (valuesCount < PROBE_MINIMUM_SIGNIFICANT_VALUES) {
        return NAN;
    }

    int medianIndex = (valuesCount + 1) / 2;
    float medianValue = buffer[medianIndex];
    Serial.println("Median value: " + String(medianValue) + " from " + String(valuesCount) + " values.");
    return medianValue;
}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        uint8_t mydata[128];
        memcpy(mydata, lpp.getBuffer(), lpp.getSize());
        LMIC_setTxData2(1, lpp.getBuffer(), sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void doWork() {

    int probingDelay = 2000;
    byte errorCount = 0;
    byte probeCount = PROBE_MINIMUM_SIGNIFICANT_VALUES;
    byte maxProbeCount = PROBE_MAXIMUM_PROBING_ITERATION;

    float batVoltageValues[maxProbeCount];
    float bme280tempValues[maxProbeCount];
    float bme280HumidityValues[maxProbeCount];
    float bme280PressureValues[maxProbeCount];
    float dht22tempValues[maxProbeCount];
    float dht22HumidityValues[maxProbeCount];

    // Fill arrays with NAN
    for (int k=0; k < maxProbeCount; k++) {
        batVoltageValues[k] = NAN;
        bme280tempValues[k] = NAN;
        bme280HumidityValues[k] = NAN;
        bme280PressureValues[k] = NAN;
        dht22tempValues[k] = NAN;
        dht22HumidityValues[k] = NAN;   
    }

    Serial.println("");

    for (int k=0; k < probeCount; k++) {
        Serial.println("Probing #" + String(k + 1) + " of " + String(probeCount) + " ...");
        
        // float batVoltage = readADC();
        // batVoltageValues[k] = batVoltage;
        //Serial.println("Battery voltage: " + String(batVoltage) + " V");

        float * bme280Datas = probeBme280();
        bool bme280Error = bme280Datas[3];
        if (!bme280Error) {
            bme280tempValues[k] = bme280Datas[0];
            bme280HumidityValues[k] = bme280Datas[1];
            bme280PressureValues[k] = bme280Datas[2];
        }
        //Serial.println("BME 280 datas: " + String(bme280Datas[0]) + " ; " + String(bme280Datas[1]) + " ; " + String(bme280Datas[2]));

        bool dht22Error = false;
        // float * dht22Datas = probeDht22();
        // bool dht22Error = dht22Datas[2];
        // if (!dht22Error) {
        //     dht22tempValues[k] = dht22Datas[0];
        //     dht22HumidityValues[k] = dht22Datas[1];
        // }
        //Serial.println("DHT 22 datas: " + String(dht22Datas[0]) + " ; " + String(dht22Datas[1]));
        
        if (probeCount < maxProbeCount && (bme280Error || dht22Error)) {
            probeCount ++;
            errorCount ++;
        }

        // Probe delay only befor probing
        if (k < probeCount - 1) {
            //Serial.println("Waiting for probingDelay: " + String(probingDelay) + " ...");
            delay(probingDelay);
        }
    }

    if (errorCount > 0) {
        Serial.println("/!\\ Got some errors !");
    }

    // float batVoltageValue = medianOfArray(batVoltageValues);
    float bme280tempValue = medianOfArray(bme280tempValues);
    float bme280HumidityValue = medianOfArray(bme280HumidityValues);
    float bme280PressureValue = medianOfArray(bme280PressureValues);
    // float dht22tempValue = medianOfArray(dht22tempValues);
    // float dht22HumidityValue = medianOfArray(dht22HumidityValues);

    lpp.reset();
    // if (!isnan(batVoltageValue))
    //     lpp.addVoltage(LPP_BOARD_CHANNEL, batVoltageValue);
    if (!isnan(bme280tempValue))
        lpp.addTemperature(LPP_BME280_CHANNEL, bme280tempValue);
    if (!isnan(bme280HumidityValue))
        lpp.addRelativeHumidity(LPP_BME280_CHANNEL, bme280HumidityValue);
    if (!isnan(bme280PressureValue))
        lpp.addBarometricPressure(LPP_BME280_CHANNEL, bme280PressureValue);
    // if (!isnan(dht22tempValue))
    //     lpp.addTemperature(LPP_DHT22_CHANNEL, dht22tempValue);
    // if (!isnan(dht22HumidityValue))
    //     lpp.addRelativeHumidity(LPP_DHT22_CHANNEL, dht22HumidityValue);
    
    // Error count
    // lpp.addDigitalInput(LPP_BOARD_ERROR_COUNT_CHANNEL, errorCount);
    // lpp.addDigitalInput(LPP_BME280_ERROR_COUNT_CHANNEL, errorCount);
    // lpp.addDigitalInput(LPP_DHT22_ERROR_COUNT_CHANNEL, errorCount);

    // Probe count
    // lpp.addDigitalInput(LPP_BOARD_PROBE_COUNT_CHANNEL, probeCount);
    // lpp.addDigitalInput(LPP_BME280_PROBE_COUNT_CHANNEL, probeCount);
    // lpp.addDigitalInput(LPP_DHT22_PROBE_COUNT_CHANNEL, probeCount);
    
#ifdef DEBUG
    DynamicJsonDocument jsonBuffer(2048);
    JsonArray root = jsonBuffer.to<JsonArray>();
    lpp.decode(lpp.getBuffer(), lpp.getSize(), root);
    //serializeJsonPretty(root, Serial);
    serializeJson(root, Serial);
    Serial.println("");
#endif

    char str[128] = "";
    array_to_string(lpp.getBuffer(), lpp.getSize(), str);
    Serial.print("LPP: ");
    Serial.println(str);

    // Start job
    do_send(&sendjob);

    Serial.println("Work done.");
}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void setup() {
//    pinMode(13, OUTPUT);
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    delay(100);     // per sample code on RF_95 test
    Serial.println(F("Starting"));

    Wire.begin(D39, D34, 100000);

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #else
    # error Region not supported
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    // do_send(&sendjob);

    doWork();
}

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(25, HIGH);
    }
    else {
      digitalWrite(25, LOW);
    }

    os_runloop_once();

}
