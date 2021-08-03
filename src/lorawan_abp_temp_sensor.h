
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

// FIXME This should works but do not. To fix this copy the content of the file into lmic_lib/project_config/lmic_project_config.h
//#define ARDUINO_LMIC_PROJECT_CONFIG_H abp_lmic_project_config.h


#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>

//#include "esp_wifi.h"
//#include "driver/adc.h"
#include <CircularBuffer.h>

#include <U8g2lib.h>
#include <Adafruit_BME280.h>
#include <CayenneLPP.h>

//#define DEBUG
#define LOG_TO_SERIAL
//#define LOG_TO_SCREEN
#define SCREEN

#ifdef DEBUG
    #define SERIAL
#endif

#define LPP_BOARD_CHANNEL 0
#define LPP_BME280_CHANNEL 1
#define LPP_BME280_PROBE_COUNT_CHANNEL 101
#define LPP_BME280_ERROR_COUNT_CHANNEL 201
#define PROBE_MINIMUM_SIGNIFICANT_VALUES 3
#define PROBE_MAXIMUM_PROBING_ITERATION 5
#define ADC_CORRECTION_RATIO ( 1 )

#define PROBING_PERIOD_IN_SEC 60

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define ADC_BAT_PIN 35


// Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);

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
#include "abpPrivateCredentials.h"

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = LORAWAN_NWKSKEY;

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = LORAWAN_APPSKEY;

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = LORAWAN_DEVADDR ; // <-- Change this address for every node!


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }

static osjob_t sendjob;

// Max TTN payload size for SF10 and greater : 51 bytes.
CayenneLPP lpp(51);

/*
For 51 bytes in EU868 with 125kHz BW,
- at SF7:  118ms    => 254 msg / day => 10 msg / hour
- at SF8:  215.6ms  => 139 msg / day =>  5 msg / hour
- at SF9:  390.1ms  =>  76 msg / day =>  3 msg / hour
- at SF10: 698.4ms  =>  42 msg / day
- at SF11: 1560.6ms =>  19 msg / day
- at SF12: 2793.5ms =>  10 msg / day
*/

// Saves the LMIC structure during DeepSleep
RTC_DATA_ATTR lmic_t RTC_LMIC;

// TTGO ESP32 Lora pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

struct LoraStats_t {
    uint32_t dataSendingAttemptCounter;
	//uint32_t dataSentCounter;
    uint32_t joinAttemptCounter;
    uint32_t joinCounter;
    uint32_t txCounter;
    uint32_t txAckCounter;
    uint32_t rxCounter;
};

RTC_DATA_ATTR LoraStats_t loraStats;

#ifdef SCREEN
CircularBuffer<String, 5> loggingQueue;
#endif

void displayLoggingQueue() {
    u8g2.clearDisplay();
    u8g2.clearBuffer();
    int8_t ascent = u8g2.getAscent();
    int8_t maxCharHeight = u8g2.getMaxCharHeight();

    int8_t cursorY = ascent + 1;
    for (int i = loggingQueue.size() - 1 ; i >= 0 ; i--) {
        // retrieves the i-th element from the buffer without removing it
        String message = loggingQueue[i];
        u8g2.setCursor(0, cursorY);
        u8g2.print(message);
        cursorY += maxCharHeight + 2;
    }
    u8g2.sendBuffer();
}

void _log(String message, bool newLine = true, bool screen = false) {
    #ifdef LOG_TO_SERIAL
    if (newLine) {
        Serial.println(message);
    } else {
        Serial.print(message);
    }
    Serial.flush();
    #endif

    #ifdef LOG_TO_SCREEN
    if (screen) {
        String lastMessage = loggingQueue[0];
        int8_t maxCharCount = u8g2.getDisplayWidth() / u8g2.getMaxCharWidth() * 1.5;
        char lastMessageEol = lastMessage.charAt(lastMessage.length() - 1);
        if (lastMessageEol == '\n') {
            message = message.substring(0, maxCharCount);
            if (newLine) {
                message += '\n';
            }
            loggingQueue.unshift(message);
        } else {
            String concat = loggingQueue.shift();
            message = concat + message;
            message = message.substring(0, maxCharCount);
            if (newLine) {
                message += '\n';
            }
            loggingQueue.unshift(message);
        }

        displayLoggingQueue();
    }
    #endif
}

void logln(String message, bool screen = true) {
    _log(message, true, screen);
}

void log(String message, bool screen = true) {
    _log(message, false, screen);
}

void _log(int message, char base = 10, bool newLine = true, bool screen = false) {
    #if defined(LOG_TO_SERIAL) || defined(LOG_TO_SCREEN)
    _log(String(message, base), newLine, screen);
    #endif
}

void logln(int message, char base = 10, bool screen = true) {
    _log(message, base, true, screen);
}

void log(int message, char base = 10, bool screen = true) {
    _log(message, base, false, screen);
}

void deepSleep(int deepsleep_ms) {
    log(F("Entering deepsleep for "), false);
    log(deepsleep_ms, 10, false);
    logln(F(" ms"), false);
    //esp_sleep_enable_timer_wakeup(deepsleep_sec * 1000000);
    //esp_deep_sleep_start();
    esp_deep_sleep(1000.0 * deepsleep_ms);
}

void storeStats() {
    //EEPROM.put(0, loraStats);
    //EEPROM.commit();
}

void restoreStats() {
    //EEPROM.get(0, loraStats);
}

void displayStats() {
    String currentDatarate;
    switch(LMIC.datarate) {
        case EU868_DR_SF12: currentDatarate = "SF12"; break;
        case EU868_DR_SF11: currentDatarate = "SF11"; break;
        case EU868_DR_SF10: currentDatarate = "SF10"; break;
        case EU868_DR_SF9: currentDatarate = "SF9"; break;
        case EU868_DR_SF8: currentDatarate = "SF8"; break;
        case EU868_DR_SF7: currentDatarate = "SF7"; break;
        case EU868_DR_SF7B: currentDatarate = "SF7B"; break;
        case EU868_DR_FSK: currentDatarate = "FSK"; break;
        case EU868_DR_NONE: currentDatarate = "no"; break;
        default: currentDatarate = "-"; break;
    }

    String currentTxPower = String(LMIC.adrTxPow);
    String currentTxChannel = String(LMIC.txChnl);

    String sendingAttempt = String(loraStats.dataSendingAttemptCounter);
    String txAttempt = String(loraStats.txCounter);
    String txAck = String(loraStats.txAckCounter);

    loggingQueue.clear();
    loggingQueue.unshift("DR: " + currentDatarate + " pow: " + currentTxPower + "dbm");
    loggingQueue.unshift("chan: " + currentTxChannel);
    loggingQueue.unshift("send: " + sendingAttempt + " tx: " + txAttempt + " ack: " + txAck);
    displayLoggingQueue();
}

void storeLmic(int deepsleep_ms) {
    logln(F("Storing LMIC into RTC"), false);
    RTC_LMIC = LMIC;

    // ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
    // Therefore reset DutyCyles

    unsigned long now = millis();

    // EU Like Bands
#if defined(CFG_LMIC_EU_like)
    logln(F("Reset CFG_LMIC_EU_like bands"), false);
    for (int i = 0; i < MAX_BANDS; i++) {
        ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_ms / 1000) * OSTICKS_PER_SEC);
        if (correctedAvail < 0) {
            correctedAvail = 0;
        }
        RTC_LMIC.bands[i].avail = correctedAvail;
    }

    RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_ms / 1000) * OSTICKS_PER_SEC);
    if (RTC_LMIC.globalDutyAvail < 0) {
        RTC_LMIC.globalDutyAvail = 0;
    }
#else
    logln(F("No DutyCycle recalculation function!"));
#endif
}

void restoreLmic() {
    logln(F("Restoring LMIC from RTC"));
    LMIC = RTC_LMIC;
    log(F("seq up: "));
    logln(String(LMIC.seqnoUp));
}

/**
 * Return [temperature, humidty, pressure]
 */ 
float * probeBme280() {
    //logln(F("Probing BME280 ..."));
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
        logln(F("Could not find a valid BME280 sensor, check wiring!"), false);
        error = true;
    }

    static float array[4];
    array[0] = temp;
    array[1] = humidity;
    array[2] = pressure;
    array[3] = error;
    return array;
}

float readADC() {
    //logln(F("Reading ADC ..."));
    float ad = 0;
    //float resolution = 3.13 * (2.187+2.200) / 2.187 / 1023; //calibrate based on your voltage divider AND Vref!
    float resolution = 3.2 * 2 * ADC_CORRECTION_RATIO / 4096;
    int adcr = analogRead(ADC_BAT_PIN);
    //logln("Read ADC value: " + String(adcr));
    //logln("Resolution: " + String(resolution, 10) + "mV");
    ad = adcr * resolution;
    //logln(F("Reading ADC done."));

    return ad;
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
    logln("Median value: " + String(medianValue) + " from " + String(valuesCount) + " values.", false);
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
        logln(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        lmic_tx_error_t result = LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        if (result == LMIC_ERROR_SUCCESS) {
            u4_t seqnoUp = LMIC.seqnoUp;
            log(F("Packet queued with next seq up: "), false);
            logln(seqnoUp, 10, false);
        } else {
            log(F("Unable to queue message ! Error is: "));
            logln(result);
        }
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

    logln("");

    for (int k=0; k < probeCount; k++) {
        logln("Probing #" + String(k + 1) + " of " + String(probeCount) + " ...");
        
        float batVoltage = readADC();
        batVoltageValues[k] = batVoltage;
        //logln("Battery voltage: " + String(batVoltage) + " V");

        float * bme280Datas = probeBme280();
        bool bme280Error = bme280Datas[3];
        if (!bme280Error) {
            bme280tempValues[k] = bme280Datas[0];
            bme280HumidityValues[k] = bme280Datas[1];
            bme280PressureValues[k] = bme280Datas[2];
        }
        //logln("BME 280 datas: " + String(bme280Datas[0]) + " ; " + String(bme280Datas[1]) + " ; " + String(bme280Datas[2]));

        bool dht22Error = false;
        // float * dht22Datas = probeDht22();
        // bool dht22Error = dht22Datas[2];
        // if (!dht22Error) {
        //     dht22tempValues[k] = dht22Datas[0];
        //     dht22HumidityValues[k] = dht22Datas[1];
        // }
        //logln("DHT 22 datas: " + String(dht22Datas[0]) + " ; " + String(dht22Datas[1]));
        
        if (probeCount < maxProbeCount && (bme280Error || dht22Error)) {
            probeCount ++;
            errorCount ++;
        }

        // Probe delay only befor probing
        if (k < probeCount - 1) {
            //logln("Waiting for probingDelay: " + String(probingDelay) + " ...");
            delay(probingDelay);
        }
    }

    if (errorCount > 0) {
        logln("/!\\ Got probing errors !");
    }

    float batVoltageValue = medianOfArray(batVoltageValues);
    float bme280tempValue = medianOfArray(bme280tempValues);
    float bme280HumidityValue = medianOfArray(bme280HumidityValues);
    float bme280PressureValue = medianOfArray(bme280PressureValues);
    // float dht22tempValue = medianOfArray(dht22tempValues);
    // float dht22HumidityValue = medianOfArray(dht22HumidityValues);

    lpp.reset();
    if (!isnan(batVoltageValue))
        lpp.addVoltage(LPP_BOARD_CHANNEL, batVoltageValue);
    if (!isnan(bme280tempValue))
        lpp.addTemperature(LPP_BME280_CHANNEL, bme280tempValue);
    if (!isnan(bme280HumidityValue))
        lpp.addRelativeHumidity(LPP_BME280_CHANNEL, bme280HumidityValue);
    if (!isnan(bme280PressureValue))
        lpp.addBarometricPressure(LPP_BME280_CHANNEL, bme280PressureValue);

    if (errorCount > 0)
        lpp.addDigitalInput(LPP_BME280_CHANNEL, errorCount);

    // if (!isnan(dht22tempValue))
    //     lpp.addTemperature(LPP_DHT22_CHANNEL, dht22tempValue);
    // if (!isnan(dht22HumidityValue))
    //     lpp.addRelativeHumidity(LPP_DHT22_CHANNEL, dht22HumidityValue);
    
    // Error count
    // lpp.addDigitalInput(LPP_BOARD_ERROR_COUNT_CHANNEL, errorCount);
    //lpp.addDigitalInput(LPP_BME280_ERROR_COUNT_CHANNEL, errorCount);
    // lpp.addDigitalInput(LPP_DHT22_ERROR_COUNT_CHANNEL, errorCount);

    // Probe count
    // lpp.addDigitalInput(LPP_BOARD_PROBE_COUNT_CHANNEL, probeCount);
    //lpp.addDigitalInput(LPP_BME280_PROBE_COUNT_CHANNEL, probeCount);
    // lpp.addDigitalInput(LPP_DHT22_PROBE_COUNT_CHANNEL, probeCount);
    
#ifdef DEBUG
    DynamicJsonDocument jsonBuffer(2048);
    JsonArray root = jsonBuffer.to<JsonArray>();
    lpp.decode(lpp.getBuffer(), lpp.getSize(), root);
    //serializeJsonPretty(root, Serial);
    serializeJson(root, Serial);
    logln("");
#endif

    char str[128] = "";
    array_to_string(lpp.getBuffer(), lpp.getSize(), str);
    log("LPP: ");
    logln(str);

    logln("Work done.");
}


void onEvent (ev_t ev) {
    log(os_getTime(), 10, false);
    log(": ", false);
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            logln(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            logln(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            logln(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            logln(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            logln(F("EV_JOINING"));
            loraStats.joinAttemptCounter += 1;
            displayStats();
            break;
        case EV_JOINED:
            logln(F("EV_JOINED"));
            loraStats.joinCounter += 1;
            displayStats();
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     logln(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            logln(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            logln(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            loraStats.txCounter += 1;
            logln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                loraStats.txAckCounter += 1;
                logln(F("Received ack"));
            if (LMIC.dataLen) {
                logln(F("Received "));
                logln(LMIC.dataLen);
                logln(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            displayStats();
            break;
        case EV_LOST_TSYNC:
            logln(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            logln(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            loraStats.rxCounter += 1;
            logln(F("EV_RXCOMPLETE"));
            displayStats();
            break;
        case EV_LINK_DEAD:
            logln(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            logln(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    logln(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            logln(F("EV_TXSTART"));
            displayStats();
            break;
        case EV_TXCANCELED:
            logln(F("EV_TXCANCELED"));
            displayStats();
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            logln(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            displayStats();
            break;
        default:
            log(F("Unknown event: "));
            logln((unsigned) ev);
            break;
    }
}

void setup() {
    // Disable WiFi
    //WiFi.mode(WIFI_MODE_NULL);
    //esp_wifi_disconnect();
    //esp_wifi_stop();

    // Disable Bluetooth
    //btStop();

    // Disable ADC
    //adc_power_off(); // deprectaed in favor of adc_power_release()

    // Reduce cpu frequency
    //setCpuFrequencyMhz(10);

//    pinMode(13, OUTPUT);

    #ifdef LOG_TO_SERIAL
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    #endif

    #ifdef SCREEN
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearDisplay();
    #endif

    logln(F("Starting"));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

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

    if (RTC_LMIC.seqnoUp != 0) {
        restoreLmic();
    } else {
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
    }

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
}

void sendLoraMessage() {
    // Start job
    do_send(&sendjob);
    loraStats.dataSendingAttemptCounter += 1;
}

void loop() {
    displayStats();
    doWork();

    int startSeqUp = LMIC.seqnoUp;
    sendLoraMessage();

    displayStats();

    while (true) {
        os_runloop_once();

        const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((PROBING_PERIOD_IN_SEC * 1000)));
        if (!timeCriticalJobs && !(LMIC.opmode & OP_TXRXPEND) && !(LMIC.opmode & OP_TXDATA))
        {
            if (startSeqUp >= LMIC.seqnoUp) {
                logln(F("BUG spotted: seqnoUp not incremented !!!"));
            }
            
            int msToSleep = PROBING_PERIOD_IN_SEC * 1000 - millis();
            log(F("Sleep for "));
            log(msToSleep);
            logln(F(" ms"));

        #ifndef DEBUG
            storeLmic(msToSleep);
            deepSleep(msToSleep);
        #else
            logln("Not deepsleeping.");
            delay(1000 * msToSleep);
            break;
        #endif
        }
    }

}
