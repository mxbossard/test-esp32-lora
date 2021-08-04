
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
//#include <CircularBuffer.h>

#include <U8g2lib.h>
//#include <Adafruit_BME280.h>
#include <CayenneLPP.h>

//#define DEBUG
#define LOG_TO_SERIAL
//#define LOG_TO_SCREEN
#define SCREEN

//#define TEST_JOIN_KO_BAD_APPEUI

#ifdef DEBUG
    #define LOG_TO_SERIAL
#endif

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define LORA_NSS_PIN 18
#define LORA_RST_PIN 23
#define LORA_DIO0_PIN 26
#define LORA_DIO1_PIN 33
#define LORA_DIO2_PIN 32

#include <probing.h>

#ifdef SCREEN
// Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);
#endif

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
#include "otaaPrivateCredentials.h"


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
#ifdef TEST_JOIN_KO_BAD_APPEUI
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#else
static const u1_t PROGMEM APPEUI[8] = LORAWAN_APPEUI;
#endif
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = LORAWAN_DEVEUI;
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = LORAWAN_APPKEY;
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

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
    .nss = LORA_NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST_PIN,
    .dio = {LORA_DIO0_PIN, LORA_DIO1_PIN, LORA_DIO2_PIN},
};

struct LoraStats_t {
    uint32_t dataSendingAttemptCounter;
	//uint32_t dataSentCounter;
    uint32_t joinAttemptCounter;
    uint32_t joinedCounter;
    uint32_t txCounter;
    uint32_t txAckCounter;
    uint32_t rxCounter;
};

RTC_DATA_ATTR LoraStats_t loraStats;

#ifdef SCREEN
CircularBuffer<String, 5> loggingQueue;

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
#else
void displayLoggingQueue() {
}
#endif


static const unsigned long SCREEN_REFRESH_INTERVAL = 1000; // ms
static unsigned long lastScreenRefreshTime = millis() - SCREEN_REFRESH_INTERVAL;
uint sleepingTime = 0;

void displayStats(bool force = false) {
    if (! force && millis() <= lastScreenRefreshTime + SCREEN_REFRESH_INTERVAL) return;

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

    String joiningAttempt = String(loraStats.joinAttemptCounter);
    String joined = String(loraStats.joinedCounter);

    String sendingAttempt = String(loraStats.dataSendingAttemptCounter);
    String txAttempt = String(loraStats.txCounter);
    String txAck = String(loraStats.txAckCounter);

    loggingQueue.clear();
    loggingQueue.unshift("DR: " + currentDatarate + " pow: " + currentTxPower + "dbm");
    loggingQueue.unshift("chan: " + currentTxChannel);
    loggingQueue.unshift("joined: " + joined + " / " + joiningAttempt);
    loggingQueue.unshift("tx: " + txAttempt + " / " + sendingAttempt + " ack: " + txAck);
    if (sleepingTime > 0) {
        loggingQueue.unshift("sleeping: " + String(sleepingTime/1000) + " sec");
    }
    displayLoggingQueue();
    lastScreenRefreshTime = millis();
}

void deepSleep(u_int deepsleep_ms) {
    log(F("Entering deepsleep for "), false);
    log(deepsleep_ms, 10, false);
    logln(F(" ms"), false);

    #ifdef SCREEN
    //u8g2.setPowerSave(true);
    #endif

    LMIC_shutdown();

    pinMode(I2C_SDA_PIN, INPUT); 
    pinMode(I2C_SCL_PIN, INPUT); 
    pinMode(ADC_BAT_PIN, INPUT); 
    pinMode(LORA_NSS_PIN, INPUT); 
    pinMode(LORA_RST_PIN, INPUT); 
    pinMode(LORA_DIO0_PIN, INPUT); 
    pinMode(LORA_DIO1_PIN, INPUT); 
    pinMode(LORA_DIO2_PIN, INPUT); 
    //delay(100);
    gpio_deep_sleep_hold_en();

    sleepingTime = deepsleep_ms;
    displayStats(true);

    //esp_sleep_enable_timer_wakeup(deepsleep_sec * 1000000);
    //esp_deep_sleep_start();
    esp_deep_sleep(1000.0 * deepsleep_ms);
}

void storeLmic(int deepsleep_ms) {
    logln(F("Storing LMIC into RTC"), false);
    if (&LMIC.osjob != NULL) {
        log(F("Next job at: "));
        logln(String(LMIC.osjob.deadline));
    }

    RTC_LMIC = LMIC;

    // ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
    // Therefore reset DutyCyles

    unsigned long now = millis();


    // Update first job deadline for deepsleep
    // FIXME: update all job deadlines
    long deepsleepTicks = (deepsleep_ms / 1000.0) * OSTICKS_PER_SEC;
    RTC_LMIC.osjob.deadline -= deepsleepTicks;
    RTC_LMIC.osjob.next = NULL;
    
    // Move timings in passed for next deepsleep wakening
    long tickShift = ((now / 1000.0 + deepsleep_ms / 1000.0) * OSTICKS_PER_SEC);
    RTC_LMIC.txend -= tickShift;
    RTC_LMIC.rxtime -= tickShift;

    // EU Like Bands
#if defined(CFG_LMIC_EU_like)
    logln(F("Reset CFG_LMIC_EU_like bands"), false);
    for (int i = 0; i < MAX_BANDS; i++) {
        ostime_t correctedAvail = RTC_LMIC.bands[i].avail - tickShift;
        if (correctedAvail < 0) {
            correctedAvail = 0;
        }
        RTC_LMIC.bands[i].avail = correctedAvail;
    }

    RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - tickShift;
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
    os_setTimedCallback(&LMIC.osjob, LMIC.osjob.deadline, LMIC.osjob.func);
    log(F("seq up: "));
    logln(String(LMIC.seqnoUp));
    if (&LMIC.osjob != NULL) {
        log(F("next job at: "));
        logln(String(LMIC.osjob.deadline));
    }
}


void onEvent (ev_t ev) {
    log(millis(), 10, false);
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
            loraStats.joinAttemptCounter ++;
            logln(F("EV_JOINING"));
            break;
        case EV_JOINED:
            logln(F("EV_JOINED"));
            loraStats.joinedCounter ++;
            {
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                log("netid: ");
                logln(netid, DEC);
                log("devaddr: ");
                logln(devaddr, HEX);
                log("AppSKey: ");
                for (size_t i=0; i<sizeof(artKey); ++i) {
                    if (i != 0)
                        log("-");
                    printHex2(artKey[i]);
                }
                logln("", false);
                log("NwkSKey: ");
                for (size_t i=0; i<sizeof(nwkKey); ++i) {
                    if (i != 0)
                        log("-");
                    printHex2(nwkKey[i]);
                }
                logln("", false);
            }            
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
            loraStats.txCounter ++;
            logln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                loraStats.txAckCounter ++;
                logln(F("Received ack"));
            }
            if (LMIC.dataLen) {
                log(F("Received "));
                log(LMIC.dataLen);
                logln(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            logln(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            logln(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            loraStats.rxCounter ++;
            logln(F("EV_RXCOMPLETE"));
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
            break;
        case EV_TXCANCELED:
            logln(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            logln(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            log(F("Unknown event: "));
            logln((unsigned) ev);
            break;
    }
}

void setup() {

    // Disable ADC
    //adc_power_off(); // deprectaed in favor of adc_power_release()

    // Reduce cpu frequency
    //setCpuFrequencyMhz(10);

    #ifdef LOG_TO_SERIAL
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    #endif

    #ifdef SCREEN
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearDisplay();
    #else
    u8g2.setPowerSave(true);
    #endif

    logln(F("Starting"));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // LMIC init
    os_init();

    if (RTC_LMIC.seqnoUp != 0 || RTC_LMIC.opmode & OP_JOINING) {
        restoreLmic();
    } else {
        // Reset the MAC state. Session and pending data transfers will be discarded.
        LMIC_reset();
    }

}

void sendLppMessage(bool confirmed = false) {
    // Start job
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        logln(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        lmic_tx_error_t result = LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), confirmed);
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
    loraStats.dataSendingAttemptCounter ++;
}

void logLmicStatus() {
    log("LMIC status:");
    if (LMIC.opmode & OP_JOINING) log(" JOINING");
    if (LMIC.opmode & OP_LINKDEAD) log(" LINKDEAD");
    if (LMIC.opmode & OP_NEXTCHNL) log(" NEXTCHNL");
    if (LMIC.opmode & OP_NONE) log(" NONE");
    if (LMIC.opmode & OP_PINGABLE) log(" PINGABLE");
    if (LMIC.opmode & OP_PINGINI) log(" PINGINI");
    if (LMIC.opmode & OP_POLL) log(" POLL");
    if (LMIC.opmode & OP_REJOIN) log(" REJOIN");
    if (LMIC.opmode & OP_RNDTX) log(" RNDTX");
    if (LMIC.opmode & OP_SCAN) log(" SCAN");
    if (LMIC.opmode & OP_SHUTDOWN) log(" SHUTDOWN");
    if (LMIC.opmode & OP_TESTMODE) log(" TESTMODE");
    if (LMIC.opmode & OP_TRACK) log(" TRACK");
    if (LMIC.opmode & OP_TXDATA) log(" TXDATA");
    if (LMIC.opmode & OP_TXRXPEND) log(" TXRXPEND");
    if (LMIC.opmode & OP_UNJOIN) log(" UNJOIN");
    logln("");

    log("LMIC txend: ");
    log(LMIC.txend);
    log(" ; rxtime: ");
    log(LMIC.rxtime);
    log(" ; lbt_ticks: ");
    log(LMIC.lbt_ticks);
    log(" ; globalDutyAvail: ");
    log(LMIC.globalDutyAvail);
    logln("");
}

void loop() {
    displayStats();

    logLmicStatus();

    if (LMIC.netid == 0 && !(LMIC.opmode & OP_JOINING)) {
        // Not JOINED yet.
        LMIC_startJoining();
    }

    // Clear previous transmission
    if (!(LMIC.opmode & OP_JOINING)) {
        LMIC_clrTxData();
    }

    bool workDone = false;

    while (true) {
        os_runloop_once();

        if (! workDone && LMIC.netid != 0) {
            doProbe();

            const uint maxProbingTime = (PROBE_MAXIMUM_PROBING_ITERATION + 1) * PROBE_INTERVAL_IN_MS;
            const bool probingNotPossible = os_queryTimeCriticalJobs(ms2osticksRound(maxProbingTime));
            if (!probingNotPossible && !(LMIC.opmode & OP_TXRXPEND)) { // 
                sendLppMessage(false);
                workDone = true;
            }
        }

        const bool refreshScreenNotPossible = os_queryTimeCriticalJobs(ms2osticksRound(1 * 1000));
        if (!refreshScreenNotPossible && !(LMIC.opmode & OP_TXRXPEND)) { // 
            displayStats();
        }

        long deepsleepTime = PROBING_PERIOD_IN_SEC * 1000 - millis();
        if (deepsleepTime < 0) {
            deepsleepTime = PROBING_PERIOD_IN_SEC * 1000;
        }
        bool deepsleepNotPossible = os_queryTimeCriticalJobs(ms2osticksRound(deepsleepTime + 20000));
        if (deepsleepNotPossible) {
            // Attempt to sleep a shorter period to save battery
            deepsleepNotPossible = os_queryTimeCriticalJobs(ms2osticksRound(10000 + 2000));
            if (!deepsleepNotPossible) {
                deepsleepTime = 10000;
            }
        }

        if (! deepsleepNotPossible && !(LMIC.opmode & OP_TXRXPEND)) { //  && !(LMIC.opmode & OP_TXRXPEND) && !(LMIC.opmode & OP_TXDATA)
            // log(F("joining: "));
            // logln(String(LMIC.opmode & OP_JOINING));
            log(F("Sleep for "));
            log(deepsleepTime);
            logln(F(" ms"));

            logLmicStatus();

        #ifndef DEBUG
            storeLmic(deepsleepTime);
            deepSleep(deepsleepTime);
        #else
            logln("Not deepsleeping.");
            delay(deepsleepTime);
            break;
        #endif
        }
    }

}
