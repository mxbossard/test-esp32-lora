
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

#include "otaa_lmic_project_config.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <ArduinoDebug.h>
DEBUG_INSTANCE(80, Serial);

//#define DEBUG
#define LOG_TO_SERIAL
//#define LOG_TO_SCREEN
//#define DISABLE_SCREEN

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

#define GPS_RX 0
#define GPS_TX 4

// Embedded Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);

// Lorawan credentials in another file
#include "otaaPrivateCredentials.h"

#include <utils.h>
#include <log.h>
#include <lmic_helper.h>
#include <probing.h>

#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)

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

/*
For 51 bytes in EU868 with 125kHz BW,
- at SF7:  118ms    => 254 msg / day => 10 msg / hour
- at SF8:  215.6ms  => 139 msg / day =>  5 msg / hour
- at SF9:  390.1ms  =>  76 msg / day =>  3 msg / hour
- at SF10: 698.4ms  =>  42 msg / day
- at SF11: 1560.6ms =>  19 msg / day
- at SF12: 2793.5ms =>  10 msg / day
*/

// TTGO ESP32 Lora pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST_PIN,
    .dio = {LORA_DIO0_PIN, LORA_DIO1_PIN, LORA_DIO2_PIN},
};

void deepSleep(unsigned int deepsleep_ms) {
    displayStats(LMIC, true, deepsleep_ms);

    log(F("Entering deepsleep for "), false);
    log(deepsleep_ms, 10, false);
    logln(F(" ms"), false);

    #ifdef SCREEN
    //u8g2.setPowerSave(true);
    #endif

    //LMIC_shutdown();

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

    //esp_sleep_enable_timer_wakeup(deepsleep_sec * 1000000);
    //esp_deep_sleep_start();
    esp_deep_sleep(1000.0 * deepsleep_ms);
}

void setup() {

    // Disable ADC
    //adc_power_off(); // deprectaed in favor of adc_power_release()

    // Reduce cpu frequency
    //setCpuFrequencyMhz(10);

    logSetup();

    DBG_DEBUG("Starting");

#ifdef DISABLE_SCREEN
    u8g2.setPowerSave(true);
#else
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    //u8g2.clearDisplay();
#endif

    // LMIC init
    os_init();
    DBG_DEBUG("LMIC inited");

    if (RTC_LMIC.devaddr != 0 || RTC_LMIC.opmode & OP_JOINING) {
        restoreLmic();
        DBG_DEBUG("LMIC restored");
    } else {
        // Reset the MAC state. Session and pending data transfers will be discarded.
        LMIC_reset();
        DBG_DEBUG("LMIC reset");
    }

    //initLorawanEepromStorage();
}

// Max TTN payload size for SF10 and greater : 51 bytes.
CayenneLPP lpp(51);

void loop() {
    DBG_DEBUG("loop started");
    displayStats(LMIC);
    DBG_VERBOSE("stats displayed");
    logLmicStatus(LMIC);
    DBG_VERBOSE("LMIC status logged");

    DBG_INFO("Next Job deadline in: %d ms with message length: %d.", osticks2ms(LMIC.osjob.deadline), LMIC.pendTxLen);

    if (LMIC.devaddr == 0 && !(LMIC.opmode & OP_JOINING)) {
        DBG_DEBUG("No Dev addr, not Joining");
        // Not JOINED nor JOINING yet.
        #ifdef SAVE_TO_EEPROM
        if (isSavedLoraWanSession()) {
            restoreLoraWanSession();
        } else {
            LMIC_startJoining();
        }
        #else
        LMIC_startJoining();
        #endif //SAVE_TO_EEPROM
    } else {
        DBG_DEBUG("Dev addr: %x", LMIC.devaddr);
    }

    // Clear previous transmission if not joining
    if (!(LMIC.opmode & OP_JOINING)) {
        LMIC_clrTxData();
    }

    bool probingDone = false;
    bool sendMessageDone = false;
    bool sendMessageFailed = false;
    int smallWorkTimeWindowMs = 1000;
    unsigned long nextSmallWorkCheckTimeMs = 0;
    unsigned long minimalDeepsleepTimeMs = 10000;
    unsigned long probingTime = 0;
    unsigned long probingOutdatedTimeMs = 20000; // Probed data are valid this amount of time.
    unsigned int extraStartingTime = 5000;
    bool sleepingEnabled = true;
    
    while (true) {
        os_runloop_once();
        
        if (millis() > nextSmallWorkCheckTimeMs) {
            if (isLmicNotDoingCriticalJob(smallWorkTimeWindowMs)) {
                DBG_DEBUG("Small time window started");
                displayStats(LMIC);
                logLmicStatus(LMIC);

                if (! probingDone && LMIC.devaddr != 0) {
                    // Probing not done yet and LMIC session acquired
                    const uint maxProbingTime = (PROBE_MAXIMUM_PROBING_ITERATION + 1) * PROBE_INTERVAL_IN_MS;
                    if (isLmicNotDoingCriticalJob(maxProbingTime)) {
                        // Time window available to probe
                        DBG_DEBUG("Probing started");
                        doProbe(lpp);
                        probingTime = millis();
                        DBG_DEBUG("Probing ended");
                        probingDone = true;
                        
                    } else {
                        DBG_DEBUG("Not safe probe during %d ms.", maxProbingTime);
                    }
                }

                if (probingDone && ! sendMessageDone && LMIC_queryTxReady()) {
                    loraStats.probeCounter ++;
                    displayStats(LMIC, true);
                    bool msgSchedule = sendLppMessage(lpp, false);
                    //bool msgSchedule = sendLoraWanMessage(lpp->getBuffer(), lpp->getSize(), false);
                    if (msgSchedule) {
                        sendMessageDone = true;
                    } else {
                        sendMessageFailed = true;
                    }
                }

                if (probingDone && !sendMessageDone) {
                    // Send message not done yet
                    if (millis() < probingTime + probingOutdatedTimeMs) {
                        // Let running the loop for a while if probed data are not oudated
                        DBG_DEBUG("Failed to schedule tx. Will retry.");
                        continue;
                    }
                }

                long deepsleepTime = PROBING_PERIOD_IN_SEC * 1000 - millis();
                if (deepsleepTime < 0) {
                    // Should we produce another probing soon ? What TODO ?
                    deepsleepTime = PROBING_PERIOD_IN_SEC * 1000;
                }
                long timeWindow = deepsleepTime + extraStartingTime;
                if (! isLmicNotDoingCriticalJob(timeWindow)) {
                    // Attempt to sleep a shorter period to save battery
                    DBG_DEBUG("Not safe to sleep for %d ms.", timeWindow);
                    timeWindow = minimalDeepsleepTimeMs + extraStartingTime;
                    if (isLmicNotDoingCriticalJob(timeWindow)) {
                        // We can deepsleep for a minimal time
                        DBG_DEBUG("Safe to sleep for %d ms.", timeWindow);
                        deepsleepTime = minimalDeepsleepTimeMs;
                    } else {
                        // We cannot deepsleep at all
                        DBG_DEBUG("Not safe to sleep for %d ms.", timeWindow);
                        deepsleepTime = 0;
                    }
                }

                if (sleepingEnabled && deepsleepTime > 0) {
                    logLmicStatus(LMIC);

                    DBG_DEBUG("Next Job deadline in: %d ms with message length: %d.", osticks2ms(LMIC.osjob.deadline), LMIC.pendTxLen);

                    DBG_INFO("Will sleep for: %d ms.", deepsleepTime);
                    // log(F("joining: "));
                    // logln(String(LMIC.opmode & OP_JOINING));
                    log(F("Sleep for "));
                    log(deepsleepTime);
                    logln(F(" ms"));

                #ifndef DEBUG
                    storeLmic(deepsleepTime);
                    deepSleep(deepsleepTime);
                #else
                    logln("Not deepsleeping.");
                    delay(deepsleepTime);
                    break;
                #endif
                }




                // Next check after 1 window time
                nextSmallWorkCheckTimeMs = millis() + 1 * smallWorkTimeWindowMs;
            } else {
                // Next check after 1 window time
                nextSmallWorkCheckTimeMs = millis() + 1 * smallWorkTimeWindowMs;
            }
        }
        
    }

}
