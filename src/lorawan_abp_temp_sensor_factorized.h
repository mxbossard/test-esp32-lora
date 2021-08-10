
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

#include "abp_lmic_project_config.h"

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
//#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <ArduinoDebug.h>
DEBUG_INSTANCE(80, Serial);

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

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define ADC_BAT_PIN 35

// Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);

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


// Lorawan credentials in another file
#include "abpPrivateCredentials.h"

#include <lmic_helper.h>
#include <probing.h>


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)


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
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

void deepSleep(int deepsleep_ms) {
    log(F("Entering deepsleep for "), false);
    log(deepsleep_ms, 10, false);
    logln(F(" ms"), false);
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
    u8g2.clearDisplay();
#endif

    //Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    // DBG_DEBUG("Wire began");

    // LMIC init
    os_init();
    DBG_DEBUG("LMIC inited");

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    DBG_DEBUG("LMIC reset");

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
}

void loop() {
    DBG_DEBUG("loop started");
    displayStats();

    int startSeqUp = LMIC.seqnoUp;

    displayStats();

    logLmicStatus();

    bool probingDone = false;
    
    while (true) {
        os_runloop_once();

        if (! probingDone && LMIC.netid != 0) {
            const uint maxProbingTime = (PROBE_MAXIMUM_PROBING_ITERATION + 1) * PROBE_INTERVAL_IN_MS;
            const bool probingNotPossible = os_queryTimeCriticalJobs(ms2osticksRound(maxProbingTime));
            if (!probingNotPossible && !(LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND))) { // 
                DBG_DEBUG("Probing started");
                CayenneLPP lpp = doProbe();
                //doProbe();

                probingDone = true;
                DBG_DEBUG("Probing ended");
                sendLppMessage(lpp, false);

                DBG_DEBUG("Probing done");
            } else {
                //DBG_DEBUG("Probing not possible right now");
                //logLmicStatus();
            }
        }

        displayStats();

        const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((PROBING_PERIOD_IN_SEC * 1000)));
        if (probingDone && !timeCriticalJobs && !(LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND)))
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
