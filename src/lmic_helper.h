#ifndef lmic_helper_h
#define lmic_helper_h

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <utils.h>
#include <Preferences.h>
#include <ArduinoDebug.h>
#include <CayenneLPP.h>

#define LORAWAN_SESSION_EEPROM_NS "lorawan-session"

Preferences preferences;

struct LoraStats_t {
    uint32_t dataSendingAttemptCounter;
	//uint32_t dataSentCounter;
    uint32_t joinAttemptCounter;
    uint32_t joinedCounter;
    uint32_t probeCounter;
    uint32_t txCounter;
    uint32_t txAckCounter;
    uint32_t rxCounter;
};

RTC_DATA_ATTR LoraStats_t loraStats;

bool isLmicNotDoingCriticalJob(unsigned int timeInMs) {
    bool isItSafe = !(LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND)) 
        && !os_queryTimeCriticalJobs(ms2osticksRound(timeInMs));
    return isItSafe;
}

static const unsigned long SCREEN_REFRESH_INTERVAL = 1000; // ms
static const unsigned long SCREEN_REFRESH_TIME = 200; // ms
static long lastScreenRefreshTime = millis() - SCREEN_REFRESH_INTERVAL;
void displayStats(struct lmic_t lmic, bool force = false, unsigned int sleepingTime = 0) {
    DBG_VERBOSE("displayStats() began");

    if (! force && millis() <= lastScreenRefreshTime + SCREEN_REFRESH_INTERVAL) return;
    
    String currentDatarate;
    switch(lmic.datarate) {
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

    String currentTxPower = String(lmic.adrTxPow);
    String currentTxChannel = String(lmic.txChnl);
    String probeCounter = String(loraStats.probeCounter);

    String joiningAttempt = String(loraStats.joinAttemptCounter);
    String joined = String(loraStats.joinedCounter);

    String sendingAttempt = String(loraStats.dataSendingAttemptCounter);
    String txAttempt = String(loraStats.txCounter);
    String txAck = String(loraStats.txAckCounter);

    clearJournal();
    journalMessage("DR: " + currentDatarate + " pow: " + currentTxPower + "dbm");
    journalMessage("Probe: " + probeCounter + " chan: " + currentTxChannel);
    journalMessage("joined: " + joined + " / " + joiningAttempt);
    journalMessage("tx: " + txAttempt + " / " + sendingAttempt + " ack: " + txAck);
    if (sleepingTime > 0) {
        journalMessage("sleeping: " + String(sleepingTime/1000) + " sec");
    }
    displayJournal();
    lastScreenRefreshTime = millis();
    DBG_VERBOSE("displayStats() ended");
}

void logLmicStatus(struct lmic_t lmic) {
    DBG_VERBOSE("logLmicStatus() began");

    String opmode = F("");
    if (lmic.opmode & OP_JOINING) opmode += F(" JOINING");
    if (lmic.opmode & OP_LINKDEAD) opmode += F(" LINKDEAD");
    if (lmic.opmode & OP_NEXTCHNL) opmode += F(" NEXTCHNL");
    if (lmic.opmode & OP_PINGABLE) opmode += F(" PINGABLE");
    if (lmic.opmode & OP_PINGINI) opmode += F(" PINGINI");
    if (lmic.opmode & OP_POLL) opmode += F(" POLL");
    if (lmic.opmode & OP_REJOIN) opmode += F(" REJOIN");
    if (lmic.opmode & OP_RNDTX) opmode += F(" RNDTX");
    if (lmic.opmode & OP_SCAN) opmode += F(" SCAN");
    if (lmic.opmode & OP_SHUTDOWN) opmode += F(" SHUTDOWN");
    if (lmic.opmode & OP_TESTMODE) opmode += F(" TESTMODE");
    if (lmic.opmode & OP_TRACK) opmode += F(" TRACK");
    if (lmic.opmode & OP_TXDATA) opmode += F(" TXDATA");
    if (lmic.opmode & OP_TXRXPEND) opmode += F(" TXRXPEND");
    if (lmic.opmode & OP_UNJOIN) opmode += F(" UNJOIN");
    if (lmic.opmode == OP_NONE) opmode = F(" NONE");

    // FIXME next line is buggy. It display: [I] LMIC opmode: @��?␏
    //DBG_INFO("LMIC opmode: %s", opmode);
    log("LMIC opmode: ");
    logln(opmode);
    
    DBG_INFO("LMIC netid: %u ; devaddr: %u", lmic.netid, lmic.devaddr);

    DBG_INFO("LMIC txend: %d ; rxtime: %d ; lbt_ticks: %d ; globalDutyAvail: %d", lmic.txend, lmic.rxtime, lmic.lbt_ticks, lmic.globalDutyAvail);
    
    DBG_VERBOSE("logLmicStatus() ended");
}

void logLoraWanSessionInfos() {
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

bool sendLoraWanMessage(xref2u1_t data, u1_t dlen, bool confirmed = false) {
    bool scheduledMessage = false;
    // Start job
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        logln(F("OP_TXRXPEND, not sending"));
    } else {
        DBG_VERBOSE("sendLoraWanMessage() began");
        char str[dlen];
        array_to_string(data, dlen, str);
        if (confirmed) {
            DBG_VERBOSE("Sending LoraWan message: %s with ack", str);
        } else {
            DBG_VERBOSE("Sending LoraWan message: %s without ack", str);
        }

        // Prepare upstream data transmission at the next possible time.
        lmic_tx_error_t result = LMIC_setTxData2(1, data, dlen, confirmed);
        loraStats.dataSendingAttemptCounter ++;
        if (result == LMIC_ERROR_SUCCESS) {
            u4_t seqnoUp = LMIC.seqnoUp;
            scheduledMessage = true;
            DBG_VERBOSE("Packet queued with next seq up: %d", seqnoUp);
        } else {
            DBG_VERBOSE("Unable to queue message ! Error is: %d", result);
        }
    }
    
    DBG_VERBOSE("sendLoraWanMessage() ended");
    return scheduledMessage;
}

// Saves the LMIC structure during DeepSleep
RTC_DATA_ATTR lmic_t RTC_LMIC;

void updateJobForDeepsleep(osjob_t *job, long deepsleepTicks) {
    if (job != NULL) {
        DBG_DEBUG("Updating a job for deepsleep. Previous deadline: %d", job->deadline);
        job->deadline -= deepsleepTicks;
        DBG_DEBUG("New job deadline: %d", job->deadline);
        updateJobForDeepsleep(job->next, deepsleepTicks);
    }
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
    // RTC_LMIC.osjob.deadline -= deepsleepTicks;
    // RTC_LMIC.osjob.next = NULL;

    // Update all jobs deadline for deepsleep
    updateJobForDeepsleep(&RTC_LMIC.osjob, deepsleepTicks);
    
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
    log("ms - ");
    log(os_getTime(), 10, false);
    log("ticks : ", false);
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
            logLoraWanSessionInfos();
            //saveLoraWanSession();
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


#ifdef SAVE_TO_EEPROM
void initLorawanEepromStorage() {
    preferences.begin(LORAWAN_SESSION_EEPROM_NS, false);
}

void saveLoraWanSession() {
    DBG_VERBOSE("saveLoraWanSession() began");
    u1_t appeui[8];
    u1_t deveui[8];
    u1_t appkey[16];
    os_getArtEui(appeui);
    os_getDevEui(deveui);
    os_getDevKey(appkey);
    preferences.putBytes("APPEUI", appeui, sizeof(appeui));
    preferences.putBytes("DEVEUI", deveui, sizeof(deveui));
    preferences.putBytes("APPKEY", appkey, sizeof(appkey));

    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);

    preferences.putBytes("NWKSKEY", nwkKey, sizeof(nwkKey));
    preferences.putBytes("APPSKEY", artKey, sizeof(artKey));
    preferences.putInt("DEVADDR", devaddr);
    preferences.putInt("NETID", netid);
    DBG_VERBOSE("saveLoraWanSession() ended");
}

bool isSavedLoraWanSession() {
    DBG_VERBOSE("isSavedLoraWanSession() began");
    u1_t appeui[8];
    u1_t deveui[8];
    u1_t appkey[16];
    os_getArtEui(appeui);
    os_getDevEui(deveui);
    os_getDevKey(appkey);

    u1_t savedAppeui[8];
    u1_t savedDeveui[8];
    u1_t savedAppkey[16];
    preferences.getBytes("APPEUI", savedAppeui, sizeof(savedAppeui));
    preferences.getBytes("DEVEUI", savedDeveui, sizeof(savedDeveui));
    preferences.getBytes("APPKEY", savedAppkey, sizeof(savedAppkey));
    u4_t savedNetid = preferences.getInt("NETID", 0);

    DBG_INFO("Saved netid: %d", savedNetid);
    //logln(savedNetid, DEC);

    return savedNetid != 0 
        && memcmp(appeui, savedAppeui, sizeof(appeui)) == 0
        && memcmp(deveui, savedDeveui, sizeof(appeui)) == 0
        && memcmp(appkey, savedAppkey, sizeof(appeui)) == 0;
}

void restoreLoraWanSession() {
    DBG_VERBOSE("restoreLoraWanSession() began");
    u1_t nwkskey[16];
    u1_t appskey[16];
    preferences.getBytes("NWKSKEY", nwkskey, sizeof(nwkskey));
    preferences.getBytes("APPSKEY", appskey, sizeof(appskey));
    devaddr_t devaddr = preferences.getInt("DEVADDR");
    u4_t netid = preferences.getInt("NETID");

    LMIC_setSession (netid, devaddr, nwkskey, appskey);

    //LMIC_setLinkCheckMode(1);
    // TTN uses SF9 for its RX2 window.
    //LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power for uplink
    //LMIC_setDrTxpow(DR_SF7,14);

    DBG_VERBOSE("restoreLoraWanSession() ended");
}
#endif //SAVE_TO_EEPROM

#endif //lmic_helper_h