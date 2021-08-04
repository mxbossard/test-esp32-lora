#include <utils.h>
#include <config.h>

// BME280 sensor on I2C
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

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

float readADC(uint8_t pin) {
    //logln(F("Reading ADC ..."));
    float ad = 0;
    //float resolution = 3.13 * (2.187+2.200) / 2.187 / 1023; //calibrate based on your voltage divider AND Vref!
    float resolution = 3.2 * 2 * ADC_CORRECTION_RATIO / 4096;
    int adcr = analogRead(pin);
    //logln("Read ADC value: " + String(adcr));
    //logln("Resolution: " + String(resolution, 10) + "mV");
    ad = adcr * resolution;
    //logln(F("Reading ADC done."));

    return ad;
}


void doProbe() {

    int probingDelay = PROBE_INTERVAL_IN_MS;
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
        
        float batVoltage = readADC(ADC_BAT_PIN);
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

    float batVoltageValue = medianOfArray(batVoltageValues, PROBE_MINIMUM_SIGNIFICANT_VALUES);
    float bme280tempValue = medianOfArray(bme280tempValues, PROBE_MINIMUM_SIGNIFICANT_VALUES);
    float bme280HumidityValue = medianOfArray(bme280HumidityValues, PROBE_MINIMUM_SIGNIFICANT_VALUES);
    float bme280PressureValue = medianOfArray(bme280PressureValues, PROBE_MINIMUM_SIGNIFICANT_VALUES);
    // float dht22tempValue = medianOfArray(dht22tempValues, PROBE_MINIMUM_SIGNIFICANT_VALUES);
    // float dht22HumidityValue = medianOfArray(dht22HumidityValues, PROBE_MINIMUM_SIGNIFICANT_VALUES);

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

    logln("Probing done.");
}
