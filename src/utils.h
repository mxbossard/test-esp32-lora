#ifndef utils_h
#define utils_h

#include <Arduino.h>

#include <log.h>

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
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
float medianOfArray(float array[], int significantCount = 1) {
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
    if (valuesCount < significantCount) {
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


#ifndef DISABLE_SCREEN
#include <CircularBuffer.h>
CircularBuffer<String, 5> journalQueue;

void displayJournal() {
#ifdef LOG_TO_SCREEN
    # warning "Cannot displayJournal() if LOG_TO_SCREEN is defined !"
    return;
#endif //LOG_TO_SCREEN

    u8g2.clearDisplay();
    u8g2.clearBuffer();
    int8_t ascent = u8g2.getAscent();
    int8_t maxCharHeight = u8g2.getMaxCharHeight();

    int8_t cursorY = ascent + 1;
    for (int i = journalQueue.size() - 1 ; i >= 0 ; i--) {
        // retrieves the i-th element from the buffer without removing it
        String message = journalQueue[i];
        u8g2.setCursor(0, cursorY);
        u8g2.print(message);
        cursorY += maxCharHeight + 2;
    }
    u8g2.sendBuffer();
}

void clearJournal() {
    journalQueue.clear();
}

void journalMessage(String &message) {
    journalQueue.unshift(message);
}
#else
void displayJournal() {
}
void clearJournal() {
}
void journalMessage(String message) {
}
#endif //DISABLE_SCREEN

#endif // utils_h