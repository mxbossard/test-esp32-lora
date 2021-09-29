#include "Sampler.h"


int sortDoubleAsc(const void *cmp1, const void *cmp2) {
    // Need to cast the void * to float *
    double a = *((float *)cmp1);
    double b = *((float *)cmp2);

    // NAN values get pushed at the end of the array
    if (isnan(a)) return 1;
    if (isnan(b)) return -1;

    // The comparison
    //return a > b ? 1 : (a < b ? -1 : 0);
    // A simpler, probably faster way:
    return a - b;
}

/**
 * Return the median of a double array.
 * Exclude NAN values.
 * return a NAN value if less than PROBE_MINIMUM_SIGNIFICANT_VALUES non NAN values in the array.
 */
/*
double medianOfArray(double array[], int significantCount = 1) {
    int arrayLength = sizeof(array) - 1;
    int arraySize = arrayLength * sizeof(array[0]);
    float buffer[arrayLength];
    memcpy(buffer, array, arraySize);
    
    // qsort - last parameter is a function pointer to the sort function
    qsort(buffer, arrayLength, sizeof(array[0]), sortDoubleAsc);
    
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
    double medianValue = buffer[medianIndex];
    //logln("Median value: " + String(medianValue) + " from " + String(valuesCount) + " values.", false);
    return medianValue;
}
*/

template<typename T>
AbstractBaseSampler<T>::AbstractBaseSampler(int significantSampleCount, int maxSampleCount) {
    _significantSampleCount = significantSampleCount;
    _maxSampleCount = maxSampleCount;
    _samples = (T*) malloc(maxSampleCount*sizeof(T));
    reset();
}

template<typename T>
AbstractBaseSampler<T>::~AbstractBaseSampler() {
    free(_samples);
}

template<typename T>
void AbstractBaseSampler<T>::reset(void) {
    _sampleCount = 0;
    _cursor = 0;
    for (int k=0; k < _maxSampleCount; k++) {
        _samples[k] = NAN;
    }
    _errorCount = 0;
}

template<typename T>
void AbstractBaseSampler<T>::addSample(T value){
    if (! isFull()) {
        _samples[_cursor] = value;
        _cursor ++;
        if (! isnan(value)) {
            _sampleCount ++;
        }
    } else {
        // TODO: throw an error ?
    }
}

template<typename T>
void AbstractBaseSampler<T>::reportError(void) {
    _errorCount ++;
}

template<typename T>
int AbstractBaseSampler<T>::sampleCount(void) {
    return _cursor;
}

template<typename T>
int AbstractBaseSampler<T>::errorCount(void) {
    return _errorCount;
}

template<typename T>
bool AbstractBaseSampler<T>::isSignificant(void){
    return _sampleCount >= _significantSampleCount;
}

template<typename T>
bool AbstractBaseSampler<T>::isFull(void){
    return _cursor >= _maxSampleCount;
}

DoubleSampler::DoubleSampler(int significantSampleCount, int maxSampleCount)
    : AbstractBaseSampler<double>(significantSampleCount, maxSampleCount) {

}

DoubleSampler::~DoubleSampler() {

}

double DoubleSampler::median(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return NAN;
    }

    int arrayLength = _maxSampleCount;
    int arraySize = arrayLength * sizeof(_samples[0]);
    double buffer[arrayLength];
    memcpy(buffer, _samples, arraySize);
    
    // qsort - last parameter is a function pointer to the sort function
    qsort(buffer, arrayLength, sizeof(buffer[0]), sortDoubleAsc);

    unsigned int medianIndex = (_sampleCount + 1) / 2;
    double medianValue = buffer[medianIndex];
    //logln("Median value: " + String(medianValue) + " from " + String(valuesCount) + " values.", false);
    return medianValue;
}

double DoubleSampler::average(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return NAN;
    }

    double sampleSum = 0;
    for (int k=0; k < _maxSampleCount; k++) {
        if (! isnan(_samples[k])) {
            sampleSum += _samples[k];
        }
    }
    return sampleSum / _sampleCount;
}

double DoubleSampler::min(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return NAN;
    }

    double sampleMin = NAN;
    for (int k=0; k < _maxSampleCount; k++) {
        if (isnan(sampleMin) || sampleMin > _samples[k]) {
            sampleMin = _samples[k];
        }
    }
    return sampleMin;
}

double DoubleSampler::max(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return NAN;
    }

    double sampleMax = NAN;
    for (int k=0; k < _maxSampleCount; k++) {
        if (isnan(sampleMax) || sampleMax < _samples[k]) {
            sampleMax = _samples[k];
        }
    }
    return sampleMax;
}
