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


Sampler::Sampler(int significantSampleCount, int maxSampleCount) {
    _significantSampleCount = significantSampleCount;
    _maxSampleCount = maxSampleCount;
    _samples = (double *)malloc(maxSampleCount);
    reset();
}

Sampler::~Sampler() {
    free(_samples);
}

void Sampler::reset(void) {
    _sampleCount = 0;
    _cursor = 0;
    for (int k=0; k < _maxSampleCount; k++) {
        _samples[k] = NAN;
    }
    _errorCount = 0;
}

void Sampler::addSample(double value){
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

void Sampler::reportError(void) {
    _errorCount ++;
}

int Sampler::sampleCount(void) {
    return _cursor;
}

int Sampler::errorCount(void) {
    return _errorCount;
}

bool Sampler::isSignificant(void){
    return _sampleCount >= _significantSampleCount;
}

bool Sampler::isFull(void){
    return _cursor >= _maxSampleCount;
}

double Sampler::median(void){
    if (_sampleCount == 0) return NAN;

    return medianOfArray(_samples);
}

double Sampler::average(void){
    if (_sampleCount == 0) return NAN;

    double sampleSum = 0;
    for (int k=0; k < _maxSampleCount; k++) {
        if (! isnan(_samples[k])) {
            sampleSum += _samples[k];
        }
    }
    return sampleSum / _sampleCount;
}

double Sampler::min(void){
    if (_sampleCount == 0) return NAN;

    double sampleMin = NAN;
    for (int k=0; k < _maxSampleCount; k++) {
        if (isnan(sampleMin) || sampleMin > _samples[k]) {
            sampleMin = _samples[k];
        }
    }
    return sampleMin;
}

double Sampler::max(void){
    if (_sampleCount == 0) return NAN;

    double sampleMax = NAN;
    for (int k=0; k < _maxSampleCount; k++) {
        if (isnan(sampleMax) || sampleMax < _samples[k]) {
            sampleMax = _samples[k];
        }
    }
    return sampleMax;
}
