#include "NaiveMultiDimensionalSampler.h"


double* buildFilledArray(int dimension, double value) {
    double array[dimension];
    for (int k = 0; k < dimension ; k++) {
        array[k] = value;
    }
    return array;
}

double* buildNanArray(int dimension) {
    return buildFilledArray(dimension, NAN);
}

NaiveMultiDimensionalSampler::NaiveMultiDimensionalSampler(int significantSampleCount, int maxSampleCount, int dimension)
    : AbstractBaseSampler<double*>(significantSampleCount, maxSampleCount) {
    _dimension = dimension;
}

NaiveMultiDimensionalSampler::~NaiveMultiDimensionalSampler() {

}

double* NaiveMultiDimensionalSampler::average(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return buildNanArray(_dimension);
    }

    double* sampleSum = buildFilledArray(_dimension, 0);
    double* sampleCount = buildFilledArray(_dimension, 0);
    for (int k=0; k < _maxSampleCount; k++) {
        for (int d=0; d < _dimension; d++) {
            if (! isnan(_samples[k][d])) {
                sampleSum[d] += _samples[k][d];
                sampleCount[d] ++;
            }
        }
    }

    for (int d=0; d < _dimension; d++) {
        sampleSum[d] = sampleSum[d] / sampleCount[d];
    }
    return sampleSum;
}

double* NaiveMultiDimensionalSampler::min(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return buildNanArray(_dimension);
    }

    double* sampleMin = buildNanArray(_dimension);
    for (int k=0; k < _maxSampleCount; k++) {
        for (int d=0; d < _dimension; d++) {
            if (isnan(sampleMin[d]) || sampleMin[d] > _samples[k][d]) {
                sampleMin[d] = _samples[k][d];
            }
        }
    }

    return sampleMin;
}

double* NaiveMultiDimensionalSampler::max(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return buildNanArray(_dimension);
    }

    double* sampleMax = buildNanArray(_dimension);
    for (int k=0; k < _maxSampleCount; k++) {
        for (int d=0; d < _dimension; d++) {
            if (isnan(sampleMax[d]) || sampleMax[d] < _samples[k][d]) {
                sampleMax[d] = _samples[k][d];
            }
        }
    }

    return sampleMax;
}

double* NaiveMultiDimensionalSampler::median(void){
    // If too few significant values, return NAN.
    if (!isSignificant()) {
        return buildNanArray(_dimension);
    }

    return buildNanArray(_dimension);
}
