#ifndef DATASAMPLER_SAMPLERMANAGER_H
#define DATASAMPLER_SAMPLERMANAGER_H

#include <Arduino.h>
#include <Sampler.h>

class SamplerManager {
private:
    /* data */
    int _sampleCount;
    int _significantSampleCount;
    int _maxSampleCount;
    double * _samples;
    int _cursor;
    int _errorCount;

public:
    SamplerManager();
    ~SamplerManager();

    void reset(void);
    void addSampler(Sampler sampler);

};

#endif //DATASAMPLER_SAMPLERMANAGER_H