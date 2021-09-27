#include <Arduino.h>

class Sampler {
private:
    /* data */
    int _sampleCount;
    int _significantSampleCount;
    int _maxSampleCount;
    double * _samples;
    int _cursor;
    int _errorCount;

public:
    Sampler(int significantSampleCount, int maxSampleCount);
    ~Sampler();

    void reset(void);
    void addSample(double value);
    void reportError(void);
    int sampleCount(void);
    int errorCount(void);
    bool isSignificant(void);
    bool isFull(void);
    double median(void);
    double average(void);
    double min(void);
    double max(void);

};