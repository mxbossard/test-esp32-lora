#ifndef DATASAMPLER_SAMPLER_H
#define DATASAMPLER_SAMPLER_H

#include <Arduino.h>

template<typename T>
class Sampler {
    public:
        // pure virtual function
        virtual void reset(void) = 0;
        virtual void addSample(T value);
        virtual void reportError(void) = 0;
        virtual int sampleCount(void) = 0;
        virtual int errorCount(void) = 0;
        virtual bool isSignificant(void) = 0;
        virtual bool isFull(void) = 0;
        virtual T median(void) = 0;
        virtual T average(void) = 0;
        virtual T min(void) = 0;
        virtual T max(void) = 0;
};

template<typename T>
class AbstractBaseSampler: public Sampler<T> {
    protected:
        /* data */
        int _sampleCount;
        int _significantSampleCount;
        int _maxSampleCount;
        T * _samples;
        int _cursor;
        int _errorCount;

    public:
        AbstractBaseSampler(int significantSampleCount, int maxSampleCount);
        ~AbstractBaseSampler();

        void reset(void);
        void addSample(T value);
        void reportError(void);
        int sampleCount(void);
        int errorCount(void);
        bool isSignificant(void);
        bool isFull(void);
        virtual T median(void) = 0;
        virtual T average(void) = 0;
        virtual T min(void) = 0;
        virtual T max(void) = 0;
};

class DoubleSampler: public AbstractBaseSampler<double> {
    private:

    public:
        DoubleSampler(int significantSampleCount, int maxSampleCount);
        ~DoubleSampler();

        double median(void);
        double average(void);
        double min(void);
        double max(void);

};

#endif //DATASAMPLER_SAMPLER_H