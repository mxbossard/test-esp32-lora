#include "Sampler.h"

class NaiveMultiDimensionalSampler: public AbstractBaseSampler<double*> {
    private:
        int _dimension;
        
    public:
        NaiveMultiDimensionalSampler(int significantSampleCount, int maxSampleCount, int dimension);
        ~NaiveMultiDimensionalSampler();

        // void reset(void);
        // void addSample(double value);
        // void reportError(void);
        // int sampleCount(void);
        // int errorCount(void);
        // bool isSignificant(void);
        // bool isFull(void);
        double* median(void);
        double* average(void);
        double* min(void);
        double* max(void);

};
