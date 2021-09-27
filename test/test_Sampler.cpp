#include "Sampler.h"
#include <unity.h>

void test_init_sampler(void) {
    Sampler s(3, 5);

    TEST_ASSERT_EQUAL(0, s.sampleCount());
    TEST_ASSERT_EQUAL(0, s.errorCount());
    TEST_ASSERT_EQUAL(false, s.isSignificant());
    TEST_ASSERT_EQUAL(false, s.isFull());
    TEST_ASSERT_EQUAL(NAN, s.median());
    TEST_ASSERT_EQUAL(NAN, s.average());
    TEST_ASSERT_EQUAL(NAN, s.min());
    TEST_ASSERT_EQUAL(NAN, s.max());
}

void test_addSample(void) {
    double value = 42.1;
    Sampler s(3, 5);
    s.addSample(value);

    TEST_ASSERT_EQUAL(1, s.sampleCount());
    TEST_ASSERT_EQUAL(0, s.errorCount());
    TEST_ASSERT_EQUAL(false, s.isSignificant());
    TEST_ASSERT_EQUAL(false, s.isFull());
    TEST_ASSERT_EQUAL(value, s.median());
    TEST_ASSERT_EQUAL(value, s.average());
    TEST_ASSERT_EQUAL(value, s.min());
    TEST_ASSERT_EQUAL(value, s.max());
}

void main() {
    test_init_sampler();
    test_addSample();
}