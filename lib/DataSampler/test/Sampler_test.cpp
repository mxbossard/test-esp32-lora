#include "gtest/gtest.h"
#include "Sampler.h"

TEST(samplerTest, testInitSampler) {
    Sampler s(3, 5);

    EXPECT_EQ(s.sampleCount(), 0);
    EXPECT_EQ(s.errorCount(), 0);
    EXPECT_EQ(s.isSignificant(), false);
    EXPECT_EQ(s.isFull(), false);
    EXPECT_EQ(s.median(), NAN);
    EXPECT_EQ(s.average(), NAN);
    EXPECT_EQ(s.min(), NAN);
    EXPECT_EQ(s.max(), NAN);
}

TEST(samplerTest, testAddSample) {
    double value = 42.1;
    Sampler s(3, 5);
    s.addSample(value);

    EXPECT_EQ(s.sampleCount(), 1);
    EXPECT_EQ(s.errorCount(), 0);
    EXPECT_EQ(s.isSignificant(), false);
    EXPECT_EQ(s.isFull(), false);
    EXPECT_EQ(s.median(), value);
    EXPECT_EQ(s.average(), value);
    EXPECT_EQ(s.min(), value);
    EXPECT_EQ(s.max(), value);
}
