#include <gtest/gtest.h>
#include "HaversineDistance.h"

TEST(HaversineDistanceTest, DegToRadCorrectValue)
{
    EXPECT_NEAR(0.017453, HaversineDistance::degToRad(1.0), 0.00005);
    EXPECT_NEAR( 6.300638, HaversineDistance::degToRad(361.0), 0.000005);
    EXPECT_NEAR(-17.453292, HaversineDistance::degToRad(-1000.0), 0.000005);
}

TEST(HaversineDistanceTest, calculateDistanceCorrectValue)
{
    EXPECT_NEAR(7485487, HaversineDistance::calculateDistance(49.243824, 121.887340, 49.235347, -121.92532),5);
    EXPECT_NEAR(1861239, HaversineDistance::calculateDistance(57.985270, 54.369198, 54.470544, 84.076228), 5);
    EXPECT_NEAR(1861237, HaversineDistance::calculateDistance(-57.985270, -54.369198, -54.470544, -84.076228), 5);
    EXPECT_NEAR(17458899, HaversineDistance::calculateDistance(-57.985270, 54.369198, 54.470544, -84.076228), 5);
    EXPECT_NEAR(12786310, HaversineDistance::calculateDistance(57.985270, 54.369198, -54.470544, 84.076228), 5);
}

TEST(HaversineDistanceTest, calculateDistanceWithAltitudeCorrectValue)
{
    EXPECT_NEAR(7485487, HaversineDistance::calculateDistance(49.243824, 121.887340, 0, 49.235347, -121.92532, 0),5);
    EXPECT_NEAR(1861239, HaversineDistance::calculateDistance(57.985270, 54.369198, 0, 54.470544, 84.076228, 1000), 5);
    EXPECT_NEAR(1861237, HaversineDistance::calculateDistance(-57.985270, -54.369198, 999, -54.470544, -84.076228, 22), 5);
}
