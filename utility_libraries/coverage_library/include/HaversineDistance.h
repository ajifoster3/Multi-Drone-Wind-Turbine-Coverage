#ifndef HAVERSINEDISTANCE_H
#define HAVERSINEDISTANCE_H

#include <cmath>

namespace HaversineDistance
{
    // Function to convert degrees to radians
    double degToRad(double degrees);

    // Haversine formula to calculate distance between two points on the Earth
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);

    double calculateDistance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2);
}

#endif
