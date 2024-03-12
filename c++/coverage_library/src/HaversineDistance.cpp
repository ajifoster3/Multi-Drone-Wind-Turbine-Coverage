#include "HaversineDistance.h"
#include <cmath>

namespace HaversineDistance
{
    // Function to convert degrees to radians
    double degToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    // Haversine formula to calculate distance between two points on the Earth
    double calculateDistance(double lat1, double lon1, double lat2, double lon2)
    {
        // Earth's radius in kilometers
        const double R = 6371.0;

        // Convert latitude and longitude from degrees to radians
        lat1 = degToRad(lat1);
        lon1 = degToRad(lon1);
        lat2 = degToRad(lat2);
        lon2 = degToRad(lon2);

        // Differences in coordinates
        double deltaLat = lat2 - lat1;
        double deltaLon = lon2 - lon1;

        // Haversine formula
        double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
                   cos(lat1) * cos(lat2) *
                       sin(deltaLon / 2) * sin(deltaLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double distance = R * c;

        // Convert distance to meters
        return distance * 1000;
    }
}
