#include "HaversineDistance.h"
#include <cmath>

namespace HaversineDistance
{
    // Function to convert degrees to radians
    double degToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    // Haversine formula to calculate distance between two points on the Earth in meters
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

    // Calculate distance between two points on the Earth including altitude difference in meters
    double calculateDistance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2)
    {
        double surfaceDistance = calculateDistance(lat1, lon1, lat2, lon2);

        // Difference in altitude remains in meters
        double deltaAlt = alt2 - alt1;

        // Calculate total distance considering altitude difference
        // Using Pythagorean theorem: a^2 + b^2 = c^2, where
        // a = surface distance in meters (converted by multiplying by 1000),
        // b = altitude difference in meters,
        // c = total distance in meters.
        double totalDistance = sqrt(pow(surfaceDistance, 2) + pow(deltaAlt, 2));

        // The total distance is already in meters, so we return it directly
        return totalDistance;
    }
}
