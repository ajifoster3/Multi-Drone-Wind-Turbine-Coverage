#include "HaversineDistance.h"
#include <GeographicLib/Geodesic.hpp>
#include <cmath>

namespace HaversineDistance
{
    using namespace GeographicLib;

    // Function to calculate distance between two points on the Earth in meters
    double calculateDistance(double lat1, double lon1, double lat2, double lon2)
    {
        // Create a Geodesic object for WGS84 (the most common Earth model)
        const Geodesic& geod = Geodesic::WGS84();
        double distance;

        // Calculate the distance using GeographicLib
        geod.Inverse(lat1, lon1, lat2, lon2, distance);

        return distance; // Distance in meters
    }

    // Calculate distance between two points on the Earth including altitude difference in meters
    double calculateDistance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2)
    {
        double surfaceDistance = calculateDistance(lat1, lon1, lat2, lon2);

        // Difference in altitude in meters
        double deltaAlt = alt2 - alt1;

        // Calculate total distance considering altitude difference
        double totalDistance = sqrt(pow(surfaceDistance, 2) + pow(deltaAlt, 2));

        return totalDistance; // Total distance in meters
    }
}
