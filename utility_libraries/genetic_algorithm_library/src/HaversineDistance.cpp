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

     // Function to calculate the drone distance considering altitude and speed constraints
    double calculateDroneHaversineDistance(double lat1, double lon1, double alt1, 
                                           double lat2, double lon2, double alt2, 
                                           double ascending_speed, double descending_speed, double horizontal_speed)
    {
        // Calculate horizontal surface distance using Geodesic
        double surfaceDistance = calculateDistance(lat1, lon1, lat2, lon2);
        
        // Calculate vertical distance (altitude difference)
        double vertical_distance = alt2 - alt1;
        
        // Determine the appropriate vertical speed
        double vertical_speed = (vertical_distance >= 0) ? ascending_speed : descending_speed;
        
        // Calculate the time required for each direction
        double time_vertical = std::abs(vertical_distance) / vertical_speed;
        double time_horizontal = surfaceDistance / horizontal_speed;
        
        // Determine the shorter time (when one direction is completed)
        double common_time = std::min(time_vertical, time_horizontal);
        
        // Calculate the distance covered during the common time
        double vertical_distance_common = vertical_speed * common_time;
        double horizontal_distance_common = horizontal_speed * common_time;
        
        // Calculate the distance for the first segment where both movements occur
        double segment1_distance = std::sqrt(vertical_distance_common * vertical_distance_common + horizontal_distance_common * horizontal_distance_common);
        
        // Calculate the remaining distance in the direction that wasn't completed
        double remaining_distance = 0.0;
        if (time_vertical > time_horizontal) {
            remaining_distance = vertical_speed * (time_vertical - time_horizontal); // remaining vertical distance
        } else if (time_horizontal > time_vertical) {
            remaining_distance = horizontal_speed * (time_horizontal - time_vertical); // remaining horizontal distance
        }
        
        // Total distance is the sum of the segment1 distance and the remaining distance
        return segment1_distance + remaining_distance;
    }
}
