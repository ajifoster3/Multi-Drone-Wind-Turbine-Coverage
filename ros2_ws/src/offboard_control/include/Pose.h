#ifndef POSE_H
#define POSE_H

struct Pose
{
public:
    struct Position
    {
        double latitude, longitude, altitude;
    };

    struct Orientation
    {
        double x, y, z, w;
    };

    Position position;
    Orientation orientation;
};

#endif // POSE_HPP
