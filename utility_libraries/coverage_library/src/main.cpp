#include <vector>
#include "Pose.h"
#include "CoverageViewpoint.h"
#include "GreedyIterativeCoveragePathPlanner.h"

int main(int argc, char const *argv[])
{
    std::vector<Pose> initalPoses{Pose{Pose::Position{5.009371, 5.500150, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseClose{Pose{Pose::Position{5.009371, 5.500200, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseFar{Pose{Pose::Position{5.009371, 5.500000, 10}, Pose::Orientation{0,0,0,0}}};
    

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPoseClose, false },
        CoverageViewpoint{coverageViewpointPoseFar, false }
        };
    
    CoveragePathPlanner *planner = new GreedyIterativeCoveragePathPlanner(std::vector<int>{0}, initalPoses, coverageViewpoints);

    return 0;
}
