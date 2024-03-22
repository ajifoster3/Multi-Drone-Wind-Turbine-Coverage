#ifndef GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H
#define GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H

#include <vector>
#include <limits>
#include <cmath>
#include "CoverageViewpoint.h"
#include "CoveragePath.h" // Include the CoveragePath header
#include "HaversineDistance.h"

/**
 * @brief Logic for greedy path planning.
 * 
 * @details 
 * Contains the logic for planning coverage paths for a team of 
 * robots given their starting positions and a list of vector of 
 * goal viewpoints.
 * 
*/
class GreedyIterativeCoveragePathPlanner {
public:

    /**
     * @brief Construct the planner and performs planning.
     *
     * @details 
     * Assigns respective parameters, and plans the coverage path.
     *
     * @param robotIDs A list of integers representing the IDs of the robots in the team.
     * @param initalPoses A list of poses representing the initial poses of the robots. 
     * @param viewpoints A list of CoverageViewpoints representing the points to be covered.
    */
    GreedyIterativeCoveragePathPlanner(const std::vector<int>& robotIDs, const std::vector<Pose>& initialPoses, std::vector<CoverageViewpoint>& viewpoints);

    /**
     * @brief Simple getter of coveragePaths.
     * 
     * @return A list of coverage paths.
    */
    std::vector<CoveragePath> getCoveragePaths() const;

private:
    std::vector<int> robotIDs;
    std::vector<Pose> robotPoses;
    std::vector<CoverageViewpoint>& viewpoints;
    std::vector<CoveragePath> coveragePaths; // Store a CoveragePath for each robot

    /**
     * @brief Plans coverage in a iterative greedy manner.
     * 
     * @details 
     * For each robot in the team assign the closest viewpoint,
     * looping through the robots until all viewpoints are covered,
     * and assign coveragePaths with the result.
    */
    void planCoveragePath();

    /**
     * @brief Returns the index of the closest viewpoint to the given pose.
     * 
     * @details 
     * Uses the haversine distance to find the closest unassigned viewpoint
     * to the given pose.
     * 
     * @param pose The subject pose
     * 
     * @return the index of viewpoint assigned
    */
    int findClosestUnassignedViewpointIndex(const Pose& pose);
};

#endif // GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_HPP
