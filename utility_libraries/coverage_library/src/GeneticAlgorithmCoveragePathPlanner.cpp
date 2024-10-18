#include "GeneticAlgorithmCoveragePathPlanner.h"
#include <iostream>

GeneticAlgorithmCoveragePathPlanner::GeneticAlgorithmCoveragePathPlanner(
    const std::vector<int> &robotIDs,
    const std::vector<Pose> &initialPoses,
    std::vector<CoverageViewpoint> &viewpoints)
    : CoveragePathPlanner()
{
    this->robotIDs = robotIDs;
    this->initialRobotPoses = initialPoses;
    this->viewpoints = GoalCoverageViewpoints(viewpoints);
    this->coveragePaths = CoveragePaths(robotIDs);

    planCoveragePath();
}

void GeneticAlgorithmCoveragePathPlanner::planCoveragePath()
{
    ParthenoGeneticAlgorithmConfig config{
        EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
        ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM,
        FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
        TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION};

    ParthenoGeneticAlgorithm pga{config};
    auto cities = viewpoints.getViewpointPositions();
    std::vector<Position> initalrobotPositions;
    initalrobotPositions.resize(initialRobotPoses.size());
    std::transform(
        initialRobotPoses.begin(),
        initialRobotPoses.end(),
        initalrobotPositions.begin(),
        [](const Pose &pose) -> Position
        { return Position{
              pose.position.latitude,
              pose.position.longitude,
              pose.position.altitude}; });

    std::vector<Position> pgaCities;
    pgaCities.resize(cities.size());
    std::transform(
        cities.begin(),
        cities.end(),
        pgaCities.begin(),
        [](const Pose::Position &position) -> Position
        { return Position{
              position.latitude,
              position.longitude,
              position.altitude}; });

    auto bestChromosome = pga.run(pgaCities, initalrobotPositions.size(), initalrobotPositions);

    auto routes = extractRoutes(bestChromosome, initalrobotPositions.size());
    for (int i = 0; i < initalrobotPositions.size(); i++)
    {
        for (auto &&viewpoint : routes[i])
        {
            coveragePaths.addCoverageViewpointForRobot(robotIDs[i], viewpoints.getViewpointAtIndex(viewpoint));
        }
        auto initialViewpoint = CoverageViewpoint(this->initialRobotPoses[i], 1);
        coveragePaths.addCoverageViewpointForRobot(robotIDs[i], initialViewpoint);
    }
}

std::vector<std::vector<int>> GeneticAlgorithmCoveragePathPlanner::extractRoutes(const std::vector<int> &bestChromosome, int numberOfRobots)
{
    
    std::vector<std::vector<int>> routes(numberOfRobots);
    std::vector<int> routeLengths(bestChromosome.end() - numberOfRobots, bestChromosome.end());
    
    int currentPositionIndex = 0;
    for (int i = 0; i < numberOfRobots; ++i)
    {
        // Extract positions for each robot based on its route length
        for (int j = 0; j < routeLengths[i]; ++j)
        {
            routes[i].push_back(bestChromosome[currentPositionIndex++]);
        }
    }
    return routes;
}
