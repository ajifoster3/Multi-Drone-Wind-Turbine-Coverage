#include <gtest/gtest.h>
#include "Chromosome.h"
#include "DistanceFitnessFunction.h"

// TEST(DistanceFitnessFunctionTest, calulateChromosomeFitnessSingleRobot)
// {
//     std::vector<int> genes{0, 1, 2};
//     int numberOfCities = 2;
//     Chromosome chromosome{genes, numberOfCities};

//     std::vector<Position> initalRobotPositions{Position{5.0, 5.0, 5.0}};

//     std::vector<Position> cities{Position{5.0, 5.0, 10.0}, Position{5.0, 5.0, 20.0}};

//     DistanceFitnessFunction fitnessFunction{};
//     double fitness = fitnessFunction.calulateChromosomeFitness(chromosome, initalRobotPositions, cities);
//     std::cerr << "[          ] fitness = " << fitness << std::endl;
//     ASSERT_NEAR(fitness, 0.0333, 0.01);
// }

TEST(DistanceFitnessFunctionTest, CalculateChromosomeFitnessSingleRobotSingleCity)
{
    std::vector<int> genes{0, 1}; // One city, assigned to one robot
    int numberOfCities = 1;
    Chromosome chromosome{genes, numberOfCities};

    std::vector<Position> initialRobotPositions{{10.0, 0.0, 10.0}};
    std::vector<Position> cities{{10.0, 0.0, 0.0}};

    DistanceFitnessFunction fitnessFunction{};
    double fitness = fitnessFunction.calulateChromosomeFitness(chromosome, initialRobotPositions, cities);
    std::cerr << "[          ] fitness = " << fitness << std::endl;
    ASSERT_NEAR(fitness, 0.05, 0.01); // Expect the robot to travel 10 units
}
