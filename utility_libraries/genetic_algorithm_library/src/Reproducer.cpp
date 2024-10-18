#include "Reproducer.h"
#include <iostream>

Population Reproducer::Reproduce(
    Population& oldPopulation,
    std::vector<Position>& initialAgentPoses,
    std::vector<Position>& cities,
    int iterationNumber)
{
    return reproductionMechanism_->Reproduce(oldPopulation, initialAgentPoses, cities, iterationNumber);
}
