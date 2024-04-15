#include "Reproducer.h"

Population Reproducer::Reproduce(
    Population& oldPopulation,
    std::vector<Position>& initialAgentPoses,
    std::vector<Position>& cities)
{
    return reproductionMechanism_->Reproduce(oldPopulation, initialAgentPoses, cities);
}
