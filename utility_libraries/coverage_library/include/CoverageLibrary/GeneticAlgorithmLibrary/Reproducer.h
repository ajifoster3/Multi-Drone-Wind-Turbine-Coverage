#ifndef REPRODUCER_H
#define REPRODUCER_H

#include <memory>
#include "ReproductionMechanism.h"
#include "Population.h"
#include "FitnessCalculator.h"

class Reproducer
{

public:
    Reproducer(){};

    Reproducer(std::shared_ptr<ReproductionMechanism> reproductionMechanism)
        : reproductionMechanism_(std::move(reproductionMechanism)){};

    Population Reproduce(
        Population& oldPopulation,
        std::vector<Position>& initialAgentPoses,
        std::vector<Position>& cities);

private:
    std::shared_ptr<ReproductionMechanism> reproductionMechanism_;
    std::shared_ptr<FitnessCalculator> fitnessCalculator_;
};

#endif
