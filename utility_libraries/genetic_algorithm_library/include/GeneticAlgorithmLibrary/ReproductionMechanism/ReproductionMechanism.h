#ifndef REPRODUCTIONMECHANISM_H
#define REPRODUCTIONMECHANISM_H

#include "FitnessCalculator.h"
#include "Population.h"

enum ReproductionMechanisms
{
    IPGA_REPRODUCTION_MECHANISM,
    IPGA_ELITISM_REPRODUCTION_MECHANISM,
    IPGA_HORIZONTAL_REPRODUCTION_MECHANISM,
    NSGAII_REPRODUCTION_MECHANISM
};

class ReproductionMechanism
{
public:
    ReproductionMechanism(std::shared_ptr<FitnessCalculator> fitnessCalculator) : fitnessCalculator_(std::move(fitnessCalculator)){};

    virtual ~ReproductionMechanism() = default;

    virtual Population Reproduce(
        Population &oldPopulation,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities) = 0;

protected:
    std::shared_ptr<FitnessCalculator> fitnessCalculator_;
};

#endif
