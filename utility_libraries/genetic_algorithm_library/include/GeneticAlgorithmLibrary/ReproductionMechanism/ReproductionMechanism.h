#ifndef REPRODUCTIONMECHANISM_H
#define REPRODUCTIONMECHANISM_H

#include "FitnessCalculator.h"
#include "Population.h"

enum ReproductionMechanisms
{
    IPGA_REPRODUCTION_MECHANISM,
    IPGA_TOURNAMENT_REPRODUCTION_MECHANISM,
    IPGA_ROULETTE_REPRODUCTION_MECHANISM,
    IPGA_HORIZONTAL_REPRODUCTION_MECHANISM,
    NSGAII_REPRODUCTION_MECHANISM,
    NSGAII_REPRODUCTION_MECHANISM_HORIZONTAL,
    NSGAII_PMX_REPRODUCTION_MECHANISM,
    NSGAII_REPRODUCTION_MECHANISM_SHUAI,
    NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL
};

class ReproductionMechanism
{
public:
    ReproductionMechanism(std::shared_ptr<FitnessCalculator> fitnessCalculator) : fitnessCalculator_(std::move(fitnessCalculator)){};

    virtual ~ReproductionMechanism() = default;

    virtual Population Reproduce(
        Population &oldPopulation,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities,
        int iterationNumber) = 0;

protected:
    std::shared_ptr<FitnessCalculator> fitnessCalculator_;
};

#endif
