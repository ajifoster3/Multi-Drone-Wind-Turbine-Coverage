#ifndef TERMINATIONCRITERIA_H
#define TERMINATIONCRITERIA_H

#include <vector>
#include <map>
#include "Fitnesses.h"

enum TerminationCriteria { ITERATION_COUNT_TERMINATION_CRITERION };

class TerminationCriterion
{
public:
    virtual ~TerminationCriterion() = default;
    virtual bool isTerminationCriterionMet(std::vector<std::map<Fitness, double>>& populationFitnesses) = 0;
};

#endif
