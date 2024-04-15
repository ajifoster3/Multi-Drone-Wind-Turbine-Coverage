#ifndef TERMINATIONCRITERIA_H
#define TERMINATIONCRITERIA_H

#include <vector>

enum TerminationCriteria { ITERATION_COUNT_TERMINATION_CRITERION };

class TerminationCriterion
{
public:
    virtual ~TerminationCriterion() = default;
    virtual bool isTerminationCriterionMet(std::vector<double> &populationFitnesses) = 0;
};

#endif
