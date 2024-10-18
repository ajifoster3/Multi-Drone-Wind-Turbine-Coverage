#include "Terminator.h"

bool Terminator::isTerminationCriteriaMet(std::vector<std::map<Fitness, double>>& populationFitnesses)
{
    return terminationCriteria_->isTerminationCriterionMet(populationFitnesses);
}

