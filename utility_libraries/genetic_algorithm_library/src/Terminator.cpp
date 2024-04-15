#include "Terminator.h"

bool Terminator::isTerminationCriteriaMet(std::vector<double>& populationFitnesses)
{
    return terminationCriteria_->isTerminationCriterionMet(populationFitnesses);
}

