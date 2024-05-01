#ifndef ITERATIONCOUNTTERMINATIONCRITERIA_H
#define ITERATIONCOUNTTERMINATIONCRITERIA_H

#include "TerminationCriterion.h"
#include <vector>
#include <numeric>
#include <cmath>

class IterationCountTerminationCriterion : public TerminationCriterion
{
private:
    double
    calculateStdDev(const std::vector<double> &fitnessValues);

public:
    IterationCountTerminationCriterion(){};
    bool isTerminationCriterionMet(std::vector<double> &populationFitnesses);
};

#endif
