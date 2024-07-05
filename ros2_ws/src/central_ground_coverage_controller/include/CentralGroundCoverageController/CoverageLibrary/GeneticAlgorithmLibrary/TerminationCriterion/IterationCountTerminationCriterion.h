#ifndef ITERATIONCOUNTTERMINATIONCRITERIA_H
#define ITERATIONCOUNTTERMINATIONCRITERIA_H

#include "TerminationCriterion.h"
#include "Fitnesses.h"
#include <vector>
#include <numeric>
#include <cmath>
#include <map>

class IterationCountTerminationCriterion : public TerminationCriterion
{
private:
    double calculateStdDev(const std::vector<double> &fitnessValues);
    int numberOfIterations_;

public:
    IterationCountTerminationCriterion(int numberOfIterations);
    bool isTerminationCriterionMet(std::vector<std::map<Fitness, double>>& populationFitnesses);
};

#endif
