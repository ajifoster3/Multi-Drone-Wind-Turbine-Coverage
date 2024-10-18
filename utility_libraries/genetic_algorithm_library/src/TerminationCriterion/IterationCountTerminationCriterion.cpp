#include "IterationCountTerminationCriterion.h"
#include <iostream>

IterationCountTerminationCriterion::IterationCountTerminationCriterion(int numberOfIterations)
: numberOfIterations_(numberOfIterations)
{
}

bool IterationCountTerminationCriterion::isTerminationCriterionMet(std::vector<std::map<Fitness, double>>& populationFitnesses)
{
    auto popSize = populationFitnesses.size();
    return populationFitnesses.size() > numberOfIterations_;
    // double stdDev = calculateStdDev(populationFitnesses);
    // return stdDev > 0.1;
}

// double IterationCountTerminationCriterion::calculateStdDev(const std::vector<double> &fitnessValues)
// {
//     if (fitnessValues.size() < 500)
//     {
//         return false;
//     }
//     double sum = std::accumulate(fitnessValues.begin(), fitnessValues.end(), 0.0);
//     double mean = sum / fitnessValues.size();

//     double squareSum = std::inner_product(
//         fitnessValues.begin(), fitnessValues.end(), fitnessValues.begin(), 0.0,
//         [](double a, double b)
//         { return a + b; },
//         [mean](double a, double b)
//         { return (a - mean) * (b - mean); });

//     return std::sqrt(squareSum / fitnessValues.size());
// }
