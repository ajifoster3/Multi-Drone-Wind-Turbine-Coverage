#ifndef TERMINATOR_H
#define TERMINATOR_H


#include "TerminationCriterion.h"
#include "Fitnesses.h"
#include <map>
#include <vector>
#include <memory>

class Terminator
{
public:
    Terminator(){};
    Terminator(std::shared_ptr<TerminationCriterion> terminationCriteria)
        : terminationCriteria_(std::move(terminationCriteria)){};
    bool isTerminationCriteriaMet(std::vector<std::map<Fitness, double>>& populationFitnesses);

private:
    std::shared_ptr<TerminationCriterion> terminationCriteria_;
};

#endif
