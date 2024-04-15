#ifndef PARTHENOGENETICALGORITHM_H
#define PARTHENOGENETICALGORITHM_H

#include <vector>
#include <memory>
#include "Position.h"
#include "Population.h"
#include "PopulationInitialiser.h"
#include "EncodingMechanism.h"
#include "ReproductionMechanism.h"
#include "FitnessFunction.h"
#include "TerminationCriterion.h"
#include "ChromosomeBuilder.h"
#include "Reproducer.h"
#include "FitnessCalculator.h"
#include "Terminator.h"
#include "SequenceEncodingMechanism.h"
#include "IPGAReproductionMechanism.h"
#include "DistanceFitnessFunction.h"
#include "IterationCountTerminationCriterion.h"
#include "ParthenoGeneticAlgorithmConfig.h"

class ParthenoGeneticAlgorithm
{

public:
    ParthenoGeneticAlgorithm(ParthenoGeneticAlgorithmConfig);

    std::vector<int> run(std::vector<Position>& cities, int agents, std::vector<Position>& agentStartPositions);


private:
    std::vector<Population> generations_;
    ChromosomeBuilder chromosomeBuilder_;
    Reproducer reproducer_;
    FitnessCalculator fitnessCalculator_;
    Terminator terminator_;
    std::shared_ptr<EncodingMechanism> encodingMechanism_;
    std::shared_ptr<ReproductionMechanism> reproductionMechanism_;
    std::shared_ptr<FitnessFunction> fitnessFunction_;
    std::shared_ptr<TerminationCriterion> terminationCriterion_;
};

#endif
