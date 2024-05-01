#ifndef PARTHENOGENETICALGORITHMCONFIG_H
#define PARTHENOGENETICALGORITHMCONFIG_H

#include "EncodingMechanism.h"
#include "ReproductionMechanism.h"
#include "FitnessFunction.h"
#include "TerminationCriterion.h"
#include "SequenceEncodingMechanism.h"
#include "IPGAReproductionMechanism.h"
#include "DistanceFitnessFunction.h"
#include "IterationCountTerminationCriterion.h"

class ParthenoGeneticAlgorithmConfig
{

public:
    ParthenoGeneticAlgorithmConfig(
        EncodingMechanisms,
        ReproductionMechanisms,
        FitnessFunctions,
        TerminationCriteria);

    EncodingMechanisms encodingMechanism;
    ReproductionMechanisms reproductionMechanism;
    FitnessFunctions fitnessFunction;
    TerminationCriteria terminationCriteria;


private:
    std::shared_ptr<EncodingMechanism> chromosomeBuilder_;
    std::shared_ptr<ReproductionMechanism> reproducer_;
    std::shared_ptr<FitnessFunction> fitnessCalculator_;
    std::shared_ptr<TerminationCriterion> terminator_;
};

#endif
