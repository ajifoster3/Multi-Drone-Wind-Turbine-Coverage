#include "ParthenoGeneticAlgorithm.h"
#include <iostream>

ParthenoGeneticAlgorithmConfig::ParthenoGeneticAlgorithmConfig(
    EncodingMechanisms encodingMechanism,
    ReproductionMechanisms reproductionMechanism,
    FitnessFunctions fitnessFunction,
    TerminationCriteria terminationCriterion) : encodingMechanism(encodingMechanism),
                                                reproductionMechanism(reproductionMechanism),
                                                fitnessFunction(fitnessFunction),
                                                terminationCriteria(terminationCriterion){};

ParthenoGeneticAlgorithm::ParthenoGeneticAlgorithm(ParthenoGeneticAlgorithmConfig config)
{
    if (config.encodingMechanism == EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM)
    {
        chromosomeBuilder_ = ChromosomeBuilder{encodingMechanism_ = std::shared_ptr<EncodingMechanism>(new SequenceEncodingMechanism)};
    }

    if (config.fitnessFunction == FitnessFunctions::DISTANCE_FITNESS_FUNCTION)
    {
        fitnessCalculator_ = FitnessCalculator{fitnessFunction_ = std::shared_ptr<FitnessFunction>(new DistanceFitnessFunction)};
    }

    if (config.reproductionMechanism == ReproductionMechanisms::IPGA_REPRODUCTION_MECHANISM)
    {
        reproducer_ = Reproducer{reproductionMechanism_ = std::shared_ptr<ReproductionMechanism>(new IPGAReproductionMechanism(std::make_shared<FitnessCalculator>(fitnessCalculator_)))};
    }

    if (config.terminationCriteria == TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION)
    {
        terminator_ = Terminator{terminationCriterion_ = std::shared_ptr<TerminationCriterion>(new IterationCountTerminationCriterion)};
        ;
    }
}

std::vector<int> ParthenoGeneticAlgorithm::run(std::vector<Position>& cities, int agents, std::vector<Position>& agentStartPositions)
{
    PopulationInitialiser populationInitialiser{chromosomeBuilder_};
    Population currentPopulation = populationInitialiser.InitialisePopulation(cities.size(), agents);
    std::vector<double> populationFitnesses;
    fitnessCalculator_.populateCostMap(cities, agentStartPositions);
    while (!terminator_.isTerminationCriteriaMet(populationFitnesses))
    {
        Population pop = reproducer_.Reproduce(currentPopulation, agentStartPositions, cities);
        double fitness = pop.getPopulationFitness(fitnessCalculator_, agentStartPositions, cities);
        populationFitnesses.push_back(fitness);
        std::cout << fitness << "\n";
        currentPopulation = pop;
    }
    return currentPopulation.getFittestChromosomeGenes(fitnessCalculator_, agentStartPositions, cities);
}

