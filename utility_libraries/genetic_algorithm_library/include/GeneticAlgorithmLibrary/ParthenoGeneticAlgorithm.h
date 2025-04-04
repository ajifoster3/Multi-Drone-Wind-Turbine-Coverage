#ifndef PARTHENOGENETICALGORITHM_H
#define PARTHENOGENETICALGORITHM_H

#include <vector>
#include <memory>
#include <toml.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>
#include <map>
#include <variant>
#include "ProblemLogUtility.h"
#include "ConfigHeaderPath.h"
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
#include "IPGATournamentReproductionMechanism.h"
#include "DistanceFitnessFunction.h"
#include "IterationCountTerminationCriterion.h"
#include "ParthenoGeneticAlgorithmConfig.h"
#include "MultiDistanceFitnessFunction.h"
#include "Fitnesses.h"
#include "NSGAIIReproductionMechanism.h"
#include "ProblemLogUtility.h"
#include "IPGAHorizontalReproductionMechanism.h"
#include "NSGAIIReproductionMechanismHorizontal.h"
#include "IPGARouletteReproductionMechanism.h"
#include "NSGAIIPMXReproductionMechanism.h"
#include "NSGAIIReproductionMechanismShuai.h"
#include "NSGAIIReproductionMechanismShuaiHorizontal.h"
#include "NSGAIIReproductionMechanismShuaiHorizontalFocused.h"
#include <iomanip>

class ParthenoGeneticAlgorithm
{

public:
    ParthenoGeneticAlgorithm(ParthenoGeneticAlgorithmConfig);

    std::vector<int> run(std::vector<Position> &cities, int agents, std::vector<Position> &agentStartPositions, std::pair<double, double> nadir);

    void logIterations(Population population, std::vector<std::map<Fitness, double>> populationFitnesses, std::vector<Position> &cities, std::vector<Position> &agentStartPositions);
    std::vector<Chromosome> getUniqueParetoFront(const std::vector<Chromosome> &paretoFront);
    void logParetoFront(Population population, std::vector<Position> &cities, std::vector<Position> &agentStartPositions);

private:
    void logData(const std::string &fileName, const std::vector<Position> &cities, int agents, const std::vector<Position> &agentStartPositions);
    bool readData(const std::string &fileName, std::vector<Position> &cities, int &agents, std::vector<Position> &agentStartPositions);
    std::vector<Population> generations_;
    ChromosomeBuilder chromosomeBuilder_;
    Reproducer reproducer_;
    FitnessCalculator fitnessCalculator_;
    std::shared_ptr<FitnessCalculator> fitnessCalculatorPtr_;
    Terminator terminator_;
    ParthenoGeneticAlgorithmConfig config_;
    std::shared_ptr<EncodingMechanism> encodingMechanism_;
    std::shared_ptr<ReproductionMechanism> reproductionMechanism_;
    std::shared_ptr<FitnessFunction> fitnessFunction_;
    std::shared_ptr<TerminationCriterion> terminationCriterion_;
    void populateGASettings();
    double citiesPerSalesmanMutationProbability_;
    double routeMutationProbability_;
    int sampleSize_;
    int populationSize_;
    int numberOfIterations_;
};

#endif
