#include "ParthenoGeneticAlgorithm.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>

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
    populateGASettings();
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
        reproducer_ = Reproducer{reproductionMechanism_ = std::shared_ptr<ReproductionMechanism>(
                                     new IPGAReproductionMechanism(
                                         std::make_shared<FitnessCalculator>(fitnessCalculator_),
                                         citiesPerSalesmanMutationProbability_,
                                         routeMutationProbability_,
                                         sampleSize_))};
    }

    if (config.terminationCriteria == TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION)
    {
        terminator_ = Terminator{terminationCriterion_ = std::shared_ptr<TerminationCriterion>(new IterationCountTerminationCriterion(numberOfIterations_))};
        ;
    }
}

int getNextLogFileNumber()
{
    int maxNumber = 0;
    std::regex logFilePattern("algorithm_log_(\\d+)\\.txt");
    for (const auto &entry : std::filesystem::directory_iterator("."))
    {
        std::string filename = entry.path().filename().string();
        std::smatch match;
        if (std::regex_match(filename, match, logFilePattern))
        {
            int number = std::stoi(match[1]);
            if (number > maxNumber)
            {
                maxNumber = number;
            }
        }
    }
    return maxNumber + 1;
}

void ParthenoGeneticAlgorithm::populateGASettings()
{
    toml::table tbl;
    try
    {
        tbl = toml::parse_file(config_header_path);
        citiesPerSalesmanMutationProbability_ = tbl["genetic_algorithm_settings"]["citiesPerSalesmanMutationProbability"].value_or<double>(0.0);
        routeMutationProbability_ = tbl["genetic_algorithm_settings"]["routeMutationProbability"].value_or<double>(0.0);
        sampleSize_ = tbl["genetic_algorithm_settings"]["sampleSize"].value_or<int>(0);
        populationSize_ = tbl["genetic_algorithm_settings"]["populationSize"].value_or<int>(0);
        numberOfIterations_ = tbl["genetic_algorithm_settings"]["numberOfIterations"].value_or<int>(0);
    }
    catch (const toml::parse_error &err)
    {
        std::cout << "Parsing failed\n";
    }
}

std::vector<int> ParthenoGeneticAlgorithm::run(std::vector<Position> &cities, int agents, std::vector<Position> &agentStartPositions)
{

    PopulationInitialiser populationInitialiser{chromosomeBuilder_, populationSize_};
    Population currentPopulation = populationInitialiser.InitialisePopulation(cities.size(), agents);
    std::vector<double> populationFitnesses;
    fitnessCalculator_.populateCostMap(cities, agentStartPositions);

    int logFileNumber = getNextLogFileNumber();
    std::string logFileName = "algorithm_log_" + std::to_string(logFileNumber) + ".txt";
    std::ofstream logFile(logFileName);
    if (!logFile)
    {
        std::cerr << "Failed to open log file." << std::endl;
        return {};
    }
    while (!terminator_.isTerminationCriteriaMet(populationFitnesses))
    {
        Population pop = reproducer_.Reproduce(currentPopulation, agentStartPositions, cities);
        double fitness = pop.getPopulationFitness(fitnessCalculator_, agentStartPositions, cities);
        populationFitnesses.push_back(fitness);
        std::cout << fitness << "\n";
        currentPopulation = pop;
    }

    std::vector<int> fittestGenes = currentPopulation.getFittestChromosomeGenes(fitnessCalculator_, agentStartPositions, cities);
    logFile << "Fittest Chromosome Genes: ";
    for (int gene : fittestGenes)
    {
        logFile << gene << " ";
    }
    logFile << "\n";

    logFile << "All Population Fitnesses: ";
    for (auto fit : populationFitnesses)
    {
        logFile << fit << " ";
    }
    logFile << "\n";

    logFile.close();
    return fittestGenes;
}
