#include "ParthenoGeneticAlgorithm.h"

ParthenoGeneticAlgorithmConfig::ParthenoGeneticAlgorithmConfig(
    EncodingMechanisms encodingMechanism,
    ReproductionMechanisms reproductionMechanism,
    FitnessFunctions fitnessFunction,
    TerminationCriteria terminationCriterion) : encodingMechanism(encodingMechanism),
                                                reproductionMechanism(reproductionMechanism),
                                                fitnessFunction(fitnessFunction),
                                                terminationCriteria(terminationCriterion){};

ParthenoGeneticAlgorithm::ParthenoGeneticAlgorithm(ParthenoGeneticAlgorithmConfig config) : config_{config}
{
    if (!config.isManualConfig)
    {
        populateGASettings();
    }
    else
    {
        citiesPerSalesmanMutationProbability_ = config.citiesPerSalesmanMutationProbability;
        routeMutationProbability_ = config.routeMutationProbability;
        sampleSize_ = config.sampleSize;
        populationSize_ = config.populationSize;
        numberOfIterations_ = config.numberOfIterations;
    }

    if (config.encodingMechanism == EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM)
    {
        chromosomeBuilder_ = ChromosomeBuilder{encodingMechanism_ = std::shared_ptr<EncodingMechanism>(new SequenceEncodingMechanism)};
    }

    if (config.fitnessFunction == FitnessFunctions::DISTANCE_FITNESS_FUNCTION)
    {
        fitnessCalculator_ = FitnessCalculator{fitnessFunction_ = std::shared_ptr<DistanceFitnessFunction>(new DistanceFitnessFunction)};
        fitnessCalculatorPtr_ = std::make_shared<FitnessCalculator>(fitnessCalculator_);
    }
    else if (config.fitnessFunction == FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION)
    {
        fitnessCalculator_ = FitnessCalculator{fitnessFunction_ = std::shared_ptr<MultiDistanceFitnessFunction>(new MultiDistanceFitnessFunction)};
        fitnessCalculatorPtr_ = std::make_shared<FitnessCalculator>(fitnessCalculator_);
    }

    if (config.reproductionMechanism == ReproductionMechanisms::IPGA_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<IPGAReproductionMechanism>(
                                         fitnessCalculatorPtr_,
                                         citiesPerSalesmanMutationProbability_,
                                         routeMutationProbability_,
                                         sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::IPGA_ELITISM_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<IPGAElitismReproductionMechanism>(
                                         fitnessCalculatorPtr_,
                                         citiesPerSalesmanMutationProbability_,
                                         routeMutationProbability_,
                                         sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::IPGA_HORIZONTAL_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<IPGAHorizontalReproductionMechanism>(
                                         fitnessCalculatorPtr_,
                                         citiesPerSalesmanMutationProbability_,
                                         routeMutationProbability_,
                                         sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIReproductionMechanism>(
                                         std::make_shared<FitnessCalculator>(fitnessCalculator_),
                                         citiesPerSalesmanMutationProbability_,
                                         routeMutationProbability_,
                                         sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }

    if (config.terminationCriteria == TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION)
    {
        terminator_ = Terminator{terminationCriterion_ = std::shared_ptr<TerminationCriterion>(new IterationCountTerminationCriterion(numberOfIterations_))};
    }
}

int getNextLogFileNumber()
{

    int maxNumber = 0;
    std::regex logFilePattern(R"(algorithm_log_(\d+)\.txt)");

    try
    {
        for (const auto &entry : std::filesystem::directory_iterator("galog"))
        {
            if (entry.is_regular_file()) // Ensure it's a file
            {
                std::string filename = entry.path().filename().string();
                std::smatch match;
                if (std::regex_match(filename, match, logFilePattern))
                {
                    int number = std::stoi(match[1].str());
                    if (number > maxNumber)
                    {
                        maxNumber = number;
                    }
                }
            }
        }
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
    catch (const std::regex_error &e)
    {
        std::cerr << "Regex error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
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
    Fitness fitnessChoice = Fitness::MAXPATHTOTALPATHWEIGHTEDSUM;

    PopulationInitialiser populationInitialiser{chromosomeBuilder_, populationSize_};
    Population currentPopulation = populationInitialiser.InitialisePopulation(cities.size(), agents);
    std::vector<double> populationFitnesses;
    fitnessCalculator_.populateCostMap(cities, agentStartPositions);
    ProblemLogUtility::logData("log.txt", cities, agents, agentStartPositions);

    int i = 0;
    while (!terminator_.isTerminationCriteriaMet(populationFitnesses))
    {
        Population pop = reproducer_.Reproduce(currentPopulation, agentStartPositions, cities);
        double fitness = pop.getPopulationFitness(fitnessCalculator_, agentStartPositions, cities, fitnessChoice);
        populationFitnesses.push_back(fitness);
        std::cout << "Iteration:" << i << " " << fitness << "\n";
        currentPopulation = pop;
        i++;
    }

    std::vector<int> fittestGenes = currentPopulation.getFittestChromosomeGenes(fitnessCalculator_, agentStartPositions, cities, Fitness::MAXPATHTOTALPATHWEIGHTEDSUM);

    logIterations(fittestGenes, populationFitnesses, fitnessChoice);
    return fittestGenes;
}

void ParthenoGeneticAlgorithm::logIterations(std::vector<int> &fittestGenes, std::vector<double> &populationFitnesses, Fitness fitnessChoice)
{
    int logFileNumber = getNextLogFileNumber();
    std::string logDirectoryName = "galog"; // Directory name
    std::string logFileName = logDirectoryName + "/algorithm_log_" + std::to_string(logFileNumber) + ".txt";

    // Check if the directory exists, if not, create it
    if (!std::filesystem::exists(logDirectoryName))
    {
        std::filesystem::create_directory(logDirectoryName);
    }

    std::ofstream logFile(logFileName);
    if (!logFile)
    {
        std::cerr << "Failed to open log file." << std::endl;
        return; // Exit if file could not be opened
    }

    // Log genetic algorithm settings
    logFile << "GA_Settings \n";
    logFile << "EncodingMechanism:" << config_.encodingMechanism << "\n";
    logFile << "FitnessFunction:" << config_.fitnessFunction << "\n";
    logFile << "ReproductionMechanism:" << config_.reproductionMechanism << "\n";
    logFile << "TerminationCriteria:" << config_.terminationCriteria << "\n";
    logFile << "citiesPerSalesmanMutationProbability:" << citiesPerSalesmanMutationProbability_ << "\n";
    logFile << "routeMutationProbability:" << routeMutationProbability_ << "\n";
    logFile << "sampleSize:" << sampleSize_ << "\n";
    logFile << "populationSize:" << populationSize_ << "\n";
    logFile << "numberOfIterations:" << numberOfIterations_ << "\n\n";

    // Log fittest chromosome genes
    logFile << "Fittest_Chromosome_Genes: \n";
    for (int gene : fittestGenes)
    {
        logFile << gene << "\n";
    }
    logFile << "\n\n";

    // Log all population fitnesses
    logFile << "All_Population_Fitnesses: \n";
    for (auto fit : populationFitnesses)
    {
        logFile << fit << "\n";
    }
    logFile << "\n\n";

    logFile.close();
}
