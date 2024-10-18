#include "ParthenoGeneticAlgorithm.h"

ParthenoGeneticAlgorithmConfig::ParthenoGeneticAlgorithmConfig(
    EncodingMechanisms encodingMechanism,
    ReproductionMechanisms reproductionMechanism,
    FitnessFunctions fitnessFunction,
    TerminationCriteria terminationCriterion) : encodingMechanism(encodingMechanism),
                                                reproductionMechanism(reproductionMechanism),
                                                fitnessFunction(fitnessFunction),
                                                terminationCriteria(terminationCriterion) {};

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
        fitnessCalculator_ = FitnessCalculator{fitnessFunction_ = std::shared_ptr<MultiDistanceFitnessFunction>(new MultiDistanceFitnessFunction(config.alphaObjective, config.alphaFitness))};
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
    else if (config.reproductionMechanism == ReproductionMechanisms::IPGA_TOURNAMENT_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<IPGATournamentReproductionMechanism>(
            fitnessCalculatorPtr_,
            citiesPerSalesmanMutationProbability_,
            routeMutationProbability_,
            sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::IPGA_ROULETTE_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<IPGARouletteReproductionMechanism>(
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
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIReproductionMechanismShuai>(
            std::make_shared<FitnessCalculator>(fitnessCalculator_),
            citiesPerSalesmanMutationProbability_,
            routeMutationProbability_,
            sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_PMX_REPRODUCTION_MECHANISM)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIPMXReproductionMechanism>(
            std::make_shared<FitnessCalculator>(fitnessCalculator_),
            citiesPerSalesmanMutationProbability_,
            routeMutationProbability_,
            sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_HORIZONTAL)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIReproductionMechanismHorizontal>(
            std::make_shared<FitnessCalculator>(fitnessCalculator_),
            citiesPerSalesmanMutationProbability_,
            routeMutationProbability_,
            sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_HORIZONTAL)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIReproductionMechanismHorizontal>(
            std::make_shared<FitnessCalculator>(fitnessCalculator_),
            citiesPerSalesmanMutationProbability_,
            routeMutationProbability_,
            sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIReproductionMechanismShuaiHorizontal>(
            std::make_shared<FitnessCalculator>(fitnessCalculator_),
            citiesPerSalesmanMutationProbability_,
            routeMutationProbability_,
            sampleSize_);
        reproducer_ = Reproducer(reproductionMechanism_);
    }
    else if (config.reproductionMechanism == ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL_FOCUSED)
    {
        reproductionMechanism_ = std::make_shared<NSGAIIReproductionMechanismShuaiHorizontalFocused>(
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

int getNextLogFileNumber(const std::string &directoryPath)
{
    int maxNumber = 0;
    std::regex logFilePattern(R"(algorithm_log_(\d+)\.txt)");

    try
    {
        if (std::filesystem::exists(directoryPath))
        {
            for (const auto &entry : std::filesystem::directory_iterator(directoryPath))
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

std::vector<int> ParthenoGeneticAlgorithm::run(std::vector<Position> &cities, int agents, std::vector<Position> &agentStartPositions, std::pair<double, double> nadir)
{
    std::cout << "Inside run\n";
    Fitness fitnessChoice = Fitness::HYPERVOLUME;

    PopulationInitialiser populationInitialiser{chromosomeBuilder_, populationSize_};
    Population currentPopulation = populationInitialiser.InitialisePopulation(cities.size(), agents);
    std::vector<std::map<Fitness, double>> populationFitnesses;

    fitnessCalculator_.populateCostMap(cities, agentStartPositions);

    auto startTime = std::chrono::steady_clock::now();
    populationFitnesses.push_back(currentPopulation.getPopulationFitness(fitnessCalculator_, agentStartPositions, cities, fitnessChoice, startTime, nadir));

    std::cout << "Iteration:-1 " << populationFitnesses[0][fitnessChoice] << "\n";
    ProblemLogUtility::logData("log.txt", cities, agents, agentStartPositions);
    std::cout << "Entering loop\n";
    int i = 0;
    while (!terminator_.isTerminationCriteriaMet(populationFitnesses))
    {
        Population pop = reproducer_.Reproduce(currentPopulation, agentStartPositions, cities, i);
        auto fitness = pop.getPopulationFitness(fitnessCalculator_, agentStartPositions, cities, fitnessChoice, startTime, nadir);
        populationFitnesses.push_back(fitness);
        std::cout << "Iteration:" << i << " " << fitness[fitnessChoice] << "\n";
        currentPopulation = pop;
        i++;
    }

    std::vector<int> fittestGenes = currentPopulation.getFittestChromosomeGenes(fitnessCalculator_, agentStartPositions, cities, Fitness::MAXPATHTOTALPATHWEIGHTEDSUM);

    logIterations(currentPopulation, populationFitnesses, cities, agentStartPositions);
    return fittestGenes;
}

void ParthenoGeneticAlgorithm::logIterations(Population population, std::vector<std::map<Fitness, double>> populationFitnesses, std::vector<Position> &cities, std::vector<Position> &agentStartPositions)
{
    // Get directory name based on reproduction mechanism, number of cities, and number of agents
    std::string logDirectoryName = "galog/" + std::to_string(cities.size()) + "_cities_" + std::to_string(agentStartPositions.size()) + "_agents_" + std::to_string(static_cast<int>(config_.reproductionMechanism));

    // Check if the directory exists, if not, create it
    if (!std::filesystem::exists(logDirectoryName))
    {
        std::filesystem::create_directories(logDirectoryName); // Create the entire directory path if needed
    }

    int logFileNumber = getNextLogFileNumber(logDirectoryName);
    std::string logFileName = logDirectoryName + "/algorithm_log_" + std::to_string(logFileNumber) + ".txt";

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

    // Log fitness values per iteration
    logFile << "Iteration_Log\n";
    logFile << "Iteration, MAXPATHLENGTH, TOTALPATHDISTANCE, HYPERVOLUME, TIMEELAPSED\n";
    for (size_t i = 0; i < populationFitnesses.size(); ++i)
    {
        logFile << i << ", "
                << populationFitnesses[i][Fitness::MAXPATHLENGTH] << ", "
                << populationFitnesses[i][Fitness::TOTALPATHDISTANCE] << ", "
                << populationFitnesses[i][Fitness::HYPERVOLUME] << ", "
                << populationFitnesses[i][Fitness::TIMEELAPSED] << "\n";
    }

    // Get Pareto front
    std::vector<Fitness> fitnessChoices = {Fitness::MAXPATHLENGTH, Fitness::TOTALPATHDISTANCE};
    std::vector<Chromosome> paretoFront = population.getParetoFront(fitnessCalculator_, agentStartPositions, cities);
    paretoFront = getUniqueParetoFront(paretoFront);
    // Log Pareto front fitnesses and chromosome values
    logFile << "\nPareto_Front: \n";
    logFile << "Chromosome_Genes, MAXPATHLENGTH, TOTALPATHDISTANCE \n";
    for (auto &chromosome : paretoFront)
    {
        auto fitness = fitnessCalculator_.calculateFitness(chromosome, cities);
        logFile << "Genes: ";
        for (int gene : chromosome.getGenes())
        {
            logFile << gene << " ";
        }
        logFile << std::setprecision(10)
            << std::fixed
            << std::showpoint;
        logFile << ", Fitnesses: " << fitness[Fitness::MAXPATHLENGTH] << ", " << fitness[Fitness::TOTALPATHDISTANCE] << "\n";
    }

    logFile.close();
}
// A hash function for the chromosome's genes can be defined, depending on the type of genes
struct ChromosomeHash
{
    std::size_t operator()(const Chromosome &chromosome) const
    {
        const auto &genes = chromosome.getGenes(); // Assuming this returns std::vector<int>
        std::size_t seed = genes.size(); // Start with the size of the genes as a basis for the hash

        // Combine the hash of each gene element
        for (const int &gene : genes)
        {
            seed ^= std::hash<int>{}(gene) + 0x9e3779b9 + (seed << 6) + (seed >> 2); // Hash combine
        }

        return seed;
    }
};
struct ChromosomeEqual
{
    bool operator()(const Chromosome &lhs, const Chromosome &rhs) const
    {
        return lhs.getGenes() == rhs.getGenes();
    }
};

std::vector<Chromosome> ParthenoGeneticAlgorithm::getUniqueParetoFront(const std::vector<Chromosome> &paretoFront)
{
    std::unordered_set<Chromosome, ChromosomeHash, ChromosomeEqual> uniqueChromosomes;
    std::vector<Chromosome> uniqueParetoFront;

    for (const auto &chromosome : paretoFront)
    {
        if (uniqueChromosomes.find(chromosome) == uniqueChromosomes.end())
        {
            uniqueChromosomes.insert(chromosome);
            uniqueParetoFront.push_back(chromosome);
        }
    }

    return uniqueParetoFront;
}

void ParthenoGeneticAlgorithm::logParetoFront(Population population, std::vector<Position> &cities, std::vector<Position> &agentStartPositions)
{
    // Get directory name based on reproduction mechanism, number of cities, and number of agents
    std::string logDirectoryName = "galog/" + std::to_string(cities.size()) + "_cities_" + std::to_string(agentStartPositions.size()) + "_agents_" + std::to_string(static_cast<int>(config_.reproductionMechanism));

    // Check if the directory exists, if not, create it
    if (!std::filesystem::exists(logDirectoryName))
    {
        std::filesystem::create_directories(logDirectoryName); // Create the entire directory path if needed
    }

    int logFileNumber = getNextLogFileNumber(logDirectoryName);
    std::string logFileName = logDirectoryName + "/pareto_front_log_" + std::to_string(logFileNumber) + ".txt";

    std::ofstream logFile(logFileName);
    if (!logFile)
    {
        std::cerr << "Failed to open log file." << std::endl;
        return; // Exit if file could not be opened
    }

    // Get Pareto front
    std::vector<Fitness> fitnessChoices = {Fitness::MAXPATHLENGTH, Fitness::TOTALPATHDISTANCE};
    std::vector<Chromosome> paretoFront = population.getParetoFront(fitnessCalculator_, agentStartPositions, cities);

    // Log Pareto front fitnesses and chromosome values
    logFile << "Pareto_Front: \n";
    logFile << "MAXPATHLENGTH, TOTALPATHDISTANCE \n";

    logFile << std::setprecision(10)
            << std::fixed
            << std::showpoint;
    for (auto &chromosome : paretoFront)
    {
        auto fitness = fitnessCalculator_.calculateFitness(chromosome, cities);
        logFile << fitness[Fitness::MAXPATHLENGTH] << ", " << fitness[Fitness::TOTALPATHDISTANCE] << "\n";
    }

    logFile.close();
}
