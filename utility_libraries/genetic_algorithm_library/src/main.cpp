#include <vector>
#include <Position.h>
#include <ProblemLogUtility.h>
#include <ParthenoGeneticAlgorithmConfig.h>
#include <ParthenoGeneticAlgorithm.h>

int main(int argc, char const *argv[])
{
    std::vector<Position> cities;
    int agents;
    std::vector<Position> agentStartPositions;


    // *** WindTurbine60-5 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine60-5.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({10000, 5000}));
    }

    // *** WindTurbine60-10 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine60-10.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({7000, 4000}));
    }

    // *** WindTurbine60-15 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine60-15.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({7000, 3000}));
    }

    // *** WindTurbine180-5 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine180-5.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({40000, 15000}));
    }

    // *** WindTurbine180-10 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine180-10.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({20000, 10000}));
    }

    // *** WindTurbine180-15 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine180-15.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({30000, 6000}));
    }

    // *** WindTurbine180-20 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine180-20.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({40000, 5000}));
    }

    // *** WindTurbine360-5 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine360-5.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({120000, 40000}));
    }

    // *** WindTurbine360-10 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine360-10.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({120000, 20000}));
    }

    // *** WindTurbine360-15 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine360-15.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({90000, 20000}));
    }

    // *** WindTurbine360-20 *** ========================================================================================================

    // Read data from the file
    if (!ProblemLogUtility::readData("/home/ajifoster3/Desktop/problemfiles/WindTurbine360-20.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < 1; i++)
    {

        // Initialize the genetic algorithm configuration
        ParthenoGeneticAlgorithmConfig config(
            EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
            ReproductionMechanisms::NSGAII_REPRODUCTION_MECHANISM_SHUAI_HORIZONTAL,
            FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
            TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

        config.isManualConfig = true;
        config.citiesPerSalesmanMutationProbability = 0.8;
        config.routeMutationProbability = 0.9;
        config.sampleSize = 10;
        config.populationSize = 1000;
        config.numberOfIterations = 1000;
        config.alphaObjective = 0.75;
        config.alphaFitness = 0;

        ParthenoGeneticAlgorithm pga(config);

        pga.run(cities, agents, agentStartPositions, std::pair<double, double>({90000, 10000}));
    }
}
