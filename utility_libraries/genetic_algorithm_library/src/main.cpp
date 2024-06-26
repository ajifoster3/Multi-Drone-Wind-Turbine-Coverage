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

    // Read data from the file
    if (!ProblemLogUtility::readData("log.txt", cities, agents, agentStartPositions))
    {
        std::cerr << "Failed to read data from log.txt" << std::endl;
        return 1;
    }

    // Initialize the genetic algorithm configuration
    ParthenoGeneticAlgorithmConfig config(
        EncodingMechanisms::SEQUENCE_ENCODING_MECHANISM,
        ReproductionMechanisms::IPGA_REPRODUCTION_MECHANISM,
        FitnessFunctions::MULTI_DISTANCE_FITNESS_FUNCTION,
        TerminationCriteria::ITERATION_COUNT_TERMINATION_CRITERION);

    auto citiesPerSalesmanMutationProbability = 0.02;

    //while (citiesPerSalesmanMutationProbability <= 0.5)
    //{
    //    for (int i = 0; i < 100; i++)
    //    { 
    config.isManualConfig = true;
    config.citiesPerSalesmanMutationProbability = 0.25;
    config.routeMutationProbability = 0.8;
    config.sampleSize = 10;
    config.populationSize = 1000;
    config.numberOfIterations = 10000;

    // Create an instance of the genetic algorithm
    ParthenoGeneticAlgorithm pga(config);

    pga.run(cities, agents, agentStartPositions);
    //}

//    citiesPerSalesmanMutationProbability = citiesPerSalesmanMutationProbability + 0.02;
//}

    return 0;
}
