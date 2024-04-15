#include "PopulationInitialiser.h"

PopulationInitialiser::PopulationInitialiser(ChromosomeBuilder& chromosomeBuilder)
{
    chromosomeBuilder_ = chromosomeBuilder;
}

Population PopulationInitialiser::InitialisePopulation(int numberOfCites, int agents)
{
    std::vector<Chromosome> initialChromosomeList{};
    initialChromosomeList.reserve(1000);
    
    for (size_t i = 0; i < 1000; i++)
    {
        initialChromosomeList.push_back(chromosomeBuilder_.buildChromosome(numberOfCites, agents));
    }
    
    return Population(initialChromosomeList);
}
