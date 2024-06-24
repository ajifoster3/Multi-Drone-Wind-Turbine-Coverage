#include "PopulationInitialiser.h"

PopulationInitialiser::PopulationInitialiser(ChromosomeBuilder& chromosomeBuilder, int populationSize)
: populationSize_(populationSize)
{
    chromosomeBuilder_ = chromosomeBuilder;
}

Population PopulationInitialiser::InitialisePopulation(int numberOfCites, int agents)
{
    std::vector<Chromosome> initialChromosomeList{};
    initialChromosomeList.reserve(populationSize_);
    
    for (size_t i = 0; i < populationSize_; i++)
    {
        initialChromosomeList.push_back(chromosomeBuilder_.buildChromosome(numberOfCites, agents));
    }
    
    return Population(initialChromosomeList);
}
