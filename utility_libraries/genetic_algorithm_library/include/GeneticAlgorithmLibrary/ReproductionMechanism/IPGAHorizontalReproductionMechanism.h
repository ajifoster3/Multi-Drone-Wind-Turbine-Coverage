#ifndef IPGAHorizontalReproductionMechanism_H
#define IPGAHorizontalReproductionMechanism_H

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <map>
#include "ReproductionMechanism.h"
#include "Population.h"

class IPGAHorizontalReproductionMechanism : public ReproductionMechanism
{
public:
    class ReproductionChromosome;

    IPGAHorizontalReproductionMechanism(
        std::shared_ptr<FitnessCalculator>,
        double citiesPerSalesmanMutationProbability,
        double routeMutationProbability,
        int sampleSize);

    Population Reproduce(
        Population &oldPopulation,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities);
        
    void shuffleReproductionChromosomeList(std::vector<IPGAHorizontalReproductionMechanism::ReproductionChromosome> &chromosomeFitness);

    class ReproductionChromosome
    {
    public:
        ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> initialAgentPoses, std::vector<Position> cities);
        std::map<Fitness, double> getFitness() const;
        Chromosome &getChromosome();

    private:
        Chromosome chromosome_;
        std::map<Fitness, double> fitness_;
    };

    class ReproductionPopulation
    {
    public:
        ReproductionPopulation(Population population);

    private:
        Population population_;
        double maxFitness;
        double minFitness;
    };


private:
    int teamSize_;
    const int sampleSize_;
    const double citiesPerSalesmanMutationProbability_;
    const double routeMutationProbability_;

    std::mt19937 gen_;
    void flipInsert(std::vector<int> &vec, int numberOfCities);
    void swapInsert(std::vector<int> &vec, int numberOfCities);
    void lSlideInsert(std::vector<int> &vec, int numberOfCities);
    void rSlideInsert(std::vector<int> &vec, int numberOfCities);
    void randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities);
    void horizontalGeneTransfer(std::vector<int> &vec, std::vector<std::pair<std::vector<int>, double>> &library, int numberOfCities, std::vector<Position> &cities);
    std::vector<double> getSubChromosomeLibraryFitnesses(std::vector<std::pair<std::vector<int>, double>> &library, std::vector<Position> &cities);
    std::vector<int> getLongestSubChromosomePath(std::vector<std::vector<int>> &subChromosomes, std::vector<Position> &cities);
    std::vector<std::vector<int>> extractSubChromosomes(std::vector<int> &chromosome, int numberOfCities);
    std::vector<std::pair<std::vector<int>, double>> getHighValueSubChromosomeLibrary(Population &, std::vector<Position> &cities);
    bool hasNoCommonElements(const std::vector<int> &vec1, const std::vector<int> &vec2);
    void distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents);
};
#endif
