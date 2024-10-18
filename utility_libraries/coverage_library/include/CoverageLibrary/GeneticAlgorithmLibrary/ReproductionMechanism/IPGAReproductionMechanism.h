#ifndef IPGAREPRODUCTIONMECHANISM_H
#define IPGAREPRODUCTIONMECHANISM_H

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <map>
#include "ReproductionMechanism.h"
#include "Population.h"

class IPGAReproductionMechanism : public ReproductionMechanism
{
public:
    class ReproductionChromosome;

    IPGAReproductionMechanism(
        std::shared_ptr<FitnessCalculator>,
        double citiesPerSalesmanMutationProbability,
        double routeMutationProbability,
        int sampleSize);

    Population Reproduce(
        Population &oldPopulation,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities);

    void shuffleReproductionChromosomeList(std::vector<IPGAReproductionMechanism::ReproductionChromosome> &chromosomeFitness);

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
    const int sampleSize_;
    const double citiesPerSalesmanMutationProbability_;
    const double routeMutationProbability_;

    std::mt19937 gen_;
    void flipInsert(std::vector<int> &vec, int numberOfCities);
    void swapInsert(std::vector<int> &vec, int numberOfCities);
    void lSlideInsert(std::vector<int> &vec, int numberOfCities);
    void rSlideInsert(std::vector<int> &vec, int numberOfCities);
    void randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities);
    void distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents);
};

#endif
