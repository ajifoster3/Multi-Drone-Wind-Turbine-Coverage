#ifndef NSGAIIREPRODUCTIONMECHANISM_H
#define NSGAIIREPRODUCTIONMECHANISM_H

#include "ReproductionMechanism.h"
#include "Population.h"
#include "Position.h"
#include "FitnessCalculator.h"
#include "MultiDistanceFitnessFunction.h"
#include <memory>
#include <vector>
#include <random>
#include <map>

class NSGAIIReproductionMechanism : public ReproductionMechanism
{
public:
    NSGAIIReproductionMechanism(
        std::shared_ptr<FitnessCalculator>,
        double citiesPerSalesmanMutationProbability,
        double routeMutationProbability,
        int sampleSize);

    Population Reproduce(
        Population &oldPopulation,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities,
    int iterationNumber);

private:
    struct ReproductionChromosome
    {
        ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities);

        bool dominates(const ReproductionChromosome &other) const;
        double getFitness(Fitness index) const;
        double getObjectiveFitness(Fitness index) const;
        Chromosome &getChromosome();
        bool operator<(const ReproductionChromosome& other) const;
        Chromosome chromosome_;
        std::map<Fitness, double> fitnessValues_;
        double crowdingDistance = 0.0;
        int rank = 0;
    };

    void AssignCrowdingDistance(std::vector<ReproductionChromosome> &front);
    NSGAIIReproductionMechanism::ReproductionChromosome TournamentSelection(const std::vector<ReproductionChromosome> &population);
    void Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities);
    std::vector<std::vector<ReproductionChromosome>> FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities);

    void shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness);
    void flipInsert(std::vector<int> &vec, int numberOfCities);
    void swapInsert(std::vector<int> &vec, int numberOfCities);
    void lSlideInsert(std::vector<int> &vec, int numberOfCities);
    void rSlideInsert(std::vector<int> &vec, int numberOfCities);
    void randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities);
    void distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents);

    double citiesPerSalesmanMutationProbability_;
    double routeMutationProbability_;
    int sampleSize_;
    std::mt19937 gen_;
};

#endif // NSGAIIREPRODUCTIONMECHANISM_H
