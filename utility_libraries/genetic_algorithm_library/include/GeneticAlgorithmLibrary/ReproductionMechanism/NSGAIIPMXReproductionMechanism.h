#ifndef NSGAIIPMXREPRODUCTIONMECHANISM_H
#define NSGAIIPMXREPRODUCTIONMECHANISM_H

#include "ReproductionMechanism.h"
#include "Population.h"
#include "Position.h"
#include "FitnessCalculator.h"
#include "MultiDistanceFitnessFunction.h"
#include <memory>
#include <vector>
#include <random>
#include <map>
#include <iostream>
#include <algorithm>
#include <limits>
#include <unordered_set>

class NSGAIIPMXReproductionMechanism : public ReproductionMechanism
{
public:
    NSGAIIPMXReproductionMechanism(
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
    std::vector<NSGAIIPMXReproductionMechanism::ReproductionChromosome> ElitismSelection(const std::vector<ReproductionChromosome> &population);
    void PMXCrossover(ReproductionChromosome& parent1, ReproductionChromosome& parent2, ReproductionChromosome& offspring1, ReproductionChromosome& offspring2, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities);
    void Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities);
    std::vector<std::vector<ReproductionChromosome>> FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities);

    ReproductionChromosome TournamentSelection(const std::vector<ReproductionChromosome> &population);

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
