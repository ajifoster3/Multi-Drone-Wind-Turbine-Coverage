#ifndef NSGAIIPMXREPRODUCTIONMECHANISMSHUAIHORIZONTAL_H
#define NSGAIIPMXREPRODUCTIONMECHANISMSHUAIHORIZONTAL_H

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

class NSGAIIReproductionMechanismShuaiHorizontal : public ReproductionMechanism
{
public:
    NSGAIIReproductionMechanismShuaiHorizontal(
        std::shared_ptr<FitnessCalculator>,
        double citiesPerSalesmanMutationProbability,
        double routeMutationProbability,
        int sampleSize);

    Population Reproduce(Population &oldPopulation, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities, int iterationNumber);

    

private:
    int teamSize_;
    struct ReproductionChromosome
    {
        ReproductionChromosome(Chromosome chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities);

        bool dominates(const ReproductionChromosome &other) const;
        double getFitness(Fitness index) const;
        double getObjectiveFitness(Fitness index) const;
        Chromosome getChromosome();
        bool operator<(const ReproductionChromosome &other) const;
        bool isParentMalformed(Chromosome chromosome, int teamSize);
        Chromosome chromosome_;
        std::map<Fitness, double> fitnessValues_;
        double crowdingDistance = 0.0;
        int rank = 0;
    };


    void AssignCrowdingDistance(std::vector<ReproductionChromosome> &front);
    std::vector<NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome> ElitismSelection(const std::vector<ReproductionChromosome> &population);
    void ProximityBasedCrossover(
        ReproductionChromosome &parent1,
        ReproductionChromosome &parent2,
        ReproductionChromosome &offspring,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities,
        bool forward);
    void Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities, std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> library);
    std::vector<std::vector<ReproductionChromosome>> FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities);

    ReproductionChromosome TournamentSelection(const std::vector<ReproductionChromosome> &population);

    NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome HighMinMaxTournamentSelection(const std::vector<ReproductionChromosome> &population);

    void shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness);
    void flipInsert(std::vector<int> &vec, int numberOfCities);
    void swapInsert(std::vector<int> &vec, int numberOfCities);
    void lSlideInsert(std::vector<int> &vec, int numberOfCities);
    void rSlideInsert(std::vector<int> &vec, int numberOfCities);
    void randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities);
    void distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents);

    std::vector<int> horizontalGeneTransfer(std::vector<int> &chromosome, std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> &library, int numberOfCities, std::vector<Position> &cities);

    std::vector<double> getSubChromosomeLibraryFitnesses(std::vector<std::pair<std::vector<int>, double>> &library, std::vector<Position> &cities);

    std::vector<int> getLongestSubChromosomePath(std::vector<std::vector<int>> &subChromosomes, std::vector<Position> &cities);

    std::vector<std::vector<int>> extractSubChromosomes(std::vector<int> &chromosome, int numberOfCities);

    std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> getHighValueSubChromosomeLibrary(Population &population, std::vector<Position> &cities, int teamsize);

    bool hasNoCommonElements(const std::vector<int> &vec1, const std::vector<int> &vec2);

    bool isParentMalformed(ReproductionChromosome chromosome, int teamSize);
    bool isParentMalformed(Chromosome chromosome, int teamSize);
    double citiesPerSalesmanMutationProbability_;
    double routeMutationProbability_;
    int sampleSize_;
    std::mt19937 gen_;
};

#endif // NSGAIIREPRODUCTIONMECHANISM_H
