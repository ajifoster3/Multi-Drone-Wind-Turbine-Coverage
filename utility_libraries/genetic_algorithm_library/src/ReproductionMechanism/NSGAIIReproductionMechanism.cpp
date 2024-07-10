#include "NSGAIIReproductionMechanism.h"
#include <iostream>
#include <algorithm>
#include <limits>

NSGAIIReproductionMechanism::NSGAIIReproductionMechanism(
    std::shared_ptr<FitnessCalculator> fitnessCalculator,
    double citiesPerSalesmanMutationProbability,
    double routeMutationProbability,
    int sampleSize)
    : ReproductionMechanism(std::move(fitnessCalculator)),
      citiesPerSalesmanMutationProbability_(citiesPerSalesmanMutationProbability),
      routeMutationProbability_(routeMutationProbability),
      sampleSize_(sampleSize)
{
    std::cout << "Constructed NSGAII\n";
    std::random_device rd;
    gen_ = std::mt19937(rd());
}

Population NSGAIIReproductionMechanism::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    auto fronts = FastNonDominatedSort(oldPopulation, initialAgentPoses, cities);

    for (auto &front : fronts)
    {
        AssignCrowdingDistance(front);
    }

    std::vector<ReproductionChromosome> newGeneration;
    newGeneration.reserve(oldPopulation.getPopulationList().size());
    std::vector<ReproductionChromosome> tempNewGeneration;
    tempNewGeneration.reserve(oldPopulation.getPopulationList().size());
    
    for (auto &front : fronts)
    {
        std::sort(front.begin(), front.end(), [](const ReproductionChromosome &a, const ReproductionChromosome &b)
                  {
                      if (a.rank != b.rank)
                          return a.rank < b.rank;
                      return a.crowdingDistance > b.crowdingDistance; });

        for (auto &chromosome : front)
        {
            if (tempNewGeneration.size() < oldPopulation.getPopulationList().size())
            {
                tempNewGeneration.push_back(chromosome);
            }
        }
    }

    // Reproduction to create the new population
    while (newGeneration.size() < oldPopulation.getPopulationList().size())
    {
        // Tournament selection
        ReproductionChromosome parent = TournamentSelection(tempNewGeneration);

        // Mutation
        ReproductionChromosome offspring = parent;
        Mutate(offspring, cities);

        // Ensure offspring is unique before adding to new generation
        if (std::find_if(newGeneration.begin(), newGeneration.end(), [&](ReproductionChromosome &chr)
                         { return chr.getChromosome() == offspring.getChromosome(); }) == newGeneration.end())
        {
            newGeneration.push_back(offspring);
        }
    }

    std::vector<Chromosome> finalGeneration;
    for (auto &chromosome : newGeneration)
    {
        finalGeneration.push_back(chromosome.getChromosome());
    }

    return Population(finalGeneration);
}

void NSGAIIReproductionMechanism::AssignCrowdingDistance(std::vector<ReproductionChromosome> &front)
{
    int numObjectives = 2; // Number of objectives: TOTALPATHDISTANCE, MAXPATHLENGTH
    int numChromosomes = front.size();

    for (auto &chromosome : front)
    {
        chromosome.crowdingDistance = 0.0;
    }

    for (int m = 0; m < numObjectives; ++m)
    {
        Fitness objective = (m == 0) ? Fitness::TOTALPATHDISTANCE : Fitness::MAXPATHLENGTH;

        std::sort(front.begin(), front.end(), [objective](const ReproductionChromosome &a, const ReproductionChromosome &b)
                  { return a.getObjectiveFitness(objective) < b.getObjectiveFitness(objective); });

        front[0].crowdingDistance = std::numeric_limits<double>::infinity();
        front[numChromosomes - 1].crowdingDistance = std::numeric_limits<double>::infinity();

        double minFitness = front[0].getObjectiveFitness(objective);
        double maxFitness = front[numChromosomes - 1].getObjectiveFitness(objective);

        if (maxFitness - minFitness == 0)
        {
            continue;
        }

        for (int i = 1; i < numChromosomes - 1; ++i)
        {
            front[i].crowdingDistance += (front[i + 1].getObjectiveFitness(objective) - front[i - 1].getObjectiveFitness(objective)) / (maxFitness - minFitness);
        }
    }
}

NSGAIIReproductionMechanism::ReproductionChromosome NSGAIIReproductionMechanism::TournamentSelection(const std::vector<ReproductionChromosome> &population)
{
    std::uniform_int_distribution<> dis(0, population.size() - 1);

    ReproductionChromosome best = population[dis(gen_)];
    for (int i = 1; i < sampleSize_; ++i)
    {
        ReproductionChromosome candidate = population[dis(gen_)];
        if (candidate.rank < best.rank || (candidate.rank == best.rank && candidate.crowdingDistance > best.crowdingDistance))
        {
            best = candidate;
        }
    }
    return best;
}

void NSGAIIReproductionMechanism::Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities)
{
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    auto genes = chromosome.getChromosome().getGenes();

    if (distribution(gen_) < routeMutationProbability_)
    {
        auto roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            flipInsert(genes, chromosome.getChromosome().getNumberOfCities());
        }
        roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            swapInsert(genes, chromosome.getChromosome().getNumberOfCities());
        }
        roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            lSlideInsert(genes, chromosome.getChromosome().getNumberOfCities());
        }
        roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            rSlideInsert(genes, chromosome.getChromosome().getNumberOfCities());
        }
    }
    if (distribution(gen_) < citiesPerSalesmanMutationProbability_)
    {
        distributeCities(genes, chromosome.getChromosome().getNumberOfCities(), chromosome.getChromosome().getNumberOfAgents());
    }

    Chromosome newChromosome(genes, chromosome.getChromosome().getNumberOfCities());
    chromosome.chromosome_ = newChromosome;
    chromosome.fitnessValues_ = fitnessCalculator_->calculateFitness(newChromosome, cities);
}

std::vector<std::vector<NSGAIIReproductionMechanism::ReproductionChromosome>> NSGAIIReproductionMechanism::FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
{
    std::vector<ReproductionChromosome> chromosomes;
    for (auto &chromosome : population.getPopulationList())
    {
        chromosomes.emplace_back(chromosome, fitnessCalculator_, agentStartPositions, cities);
    }

    std::vector<std::vector<ReproductionChromosome>> fronts(1);
    std::vector<int> dominationCount(chromosomes.size(), 0);
    std::vector<std::vector<int>> dominatedChromosomes(chromosomes.size());

    for (size_t p = 0; p < chromosomes.size(); ++p)
    {
        for (size_t q = 0; q < chromosomes.size(); ++q)
        {
            if (p == q)
                continue;
            if (chromosomes[p].dominates(chromosomes[q]))
            {
                dominatedChromosomes[p].push_back(q);
            }
            else if (chromosomes[q].dominates(chromosomes[p]))
            {
                dominationCount[p]++;
            }
        }
        if (dominationCount[p] == 0)
        {
            chromosomes[p].rank = 0;
            fronts[0].push_back(chromosomes[p]);
        }
    }

    int i = 0;
    while (!fronts[i].empty())
    {
        std::vector<ReproductionChromosome> nextFront;
        for (auto chromosome : fronts[i])
        {
            for (size_t q = 0; q < dominatedChromosomes.size(); ++q)//auto q : dominatedChromosomes
            {
                dominationCount[q]--;
                if (dominationCount[q] == 0)
                {
                    chromosomes[q].rank = i + 1;
                    nextFront.push_back(chromosomes[q]);
                }
            }
        }
        i++;
        fronts.push_back(nextFront);
    }

    fronts.pop_back(); // remove the last empty front
    return fronts;
}


void NSGAIIReproductionMechanism::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{
    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void NSGAIIReproductionMechanism::flipInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }
    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    int index1 = dis(gen_);
    int index2;
    do
    {
        index2 = dis(gen_);
    } while (index1 == index2);

    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    std::reverse(vec.begin() + index1, vec.begin() + index2 + 1);
    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void NSGAIIReproductionMechanism::swapInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }

    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    int index1 = dis(gen_);
    int index2;
    do
    {
        index2 = dis(gen_);
    } while (index1 == index2);

    std::swap(vec[index1], vec[index2]);

    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void NSGAIIReproductionMechanism::lSlideInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }

    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    int index1 = dis(gen_);
    int index2;
    do
    {
        index2 = dis(gen_);
    } while (index1 == index2);

    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    std::rotate(vec.begin() + index1, vec.begin() + index1 + 1, vec.begin() + index2 + 1);
    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void NSGAIIReproductionMechanism::rSlideInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }

    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    int index1 = dis(gen_);
    int index2;
    do
    {
        index2 = dis(gen_);
    } while (index1 == index2);

    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    std::rotate(vec.begin() + index1, vec.begin() + index2, vec.begin() + index2 + 1);
    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void NSGAIIReproductionMechanism::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
{
    int range1 = index1;
    int range2 = numberOfCities - (index2 + 1);

    std::vector<int> weights = {range1, range2};
    std::discrete_distribution<> rangeDist(weights.begin(), weights.end());
    int rangeSelected = rangeDist(gen_);

    int newPosition;
    if (rangeSelected == 0)
    {
        if (range1 > 0)
        {
            std::uniform_int_distribution<> newPosDist(0, index1 - 1);
            newPosition = newPosDist(gen_);
        }
        else
        {
            newPosition = 0;
        }
    }
    else
    {
        if (range2 > 0)
        {
            std::uniform_int_distribution<> newPosDist(index2 + 1, numberOfCities - 1);
            newPosition = newPosDist(gen_);
        }
        else
        {
            newPosition = numberOfCities - 1;
        }
    }

    std::vector<int> tempBlock(vec.begin() + index1, vec.begin() + index2 + 1);
    vec.erase(vec.begin() + index1, vec.begin() + index2 + 1);
    if (newPosition > index1)
        newPosition -= tempBlock.size();

    vec.insert(vec.begin() + newPosition, tempBlock.begin(), tempBlock.end());
}

void NSGAIIReproductionMechanism::distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents)
{
    std::vector<int> partitionPoints(numberOfAgents - 1);

    std::uniform_int_distribution<> dis(1, numberOfCities - 1);

    for (int &point : partitionPoints)
    {
        point = dis(gen_);
    }

    std::sort(partitionPoints.begin(), partitionPoints.end());

    partitionPoints.insert(partitionPoints.begin(), 0);
    partitionPoints.push_back(numberOfCities);

    for (int i = 0; i < numberOfAgents; ++i)
    {
        vec[vec.size() - numberOfAgents + i] = partitionPoints[i + 1] - partitionPoints[i];
    }
}

NSGAIIReproductionMechanism::ReproductionChromosome::ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities)
{
    chromosome_ = chromosome;
    fitnessValues_ = fitnessCalculator->calculateFitness(chromosome, cities);
}

bool NSGAIIReproductionMechanism::ReproductionChromosome::dominates(const ReproductionChromosome &other) const
{
    return (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) < other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) <= other.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) < other.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}

double NSGAIIReproductionMechanism::ReproductionChromosome::getFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

double NSGAIIReproductionMechanism::ReproductionChromosome::getObjectiveFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

Chromosome &NSGAIIReproductionMechanism::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}

bool NSGAIIReproductionMechanism::ReproductionChromosome::operator<(const ReproductionChromosome& other) const {
    auto primary_fitness = getFitness(Fitness::MAXPATHLENGTH); // Replace with actual fitness enum or identifier
    auto other_primary_fitness = other.getFitness(Fitness::TOTALPATHDISTANCE);
    if (primary_fitness != other_primary_fitness) {
        return primary_fitness < other_primary_fitness;
    }
    return getFitness(Fitness::TOTALPATHDISTANCE) < other.getFitness(Fitness::TOTALPATHDISTANCE); // Replace with actual fitness enum or identifier
}
