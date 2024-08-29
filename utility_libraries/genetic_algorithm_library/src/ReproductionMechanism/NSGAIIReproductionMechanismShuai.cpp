#include "NSGAIIReproductionMechanismShuai.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <set>

NSGAIIReproductionMechanismShuai::NSGAIIReproductionMechanismShuai(
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

Population NSGAIIReproductionMechanismShuai::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities,
    int iterationNumber)
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

    // Elitism selection
    auto parents = ElitismSelection(tempNewGeneration);

    newGeneration.insert(newGeneration.end(), parents.begin(), parents.end());

    // Reproduction to create the new population
    while (newGeneration.size() < oldPopulation.getPopulationList().size())
    {
        // Tournament selection to select parents
        ReproductionChromosome parent1 = TournamentSelection(parents);
        ReproductionChromosome parent2 = TournamentSelection(parents);

        // PMX crossover
        ReproductionChromosome offspring1(parent1.getChromosome(), fitnessCalculator_, initialAgentPoses, cities);
        ReproductionChromosome offspring2(parent2.getChromosome(), fitnessCalculator_, initialAgentPoses, cities);
        ProximityBasedCrossover(parent1, parent2, offspring1, initialAgentPoses, cities, true);
        ProximityBasedCrossover(parent1, parent2, offspring2, initialAgentPoses, cities, false);

        // Mutation
        Mutate(offspring1, cities);
        Mutate(offspring2, cities);

        // Ensure offspring are unique before adding to new generation
        if (std::find_if(newGeneration.begin(), newGeneration.end(), [&](ReproductionChromosome &chr)
                         { return chr.getChromosome() == offspring1.getChromosome(); }) == newGeneration.end())
        {
            newGeneration.push_back(offspring1);
        }

        if (std::find_if(newGeneration.begin(), newGeneration.end(), [&](ReproductionChromosome &chr)
                         { return chr.getChromosome() == offspring2.getChromosome(); }) == newGeneration.end())
        {
            newGeneration.push_back(offspring2);
        }
    }

    std::vector<Chromosome> finalGeneration;
    for (auto &chromosome : newGeneration)
    {
        finalGeneration.push_back(chromosome.getChromosome());
    }

    return Population(finalGeneration);
}

void NSGAIIReproductionMechanismShuai::AssignCrowdingDistance(std::vector<ReproductionChromosome> &front)
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

std::vector<NSGAIIReproductionMechanismShuai::ReproductionChromosome> NSGAIIReproductionMechanismShuai::ElitismSelection(const std::vector<ReproductionChromosome> &population)
{
    std::vector<ReproductionChromosome> selected;
    int halfPopulationSize = population.size() / 2;

    // Group individuals by rank
    std::map<int, std::vector<ReproductionChromosome>> rankGroups;
    for (const auto &individual : population)
    {
        rankGroups[individual.rank].push_back(individual);
    }

    // Select individuals by rank
    for (auto &rankGroup : rankGroups)
    {
        if (selected.size() + rankGroup.second.size() <= halfPopulationSize)
        {
            // Add all individuals in this rank
            selected.insert(selected.end(), rankGroup.second.begin(), rankGroup.second.end());
        }
        else
        {
            // Sort by crowding distance if needed
            std::sort(rankGroup.second.begin(), rankGroup.second.end(), [](const ReproductionChromosome &a, const ReproductionChromosome &b)
                      { return a.crowdingDistance > b.crowdingDistance; });

            // Add only the required number of individuals
            selected.insert(selected.end(), rankGroup.second.begin(), rankGroup.second.begin() + (halfPopulationSize - selected.size()));
            break;
        }
    }

    return selected;
}

NSGAIIReproductionMechanismShuai::ReproductionChromosome NSGAIIReproductionMechanismShuai::TournamentSelection(const std::vector<ReproductionChromosome> &population)
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

void NSGAIIReproductionMechanismShuai::ProximityBasedCrossover(
    ReproductionChromosome &parent1,
    ReproductionChromosome &parent2,
    ReproductionChromosome &offspring,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities,
    bool forward)
{
    auto parent1Genes = parent1.getChromosome().getGenes();
    auto parent2Genes = parent2.getChromosome().getGenes();
    auto numberOfGenes = parent1Genes.size();
    auto fixedElementsStart = numberOfGenes - initialAgentPoses.size();

    std::vector<int> offspringGenes;
    offspringGenes.reserve(numberOfGenes);

    parent1Genes.resize(parent1Genes.size() - initialAgentPoses.size());
    parent2Genes.resize(parent2Genes.size() - initialAgentPoses.size());

    // Select a random city to start with
    std::uniform_int_distribution<> dist(0, numberOfGenes - 1);
    int selectedIndexParent1 = dist(gen_);
    int selectedIndexParent2 = std::distance(parent2Genes.begin(), std::find(parent2Genes.begin(), parent2Genes.end(), parent1Genes[selectedIndexParent1]));
    int currentCity = parent1Genes[selectedIndexParent1];
    offspringGenes.push_back(currentCity);

    // Remove the selected city from both parent chromosomes
    parent1Genes.erase(std::remove(parent1Genes.begin(), parent1Genes.end(), currentCity), parent1Genes.end());
    parent2Genes.erase(std::remove(parent2Genes.begin(), parent2Genes.end(), currentCity), parent2Genes.end());

    // Direction of iteration: forward or backward
    int direction = forward ? 0 : -1;

    while (offspringGenes.size() < numberOfGenes - initialAgentPoses.size())
    {
        // Find the next city based on proximity
        int nextCity = -1;
        double minDistance = std::numeric_limits<double>::max();

        // Check the next or previous city in parent1's chromosome
        int nextIndexParent1 = (selectedIndexParent1 + direction + parent1Genes.size()) % parent1Genes.size();
        if (nextIndexParent1 >= 0 && nextIndexParent1 < parent1Genes.size())
        {
            int candidateCity = parent1Genes[nextIndexParent1];
            double distance = HaversineDistance::calculateDistance(
                cities[currentCity].latitude, cities[currentCity].longitude, cities[currentCity].altitude,
                cities[candidateCity].latitude, cities[candidateCity].longitude, cities[candidateCity].altitude);

            if (distance < minDistance)
            {
                minDistance = distance;
                nextCity = candidateCity;
            }
        }

        int nextIndexParent2 = (selectedIndexParent2 + direction + parent2Genes.size()) % parent2Genes.size();
        if (nextIndexParent2 >= 0 && nextIndexParent2 < parent2Genes.size())
        {
            int candidateCity = parent2Genes[nextIndexParent2];
            double distance = HaversineDistance::calculateDistance(
                cities[currentCity].latitude, cities[currentCity].longitude, cities[currentCity].altitude,
                cities[candidateCity].latitude, cities[candidateCity].longitude, cities[candidateCity].altitude);

            if (distance < minDistance)
            {
                minDistance = distance;
                nextCity = candidateCity;
            }
        }

        if (nextCity != -1)
        {
            offspringGenes.push_back(nextCity);
            currentCity = nextCity;

            auto itParent1 = std::find(parent1Genes.begin(), parent1Genes.end(), nextCity);
            selectedIndexParent1 = std::distance(parent1Genes.begin(), itParent1);
            auto itParent2 = std::find(parent2Genes.begin(), parent2Genes.end(), nextCity);
            selectedIndexParent2 = std::distance(parent2Genes.begin(), itParent2);

            // Remove the selected city from both parent chromosomes
            if (itParent1 != parent1Genes.end())
            {
                parent1Genes.erase(itParent1);
            }

            if (itParent2 != parent2Genes.end())
            {
                parent2Genes.erase(itParent2);
            }
        }
        else
        {
            break;
        }
    }

    for (int i = 0; i < initialAgentPoses.size(); ++i)
    {
        offspringGenes.emplace_back(parent1.getChromosome().getGenes()[fixedElementsStart + i]);
    }

    Chromosome newOffspring(offspringGenes, parent1.getChromosome().getNumberOfCities());

    offspring = ReproductionChromosome(newOffspring, fitnessCalculator_, initialAgentPoses, cities);
}

void NSGAIIReproductionMechanismShuai::Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities)
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

std::vector<std::vector<NSGAIIReproductionMechanismShuai::ReproductionChromosome>> NSGAIIReproductionMechanismShuai::FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
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
            for (size_t q = 0; q < dominatedChromosomes.size(); ++q) // auto q : dominatedChromosomes
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

void NSGAIIReproductionMechanismShuai::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{
    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void NSGAIIReproductionMechanismShuai::flipInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuai::swapInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuai::lSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuai::rSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuai::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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


void NSGAIIReproductionMechanismShuai::distributeCities(std::vector<int> &outVec, int numberOfCities, int numberOfAgents)
{
    if (numberOfAgents == 0)
    {
        throw std::runtime_error("Number of agents cannot be zero.");
    }

    if (numberOfCities < numberOfAgents)
    {
        throw std::runtime_error("Number of cities must be at least as large as the number of agents.");
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    std::set<int> separatorsSet;
    while (separatorsSet.size() < numberOfAgents - 1)
    {
        int separator = std::uniform_int_distribution<>(1, numberOfCities - 1)(gen);
        separatorsSet.insert(separator);
    }

    std::vector<int> separators(separatorsSet.begin(), separatorsSet.end());

    separators.insert(separators.begin(), 0);
    separators.push_back(numberOfCities);

    std::vector<int> vec;
    for (int i = 1; i < separators.size(); ++i)
    {
        int diff = separators[i] - separators[i - 1];
        vec.push_back(diff);
    }

    if (outVec.size() < numberOfAgents)
    {
        throw std::runtime_error("Output vector size is smaller than the number of agents.");
    }

    for (size_t i = 0; i < numberOfAgents; ++i)
    {
        if (vec[i] == 0)
        {
            throw std::runtime_error("Number of cities must be at least as large as the number of agents.");
        }

        outVec[outVec.size() - numberOfAgents + i] = vec[i];
    }
}

NSGAIIReproductionMechanismShuai::ReproductionChromosome::ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities)
{
    chromosome_ = chromosome;
    fitnessValues_ = fitnessCalculator->calculateFitness(chromosome, cities);
}

bool NSGAIIReproductionMechanismShuai::ReproductionChromosome::dominates(const ReproductionChromosome &other) const
{
    return (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) < other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) <= other.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) < other.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}

double NSGAIIReproductionMechanismShuai::ReproductionChromosome::getFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

double NSGAIIReproductionMechanismShuai::ReproductionChromosome::getObjectiveFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

Chromosome &NSGAIIReproductionMechanismShuai::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}

bool NSGAIIReproductionMechanismShuai::ReproductionChromosome::operator<(const ReproductionChromosome &other) const
{
    auto primary_fitness = getFitness(Fitness::MAXPATHLENGTH); // Replace with actual fitness enum or identifier
    auto other_primary_fitness = other.getFitness(Fitness::TOTALPATHDISTANCE);
    if (primary_fitness != other_primary_fitness)
    {
        return primary_fitness < other_primary_fitness;
    }
    return getFitness(Fitness::TOTALPATHDISTANCE) < other.getFitness(Fitness::TOTALPATHDISTANCE); // Replace with actual fitness enum or identifier
}
