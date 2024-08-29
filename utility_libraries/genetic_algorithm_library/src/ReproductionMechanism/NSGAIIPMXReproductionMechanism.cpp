#include "NSGAIIPMXReproductionMechanism.h"
#include <iostream>
#include <algorithm>
#include <limits>

NSGAIIPMXReproductionMechanism::NSGAIIPMXReproductionMechanism(
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

Population NSGAIIPMXReproductionMechanism::Reproduce(
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
        PMXCrossover(parent1, parent2, offspring1, offspring2, initialAgentPoses, cities);

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

void NSGAIIPMXReproductionMechanism::AssignCrowdingDistance(std::vector<ReproductionChromosome> &front)
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

std::vector<NSGAIIPMXReproductionMechanism::ReproductionChromosome> NSGAIIPMXReproductionMechanism::ElitismSelection(const std::vector<ReproductionChromosome>& population) {
    std::vector<ReproductionChromosome> selected;
    int halfPopulationSize = population.size() / 2;

    // Group individuals by rank
    std::map<int, std::vector<ReproductionChromosome>> rankGroups;
    for (const auto& individual : population) {
        rankGroups[individual.rank].push_back(individual);
    }

    // Select individuals by rank
    for (auto& rankGroup : rankGroups) {
        if (selected.size() + rankGroup.second.size() <= halfPopulationSize) {
            // Add all individuals in this rank
            selected.insert(selected.end(), rankGroup.second.begin(), rankGroup.second.end());
        } else {
            // Sort by crowding distance if needed
            std::sort(rankGroup.second.begin(), rankGroup.second.end(), [](const ReproductionChromosome& a, const ReproductionChromosome& b) {
                return a.crowdingDistance > b.crowdingDistance;
            });

            // Add only the required number of individuals
            selected.insert(selected.end(), rankGroup.second.begin(), rankGroup.second.begin() + (halfPopulationSize - selected.size()));
            break;
        }
    }

    return selected;
}

NSGAIIPMXReproductionMechanism::ReproductionChromosome NSGAIIPMXReproductionMechanism::TournamentSelection(const std::vector<ReproductionChromosome> &population)
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

void NSGAIIPMXReproductionMechanism::PMXCrossover(
    ReproductionChromosome &parent1,
    ReproductionChromosome &parent2,
    ReproductionChromosome &offspring1,
    ReproductionChromosome &offspring2,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    auto parent1Genes = parent1.getChromosome().getGenes();
    auto parent2Genes = parent2.getChromosome().getGenes();
    auto numberOfGenes = parent1Genes.size();
    auto fixedElementsStart = numberOfGenes - initialAgentPoses.size();

    std::uniform_int_distribution<> dist(0, fixedElementsStart - 1);
    int crossoverPoint1 = dist(gen_);
    int crossoverPoint2 = dist(gen_);

    while (crossoverPoint1 == crossoverPoint2)
    {
        crossoverPoint2 = dist(gen_);
    }

    if (crossoverPoint1 > crossoverPoint2)
    {
        std::swap(crossoverPoint1, crossoverPoint2);
    }

    auto offspring1Genes = parent1Genes;
    auto offspring2Genes = parent2Genes;

    // PMX Crossover between crossoverPoint1 and crossoverPoint2
    std::unordered_map<int, int> mapping1;
    std::unordered_map<int, int> mapping2;

    for (int i = crossoverPoint1; i <= crossoverPoint2; ++i)
    {
        offspring1Genes[i] = parent2Genes[i];
        offspring2Genes[i] = parent1Genes[i];
        mapping1[parent2Genes[i]] = parent1Genes[i];
        mapping2[parent1Genes[i]] = parent2Genes[i];
    }

    auto resolveConflicts = [&](std::vector<int>& offspringGenes, const std::unordered_map<int, int>& mapping, int crossoverPoint1, int crossoverPoint2)
    {
        auto resolveGene = [&](int gene) {
            while (mapping.find(gene) != mapping.end()) {
                gene = mapping.at(gene);
            }
            return gene;
        };

        for (int i = 0; i < crossoverPoint1; ++i)
        {
            offspringGenes[i] = resolveGene(offspringGenes[i]);
        }

        for (int i = crossoverPoint2 + 1; i < fixedElementsStart; ++i)
        {
            offspringGenes[i] = resolveGene(offspringGenes[i]);
        }
    };

    resolveConflicts(offspring1Genes, mapping1, crossoverPoint1, crossoverPoint2);
    resolveConflicts(offspring2Genes, mapping2, crossoverPoint1, crossoverPoint2);

    // Ensure the last initialAgentPoses.size() elements are not modified
    for (size_t i = fixedElementsStart; i < numberOfGenes; ++i)
    {
        offspring1Genes[i] = parent1Genes[i];
        offspring2Genes[i] = parent2Genes[i];
    }

    Chromosome newOffspring1(offspring1Genes, parent1.getChromosome().getNumberOfCities());
    Chromosome newOffspring2(offspring2Genes, parent1.getChromosome().getNumberOfCities());

    offspring1 = ReproductionChromosome(newOffspring1, fitnessCalculator_, initialAgentPoses, cities);
    offspring2 = ReproductionChromosome(newOffspring2, fitnessCalculator_, initialAgentPoses, cities);
}



void NSGAIIPMXReproductionMechanism::Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities)
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

std::vector<std::vector<NSGAIIPMXReproductionMechanism::ReproductionChromosome>> NSGAIIPMXReproductionMechanism::FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
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

void NSGAIIPMXReproductionMechanism::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{
    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void NSGAIIPMXReproductionMechanism::flipInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIPMXReproductionMechanism::swapInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIPMXReproductionMechanism::lSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIPMXReproductionMechanism::rSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIPMXReproductionMechanism::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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

void NSGAIIPMXReproductionMechanism::distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents)
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

NSGAIIPMXReproductionMechanism::ReproductionChromosome::ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities)
{
    chromosome_ = chromosome;
    fitnessValues_ = fitnessCalculator->calculateFitness(chromosome, cities);
}

bool NSGAIIPMXReproductionMechanism::ReproductionChromosome::dominates(const ReproductionChromosome &other) const
{
    return (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) < other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) <= other.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) < other.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}

double NSGAIIPMXReproductionMechanism::ReproductionChromosome::getFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

double NSGAIIPMXReproductionMechanism::ReproductionChromosome::getObjectiveFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

Chromosome &NSGAIIPMXReproductionMechanism::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}

bool NSGAIIPMXReproductionMechanism::ReproductionChromosome::operator<(const ReproductionChromosome &other) const
{
    auto primary_fitness = getFitness(Fitness::MAXPATHLENGTH); // Replace with actual fitness enum or identifier
    auto other_primary_fitness = other.getFitness(Fitness::TOTALPATHDISTANCE);
    if (primary_fitness != other_primary_fitness)
    {
        return primary_fitness < other_primary_fitness;
    }
    return getFitness(Fitness::TOTALPATHDISTANCE) < other.getFitness(Fitness::TOTALPATHDISTANCE); // Replace with actual fitness enum or identifier
}
