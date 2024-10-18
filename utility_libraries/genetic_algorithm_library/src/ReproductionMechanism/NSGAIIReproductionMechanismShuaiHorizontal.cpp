#include "NSGAIIReproductionMechanismShuaiHorizontal.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <set>

NSGAIIReproductionMechanismShuaiHorizontal::NSGAIIReproductionMechanismShuaiHorizontal(
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

Population NSGAIIReproductionMechanismShuaiHorizontal::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities,
    int iterationNumber)
{

    teamSize_ = initialAgentPoses.size();

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

    auto libary = getHighValueSubChromosomeLibrary(tempNewGeneration, cities, initialAgentPoses.size());

    auto parents = ElitismSelection(tempNewGeneration);

    newGeneration.insert(newGeneration.end(), parents.begin(), parents.end());

    size_t targetPopulationSize = oldPopulation.getPopulationList().size();

    while (newGeneration.size() < targetPopulationSize)
    {
        ReproductionChromosome parent1 = TournamentSelection(parents);
        while (isParentMalformed(parent1, initialAgentPoses.size()))
        {
            parent1 = TournamentSelection(parents);
        }
        ReproductionChromosome parent2 = TournamentSelection(parents);
        while (isParentMalformed(parent2, initialAgentPoses.size()))
        {
            parent2 = TournamentSelection(parents);
        }

        ReproductionChromosome offspring1(parent1.getChromosome(), fitnessCalculator_, initialAgentPoses, cities);
        ReproductionChromosome offspring2(parent2.getChromosome(), fitnessCalculator_, initialAgentPoses, cities);

        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        auto roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            // PMX crossover
            ProximityBasedCrossover(parent1, parent2, offspring1, initialAgentPoses, cities, true);
            ProximityBasedCrossover(parent1, parent2, offspring2, initialAgentPoses, cities, false);
        }
        else if (roulette < 0.75)
        {
            ReproductionChromosome parent1 = HighMinMaxTournamentSelection(parents);
            while (isParentMalformed(parent1, initialAgentPoses.size()))
            {
                parent1 = HighMinMaxTournamentSelection(parents);
            }
            ReproductionChromosome parent2 = HighMinMaxTournamentSelection(parents);
            while (isParentMalformed(parent2, initialAgentPoses.size()))
            {
                parent2 = HighMinMaxTournamentSelection(parents);
            }
            std::vector<int> offspringGenes1;
            offspringGenes1 = parent1.getChromosome().getGenes();
            std::vector<int> offspringGenes2;
            offspringGenes2 = parent2.getChromosome().getGenes();
            offspring1.chromosome_.setGenes(horizontalGeneTransfer(offspringGenes1, libary, parent1.getChromosome().getNumberOfCities(), cities));
            offspring2.chromosome_.setGenes(horizontalGeneTransfer(offspringGenes1, libary, parent2.getChromosome().getNumberOfCities(), cities));
        }
        else
        {
            Mutate(offspring1, cities, libary);
            Mutate(offspring2, cities, libary);
        }

        // Check uniqueness and ensure we don't exceed targetPopulationSize
        auto it1 = std::find_if(newGeneration.begin(), newGeneration.end(),
                                [&offspring1](ReproductionChromosome &existing)
                                {
                                    return existing.getChromosome().getGenes() == offspring1.getChromosome().getGenes();
                                });

        if (it1 == newGeneration.end())
        {
            newGeneration.push_back(offspring1);

            // Check if the population size has reached the limit
            if (newGeneration.size() >= targetPopulationSize)
            {
                break;
            }
        }
        else
        {
            auto a = 1;
        }

        auto it2 = std::find_if(newGeneration.begin(), newGeneration.end(),
                                [&offspring2](ReproductionChromosome &existing)
                                {
                                    return existing.getChromosome().getGenes() == offspring2.getChromosome().getGenes();
                                });

        if (it2 == newGeneration.end())
        {
            newGeneration.push_back(offspring2);

            // Check if the population size has reached the limit
            if (newGeneration.size() >= targetPopulationSize)
            {
                break;
            }
        }
        else
        {
            auto a = 1;
        }
    }

    std::vector<Chromosome> finalGeneration;
    for (auto &chromosome : newGeneration)
    {
        finalGeneration.push_back(chromosome.getChromosome());
    }

    return Population(finalGeneration);
}

bool NSGAIIReproductionMechanismShuaiHorizontal::isParentMalformed(ReproductionChromosome chromosome, int teamSize)
{
    const std::vector<int> &genes = chromosome.getChromosome().getGenes();
    int numberOfCities = chromosome.getChromosome().getNumberOfCities();

    // Check if the chromosome size is not equal to numberOfCities + teamSize
    if (genes.size() != numberOfCities + teamSize)
    {
        return true;
    }

    // Check if the first numberOfCities genes are not within the range of [0, numberOfCities-1]
    std::unordered_set<int> uniqueGenes;
    for (int i = 0; i < numberOfCities; ++i)
    {
        if (genes[i] < 0 || genes[i] >= numberOfCities)
        {
            return true;
        }
        uniqueGenes.insert(genes[i]);
    }

    // Check for duplicates in the first numberOfCities genes
    if (uniqueGenes.size() != numberOfCities)
    {
        return true;
    }

    // Check if there are any duplicates across the entire chromosome except the last teamSize elements
    uniqueGenes.clear();
    for (int i = 0; i < genes.size() - teamSize; ++i)
    {
        if (uniqueGenes.count(genes[i]))
        {
            return true;
        }
        uniqueGenes.insert(genes[i]);
    }

    return false;
}

bool NSGAIIReproductionMechanismShuaiHorizontal::isParentMalformed(Chromosome chromosome, int teamSize)
{
    const std::vector<int> &genes = chromosome.getGenes();
    int numberOfCities = chromosome.getNumberOfCities();

    // Check if the chromosome size is not equal to numberOfCities + teamSize
    if (genes.size() != numberOfCities + teamSize)
    {
        return true;
    }

    // Check if the first numberOfCities genes are not within the range of [0, numberOfCities-1]
    std::unordered_set<int> uniqueGenes;
    for (int i = 0; i < numberOfCities; ++i)
    {
        if (genes[i] < 0 || genes[i] >= numberOfCities)
        {
            return true;
        }
        uniqueGenes.insert(genes[i]);
    }

    // Check for duplicates in the first numberOfCities genes
    if (uniqueGenes.size() != numberOfCities)
    {
        return true;
    }

    // Check if there are any duplicates across the entire chromosome except the last teamSize elements
    uniqueGenes.clear();
    for (int i = 0; i < genes.size() - teamSize; ++i)
    {
        if (uniqueGenes.count(genes[i]))
        {
            return true;
        }
        uniqueGenes.insert(genes[i]);
    }

    return false;
}

void NSGAIIReproductionMechanismShuaiHorizontal::AssignCrowdingDistance(std::vector<ReproductionChromosome> &front)
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

std::vector<NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome> NSGAIIReproductionMechanismShuaiHorizontal::ElitismSelection(const std::vector<ReproductionChromosome> &population)
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

NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome NSGAIIReproductionMechanismShuaiHorizontal::TournamentSelection(const std::vector<ReproductionChromosome> &population)
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

NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome NSGAIIReproductionMechanismShuaiHorizontal::HighMinMaxTournamentSelection(const std::vector<ReproductionChromosome> &population)
{
    std::uniform_int_distribution<> dis(0, population.size() - 1);

    ReproductionChromosome best = population[dis(gen_)];
    for (int i = 1; i < sampleSize_; ++i)
    {
        ReproductionChromosome candidate = population[dis(gen_)];
        if (candidate.fitnessValues_[Fitness::MAXPATHLENGTH] < best.fitnessValues_[Fitness::MAXPATHLENGTH])
        {
            best = candidate;
        }
    }
    return best;
}

void NSGAIIReproductionMechanismShuaiHorizontal::ProximityBasedCrossover(
    ReproductionChromosome &parent1,
    ReproductionChromosome &parent2,
    ReproductionChromosome &offspring,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities,
    bool forward)
{
    int initialSize = parent1.chromosome_.getGenes().size();
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

    if (offspring.chromosome_.getGenes().size() > initialSize)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
    }

    offspring = ReproductionChromosome(newOffspring, fitnessCalculator_, initialAgentPoses, cities);
}

void NSGAIIReproductionMechanismShuaiHorizontal::Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities, std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> library)
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
        auto roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            distributeCities(genes, chromosome.getChromosome().getNumberOfCities(), chromosome.getChromosome().getNumberOfAgents());
        }
        else
        {
            distributeCitiesStep(genes, chromosome.getChromosome().getNumberOfCities(), chromosome.getChromosome().getNumberOfAgents());
        }
    }

    Chromosome newChromosome(genes, chromosome.getChromosome().getNumberOfCities());
    chromosome.chromosome_ = newChromosome;
    chromosome.fitnessValues_ = fitnessCalculator_->calculateFitness(newChromosome, cities);
}

std::vector<std::vector<NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome>> NSGAIIReproductionMechanismShuaiHorizontal::FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
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

void NSGAIIReproductionMechanismShuaiHorizontal::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{
    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void NSGAIIReproductionMechanismShuaiHorizontal::flipInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }
    if (numberOfCities == 0)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
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

void NSGAIIReproductionMechanismShuaiHorizontal::swapInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }
    if (numberOfCities == 0)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
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

void NSGAIIReproductionMechanismShuaiHorizontal::lSlideInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }
    if (numberOfCities == 0)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
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

void NSGAIIReproductionMechanismShuaiHorizontal::rSlideInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }
    if (numberOfCities == 0)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
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

void NSGAIIReproductionMechanismShuaiHorizontal::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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
            if (index1 == 0)
            {
                throw std::runtime_error("Population is empty, cannot perform tournament selection.");
            }
            std::uniform_int_distribution<> newPosDist(0, index1 - 1);
            newPosition = newPosDist(gen_);
        }
        {
            newPosition = 0;
        }
    }
    else
    {
        if (range2 > 0)
        {
            if (index2 + 1 == numberOfCities)
            {
                throw std::runtime_error("Population is empty, cannot perform tournament selection.");
            }
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

void NSGAIIReproductionMechanismShuaiHorizontal::distributeCities(std::vector<int> &outVec, int numberOfCities, int numberOfAgents)
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

        outVec[numberOfCities + i] = vec[i];
    }
}

void NSGAIIReproductionMechanismShuaiHorizontal::distributeCitiesMinSum(std::vector<int> &outVec, int numberOfCities, int numberOfAgents)
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

    // Choose the number of large paths and their indices
    std::set<int> largePathIndices;
    int numLargePaths = std::uniform_int_distribution<>(1, numberOfAgents / 3)(gen);

    while (largePathIndices.size() < numLargePaths)
    {
        int index = std::uniform_int_distribution<>(0, numberOfAgents - 1)(gen);
        largePathIndices.insert(index);
    }

    // Calculate maximum size for small paths
    int maxSmallPathSize = numberOfCities / numberOfAgents;

    // Determine the size for each path
    std::vector<int> pathSizes(numberOfAgents, 0);
    int remainingCities = numberOfCities;

    for (int i = 0; i < numberOfAgents; ++i)
    {
        if (largePathIndices.find(i) == largePathIndices.end())
        {
            // Small path
            int randomValue = std::uniform_int_distribution<>(0, 2 * maxSmallPathSize)(gen) - maxSmallPathSize;
            int smallPathSize = std::max(1, randomValue);                                        // Ensure at least 1 city
            smallPathSize = std::min(smallPathSize, remainingCities - (numberOfAgents - i - 1)); // Ensure enough cities left for others

            pathSizes[i] = smallPathSize;
            remainingCities -= smallPathSize;
        }
    }

    // Distribute remaining cities to large paths
    for (int i : largePathIndices)
    {
        if (remainingCities > 0)
        {
            int largePathSize = std::uniform_int_distribution<>(1, remainingCities)(gen);
            pathSizes[i] = largePathSize;
            remainingCities -= largePathSize;
        }
    }

    // If there's any remaining city, add them to the last large path
    if (remainingCities > 0)
    {
        int lastLargePathIndex = *largePathIndices.rbegin();
        pathSizes[lastLargePathIndex] += remainingCities;
    }

    // Ensure no path has zero length
    for (size_t i = 0; i < numberOfAgents; ++i)
    {
        if (pathSizes[i] == 0)
        {
            // Find a non-zero path to take from
            std::vector<int> candidates;
            for (size_t j = 0; j < numberOfAgents; ++j)
            {
                if (j != i && pathSizes[j] > 1) // Ensure the source path remains non-zero after giving away a city
                {
                    candidates.push_back(j);
                }
            }

            if (!candidates.empty())
            {
                int donorIndex = candidates[std::uniform_int_distribution<>(0, candidates.size() - 1)(gen)];
                pathSizes[donorIndex] -= 1;
                pathSizes[i] += 1;
            }
            else
            {
                throw std::runtime_error("Failed to adjust paths to avoid zero-length paths.");
            }
        }
    }

    if (outVec.size() < numberOfAgents)
    {
        throw std::runtime_error("Output vector size is smaller than the number of agents.");
    }

    for (size_t i = 0; i < numberOfAgents; ++i)
    {
        if (pathSizes[i] == 0)
        {
            throw std::runtime_error("PathSize is equal to zero.");
        }
        outVec[numberOfCities + i] = pathSizes[i];
    }
}

void NSGAIIReproductionMechanismShuaiHorizontal::distributeCitiesStep(std::vector<int> &outVec, int numberOfCities, int numberOfAgents)
{
    std::vector<int> citiesPerAgent;

    int largestPath = 0;
    for (size_t i = numberOfCities; i < numberOfCities + numberOfAgents; i++)
    {
        if (outVec[i] > largestPath)
        {
            largestPath = outVec[i];
        }
        citiesPerAgent.emplace_back(outVec[i]);
    }

    std::uniform_int_distribution<> distr(numberOfCities, numberOfCities + numberOfAgents - 1);

    double k = 5.0;

    double p = 1.0 - exp(-k * (numberOfAgents) / numberOfCities);

    std::geometric_distribution<> geodis(p);

    int donation{geodis(gen_) + 1};

    while (donation >= largestPath)
    {
        donation = geodis(gen_) + 1;
    }

    int donor = distr(gen_);
    while (outVec[donor] <= donation)
    {
        donor = distr(gen_);
    }
    int donee = distr(gen_);
    while (donee == donor)
    {
        donee = distr(gen_);
    }

    outVec[donor] -= donation;
    outVec[donee] += donation;
}

void NSGAIIReproductionMechanismShuaiHorizontal::distributeCitiesBalanced(std::vector<int> &outVec, int numberOfCities, int numberOfAgents)
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

    // Calculate the mean path size
    int meanPathSize = numberOfCities / numberOfAgents;

    // Use a smaller standard deviation for the normal distribution
    double standardDeviation = meanPathSize / 10.0; // Smaller standard deviation for reduced variability

    // Determine the size for each path
    std::vector<int> pathSizes(numberOfAgents, 0);
    int remainingCities = numberOfCities;

    for (int i = 0; i < numberOfAgents; ++i)
    {
        // Centered around the meanPathSize, use a normal distribution
        std::normal_distribution<> d(meanPathSize, standardDeviation);

        int pathSize = std::round(d(gen));                                         // Generate random size from the distribution
        pathSize = std::max(1, pathSize);                                          // Ensure at least 1 city
        pathSize = std::min(pathSize, remainingCities - (numberOfAgents - i - 1)); // Ensure enough cities left for others

        pathSizes[i] = pathSize;
        remainingCities -= pathSize;
    }

    // If there's any remaining city, distribute them randomly to agents
    while (remainingCities > 0)
    {
        int randomAgent = std::uniform_int_distribution<>(0, numberOfAgents - 1)(gen);
        pathSizes[randomAgent]++;
        remainingCities--;
    }

    // Ensure no path has zero length
    for (size_t i = 0; i < numberOfAgents; ++i)
    {
        if (pathSizes[i] == 0)
        {
            // Find a non-zero path to take from
            std::vector<int> candidates;
            for (size_t j = 0; j < numberOfAgents; ++j)
            {
                if (j != i && pathSizes[j] > 1) // Ensure the source path remains non-zero after giving away a city
                {
                    candidates.push_back(j);
                }
            }

            if (!candidates.empty())
            {
                int donorIndex = candidates[std::uniform_int_distribution<>(0, candidates.size() - 1)(gen)];
                pathSizes[donorIndex] -= 1;
                pathSizes[i] += 1;
            }
            else
            {
                throw std::runtime_error("Failed to adjust paths to avoid zero-length paths.");
            }
        }
    }

    // Ensure no path has zero length
    for (size_t i = 0; i < numberOfAgents; ++i)
    {
        if (pathSizes[i] == 0)
        {
            throw std::runtime_error("Still have zero path lengths.");
        }
    }

    if (outVec.size() < numberOfAgents)
    {
        throw std::runtime_error("Output vector size is smaller than the number of agents.");
    }

    for (size_t i = 0; i < numberOfAgents; ++i)
    {
        if (pathSizes[i] == 0)
        {
            throw std::runtime_error("PathSize is equal to zero.");
        }
        outVec[numberOfCities + i] = pathSizes[i];
    }
}

std::vector<int> NSGAIIReproductionMechanismShuaiHorizontal::horizontalGeneTransfer(
    std::vector<int> &chromosome,
    std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> &library,
    int numberOfCities,
    std::vector<Position> &cities)
{
    int initialSize = chromosome.size();

    auto subChromosomes = extractSubChromosomes(chromosome, numberOfCities);

    std::vector<int> cityChromosomes(chromosome.begin(), chromosome.begin() + numberOfCities);

    std::pair<std::vector<int>, int> longestPathSubChromosome = getLongestSubChromosomePath(subChromosomes, cities);
    int longestPathRobotID = longestPathSubChromosome.second;
    // Create new chromosome
    std::vector<std::vector<int>> newChromosome(teamSize_);
    // newChromosome.emplace_back(longestPathSubChromosome);

    std::vector<int> newChromosomeWhole;
    // newChromosomeWhole.insert(newChromosomeWhole.end(), longestPathSubChromosome.first.begin(), longestPathSubChromosome.first.end());

    for (int i = 0; i < teamSize_; i++)
    {
        bool isInitialized = false;
        for (size_t attempts = 0; attempts < library.first.size(); attempts++)
        {
            if (i == longestPathRobotID)
            {
                continue;
            }
            int index = library.second(gen_);
            auto selectedSubChromosome = library.first[index];
            if (hasNoCommonElements(newChromosomeWhole, selectedSubChromosome) && hasNoCommonElements(selectedSubChromosome, longestPathSubChromosome.first))
            {
                newChromosome[i] = selectedSubChromosome;
                newChromosomeWhole.insert(newChromosomeWhole.end(), selectedSubChromosome.begin(), selectedSubChromosome.end());
                isInitialized = true;
                break;
            }
        }
    }

    // Place the longest path subchromosome at the correct index (robotID) last
    newChromosome[longestPathRobotID] = longestPathSubChromosome.first;

    bool inserted = false;
    if (longestPathRobotID > 0)
    {
        // Check previous subchromosomes for a non-empty one
        for (int i = longestPathRobotID - 1; i >= 0; --i)
        {
            if (!newChromosome[i].empty())
            {
                // Find the insertion position after the last element of the previous subchromosome
                auto insertionPos = std::find(newChromosomeWhole.begin(), newChromosomeWhole.end(), newChromosome[i].back());
                if (insertionPos != newChromosomeWhole.end())
                {
                    // Insert after the last element of the found subchromosome
                    newChromosomeWhole.insert(insertionPos + 1, longestPathSubChromosome.first.begin(), longestPathSubChromosome.first.end());
                    inserted = true;
                    break;
                }
            }
        }
    }

    // If no valid previous subchromosome was found or the robotID is zero, insert at the start
    if (!inserted)
    {
        newChromosomeWhole.insert(newChromosomeWhole.begin(), longestPathSubChromosome.first.begin(), longestPathSubChromosome.first.end());
    }

    std::set<int> allCityIndexes;
    for (int i = 0; i < numberOfCities; i++)
    {
        allCityIndexes.insert(i);
    }

    std::set<int> presentCityIndexes(newChromosomeWhole.begin(), newChromosomeWhole.end());

    std::vector<int> missingCityIndexes;
    std::set_difference(allCityIndexes.begin(), allCityIndexes.end(), presentCityIndexes.begin(), presentCityIndexes.end(),
                        std::inserter(missingCityIndexes, missingCityIndexes.end()));

    if (!missingCityIndexes.empty())
    {
        std::unordered_map<int, std::vector<int>> cityNeighbors;
        for (size_t i = 0; i < cityChromosomes.size(); i++)
        {
            int city = cityChromosomes[i];
            if (std::find(missingCityIndexes.begin(), missingCityIndexes.end(), city) != missingCityIndexes.end())
            {
                // Find neighbors
                std::vector<int> neighbors;
                if (i > 0)
                    neighbors.push_back(cityChromosomes[i - 1]);
                if (i < chromosome.size() - 1)
                    neighbors.push_back(cityChromosomes[i + 1]);
                cityNeighbors[city] = neighbors;
            }
        }

        std::vector<int> remainingMissingCities = missingCityIndexes;

        while (!remainingMissingCities.empty())
        {
            std::vector<int> citiesToReprocess;
            for (auto missingCity : remainingMissingCities)
            {
                bool placed = false;
                if (cityNeighbors.find(missingCity) != cityNeighbors.end())
                {
                    std::vector<int> neighbors = cityNeighbors[missingCity];

                    std::shuffle(neighbors.begin(), neighbors.end(), gen_);

                    for (auto neighbor : neighbors)
                    {
                        // Check if neighbor exists in the new chromosome
                        auto it = std::find(newChromosomeWhole.begin(), newChromosomeWhole.end(), neighbor);
                        if (it != newChromosomeWhole.end())
                        {
                            int index = std::distance(newChromosomeWhole.begin(), it);

                            // Insert into the newChromosomeWhole
                            newChromosomeWhole.insert(newChromosomeWhole.begin() + index + 1, missingCity);

                            // Insert into the corresponding subchromosome using index
                            bool inserted = false;
                            for (auto &subChr : newChromosome)
                            {
                                auto subIt = std::find(subChr.begin(), subChr.end(), neighbor);
                                if (subIt != subChr.end())
                                {
                                    int subIndex = std::distance(subChr.begin(), subIt);
                                    subChr.insert(subChr.begin() + subIndex + 1, missingCity);

                                    // Verify insertion using index
                                    if (subChr[subIndex + 1] == missingCity)
                                    {
                                        inserted = true;
                                    }
                                    else
                                    {
                                        throw std::runtime_error("Verification failed: missing city was not inserted correctly into the subchromosome.");
                                    }

                                    break;
                                }
                            }

                            // If insertion into newChromosome fails, throw an error
                            if (!inserted)
                            {
                                throw std::runtime_error("Failed to insert the missing city into the subchromosome.");
                            }

                            placed = true;
                            break;
                        }
                    }
                }

                if (!placed)
                {
                    citiesToReprocess.push_back(missingCity);
                }
            }

            // Explicitly check for missing cities at the end of every iteration
            std::set<int> stillMissing(allCityIndexes.begin(), allCityIndexes.end());
            for (const auto &city : newChromosomeWhole)
            {
                stillMissing.erase(city);
            }

            remainingMissingCities = citiesToReprocess;
        }
    }

    for (size_t i = 0; i < newChromosome.size(); ++i)
    {
        if (newChromosome[i].empty())
        {
            bool elementMoved = false;

            for (size_t j = 0; j < newChromosome.size(); ++j)
            {
                if (newChromosome[j].size() > 1)
                {
                    int donorElement = newChromosome[j].front();

                    // Remove the element from newChromosomeWhole
                    auto it = std::find(newChromosomeWhole.begin(), newChromosomeWhole.end(), donorElement);
                    if (it != newChromosomeWhole.end())
                    {
                        newChromosomeWhole.erase(it);
                    }
                    else
                    {
                        throw std::runtime_error("Element not found in newChromosomeWhole.");
                    }

                    // Donate the element to the empty sub-chromosome
                    newChromosome[i].push_back(donorElement);

                    // Remove the element from the donor sub-chromosome
                    newChromosome[j].erase(newChromosome[j].begin());

                    if (i == 0)
                    {
                        // If i == 0, append donorElement to the start of newChromosomeWhole
                        newChromosomeWhole.insert(newChromosomeWhole.begin(), donorElement);
                    }
                    else
                    {
                        // Find the iterator to the position where you want to insert donorElement
                        auto pos = std::find(newChromosomeWhole.begin(), newChromosomeWhole.end(), newChromosome[i - 1].back());

                        if (pos != newChromosomeWhole.end())
                        {
                            // Insert donorElement after the found position
                            newChromosomeWhole.insert(pos + 1, donorElement);
                        }
                        else
                        {
                            throw std::runtime_error("Element not found in newChromosomeWhole.");
                        }
                    }

                    elementMoved = true;
                    break;
                }
            }

            if (!elementMoved)
            {
                throw std::runtime_error("No elements found to move to the empty sub-chromosome.");
            }
        }
    }

    // Ensure the size of newChromosome and newChromosomeWhole match
    if (newChromosomeWhole.size() != std::accumulate(newChromosome.begin(), newChromosome.end(), 0,
                                                     [](int sum, const std::vector<int> &chr)
                                                     { return sum + chr.size(); }))
    {
        throw std::runtime_error("Size discrepancy between newChromosome and newChromosomeWhole detected.");
    }

    // Ensure the sum of cities in newChromosome equals numberOfCities
    int totalCities = std::accumulate(newChromosome.begin(), newChromosome.end(), 0,
                                      [](int sum, const std::vector<int> &chr)
                                      { return sum + chr.size(); });
    if (totalCities != numberOfCities)
    {
        throw std::runtime_error("The sum of cities in newChromosome does not equal the number of cities.");
    }

    // Check for duplicates in newChromosomeWhole
    std::set<int> uniqueCities(newChromosomeWhole.begin(), newChromosomeWhole.end());
    if (uniqueCities.size() != newChromosomeWhole.size())
    {
        throw std::runtime_error("Duplicate cities found in newChromosomeWhole.");
    }

    // Check that no path lengths are 0 after all operations
    for (const auto &path : newChromosome)
    {
        if (path.size() == 0)
        {
            throw std::runtime_error("A sub-chromosome has a path length of 0 after operations.");
        }
    }

    int size = chromosome.size();
    if (size > initialSize)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
    }

    for (size_t i = 0; i < newChromosome.size(); i++)
    {
        newChromosomeWhole.emplace_back(newChromosome[i].size());
    }

    return newChromosomeWhole;
}

std::vector<double> NSGAIIReproductionMechanismShuaiHorizontal::getSubChromosomeLibraryFitnesses(std::vector<std::pair<std::vector<int>, double>> &library, std::vector<Position> &cities)
{
    std::vector<double> fitnessValues(library.size());
    for (size_t i = 0; i < library.size(); ++i)
    {
        fitnessValues[i] = library[i].second;
    }
    return fitnessValues;
}

std::pair<std::vector<int>, int> NSGAIIReproductionMechanismShuaiHorizontal::getLongestSubChromosomePath(std::vector<std::vector<int>> &subChromosomes, std::vector<Position> &cities)
{
    int longestPathIndex;
    double highestValue = 0;
    int i = 0;
    while (i < teamSize_)
    {
        double fitnessValue = fitnessCalculator_->calculateSubvectorFitness(subChromosomes[i], i, cities);
        if (fitnessValue > highestValue)
        {
            highestValue = fitnessValue;
            longestPathIndex = i;
        }
        i++;
    }
    return std::pair<std::vector<int>, int>{subChromosomes[longestPathIndex], longestPathIndex};
}

std::vector<std::vector<int>> NSGAIIReproductionMechanismShuaiHorizontal::extractSubChromosomes(std::vector<int> &chromosome, int numberOfCities)
{
    std::vector<std::vector<int>> subChromosomes;
    subChromosomes.reserve(teamSize_);
    int positionIndex = 0;
    for (int i = 0; i < teamSize_; i++)
    {
        subChromosomes.emplace_back(std::vector<int>());
        for (int j = 0; j < chromosome[numberOfCities + i]; j++)
        {
            subChromosomes[i].emplace_back(chromosome[positionIndex + j]);
        }
        positionIndex = positionIndex + chromosome[numberOfCities + i];
    }
    return subChromosomes;
}

std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> NSGAIIReproductionMechanismShuaiHorizontal::getHighValueSubChromosomeLibrary(std::vector<ReproductionChromosome> const &population, std::vector<Position> &cities, int teamsize)
{
    auto chromosomes = population;
    std::vector<double> chromosomeFitnesses;
    std::vector<std::pair<Chromosome, std::map<Fitness, double>>> chromosomeFitnessPairs;
    chromosomeFitnessPairs.reserve(chromosomes.size());
    for (auto &chromosome : chromosomes)
    {
        if (isParentMalformed(chromosome, teamsize))
        {
            std::map<Fitness, double> fitness = {
                {Fitness::MAXPATHLENGTH, std::numeric_limits<double>::max()},
                {Fitness::TOTALPATHDISTANCE, std::numeric_limits<double>::max()}};
            chromosomeFitnessPairs.emplace_back(chromosome.chromosome_, fitness);
        }
        else
        {
            std::map<Fitness, double> fitness = fitnessCalculator_->calculateFitness(chromosome.chromosome_, cities);
            chromosomeFitnessPairs.emplace_back(chromosome.chromosome_, fitness);
        }
    }

    std::vector<ReproductionChromosome> selected;
    int halfPopulationSize = population.size() * 0.05;

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

    std::vector<std::vector<int>> subChromosomes;
    subChromosomes.reserve(population.size() * 0.2 * teamSize_);
    std::vector<double> weights;
    weights.reserve(population.size() * 0.2 * teamSize_);

    double expectedSubChromosomeSize = static_cast<double>(cities.size()) / static_cast<double>(teamsize);

    for (auto chromosome : selected)
    {
        int progressIndex = 0;
        for (int i = 0; i < teamSize_; i++)
        {
            auto subChromosome = chromosome.chromosome_.getGenesBetweenIndices(progressIndex, progressIndex + chromosome.chromosome_.getGenesAtIndex(chromosome.chromosome_.getNumberOfCities() + i));
            subChromosomes.emplace_back(std::vector<int>(subChromosome));

            // Calculate weight based on the deviation from the expected subchromosome size
            double subChromosomeSize = static_cast<double>(subChromosome.size());
            double deviation = std::abs(subChromosomeSize - expectedSubChromosomeSize);
            weights.emplace_back(1.0 / (deviation + 1.0)); // Inverse proportional weight

            progressIndex = progressIndex + chromosome.chromosome_.getGenesAtIndex(chromosome.chromosome_.getNumberOfCities() + i);
        }
    }

    std::discrete_distribution<> dist(weights.begin(), weights.end());

    return std::pair(subChromosomes, dist);
}

bool NSGAIIReproductionMechanismShuaiHorizontal::hasNoCommonElements(const std::vector<int> &vec1, const std::vector<int> &vec2)
{
    return std::none_of(vec1.begin(), vec1.end(), [&](int elem1)
                        { return std::any_of(vec2.begin(), vec2.end(), [&](int elem2)
                                             { return elem1 == elem2; }); });
}

NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::ReproductionChromosome(Chromosome chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities)
{
    chromosome_ = chromosome;
    if (isParentMalformed(chromosome, initialAgentPoses.size()))
    {
        fitnessValues_ = {
            {Fitness::MAXPATHLENGTH, std::numeric_limits<double>::max()},
            {Fitness::TOTALPATHDISTANCE, std::numeric_limits<double>::max()}};
    }
    else
    {

        fitnessValues_ = fitnessCalculator->calculateFitness(chromosome, cities);
    }
}

bool NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::dominates(const ReproductionChromosome &other) const
{
    return (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) < other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) <= other.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) < other.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}

double NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::getFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

double NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::getObjectiveFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

Chromosome NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}

bool NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::operator<(const ReproductionChromosome &other) const
{
    auto primary_fitness = getFitness(Fitness::MAXPATHLENGTH); // Replace with actual fitness enum or identifier
    auto other_primary_fitness = other.getFitness(Fitness::TOTALPATHDISTANCE);
    if (primary_fitness != other_primary_fitness)
    {
        return primary_fitness < other_primary_fitness;
    }
    return getFitness(Fitness::TOTALPATHDISTANCE) < other.getFitness(Fitness::TOTALPATHDISTANCE); // Replace with actual fitness enum or identifier
}

bool NSGAIIReproductionMechanismShuaiHorizontal::ReproductionChromosome::isParentMalformed(Chromosome chromosome, int teamSize)
{
    const std::vector<int> &genes = chromosome.getGenes();
    int numberOfCities = chromosome.getNumberOfCities();

    // Check if the chromosome size is not equal to numberOfCities + teamSize
    if (genes.size() != numberOfCities + teamSize)
    {
        return true;
    }

    // Check if the first numberOfCities genes are not within the range of [0, numberOfCities-1]
    std::unordered_set<int> uniqueGenes;
    for (int i = 0; i < numberOfCities; ++i)
    {
        if (genes[i] < 0 || genes[i] >= numberOfCities)
        {
            return true;
        }
        uniqueGenes.insert(genes[i]);
    }

    // Check for duplicates in the first numberOfCities genes
    if (uniqueGenes.size() != numberOfCities)
    {
        return true;
    }

    // Check if there are any duplicates across the entire chromosome except the last teamSize elements
    uniqueGenes.clear();
    for (int i = 0; i < genes.size() - teamSize; ++i)
    {
        if (uniqueGenes.count(genes[i]))
        {
            return true;
        }
        uniqueGenes.insert(genes[i]);
    }

    return false;
}
