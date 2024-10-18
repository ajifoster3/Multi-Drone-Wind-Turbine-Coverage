#include "NSGAIIReproductionMechanismShuaiHorizontalFocused.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <set>

NSGAIIReproductionMechanismShuaiHorizontalFocused::NSGAIIReproductionMechanismShuaiHorizontalFocused(
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

Population NSGAIIReproductionMechanismShuaiHorizontalFocused::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities,
    int iterationNumber)
{

    teamSize_ = initialAgentPoses.size();
    auto libary = getHighValueSubChromosomeLibrary(oldPopulation, cities, initialAgentPoses.size());

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

        if (distribution(gen_) < 0.95)
        {
            // PMX crossover
            ProximityBasedCrossover(parent1, parent2, offspring1, initialAgentPoses, cities, true);
            ProximityBasedCrossover(parent1, parent2, offspring2, initialAgentPoses, cities, false);

            // Mutation
            Mutate(offspring1, cities, libary);
            Mutate(offspring2, cities, libary);
        }
        else
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
            if (distribution(gen_) < 0.5)
            {
                std::vector<int> offspringGenes1;
                offspringGenes1 = parent1.getChromosome().getGenes();
                std::vector<int> offspringGenes2;
                offspringGenes2 = parent2.getChromosome().getGenes();
                offspring1.chromosome_.setGenes(horizontalGeneTransfer(offspringGenes1, libary, parent1.getChromosome().getNumberOfCities(), cities));
                offspring2.chromosome_.setGenes(horizontalGeneTransfer(offspringGenes1, libary, parent2.getChromosome().getNumberOfCities(), cities));
            }
            else
            {
                // PMX crossover
                ProximityBasedCrossoverFixed(parent1, parent2, offspring1, initialAgentPoses, cities, true);
                ProximityBasedCrossoverFixed(parent1, parent2, offspring2, initialAgentPoses, cities, false);
            }
        }

        newGeneration.push_back(offspring1);

        newGeneration.push_back(offspring2);
    }

    std::vector<Chromosome> finalGeneration;
    for (auto &chromosome : newGeneration)
    {
        finalGeneration.push_back(chromosome.getChromosome());
    }

    return Population(finalGeneration);
}

bool NSGAIIReproductionMechanismShuaiHorizontalFocused::isParentMalformed(ReproductionChromosome chromosome, int teamSize)
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

bool NSGAIIReproductionMechanismShuaiHorizontalFocused::isParentMalformed(Chromosome chromosome, int teamSize)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::AssignCrowdingDistance(std::vector<ReproductionChromosome> &front)
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

std::vector<NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome> NSGAIIReproductionMechanismShuaiHorizontalFocused::ElitismSelection(const std::vector<ReproductionChromosome> &population)
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

NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome NSGAIIReproductionMechanismShuaiHorizontalFocused::TournamentSelection(const std::vector<ReproductionChromosome> &population)
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

NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome NSGAIIReproductionMechanismShuaiHorizontalFocused::HighMinMaxTournamentSelection(const std::vector<ReproductionChromosome> &population)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::ProximityBasedCrossover(
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
            double distance = HaversineDistance::calculateDroneHaversineDistance(
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
            double distance = HaversineDistance::calculateDroneHaversineDistance(
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::MutationProximityBasedCrossover(
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

    std::uniform_int_distribution<> chanceDist(1, 50);

    while (offspringGenes.size() < numberOfGenes - initialAgentPoses.size())
    {
        // Find the next city based on proximity
        int nextCity = -1;
        double minDistance = std::numeric_limits<double>::max();
        double maxDistance = std::numeric_limits<double>::lowest();

        int candidateCityMin = -1;
        int candidateCityMax = -1;

        // Check the next or previous city in parent1's chromosome
        int nextIndexParent1 = (selectedIndexParent1 + direction + parent1Genes.size()) % parent1Genes.size();
        if (nextIndexParent1 >= 0 && nextIndexParent1 < parent1Genes.size())
        {
            int candidateCity = parent1Genes[nextIndexParent1];
            double distance = HaversineDistance::calculateDroneHaversineDistance(
                cities[currentCity].latitude, cities[currentCity].longitude, cities[currentCity].altitude,
                cities[candidateCity].latitude, cities[candidateCity].longitude, cities[candidateCity].altitude);

            if (distance < minDistance)
            {
                minDistance = distance;
                candidateCityMin = candidateCity;
            }
            if (distance > maxDistance)
            {
                maxDistance = distance;
                candidateCityMax = candidateCity;
            }
        }

        int nextIndexParent2 = (selectedIndexParent2 + direction + parent2Genes.size()) % parent2Genes.size();
        if (nextIndexParent2 >= 0 && nextIndexParent2 < parent2Genes.size())
        {
            int candidateCity = parent2Genes[nextIndexParent2];
            double distance = HaversineDistance::calculateDroneHaversineDistance(
                cities[currentCity].latitude, cities[currentCity].longitude, cities[currentCity].altitude,
                cities[candidateCity].latitude, cities[candidateCity].longitude, cities[candidateCity].altitude);

            if (distance < minDistance)
            {
                minDistance = distance;
                candidateCityMin = candidateCity;
            }
            if (distance > maxDistance)
            {
                maxDistance = distance;
                candidateCityMax = candidateCity;
            }
        }

        if (chanceDist(gen_) == 1)
        {
            // 1/20 chance to choose the less close city
            nextCity = candidateCityMax;
        }
        else
        {
            // Otherwise, choose the closest city
            nextCity = candidateCityMin;
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


void NSGAIIReproductionMechanismShuaiHorizontalFocused::ProximityBasedCrossoverFixed(
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

    // Step 0: Remove fixed elements (initial agent poses) from parent1 and parent2 genes
    parent1Genes.resize(parent1Genes.size() - initialAgentPoses.size());
    parent2Genes.resize(parent2Genes.size() - initialAgentPoses.size());

    // Step 1: Extract sub-chromosomes and find the longest path sub-chromosome
    auto subChromosomes = extractSubChromosomes(parent1Genes, cities.size());
    auto longestPathSubChromosome = getLongestSubChromosomePath(subChromosomes, cities);
    int insertionIndex = std::distance(parent1Genes.begin(),

                                       std::find(parent1Genes.begin(),
                                                 parent1Genes.end(),
                                                 longestPathSubChromosome[0]));

    // Step 2: Remove the longest path sub-chromosome from parent1
    for (auto gene : longestPathSubChromosome)
    {
        parent1Genes.erase(std::remove(parent1Genes.begin(), parent1Genes.end(), gene), parent1Genes.end());
    }

    // Step 3: Remove the same genes from parent2
    for (auto gene : longestPathSubChromosome)
    {
        parent2Genes.erase(std::remove(parent2Genes.begin(), parent2Genes.end(), gene), parent2Genes.end());
    }

    // Step 4: Perform proximity-based crossover on the remaining genes
    std::vector<int> offspringGenes;
    offspringGenes.reserve(numberOfGenes);

    // Select a random city to start with
    std::uniform_int_distribution<> dist(0, parent1Genes.size() - 1);
    int selectedIndexParent1 = dist(gen_);
    int selectedIndexParent2 = std::distance(parent2Genes.begin(), std::find(parent2Genes.begin(), parent2Genes.end(), parent1Genes[selectedIndexParent1]));
    int currentCity = parent1Genes[selectedIndexParent1];
    offspringGenes.push_back(currentCity);

    // Remove the selected city from both parent chromosomes
    parent1Genes.erase(std::remove(parent1Genes.begin(), parent1Genes.end(), currentCity), parent1Genes.end());
    parent2Genes.erase(std::remove(parent2Genes.begin(), parent2Genes.end(), currentCity), parent2Genes.end());

    int direction = forward ? 0 : -1;

    while (offspringGenes.size() < numberOfGenes - initialAgentPoses.size()-longestPathSubChromosome.size())
    {
        int nextCity = -1;
        double minDistance = std::numeric_limits<double>::max();

        // Check the next or previous city in parent1's chromosome
        int nextIndexParent1 = (selectedIndexParent1 + direction + parent1Genes.size()) % parent1Genes.size();
        if (nextIndexParent1 >= 0 && nextIndexParent1 < parent1Genes.size())
        {
            int candidateCity = parent1Genes[nextIndexParent1];
            double distance = HaversineDistance::calculateDroneHaversineDistance(
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
            double distance = HaversineDistance::calculateDroneHaversineDistance(
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

    offspringGenes.insert(offspringGenes.begin() + insertionIndex,
                          longestPathSubChromosome.begin(),
                          longestPathSubChromosome.end());

    // Insert initial agent poses at the end
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::Mutate(ReproductionChromosome &chromosome, std::vector<Position> &cities, std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> library)
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

std::vector<std::vector<NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome>> NSGAIIReproductionMechanismShuaiHorizontalFocused::FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{
    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void NSGAIIReproductionMechanismShuaiHorizontalFocused::flipInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::swapInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::lSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::rSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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

void NSGAIIReproductionMechanismShuaiHorizontalFocused::distributeCities(std::vector<int> &outVec, int numberOfCities, int numberOfAgents)
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

std::vector<int> NSGAIIReproductionMechanismShuaiHorizontalFocused::horizontalGeneTransfer(
    std::vector<int> &chromosome,
    std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> &library,
    int numberOfCities,
    std::vector<Position> &cities)
{
    int initialSize = chromosome.size();

    auto subChromosomes = extractSubChromosomes(chromosome, numberOfCities);

    auto longestPathSubChromosome = getLongestSubChromosomePath(subChromosomes, cities);

    // Create new chromosome
    std::vector<std::vector<int>> newChromosome;
    newChromosome.emplace_back(longestPathSubChromosome);

    std::vector<int> newChromosomeWhole;
    newChromosomeWhole.insert(newChromosomeWhole.end(), longestPathSubChromosome.begin(), longestPathSubChromosome.end());

    // Calculate fitness values for the library subchromosomes

    for (int i = 1; i < teamSize_; i++)
    {
        bool isInitialized = false;
        for (size_t attempts = 0; attempts < library.first.size(); attempts++)
        {
            int index = library.second(gen_);
            auto selectedSubChromosome = library.first[index];
            if (hasNoCommonElements(newChromosomeWhole, selectedSubChromosome))
            {
                newChromosome.emplace_back(selectedSubChromosome);
                newChromosomeWhole.insert(newChromosomeWhole.end(), selectedSubChromosome.begin(), selectedSubChromosome.end());
                isInitialized = true;
                break;
            }
        }
        if (!isInitialized)
        {
            newChromosome.emplace_back(std::vector<int>());
        }
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
        int emptyChromosomeCount = std::count_if(newChromosome.begin(), newChromosome.end(), [](const auto &chr)
                                                 { return chr.empty(); });

        if (emptyChromosomeCount > 0)
        {
            int index = 0;
            for (auto &chr : newChromosome)
            {
                if (chr.empty())
                {
                    chr.push_back(missingCityIndexes[index]);
                    newChromosomeWhole.push_back(missingCityIndexes[index]);
                    index++;
                    if (index >= missingCityIndexes.size())
                        break;
                }
            }

            if (index < missingCityIndexes.size())
            {
                newChromosome.back().insert(newChromosome.back().end(),
                                            missingCityIndexes.begin() + index, missingCityIndexes.end());
                newChromosomeWhole.insert(newChromosomeWhole.end(), missingCityIndexes.begin() + index, missingCityIndexes.end());
            }
        }
        else
        {
            newChromosome.back().insert(newChromosome.back().end(),
                                        missingCityIndexes.begin(), missingCityIndexes.end());
            newChromosomeWhole.insert(newChromosomeWhole.end(), missingCityIndexes.begin(), missingCityIndexes.end());
        }
    }

    for (auto &chr : newChromosome)
    {
        if (chr.empty())
        {
            // Find a non-empty sub-chromosome
            for (auto &donorChromosome : newChromosome)
            {
                if (!donorChromosome.empty())
                {
                    // Move the first element from the donorChromosome to the empty chromosome
                    int donorElement = donorChromosome.front();
                    chr.push_back(donorElement);
                    newChromosomeWhole.push_back(donorElement);

                    // Remove the element from the donor chromosome
                    donorChromosome.erase(donorChromosome.begin());
                    break;
                }
            }
        }
    }

    for (int i = 0; i < teamSize_; i++)
    {
        newChromosomeWhole.emplace_back(newChromosome[i].size());
    }
    int size = chromosome.size();
    if (size > initialSize)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
    }

    return newChromosomeWhole;
}

std::vector<double> NSGAIIReproductionMechanismShuaiHorizontalFocused::getSubChromosomeLibraryFitnesses(std::vector<std::pair<std::vector<int>, double>> &library, std::vector<Position> &cities)
{
    std::vector<double> fitnessValues(library.size());
    for (size_t i = 0; i < library.size(); ++i)
    {
        fitnessValues[i] = library[i].second;
    }
    return fitnessValues;
}

std::vector<int> NSGAIIReproductionMechanismShuaiHorizontalFocused::getLongestSubChromosomePath(std::vector<std::vector<int>> &subChromosomes, std::vector<Position> &cities)
{
    int longestPathIndex;
    double highestValue = 0;
    for (size_t i = 0; i < teamSize_; i++)
    {
        double fitnessValue = fitnessCalculator_->calculateSubvectorFitness(subChromosomes[i], i, cities);
        if (fitnessValue > highestValue)
        {
            highestValue = fitnessValue;
            longestPathIndex = i;
        }
    }
    return subChromosomes[longestPathIndex];
}

std::vector<std::vector<int>> NSGAIIReproductionMechanismShuaiHorizontalFocused::extractSubChromosomes(std::vector<int> &chromosome, int numberOfCities)
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

std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> NSGAIIReproductionMechanismShuaiHorizontalFocused::getHighValueSubChromosomeLibrary(Population &population, std::vector<Position> &cities, int teamsize)
{
    auto chromosomes = population.getPopulationList();
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
            chromosomeFitnessPairs.emplace_back(chromosome, fitness);
        }
        else
        {
            std::map<Fitness, double> fitness = fitnessCalculator_->calculateFitness(chromosome, cities);
            chromosomeFitnessPairs.emplace_back(chromosome, fitness);
        }
    }

    // Sort chromosomes based on their fitness values (descending order)
    std::sort(chromosomeFitnessPairs.begin(), chromosomeFitnessPairs.end(),
              [](const std::pair<Chromosome, std::map<Fitness, double>> &a, const std::pair<Chromosome, std::map<Fitness, double>> &b)
              {
                  return a.second.at(Fitness::TOTALPATHDISTANCE) < b.second.at(Fitness::TOTALPATHDISTANCE); // Compare fitness values
              });

    // Sample the best 20% of chromosomes (in regards to total path distance)
    std::vector<Chromosome> sortedChromosomes;
    sortedChromosomes.reserve(chromosomeFitnessPairs.size());
    for (size_t i = 0; i < population.getPopulationList().size() * 0.05; i++)
    {
        sortedChromosomes.push_back(chromosomeFitnessPairs[i].first);
    }

    std::vector<std::vector<int>> subChromosomes;
    subChromosomes.reserve(population.getPopulationList().size() * 0.2 * teamSize_);
    std::vector<double> weights;
    weights.reserve(population.getPopulationList().size() * 0.2 * teamSize_);

    double expectedSubChromosomeSize = static_cast<double>(cities.size()) / static_cast<double>(teamsize);

    for (auto chromosome : sortedChromosomes)
    {
        int progressIndex = 0;
        for (int i = 0; i < teamSize_; i++)
        {
            auto subChromosome = chromosome.getGenesBetweenIndices(progressIndex, progressIndex + chromosome.getGenesAtIndex(chromosome.getNumberOfCities() + i));
            subChromosomes.emplace_back(std::vector<int>(subChromosome));

            // Calculate weight based on the deviation from the expected subchromosome size
            double subChromosomeSize = static_cast<double>(subChromosome.size());
            double deviation = std::abs(subChromosomeSize - expectedSubChromosomeSize);
            weights.emplace_back(1.0 / (deviation + 1.0)); // Inverse proportional weight

            progressIndex = progressIndex + chromosome.getGenesAtIndex(chromosome.getNumberOfCities() + i);
        }
    }

    std::discrete_distribution<> dist(weights.begin(), weights.end());

    return std::pair(subChromosomes, dist);
}

bool NSGAIIReproductionMechanismShuaiHorizontalFocused::hasNoCommonElements(const std::vector<int> &vec1, const std::vector<int> &vec2)
{
    return std::none_of(vec1.begin(), vec1.end(), [&](int elem1)
                        { return std::any_of(vec2.begin(), vec2.end(), [&](int elem2)
                                             { return elem1 == elem2; }); });
}

NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::ReproductionChromosome(Chromosome chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities)
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

bool NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::dominates(const ReproductionChromosome &other) const
{
    return (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) < other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) <= other.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) < other.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}

double NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::getFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

double NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::getObjectiveFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

Chromosome NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}

bool NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::operator<(const ReproductionChromosome &other) const
{
    auto primary_fitness = getFitness(Fitness::MAXPATHLENGTH); // Replace with actual fitness enum or identifier
    auto other_primary_fitness = other.getFitness(Fitness::TOTALPATHDISTANCE);
    if (primary_fitness != other_primary_fitness)
    {
        return primary_fitness < other_primary_fitness;
    }
    return getFitness(Fitness::TOTALPATHDISTANCE) < other.getFitness(Fitness::TOTALPATHDISTANCE); // Replace with actual fitness enum or identifier
}

bool NSGAIIReproductionMechanismShuaiHorizontalFocused::ReproductionChromosome::isParentMalformed(Chromosome chromosome, int teamSize)
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
