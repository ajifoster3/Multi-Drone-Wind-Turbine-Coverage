#include "NSGAIIReproductionMechanismHorizontal.h"


NSGAIIReproductionMechanismHorizontal::NSGAIIReproductionMechanismHorizontal(
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

Population NSGAIIReproductionMechanismHorizontal::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities,
    int iterationNumber)
{

    teamSize_ = initialAgentPoses.size();
    auto libary = getHighValueSubChromosomeLibrary(oldPopulation, cities);


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
    
    // Insert elites into new generation
    newGeneration.insert(newGeneration.end(), parents.begin(), parents.end());

    // Reproduction to create the new population
    while (newGeneration.size() < oldPopulation.getPopulationList().size())
    {
        ReproductionChromosome offspring = TournamentSelection(parents);
        Mutate(offspring, libary ,cities);

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

void NSGAIIReproductionMechanismHorizontal::AssignCrowdingDistance(std::vector<ReproductionChromosome> &front)
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

NSGAIIReproductionMechanismHorizontal::ReproductionChromosome NSGAIIReproductionMechanismHorizontal::TournamentSelection(const std::vector<ReproductionChromosome> &population)
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

void NSGAIIReproductionMechanismHorizontal::Mutate(ReproductionChromosome &chromosome, std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> library,  std::vector<Position> &cities)
{
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    auto genes = chromosome.getChromosome().getGenes();

    if (distribution(gen_) < routeMutationProbability_)
    {
        auto roulette = distribution(gen_);
        if (roulette < 0.5)
        {
            roulette = distribution(gen_);
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
        else
        {
            horizontalGeneTransfer(genes, library, chromosome.getChromosome().getNumberOfCities(), cities);
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

std::vector<std::vector<NSGAIIReproductionMechanismHorizontal::ReproductionChromosome>> NSGAIIReproductionMechanismHorizontal::FastNonDominatedSort(Population &population, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
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

void NSGAIIReproductionMechanismHorizontal::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{
    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void NSGAIIReproductionMechanismHorizontal::flipInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismHorizontal::swapInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismHorizontal::lSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismHorizontal::rSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void NSGAIIReproductionMechanismHorizontal::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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


void NSGAIIReproductionMechanismHorizontal::horizontalGeneTransfer(
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

    for (int i = 0; i < teamSize_; i++)
    {
        newChromosomeWhole.emplace_back(newChromosome[i].size());
    }

    if (chromosome.size() > initialSize)
    {
        throw std::runtime_error("Population is empty, cannot perform tournament selection.");
    }
    chromosome = newChromosomeWhole;
}

std::vector<double> NSGAIIReproductionMechanismHorizontal::getSubChromosomeLibraryFitnesses(std::vector<std::pair<std::vector<int>, double>> &library, std::vector<Position> &cities)
{
    std::vector<double> fitnessValues(library.size());
    for (size_t i = 0; i < library.size(); ++i)
    {
        fitnessValues[i] = library[i].second;
    }
    return fitnessValues;
}

std::vector<int> NSGAIIReproductionMechanismHorizontal::getLongestSubChromosomePath(std::vector<std::vector<int>> &subChromosomes, std::vector<Position> &cities)
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
    ;
}

std::vector<std::vector<int>> NSGAIIReproductionMechanismHorizontal::extractSubChromosomes(std::vector<int> &chromosome, int numberOfCities)
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

std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> NSGAIIReproductionMechanismHorizontal::getHighValueSubChromosomeLibrary(Population &population, std::vector<Position> &cities)
{
    auto chromosomes = population.getPopulationList();
    std::vector<double> chromosomeFitnesses;
    std::vector<std::pair<Chromosome, std::map<Fitness, double>>> chromosomeFitnessPairs;
    chromosomeFitnessPairs.reserve(chromosomes.size());
    for (auto &chromosome : chromosomes)
    {
        std::map<Fitness, double> fitness = fitnessCalculator_->calculateFitness(chromosome, cities);
        chromosomeFitnessPairs.emplace_back(chromosome, fitness);
    }

    // Sort chromosomes based on their fitness values (descending order)
    std::sort(chromosomeFitnessPairs.begin(), chromosomeFitnessPairs.end(),
              [](const std::pair<Chromosome, std::map<Fitness, double>> &a, const std::pair<Chromosome, std::map<Fitness, double>> &b)
              {
                  return a.second.at(Fitness::TOTALPATHDISTANCE) < b.second.at(Fitness::TOTALPATHDISTANCE); // Compare fitness values
              });

    // Sample the best 5% of chromosomes (in regards to total path distance)
    std::vector<Chromosome> sortedChromosomes;
    sortedChromosomes.reserve(chromosomeFitnessPairs.size());
    for (size_t i = 0; i < population.getPopulationList().size() * 0.02; i++)
    {
        sortedChromosomes.push_back(chromosomeFitnessPairs[i].first);
    }

    std::vector<std::vector<int>> subChromosomes;
    subChromosomes.reserve(population.getPopulationList().size() * 0.2 * teamSize_);
    std::vector<double> fitnessValues;
    fitnessValues.reserve(population.getPopulationList().size() * 0.2 * teamSize_);
    for (auto chromosome : sortedChromosomes)
    {
        int progressIndex = 0;
        for (int i = 0; i < teamSize_; i++)
        {
            auto subChromosome = chromosome.getGenesBetweenIndices(progressIndex, progressIndex + chromosome.getGenesAtIndex(chromosome.getNumberOfCities() + i));
            subChromosomes.emplace_back(std::vector<int>(subChromosome));
            fitnessValues.emplace_back(fitnessCalculator_->calculateSubvectorFitness(subChromosome, i, cities));
            progressIndex = progressIndex + chromosome.getGenesAtIndex(chromosome.getNumberOfCities() + i);
        }
    }

    // Calculate the mean fitness
    double totalFitness = std::accumulate(fitnessValues.begin(), fitnessValues.end(), 0.0);
    double meanFitness = totalFitness / fitnessValues.size();

    // Assign weights based on deviation from the mean
    std::vector<double> weights(subChromosomes.size());
    for (size_t i = 0; i < subChromosomes.size(); ++i)
    {
        weights[i] = 1.0 / (std::abs(fitnessValues[i] - meanFitness) + 1.0); // Inverse proportional weight
    }

    std::discrete_distribution<> dist(weights.begin(), weights.end());

    return std::pair(subChromosomes, dist);
}

bool NSGAIIReproductionMechanismHorizontal::hasNoCommonElements(const std::vector<int> &vec1, const std::vector<int> &vec2)
{
    return std::none_of(vec1.begin(), vec1.end(), [&](int elem1)
                        { return std::any_of(vec2.begin(), vec2.end(), [&](int elem2)
                                             { return elem1 == elem2; }); });
}

void NSGAIIReproductionMechanismHorizontal::distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents)
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

std::vector<NSGAIIReproductionMechanismHorizontal::ReproductionChromosome> NSGAIIReproductionMechanismHorizontal::ElitismSelection(const std::vector<ReproductionChromosome>& population) {
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

NSGAIIReproductionMechanismHorizontal::ReproductionChromosome::ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> &initialAgentPoses, std::vector<Position> &cities)
{
    chromosome_ = chromosome;
    fitnessValues_ = fitnessCalculator->calculateFitness(chromosome, cities);
}

bool NSGAIIReproductionMechanismHorizontal::ReproductionChromosome::dominates(const ReproductionChromosome &other) const
{
    return (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) < other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) <= other.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= other.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && fitnessValues_.at(Fitness::MAXPATHLENGTH) < other.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}

double NSGAIIReproductionMechanismHorizontal::ReproductionChromosome::getFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

double NSGAIIReproductionMechanismHorizontal::ReproductionChromosome::getObjectiveFitness(Fitness index) const
{
    return fitnessValues_.at(index);
}

Chromosome &NSGAIIReproductionMechanismHorizontal::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}

bool NSGAIIReproductionMechanismHorizontal::ReproductionChromosome::operator<(const ReproductionChromosome &other) const
{
    auto primary_fitness = getFitness(Fitness::MAXPATHLENGTH); // Replace with actual fitness enum or identifier
    auto other_primary_fitness = other.getFitness(Fitness::TOTALPATHDISTANCE);
    if (primary_fitness != other_primary_fitness)
    {
        return primary_fitness < other_primary_fitness;
    }
    return getFitness(Fitness::TOTALPATHDISTANCE) < other.getFitness(Fitness::TOTALPATHDISTANCE); // Replace with actual fitness enum or identifier
}

