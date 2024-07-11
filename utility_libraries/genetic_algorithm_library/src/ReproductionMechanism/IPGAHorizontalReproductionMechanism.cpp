#include "IPGAHorizontalReproductionMechanism.h"
#include <iostream>
#include <set>

IPGAHorizontalReproductionMechanism::IPGAHorizontalReproductionMechanism(std::shared_ptr<FitnessCalculator> fitnessCalculator,
                                                                         double citiesPerSalesmanMutationProbability,
                                                                         double routeMutationProbability,
                                                                         int sampleSize)
    : ReproductionMechanism(std::move(fitnessCalculator)),
      citiesPerSalesmanMutationProbability_(citiesPerSalesmanMutationProbability),
      routeMutationProbability_(routeMutationProbability),
      sampleSize_(sampleSize)
{
    std::random_device rd;
    gen_ = std::mt19937(rd());
};

Population IPGAHorizontalReproductionMechanism::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    teamSize_ = initialAgentPoses.size();
    auto libary = getHighValueSubChromosomeLibrary(oldPopulation, cities);

    std::vector<ReproductionChromosome> reproductionChromosomes{};

    reproductionChromosomes.reserve(oldPopulation.getPopulationList().size());

    for (auto chromosome : oldPopulation.getPopulationList())
    {
        reproductionChromosomes.push_back(ReproductionChromosome{chromosome, fitnessCalculator_, initialAgentPoses, cities});
    }

    shuffleReproductionChromosomeList(reproductionChromosomes);

    std::vector<Chromosome> newGeneration{};
    newGeneration.reserve(reproductionChromosomes.size());

    while (!reproductionChromosomes.empty())
    {
        size_t safeSampleSize = std::min(sampleSize_, (int)reproductionChromosomes.size());
        auto firstSample = std::vector<ReproductionChromosome>(reproductionChromosomes.begin(), reproductionChromosomes.begin() + safeSampleSize);
        auto selectedChromosome = std::min_element(firstSample.begin(), firstSample.end(), [](const auto &a, const auto &b)
                                                   { return a.getFitness()[Fitness::MAXPATHTOTALPATHWEIGHTEDSUM] < b.getFitness()[Fitness::MAXPATHTOTALPATHWEIGHTEDSUM]; });

        ReproductionChromosome selected = *selectedChromosome;

        std::vector<ReproductionChromosome> cloneChromosomes(safeSampleSize, selected);

        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        for (size_t i = 0; i < safeSampleSize; i++)
        {
            auto genes = cloneChromosomes[i].getChromosome().getGenes();

            Chromosome tempChromosome(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
            if (std::find(newGeneration.begin(), newGeneration.end(), tempChromosome) == newGeneration.end())
            {
                newGeneration.emplace_back(tempChromosome);
            }
            else
            {
                if (distribution(gen_) < routeMutationProbability_)
                {
                    auto roulette = distribution(gen_);
                    if (roulette < 0.75)
                    {
                        roulette = distribution(gen_);
                        if (roulette < 0.5)
                        {
                            flipInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                        }
                        roulette = distribution(gen_);
                        if (roulette < 0.5)
                        {
                            swapInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                        }
                        roulette = distribution(gen_);
                        if (roulette < 0.5)
                        {
                            lSlideInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                        }
                        roulette = distribution(gen_);
                        if (roulette < 0.5)
                        {
                            rSlideInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                        }
                    }
                    else
                    {
                        horizontalGeneTransfer(genes, libary, cloneChromosomes[i].getChromosome().getNumberOfCities(), cities);
                    }
                }
                if (distribution(gen_) < citiesPerSalesmanMutationProbability_)
                {
                    distributeCities(genes, cloneChromosomes[i].getChromosome().getNumberOfCities(), cloneChromosomes[i].getChromosome().getNumberOfAgents());
                }
                newGeneration.emplace_back(Chromosome(genes, cloneChromosomes[i].getChromosome().getNumberOfCities()));
            }
        }
        reproductionChromosomes.erase(reproductionChromosomes.begin(), reproductionChromosomes.begin() + safeSampleSize);
    }
    return Population(newGeneration);
}

void IPGAHorizontalReproductionMechanism::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{

    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void IPGAHorizontalReproductionMechanism::flipInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return; // Not enough elements to perform operations
    }
    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    int index1{dis(gen_)};
    int index2{};
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

void IPGAHorizontalReproductionMechanism::swapInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return; // Not enough elements to perform operations
    }

    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    // Generate two random indices and ensure they are distinct
    int index1 = dis(gen_);
    int index2 = dis(gen_);
    while (index1 == index2)
    {
        index2 = dis(gen_);
    }

    // Swap the values at the two indices
    std::swap(vec[index1], vec[index2]);

    // Ensure index1 < index2 for block manipulation
    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void IPGAHorizontalReproductionMechanism::lSlideInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return; // Not enough elements to perform operations
    }

    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    // Generate two random indices and ensure they are distinct
    int index1 = dis(gen_);
    int index2 = dis(gen_);
    while (index1 == index2)
    {
        index2 = dis(gen_);
    }

    // Ensure index1 < index2 for block manipulation
    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    // Left cyclic shift within the selected range
    std::rotate(vec.begin() + index1, vec.begin() + index1 + 1, vec.begin() + index2 + 1);

    // Call your mechanism for inserting the subvector at a new random position
    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void IPGAHorizontalReproductionMechanism::rSlideInsert(std::vector<int> &vec, int numberOfCities)
{
    if (vec.size() < 2)
    {
        return;
    }

    std::uniform_int_distribution<> dis(0, numberOfCities - 1);

    // Generate two random indices and ensure they are distinct
    int index1 = dis(gen_);
    int index2 = dis(gen_);
    while (index1 == index2)
    {
        index2 = dis(gen_);
    }

    // Ensure index1 < index2 for block manipulation
    if (index1 > index2)
    {
        std::swap(index1, index2);
    }

    // Right cyclic shift within the selected range
    std::rotate(vec.begin() + index1, vec.begin() + index2, vec.begin() + index2 + 1);

    // Call your mechanism for inserting the subvector at a new random position
    randomlyInsertSubvector(vec, index1, index2, numberOfCities);
}

void IPGAHorizontalReproductionMechanism::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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

void IPGAHorizontalReproductionMechanism::horizontalGeneTransfer(
    std::vector<int> &chromosome,
    std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> &library,
    int numberOfCities,
    std::vector<Position> &cities)
{
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

    chromosome = newChromosomeWhole;
}

std::vector<double> IPGAHorizontalReproductionMechanism::getSubChromosomeLibraryFitnesses(std::vector<std::pair<std::vector<int>, double>> &library, std::vector<Position> &cities)
{
    std::vector<double> fitnessValues(library.size());
    for (size_t i = 0; i < library.size(); ++i)
    {
        fitnessValues[i] = library[i].second;
    }
    return fitnessValues;
}

std::vector<int> IPGAHorizontalReproductionMechanism::getLongestSubChromosomePath(std::vector<std::vector<int>> &subChromosomes, std::vector<Position> &cities)
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

std::vector<std::vector<int>> IPGAHorizontalReproductionMechanism::extractSubChromosomes(std::vector<int> &chromosome, int numberOfCities)
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

std::pair<std::vector<std::vector<int>>, std::discrete_distribution<>> IPGAHorizontalReproductionMechanism::getHighValueSubChromosomeLibrary(Population &population, std::vector<Position> &cities)
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

    // Sample the best 20% of chromosomes (in regards to total path distance)
    std::vector<Chromosome> sortedChromosomes;
    sortedChromosomes.reserve(chromosomeFitnessPairs.size());
    for (size_t i = 0; i < population.getPopulationList().size() * 0.2; i++)
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

bool IPGAHorizontalReproductionMechanism::hasNoCommonElements(const std::vector<int> &vec1, const std::vector<int> &vec2)
{
    return std::none_of(vec1.begin(), vec1.end(), [&](int elem1)
                        { return std::any_of(vec2.begin(), vec2.end(), [&](int elem2)
                                             { return elem1 == elem2; }); });
}

void IPGAHorizontalReproductionMechanism::distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents)
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

IPGAHorizontalReproductionMechanism::ReproductionPopulation::ReproductionPopulation(Population population)
{
    population_;
}

IPGAHorizontalReproductionMechanism::ReproductionChromosome::ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> initialAgentPoses, std::vector<Position> cities)
{
    chromosome_ = chromosome;
    fitness_ = fitnessCalculator->calculateFitness(chromosome, cities);
}

std::map<Fitness, double> IPGAHorizontalReproductionMechanism::ReproductionChromosome::getFitness() const { return fitness_; };

Chromosome &IPGAHorizontalReproductionMechanism::ReproductionChromosome::getChromosome()
{
    return chromosome_;
}
