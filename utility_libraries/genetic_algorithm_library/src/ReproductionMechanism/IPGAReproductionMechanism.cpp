#include "IPGAReproductionMechanism.h"

IPGAReproductionMechanism::IPGAReproductionMechanism(std::shared_ptr<FitnessCalculator> fitnessCalculator)
    : ReproductionMechanism(std::move(fitnessCalculator))
{
    std::random_device rd;
    gen_ = std::mt19937(rd());
};

Population IPGAReproductionMechanism::Reproduce(
    Population &oldPopulation,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    std::vector<ReproductionChromosome> reproductionChromosomes{};
    reproductionChromosomes.reserve(oldPopulation.getPopulationList().size());
    for (auto chromosome : oldPopulation.getPopulationList())
    {
        reproductionChromosomes.emplace_back(ReproductionChromosome{chromosome, fitnessCalculator_, initialAgentPoses, cities});
    }

    shuffleReproductionChromosomeList(reproductionChromosomes);

    std::vector<Chromosome> newGeneration{};
    newGeneration.reserve(oldPopulation.getPopulationList().size());

    while (!reproductionChromosomes.empty())
    {
        size_t safeSampleSize = std::min(sampleSize_, (int)reproductionChromosomes.size());
        auto firstSample = std::vector<ReproductionChromosome>(reproductionChromosomes.begin(), reproductionChromosomes.begin() + safeSampleSize);

        auto selectedChromosome = std::max_element(firstSample.begin(), firstSample.end(), [](const auto &a, const auto &b)
                                                   { return a.getFitness() < b.getFitness(); });

        ReproductionChromosome selected = *selectedChromosome;

        std::vector<ReproductionChromosome> cloneChromosomes(safeSampleSize, selected);

        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        for (size_t i = 0; i < safeSampleSize; i++)
        {
            auto genes = cloneChromosomes[i].getChromosome().getGenes();

            if (distribution(gen_) < routeMutationProbability_)
            {
                auto roulette = distribution(gen_);
                if (roulette < 0.25)
                {
                    flipInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                }
                else if (roulette < 0.5)
                {
                    swapInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                }
                else if (roulette < 0.75)
                {
                    lSlideInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                }
                else
                {
                    rSlideInsert(genes, cloneChromosomes[i].getChromosome().getNumberOfCities());
                }
            }
            if (distribution(gen_) < citiesPerSalesmanMutationProbability_)
            {
                distributeCities(genes, cloneChromosomes[i].getChromosome().getNumberOfCities(), cloneChromosomes[i].getChromosome().getNumberOfAgents());
            }
            newGeneration.emplace_back(Chromosome(genes, cloneChromosomes[i].getChromosome().getNumberOfCities()));
        }

        reproductionChromosomes.erase(reproductionChromosomes.begin(), reproductionChromosomes.begin() + safeSampleSize);
    }

    return Population(newGeneration);
}

void IPGAReproductionMechanism::shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness)
{

    std::shuffle(chromosomeFitness.begin(), chromosomeFitness.end(), gen_);
}

void IPGAReproductionMechanism::flipInsert(std::vector<int> &vec, int numberOfCities)
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

void IPGAReproductionMechanism::swapInsert(std::vector<int> &vec, int numberOfCities)
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

void IPGAReproductionMechanism::lSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void IPGAReproductionMechanism::rSlideInsert(std::vector<int> &vec, int numberOfCities)
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

void IPGAReproductionMechanism::randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities)
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

void IPGAReproductionMechanism::distributeCities(std::vector<int> &vec, int numberOfCities, int numberOfAgents)
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

ReproductionPopulation::ReproductionPopulation(Population population)
{
    population_;
}

ReproductionChromosome::ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> initialAgentPoses, std::vector<Position> cities)
{
    chromosome_ = chromosome;
    fitness_ = fitnessCalculator->calculateFitness(chromosome, cities);
}

double ReproductionChromosome::getFitness() const { return fitness_; };

Chromosome &ReproductionChromosome::getChromosome()
{
    return chromosome_;
}
