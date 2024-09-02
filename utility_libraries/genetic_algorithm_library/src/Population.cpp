#include "Population.h"
#include "Fitnesses.h"

Population::Population(std::vector<Chromosome> &populationList)
{
    populationList_ = populationList;
}

std::vector<Chromosome> Population::getPopulationList()
{
    return populationList_;
}

double Population::calculateHypervolume(const std::vector<std::pair<double, double>> &paretoFront, std::pair<double, double> nadir)
{
    double hypervolume = 0.0;

    // Sort the Pareto front by the first objective (ascending)
    std::vector<std::pair<double, double>> sortedParetoFront = paretoFront;
    std::sort(sortedParetoFront.rbegin(), sortedParetoFront.rend());

    // Calculate the hypervolume
    double previousX = nadir.first;
    for (const auto &point : sortedParetoFront)
    {
        double width = previousX - point.first;
        double height = nadir.second - point.second;
        hypervolume += width * height;
        previousX = point.first;
    }

    return hypervolume;
}

std::map<Fitness, double> Population::getPopulationFitness(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities, Fitness fitnessChoice, std::chrono::_V2::steady_clock::time_point startTime, std::pair<double, double> nadir)
{
    std::vector<Fitness> fitnessChoices = {Fitness::MAXPATHLENGTH, Fitness::TOTALPATHDISTANCE};
    std::vector<Chromosome> paretoFront = getParetoFront(fitnessCalculator, agentStartPositions, cities);

    std::vector<std::pair<double, double>> paretoPoints;
    for (auto &chromosome : paretoFront)
    {
        auto fitness = fitnessCalculator.calculateFitness(chromosome, cities);
        paretoPoints.push_back({fitness[Fitness::MAXPATHLENGTH], fitness[Fitness::TOTALPATHDISTANCE]});
    }

    // Calculate the hypervolume
    double hypervolume = calculateHypervolume(paretoPoints, nadir);

    // Record the end time and calculate the elapsed time
    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsedSeconds = endTime - startTime;
    double calculationTime = elapsedSeconds.count();

    // We will return the hypervolume and calculation time in the map with dummy keys since the original function signature expects a map
    std::map<Fitness, double> fitnessMap;
    fitnessMap[Fitness::HYPERVOLUME] = hypervolume;
    fitnessMap[Fitness::TIMEELAPSED] = calculationTime;
    return fitnessMap;
}

std::vector<int> Population::getFittestChromosomeGenes(
    FitnessCalculator &fitnessCalculator,
    std::vector<Position> &agentStartPositions,
    std::vector<Position> &cities,
    Fitness fitnessChoice)
{
    std::vector<int> fittestChromosomeGenes;
    double maxFitness = 0;
    for (auto chromosome : this->getPopulationList())
    {
        auto fitness = fitnessCalculator.calculateFitness(chromosome, cities);
        if (maxFitness < fitness[fitnessChoice])
        {
            fittestChromosomeGenes = chromosome.getGenes();
            maxFitness = fitness[fitnessChoice];
        }
    }
    return fittestChromosomeGenes;
}

bool Population::dominates(const std::map<Fitness, double> &fitness1, const std::map<Fitness, double> &fitness2, const std::vector<Fitness> &fitnessChoices)
{
    bool betterInAtLeastOne = false;
    for (const auto &choice : fitnessChoices)
    {
        if (fitness1.at(choice) > fitness2.at(choice))
            return false;
        if (fitness1.at(choice) < fitness2.at(choice))
            betterInAtLeastOne = true;
    }
    return betterInAtLeastOne;
}

std::vector<Chromosome> Population::getParetoFront(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
{
    return FastNonDominatedSort(agentStartPositions, cities, fitnessCalculator)[0];
}

std::vector<std::vector<Chromosome>> Population::FastNonDominatedSort(std::vector<Position> &agentStartPositions, std::vector<Position> &cities, FitnessCalculator &fitnessCalculator)
{
    std::vector<Chromosome> chromosomes;
    for (auto &chromosome : getPopulationList())
    {
        auto genes = chromosome.getGenes();
        chromosomes.emplace_back(genes, cities.size());
    }

    chromosomes.erase(
        std::remove_if(chromosomes.begin(), chromosomes.end(),
                       [&](const Chromosome &chromosome)
                       {
                           return isParentMalformed(chromosome, agentStartPositions.size());
                       }),
        chromosomes.end());

    for (auto &chromosome : chromosomes)
    {
        chromosome.fitnessValues_ = fitnessCalculator.calculateFitness(chromosome, cities);
    }

    std::vector<std::vector<Chromosome>> fronts(1);
    std::vector<int> dominationCount(chromosomes.size(), 0);
    std::vector<std::vector<int>> dominatedChromosomes(chromosomes.size());

    for (size_t p = 0; p < chromosomes.size(); ++p)
    {
        for (size_t q = 0; q < chromosomes.size(); ++q)
        {
            if (p == q)
                continue;
            if (dominates(chromosomes[p], chromosomes[q]))
            {
                dominatedChromosomes[p].push_back(q);
            }
            else if (dominates(chromosomes[q], chromosomes[p]))
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
        std::vector<Chromosome> nextFront;
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

bool Population::isParentMalformed(Chromosome chromosome, int teamSize)
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

bool Population::dominates(const Chromosome &a, const Chromosome &b)
{
    return (a.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) + 1 < b.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && a.fitnessValues_.at(Fitness::MAXPATHLENGTH) <= b.fitnessValues_.at(Fitness::MAXPATHLENGTH)) ||
           (a.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) <= b.fitnessValues_.at(Fitness::TOTALPATHDISTANCE) && a.fitnessValues_.at(Fitness::MAXPATHLENGTH) + 1 < b.fitnessValues_.at(Fitness::MAXPATHLENGTH));
}
