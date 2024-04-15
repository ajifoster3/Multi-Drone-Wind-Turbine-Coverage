#include "SequenceEncodingMechanism.h"

Chromosome SequenceEncodingMechanism::buildChromosome(int numberOfCites, int agents)
{
    std::vector<int> cities(numberOfCites);
    std::iota(std::begin(cities), std::end(cities), 0);
    auto rd = std::random_device{};
    auto rng = std::default_random_engine{rd()};
    std::shuffle(std::begin(cities), std::end(cities), rng);

    std::vector<int> numberOfCitesPerAgent = createRandomVectorWithSum(agents, numberOfCites);

    cities.insert(cities.end(), numberOfCitesPerAgent.begin(), numberOfCitesPerAgent.end());

    return Chromosome(cities, numberOfCites);
}

std::vector<int> SequenceEncodingMechanism::createRandomVectorWithSum(size_t n, int targetSum)
{
    std::vector<int> result;
    result.reserve(n);
    std::default_random_engine generator(std::random_device{}());
    std::uniform_int_distribution<int> distribution(1, targetSum / n);

    int currentSum = 0;
    for (size_t i = 0; i < n - 1; ++i)
    {
        int randomValue = distribution(generator);
        result.emplace_back(randomValue);
        currentSum += randomValue;
    }

    // Add the final element to make the sum equal to targetSum
    result.emplace_back(targetSum - currentSum);

    // Optional: Shuffle the vector to make the distribution of numbers random
    std::shuffle(result.begin(), result.end(), generator);

    return result;
}
