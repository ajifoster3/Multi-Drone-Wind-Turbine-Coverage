#ifndef SEQUENCEENCODINGMECHANISM_H
#define SEQUENCEENCODINGMECHANISM_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include "EncodingMechanism.h"
#include "Chromosome.h"

class SequenceEncodingMechanism : public EncodingMechanism
{

public:
    Chromosome buildChromosome(int numberOfCites, int agents);

private:
    std::vector<int> createRandomVectorWithSum(size_t n, int targetSum);
};

#endif
