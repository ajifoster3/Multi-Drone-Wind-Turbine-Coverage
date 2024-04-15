#ifndef ENCODINGMECHANISM_H
#define ENCODINGMECHANISM_H

#include "Chromosome.h"

enum EncodingMechanisms
{
    SEQUENCE_ENCODING_MECHANISM
};

class EncodingMechanism
{

public:
    virtual ~EncodingMechanism() = default;
    virtual Chromosome buildChromosome(int numberOfCites, int agents) = 0;
};

#endif
