#ifndef CHROMOSOMEBUILDER_H
#define CHROMOSOMEBUILDER_H

#include <memory>
#include <vector>
#include "Chromosome.h"
#include "EncodingMechanism.h"

class ChromosomeBuilder
{

public:
    ChromosomeBuilder(){};
    ChromosomeBuilder(std::shared_ptr<EncodingMechanism> encodingMechanism)
        : encodingMechanism_(std::move(encodingMechanism)){};

    Chromosome buildChromosome(int, int);

private:
    std::shared_ptr<EncodingMechanism> encodingMechanism_;
};

#endif
