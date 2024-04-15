#include "ChromosomeBuilder.h"

Chromosome ChromosomeBuilder::buildChromosome(int numberOfCites, int agents)
{
    return(encodingMechanism_->buildChromosome(numberOfCites, agents));
}

