#ifndef POPULATIONINITIALISER_H
#define POPULATIONINITIALISER_H

#include "ChromosomeBuilder.h"
#include "Population.h"
#include "Position.h"

class PopulationInitialiser
{
public:
    PopulationInitialiser(ChromosomeBuilder&, int populationSize);
    Population InitialisePopulation(int, int);

private:
    ChromosomeBuilder chromosomeBuilder_;
    int populationSize_;
};

#endif
