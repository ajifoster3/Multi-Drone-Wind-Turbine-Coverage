#ifndef CHROMOSOME_H
#define CHROMOSOME_H

#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <map>
#include "Fitnesses.h"

class Chromosome
{
public:
    Chromosome() {};
    Chromosome(std::vector<int> &genes, int numberOfCites);
    int getGenesAtIndex(int);
    std::vector<int> getGenesBetweenIndices(int, int);
    std::vector<int> getGenes() const;
    int getNumberOfCities();
    int getNumberOfAgents();
    
    void setGenes(const std::vector<int> &genes);
    bool operator==(const Chromosome& other) const;

    bool hasDuplicates(const std::vector<int> &genes, int count);
    std::map<Fitness, double> fitnessValues_;
    int rank = 0;

private:
    std::vector<int> genes_;
    int numberOfCites_;
};

#endif
