#ifndef CHROMOSOME_H
#define CHROMOSOME_H

#include <vector>

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
    
    bool operator==(const Chromosome& other) const;

private:
    std::vector<int> genes_;
    int numberOfCites_;
};

#endif
