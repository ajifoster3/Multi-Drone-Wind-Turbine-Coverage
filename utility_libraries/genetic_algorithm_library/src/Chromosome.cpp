#include "Chromosome.h"

Chromosome::Chromosome(std::vector<int> &genes, int numberOfCites)
{
    genes_ = genes;
    numberOfCites_ = numberOfCites;
}

int Chromosome::getGenesAtIndex(int i)
{
    return genes_[i];
}

std::vector<int> Chromosome::getGenesBetweenIndices(int beginning, int end)
{
    std::vector<int>::const_iterator first = genes_.begin() + beginning;
    std::vector<int>::const_iterator last = genes_.begin() + end;
    std::vector<int> subVector(first, last);
    return subVector;
}

std::vector<int> Chromosome::getGenes() const
{
    return genes_;
}

int Chromosome::getNumberOfCities()
{
    return numberOfCites_;
}

int Chromosome::getNumberOfAgents()
{
    return genes_.size() - numberOfCites_;
}

bool Chromosome::operator==(const Chromosome& other) const
{
    return (genes_ == other.genes_) && (numberOfCites_ == other.numberOfCites_);
}
