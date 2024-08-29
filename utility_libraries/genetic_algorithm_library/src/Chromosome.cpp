#include "Chromosome.h"
#include <FitnessCalculator.h>

Chromosome::Chromosome(std::vector<int> &genes, int numberOfCites)
{
    genes_ = genes;
    numberOfCites_ = numberOfCites;
    for (int i = numberOfCites_; i < genes_.size() - numberOfCites_; ++i)
    {
        if (genes_[i] == 0)
        {
            throw std::runtime_error("The final numberOfCites elements cannot be 0.");
        }
    }
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

void Chromosome::setGenes(const std::vector<int> &genes)
{
    genes_ = genes;
}

bool Chromosome::operator==(const Chromosome &other) const
{
    return (genes_ == other.genes_) && (numberOfCites_ == other.numberOfCites_);
}
