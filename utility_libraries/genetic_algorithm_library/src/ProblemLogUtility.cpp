#include "ProblemLogUtility.h"

void ProblemLogUtility::logData(const std::string &fileName, const std::vector<Position> &cities, int agents, const std::vector<Position> &agentStartPositions)
{
    std::ofstream outFile(fileName);
    if (!outFile)
    {
        std::cerr << "Failed to open file for logging." << std::endl;
        return;
    }

    // Log cities
    outFile << "Cities:\n";
    for (const auto &city : cities)
    {
        outFile << city.latitude << " " << city.longitude << " " << city.altitude << "\n";
    }

    // Log agents
    outFile << "Agents:\n";
    outFile << agents << "\n";

    // Log agent start positions
    outFile << "Agent Start Positions:\n";
    for (const auto &position : agentStartPositions)
    {
        outFile << position.latitude << " " << position.longitude << " " << position.altitude << "\n";
    }

    outFile.close();
    if (!outFile.good())
    {
        std::cerr << "Error occurred at writing time!" << std::endl;
    }
}

bool ProblemLogUtility::readData(const std::string &fileName, std::vector<Position> &cities, int &agents, std::vector<Position> &agentStartPositions)
{
    std::ifstream inFile(fileName);
    if (!inFile)
    {
        std::cerr << "Failed to open file for reading: " << fileName << std::endl;
        return false;
    }

    std::string line;
    // Read cities
    cities.clear();
    agentStartPositions.clear();
    if (std::getline(inFile, line))
    {
        std::cout << "Reading line: " << line << std::endl; // Debug print
        if (line == "Cities:")
        {
            while (std::getline(inFile, line) && !line.empty() && line != "Agents:")
            {
                std::cout << "Reading city line: " << line << std::endl; // Debug print
                std::istringstream ss(line);
                Position city;
                ss >> city.latitude >> city.longitude >> city.altitude;
                cities.push_back(city);
            }
        }
        else
        {
            std::cerr << "Failed to read 'Cities:' header" << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << "Failed to read the first line from the file" << std::endl;
        return false;
    }

    // Read agents
    if (line == "Agents:")
    {
        if (std::getline(inFile, line))
        {
            std::cout << "Reading agents line: " << line << std::endl; // Debug print
            std::istringstream ss(line);
            ss >> agents;
        }
        else
        {
            std::cerr << "Failed to read agents value" << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << "Failed to read 'Agents:' header" << std::endl;
        return false;
    }

    // Read agent start positions
    if (std::getline(inFile, line) && line == "Agent Start Positions:")
    {
        while (std::getline(inFile, line) && !line.empty())
        {
            std::cout << "Reading agent start position line: " << line << std::endl; // Debug print
            std::istringstream ss(line);
            Position position;
            ss >> position.latitude >> position.longitude >> position.altitude;
            agentStartPositions.push_back(position);
        }
    }
    else
    {
        std::cerr << "Failed to read 'Agent Start Positions:' header" << std::endl;
        return false;
    }

    inFile.close();
    if (!inFile.good() && !inFile.eof())
    {
        std::cerr << "Error occurred at reading time!" << std::endl;
        return false;
    }
    return true;
}
