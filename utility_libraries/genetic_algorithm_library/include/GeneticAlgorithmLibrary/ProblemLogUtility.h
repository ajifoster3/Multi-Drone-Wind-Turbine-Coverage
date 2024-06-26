#ifndef PROBLEMLOGUTILITY_H
#define PROBLEMLOGUTILITY_H

#include <string>
#include <vector>
#include <Position.h>
#include <iostream>
#include <fstream>
#include <regex>

class ProblemLogUtility
{
public:
    ProblemLogUtility() = delete;
    static void logData(const std::string &fileName, const std::vector<Position> &cities, int agents, const std::vector<Position> &agentStartPositions);
    static bool readData(const std::string &fileName, std::vector<Position> &cities, int &agents, std::vector<Position> &agentStartPositions);
};

#endif
