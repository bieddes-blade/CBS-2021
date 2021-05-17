#include "agent.h"

#include <iostream>
#include <fstream>
#include <random>

Agent::Agent(int agentId) {
    start_i = -1;
    start_j = -1;
    agentId = agentId;
    speed = 1;
}

void Agent::getAgent(std::ifstream& agentFile, bool online) {
    std::string word;
    agentFile >> word >> word >> word >> word;

    int first_fin_j, first_fin_i;
    agentFile >> start_j >> start_i >> first_fin_j >> first_fin_i >> word;
    // i and j are swapped because of the test format
    if (!online) {
        fin_i.push_back(first_fin_i);
        fin_j.push_back(first_fin_j);
    }

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(1, 10);

    speed = 1;
    //speed = dist6(rng); uncomment to use random speeds
}