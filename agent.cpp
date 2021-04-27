#include "agent.h"

#include <fstream>
#include <random>

Agent::Agent(int agentId) {
    start_i = -1;
    start_j = -1;
    fin_i = -1;
    fin_j = -1;
    agentId = agentId;
    speed = 1;
}

void Agent::getAgent(std::ifstream& agentFile) {
    std::string word;
    agentFile >> word >> word >> word >> word;
    agentFile >> start_j >> start_i >> fin_j >> fin_i >> word;

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(1, 10);

    speed = 1;//dist6(rng);
}