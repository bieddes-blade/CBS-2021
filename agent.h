#ifndef AGENT_H
#define AGENT_H

#include <fstream>
#include <vector>

class Agent {
    public:
        int start_i;
        int start_j;
        std::vector<int> fin_i;
        std::vector<int> fin_j;
        int agentId;
        int speed;

        Agent(int agentId);
        void getAgent(std::ifstream& agentFile, bool online);
};

#endif