#ifndef AGENT_H
#define AGENT_H

#include <fstream>

class Agent {
    public:
        int start_i;
        int start_j;
        int fin_i;
        int fin_j;
        int agentId;
        int speed;

        Agent(int agentId);
        void getAgent(std::ifstream& agentFile);
};

#endif