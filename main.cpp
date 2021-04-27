#include "map.h"
#include "agent.h"
#include "ctSolution.h"

#include <vector>
#include <fstream>

std::vector<Agent> readAgents(std::ifstream& agentFile, int size) {
    // read garbage
    std::string word;
    agentFile >> word >> word;

    // fill in a vector of agents
    std::vector<Agent> agents;
    for (int id = 0; id < size; ++id) {
        Agent agent(id);
        agent.getAgent(agentFile);
        agents.push_back(agent);
    }
    return agents;
}


int main(int argc,  char **argv) {
    // read the map from mapFile
    // argv[1] is the location of the map
    std::ifstream mapFile;
    mapFile.open(argv[1]);
    // get its options and grid
    Map map;
    map.getMapOptions(mapFile);
    map.getMapGrid(mapFile);
    mapFile.close();

    // read info on the agents (start, finish, speed) from agentFile
    // argv[2] is the location of the file
    std::ifstream agentFile;
    agentFile.open(argv[2]);
    // argv[3] is the number of agents
    std::vector<Agent> agents;
    agents = readAgents(agentFile, std::stoi(argv[3]));
    agentFile.close();

    std::vector<std::string> argList(argv + 1, argv + argc);

    // create a solution object
    CTSolution solution(map, agents, argList[3] == "true", argList[4] == "true", argList[5], argList[6] == "true", argList[7] == "true", argList[8] == "true", std::stof(argList[9]), argList[10] == "true");

    // map - argList[0]
    // scen - argList[1]
    // vector of agents
    // true -> use dijkstra precalc - argList[3]
    // true -> use CAT - argList[4]
    // heuristic (normal, normal_length, number_of_conflicts, number_of_conflicting_agents, number_of_pairs, vertex_cover) - argList[5]
    // true -> prioritize conflicts - argList[6]
    // true -> use bypass - argList[7]
    // true -> use ecbs - argList[8]
    // omega - argList[9]
    // print paths - argList[10]

    std::vector<Path> bestPaths = solution.highLevelSearch();

    return 0;
}