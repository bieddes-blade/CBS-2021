#include "map.h"
#include "agent.h"
#include "ctSolution.h"

#include <vector>
#include <fstream>

std::vector<Agent> readAgents(std::ifstream& agentFile, int size, bool online) {
    // read garbage
    std::string word;
    agentFile >> word >> word;

    // fill in a vector of agents
    std::vector<Agent> agents;
    for (int id = 0; id < size; ++id) {
        Agent agent(id);
        agent.getAgent(agentFile, online);
        agents.push_back(agent);
    }
    return agents;
}
 
std::vector<std::vector<std::pair<int, int>>> readGoals(std::ifstream& goalFile, int size) {
    int a, b;
    std::vector<std::vector<std::pair<int, int>>> goals(size, std::vector<std::pair<int, int>>());

    for (int agent = 0; agent < size; ++agent) {
        goalFile >> a;
        while (a != -1) {
            goalFile >> b;
            goals[agent].push_back({b, a});
            goalFile >> a;
        }
    }
    return goals;
}

int main(int argc,  char **argv) {
    std::vector<std::string> argList(argv + 1, argv + argc);

    // read the map from mapFile
    // argv[1] is the location of the map
    std::ifstream mapFile;
    mapFile.open(argv[1]);
    // get its options and grid
    Map map;
    map.getMapOptions(mapFile);
    map.getMapGrid(mapFile);
    mapFile.close();

    // check if this is an online or offline setting
    bool online = bool(argList[11] == "true");

    // read info on the agents (start, finish, speed) from agentFile
    // argv[2] is the location of the file
    std::ifstream agentFile;
    agentFile.open(argv[2]);
    // argv[3] is the number of agents
    std::vector<Agent> agents;
    agents = readAgents(agentFile, std::stoi(argv[3]), online);
    agentFile.close();

    // read info on the agents' goals
    std::vector<std::vector<std::pair<int, int>>> goalLocs;
    if (online) {
        std::ifstream goalFile;
        // argList[12] is the location of the file
        goalFile.open(argList[12]);
        goalLocs = readGoals(goalFile, std::stoi(argv[3]));
        goalFile.close();
    }

    // create a solution object
    CTSolution solution(map, agents, argList[3] == "true", argList[4] == "true", argList[5], argList[6] == "true", argList[7] == "true", argList[8] == "true", std::stof(argList[9]), argList[10] == "true", argList[11] == "true", goalLocs, std::stoi(argList[13]), std::stoi(argList[14]), argList[15] == "true");

    // map - argList[0]
    // scen - argList[1]
    // vector of agents
    // set true to use dijkstra precalc - argList[3]
    // set true to use CAT - argList[4]
    // choose heuristic (normal, normal_length, number_of_conflicts, number_of_conflicting_agents, number_of_pairs, vertex_cover) - argList[5]
    // set true to prioritize conflicts - argList[6]
    // set true to use bypass - argList[7]
    // set true to use ecbs - argList[8]
    // choose omega - argList[9]
    // set true to use symmetry breaking - argList[10]
    // set true to use online setting - argList[11]
    // goals for online setting - argList[12]
    // vector of goals
    // time horizon - argList[13]
    // replanning frequency - argList[14]
    // set true to print paths - argList[15]

    solution.solve();

    return 0;
}