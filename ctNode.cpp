#include "ctNode.h"

#include <cmath>
#include <queue>
#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <climits>

CTNode::CTNode() {
    paths = std::vector<std::vector<std::pair<int, int>>>();
    cost = 0;
    maxTime = 10000;
    vertexConstr = VertexConstrStruct(maxTime);
    edgeConstr = EdgeConstrStruct(maxTime);
    conflictAvoidanceTable = ConfMap();
    stateAgentMap = StateMap();
}

bool CTNode::isCover(int V, int k, int E) { 
    // for vertex cover
    int set = (1 << k) - 1; 
    int limit = (1 << V); 
    bool vis[200][200];
    while (set < limit) {
        memset(vis, 0, sizeof vis); 
        int cnt = 0;
        for (int j = 1, v = 1; j < limit; j = j << 1, v++) { 
            if (set & j) {
                for (int k = 1; k <= V; k++) { 
                    if (graph[v][k] && !vis[v][k]) { 
                        vis[v][k] = 1; 
                        vis[k][v] = 1; 
                        cnt++; 
                    } 
                } 
            } 
        }
        if (cnt == E) {
            return true; 
        }
        int c = set & -set; 
        int r = set + c; 
        set = (((r^set) >> 2) / c) | r; 
    } 
    return false; 
} 
      
int CTNode::findMinCover(int n, int m) {
    // for vertex cover
    int left = 1, right = n; 
    while (right > left) { 
        int mid = (left + right) >> 1; 
        if (isCover(n, mid, m) == false) {
            left = mid + 1; 
        } else {
            right = mid; 
        }
    }
    return left; 
} 

void CTNode::insertEdge(int u, int v) { 
    // for vertex cover
    graph[u][v] = 1; 
    graph[v][u] = 1;
}

void CTNode::countCost(std::string heuristic) {
    // there are a few heuristics we can use
    int ln = paths.size();
    int numOfConflicts = 0;
    int agentInConflict = 0;
    int pairsInConflict = 0;
    int minVertexCover = 0;
    int sumLenghts = 0;

    // sum of path sizes
    if (heuristic == "normal") {
        for (int i = 0; i < ln; ++i) {
            sumLenghts += paths[i].size() - 1;
        }
        cost = sumLenghts;
        return;
    }

    // vertex cover
    if (heuristic == "vertex_cover") {
        graph = new bool *[200];
        for (int i = 0; i < 200; ++i) {
            graph[i] = new bool[200];
        } 
    }

    // number of conflicts
    // number of conflicting agents
    // number of conflicting pairs of agents
    for (int i = 0; i < ln; ++i) {
        int wasConflict_i = 0;
        for (int j = i + 1; j < ln; ++j) {
            int wasConflict_j = 0;
            int steps = std::min(paths[i].size(), paths[j].size());
            for (int step = 0; step < steps; ++step) {
                if (paths[i][step] == paths[j][step]) {
                    ++numOfConflicts;
                    wasConflict_i = 1;
                    wasConflict_j = 1;
                    if (heuristic == "vertex_cover") {
                        insertEdge(i + 1, j + 1);
                    }
                }
            }
            pairsInConflict += wasConflict_j;
        }
        agentInConflict += wasConflict_i;
    }

    if (heuristic == "number_of_conflicts") {
        cost = numOfConflicts;
    } else if (heuristic == "number_of_conflicting_agents") {
        cost = agentInConflict;
    } else if (heuristic == "number_of_pairs") {
        cost = pairsInConflict;
    } else if (heuristic == "vertex_cover") {
        cost = findMinCover(agentInConflict, pairsInConflict);
    }
}

int CTNode::countPathCost(int i, std::string heuristic) {
    double sumLenghts = 0;

    // sum of path sizes
    if (heuristic == "normal") {
        return paths[i].size();
    }

    // sum of path lengths
    if (heuristic == "normal_diagonal") {
        for (int j = 1; j < paths[i].size(); ++j) {
            double x1 = paths[i][j].first;
            double x2 = paths[i][j - 1].first;
            double y1 = paths[i][j].second;
            double y2 = paths[i][j - 1].second;
            sumLenghts += std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        }
        return sumLenghts;
    }

    return sumLenghts;
}

void CTNode::countCAT() {
    // fill in the conflict avoidance table
    int ln = paths.size();
    // go through the paths
    for (int i = 0; i < ln; ++i) {
        for (int j = i + 1; j < ln; ++j) {
            int steps = std::min(paths[i].size(), paths[j].size());
            for (int step = 0; step < steps; ++step) {
                // change the number of agents that were in (i, j) at time t
                KeyThree state = std::make_tuple(paths[i][step].first, paths[i][step].second, step);
                // if such a state exists in the CAT, we increment its variable
                if (conflictAvoidanceTable.find(state) != conflictAvoidanceTable.end()) {
                    ++conflictAvoidanceTable[state];
                } else {
                    conflictAvoidanceTable[state] = 1;
                }
                // we also collect data on which agents have been in which state (i, j, t)
                stateAgentMap[state].insert(i);
            }
        }
    }
}

std::string CTNode::findConflictType(Map& map, bool dijkstra, std::map<pairVert, int, pvCompare>& distMap, std::vector<Agent>& agents, Conflict& conflict, std::string heuristic) {
    // we calculate the costs of the agents' individual paths
    int oldCost0 = countPathCost((conflict.agents).first, heuristic);
    int oldCost1 = countPathCost((conflict.agents).second, heuristic);

    // we calculate the costs of the agents' paths with added constraints
    int newCost0 = 0;
    int newCost1 = 0;

    for (int agent_number = 0; agent_number < 2; ++agent_number) {
        int agent; // the agent
        int confTime; // when did the conflict happen

        if (agent_number == 0) {
            agent = (conflict.agents).first;
            confTime = conflict.time1;
        } else {
            agent = (conflict.agents).second;
            confTime = conflict.time2;
        }

        CTNode node;
        node.vertexConstr = vertexConstr;
        node.edgeConstr = edgeConstr;

        if (conflict.type == "vertex") {
            node.vertexConstr[confTime].insert(conflict.v1);
        } else if (conflict.type == "edge") {
            std::pair<std::pair<int, int>, std::pair<int, int>> confTuple;
            if (agent_number == 0) {
                confTuple = {{conflict.v1.first, conflict.v1.second}, {conflict.v2.first, conflict.v2.second}};
                node.edgeConstr[conflict.time1].insert(confTuple);
            } else {
                confTuple = {{conflict.v2.first, conflict.v2.second}, {conflict.v1.first, conflict.v1.second}};
                node.edgeConstr[conflict.time2].insert(confTuple);
            }
        }

        node.paths = paths;
        node.countCAT();

        // planning a new path
        Search search(map);
        search.startSearch(map, distMap, node.vertexConstr, node.edgeConstr, node.conflictAvoidanceTable, agents[agent], node.stateAgentMap, dijkstra, false, 1.0);
        node.paths[agent] = search.fullPath;

        if (agent_number == 0) {
            newCost0 = countPathCost(agent, heuristic);
        } else {
            newCost1 = countPathCost(agent, heuristic);
        }
    }

    // if both new paths increase the cost of the solution, this is a cardinal conflict
    if (newCost0 > oldCost0 && newCost1 > oldCost1) {
        return "cardinal";
    // if one of the new paths increases the cost of the solution and the other one doesn't change it, this is a semi-cardinal conflict
    } else if (newCost0 == oldCost0 && newCost1 > oldCost1) {
        return "semi-cardinal";
    // if one of the new paths increases the cost of the solution and the other one doesn't change it, this is a semi-cardinal conflict
    } else if (newCost0 > oldCost0 && newCost1 == oldCost1) {
        return "semi-cardinal";
    // if both new paths don't change the cost of the solution, this is a non-cardinal conflict
    } else {
        return "non-cardinal";
    }
}

Conflict CTNode::findBestConflict(Map& map, bool dijkstra, std::map<pairVert, int, pvCompare>& distMap, std::vector<Agent>& agents, bool prioritizeConflicts, bool useSymmetry, std::string heuristic, int horizon) {
    int ln = paths.size();
    Conflict bestConflict("none");
    Conflict semiCardConflict("none");

    // VERTEX CONFLICTS

    std::vector<EventVertex> collectEvents;
    for (int agent = 0; agent < ln; ++agent) {
        // create event 'agent entered first vertex'

        if (paths[agent].size()) {
            collectEvents.push_back(EventVertex(agents[agent].speed * 0, agent, agents[agent].speed, 0, 0));
        }

        // conflicts that happen after the horizon don't matter
        for (int step = 1; step < std::min(horizon, int(paths[agent].size())); ++step) {
            if (paths[agent][step] != paths[agent][step - 1]) {
                // create event 'agent entered a new vertex'
                collectEvents.push_back(EventVertex(agents[agent].speed * step, agent, agents[agent].speed, step, 0));
                // create event 'agent exited the last vertex'
                collectEvents.push_back(EventVertex(agents[agent].speed * (step - 1), agent, agents[agent].speed, (step - 1), 1));
            }
        }

        if (paths[agent].size()) {
            int lastStep = paths[agent].size() - 1;
            collectEvents.push_back(EventVertex(agents[agent].speed * lastStep, agent, agents[agent].speed, lastStep, 1));
        }
    }

    // sort all events by two keys: time and type
    std::sort(collectEvents.begin(), collectEvents.end());
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> scanLine;

    for (auto curEvent : collectEvents) {
        std::pair<int, int> curVert = paths[curEvent.agent][curEvent.step];
        // if current event is of type 'in'
        if (curEvent.type == 0) {
            if (scanLine.find(curVert) != scanLine.end()) {
                // if someone else is in this vertex
                if (scanLine[curVert].first != -1) {
                    int a1 = curEvent.agent;
                    int a2 = scanLine[curVert].first;
                    int step1 = curEvent.step;
                    int step2 = scanLine[curVert].second;
                    Conflict curConflict = Conflict("vertex", {a1, a2}, step1, step2, paths[a1][step1], std::pair<int, int>());
                    bestConflict = curConflict;

                    if (useSymmetry) {
                        // lets check if this is a cardinal rectangular conflict
                        int start_ix = agents[a1].start_i;
                        int start_iy = agents[a1].start_j;
                        int goal_ix = agents[a1].fin_i[0];
                        int goal_iy = agents[a1].fin_j[0];

                        int start_jx = agents[a2].start_i;
                        int start_jy = agents[a2].start_j;
                        int goal_jx = agents[a2].fin_i[0];
                        int goal_jy = agents[a2].fin_j[0];

                        int start_it = 0;
                        int goal_it = paths[a1].size() - 1;
                        int start_jt = 0;
                        int goal_jt = paths[a2].size() - 1;


                        if (std::abs(start_ix - goal_ix) + std::abs(start_iy - goal_iy) == goal_it - start_it && goal_it - start_it > 0) {
                            if (std::abs(start_jx - goal_jx) + std::abs(start_jy - goal_jy) == goal_jt - start_jt && goal_jt - start_jt > 0) {
                                if ((start_ix - goal_ix) * (start_jx - goal_jx) >= 0) {
                                    if ((start_iy - goal_iy) * (start_jy - goal_jy) >= 0) {
                                        if ((start_ix - start_jx) * (goal_ix - goal_jx) <= 0) {
                                            if ((start_iy - start_jy) * (goal_iy - goal_jy) <= 0) {
                                                // agents a1 and a2 are involved in a cardinal rectangle conflict
                                                curConflict.type = "rectangular";
                                                return curConflict;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // if PC is active, we check the type of the conflict
                    if (prioritizeConflicts == true) {
                        std::string type = findConflictType(map, dijkstra, distMap, agents, curConflict, heuristic);
                        if (type == "cardinal") {
                            return curConflict;
                        } else if (type == "semi-cardinal") {
                            semiCardConflict = curConflict;
                        }
                    // otherwise we return the first conflict we encountered
                    } else {
                        return curConflict;
                    }
                }
            } else {
                scanLine[curVert] = {curEvent.agent, curEvent.step};
            }
        // if current event is of type 'out'
        } else {
            scanLine[curVert] = {-1, -1};
        }
    }

    if (bestConflict.type != "none") {
        return bestConflict;
    }
    




    // EDGE CONFLICT (SWAP)

    std::vector<EventEdge> collectEventsSwap;
    for (int agent = 0; agent < ln; ++agent) {
        for (int step = 1; step < std::min(horizon, int(paths[agent].size())); ++step) {
            if (paths[agent][step] != paths[agent][step - 1]) {
                int in_time = (step - 1) * agents[agent].speed;
                int out_time = step * agents[agent].speed;
                std::pair<int, int> start = paths[agent][step - 1];
                std::pair<int, int> end = paths[agent][step];

                collectEventsSwap.push_back(EventEdge(in_time, agent, start, end, (step - 1), 0));
                collectEventsSwap.push_back(EventEdge(out_time, agent, start, end, step, 1));
            }
        }
    }


    std::sort(collectEventsSwap.begin(), collectEventsSwap.end());
    std::unordered_map<KeyFour, std::set<std::pair<int, int>>, KeyFourHash, KeyFourEqual> scanLineSwap;

    for (auto curEvent : collectEventsSwap) {
        std::pair<int, int> start = curEvent.start;
        std::pair<int, int> end = curEvent.end;

        KeyFour edgeNormal = std::make_tuple(start.first, start.second, end.first, end.second);
        KeyFour edgeInvert = std::make_tuple(end.first, end.second, start.first, start.second);

        int a1 = curEvent.agent;
        int step1 = curEvent.step;

        // if current event is of type 'in'
        if (curEvent.type == 0) {
            // add pair agent-step to the set
            if (scanLineSwap.find(edgeNormal) == scanLineSwap.end()) {
                scanLineSwap[edgeNormal] = std::set<std::pair<int, int>>();
            }
            scanLineSwap[edgeNormal].insert({a1, step1});

            // check if someone was moving towards this agent
            if (scanLineSwap.find(edgeInvert) != scanLineSwap.end()) {
                if (scanLineSwap[edgeInvert].size() > 0) {
                    auto iter = scanLineSwap[edgeInvert].begin();
                    std::pair<int, int> other = *iter;
                    int a2 = other.first;
                    int step2 = other.second;

                    Conflict curConflict = Conflict("edge", {a1, a2}, step1, step2, start, end);
                    bestConflict = curConflict;

                    // if PC is active, we check the type of the conflict
                    if (prioritizeConflicts == true) {
                        std::string type = findConflictType(map, dijkstra, distMap, agents, curConflict, heuristic);
                        if (type == "cardinal") {
                            return curConflict;
                        } else if (type == "semi-cardinal") {
                            semiCardConflict = curConflict;
                        }
                    // otherwise we return the first conflict we encountered
                    } else {
                        return curConflict;
                    }
                }
            }
                
        // if current event is of type 'out'
        } else {
            auto iter = scanLineSwap[edgeNormal].find({a1, step1 - 1});
            scanLineSwap[edgeNormal].erase(iter);
        }
    }





    // EDGE CONFLICT (OVER)

    std::vector<EventEdge> collectEventsOver;
    for (int agent = 0; agent < ln; ++agent) {
        for (int step = 1; step < std::min(horizon, int(paths[agent].size())); ++step) {
            if (paths[agent][step] != paths[agent][step - 1]) {
                int in_time = (step - 1) * agents[agent].speed;
                int out_time = step * agents[agent].speed;
                std::pair<int, int> start = paths[agent][step - 1];
                std::pair<int, int> end = paths[agent][step];

                collectEventsOver.push_back(EventEdge(in_time, agent, start, end, (step - 1), 0));
                collectEventsOver.push_back(EventEdge(out_time, agent, start, end, step, 1));
            }
        }
    }


    std::sort(collectEventsOver.begin(), collectEventsOver.end());
    std::unordered_map<KeyFour, std::queue<std::pair<int, int>>, KeyFourHash, KeyFourEqual> scanLineOver;

    for (auto curEvent : collectEventsOver) {
        std::pair<int, int> start = curEvent.start;
        std::pair<int, int> end = curEvent.end;

        KeyFour edgeNormal = std::make_tuple(start.first, start.second, end.first, end.second);
        KeyFour edgeInvert = std::make_tuple(end.first, end.second, start.first, start.second);

        int a1 = curEvent.agent;
        int step1 = curEvent.step;

        // if current event is of type 'in'
        if (curEvent.type == 0) {
            // add pair agent-step to the set
            if (scanLineOver.find(edgeNormal) == scanLineOver.end()) {
                scanLineOver[edgeNormal] = std::queue<std::pair<int, int>>();
            }
            scanLineOver[edgeNormal].push({a1, step1});

        // if current event is of type 'out'
        } else {
            // if first element == our agent
            if ((scanLineOver[edgeNormal].front()).first == a1) {
                scanLineOver[edgeNormal].pop();
            } else {
                std::pair<int, int> other = scanLineOver[edgeNormal].front();
                int a2 = other.first;
                int step2 = other.second;

                Conflict curConflict = Conflict("edge", {a1, a2}, step1, step2, start, end);
                bestConflict = curConflict;

                // if PC is active, we check the type of the conflict
                if (prioritizeConflicts == true) {
                    std::string type = findConflictType(map, dijkstra, distMap, agents, curConflict, heuristic);
                    if (type == "cardinal") {
                        return curConflict;
                    } else if (type == "semi-cardinal") {
                        semiCardConflict = curConflict;
                    }
                // otherwise we return the first conflict we encountered
                } else {
                    return curConflict;
                }
            }
        }
    }

    if (prioritizeConflicts && semiCardConflict.type != "none") {
        return semiCardConflict;
    }

    return bestConflict;
}

int CTNode::findNumOfConflicts(Map& map, std::vector<Agent>& agents, int agentCheck, int horizon) {
    int ln = paths.size();
    std::set<std::tuple<int, int, int, int>> allConf;

    // VERTEX CONFLICTS

    std::vector<EventVertex> collectEvents;
    for (int agent = 0; agent < ln; ++agent) {
        if (paths[agent].size()) {
            collectEvents.push_back(EventVertex(agents[agent].speed * 0, agent, agents[agent].speed, 0, 0));
        }

        for (int step = 1; step < std::min(horizon, int(paths[agent].size())); ++step) {
            if (paths[agent][step] != paths[agent][step - 1]) {
                collectEvents.push_back(EventVertex(agents[agent].speed * step, agent, agents[agent].speed, step, 0));
                collectEvents.push_back(EventVertex(agents[agent].speed * (step - 1), agent, agents[agent].speed, (step - 1), 1));
            }
        }

        if (paths[agent].size()) {
            int lastStep = paths[agent].size() - 1;
            collectEvents.push_back(EventVertex(agents[agent].speed * lastStep, agent, agents[agent].speed, lastStep, 1));
        }
    }

    std::sort(collectEvents.begin(), collectEvents.end());
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> scanLine;

    for (auto curEvent : collectEvents) {
        std::pair<int, int> curVert = paths[curEvent.agent][curEvent.step];
        if (curEvent.type == 0) {
            if (scanLine.find(curVert) != scanLine.end()) {
                if (scanLine[curVert].first != -1) {
                    int a1 = curEvent.agent;
                    int a2 = scanLine[curVert].first;
                    int step1 = curEvent.step;
                    int step2 = scanLine[curVert].second;

                    if (a2 < a1) {
                        std::swap(a1, a2);
                        std::swap(step1, step2);
                    }
                    allConf.insert(std::make_tuple(a1, a2, step1, step2));
                    //Conflict curConflict = Conflict("vertex", {a1, a2}, step1, step2, paths[a1][step1], std::pair<int, int>());
                }
            } else {
                scanLine[curVert] = {curEvent.agent, curEvent.step};
            }
        // if current event is of type 'out'
        } else {
            scanLine[curVert] = {-1, -1};
        }
    }


    // EDGE CONFLICT (SWAP)

    std::vector<EventEdge> collectEventsSwap;
    for (int agent = 0; agent < ln; ++agent) {
        for (int step = 1; step < std::min(horizon, int(paths[agent].size())); ++step) {
            if (paths[agent][step] != paths[agent][step - 1]) {
                int in_time = (step - 1) * agents[agent].speed;
                int out_time = step * agents[agent].speed;
                std::pair<int, int> start = paths[agent][step - 1];
                std::pair<int, int> end = paths[agent][step];

                collectEventsSwap.push_back(EventEdge(in_time, agent, start, end, (step - 1), 0));
                collectEventsSwap.push_back(EventEdge(out_time, agent, start, end, step, 1));
            }
        }
    }


    std::sort(collectEventsSwap.begin(), collectEventsSwap.end());
    std::unordered_map<KeyFour, std::set<std::pair<int, int>>, KeyFourHash, KeyFourEqual> scanLineSwap;

    for (auto curEvent : collectEventsSwap) {
        std::pair<int, int> start = curEvent.start;
        std::pair<int, int> end = curEvent.end;

        KeyFour edgeNormal = std::make_tuple(start.first, start.second, end.first, end.second);
        KeyFour edgeInvert = std::make_tuple(end.first, end.second, start.first, start.second);

        int a1 = curEvent.agent;
        int step1 = curEvent.step;

        if (curEvent.type == 0) {
            if (scanLineSwap.find(edgeNormal) == scanLineSwap.end()) {
                scanLineSwap[edgeNormal] = std::set<std::pair<int, int>>();
            }
            scanLineSwap[edgeNormal].insert({a1, step1});

            if (scanLineSwap.find(edgeInvert) != scanLineSwap.end()) {
                if (scanLineSwap[edgeInvert].size() > 0) {
                    auto iter = scanLineSwap[edgeInvert].begin();
                    std::pair<int, int> other = *iter;
                    int a2 = other.first;
                    int step2 = other.second;

                    if (a2 < a1) {
                        std::swap(a1, a2);
                        std::swap(step1, step2);
                    }
                    allConf.insert(std::make_tuple(a1, a2, step1, step2));
                    //Conflict curConflict = Conflict("edge", {a1, a2}, step, step2, start, end);
                }
            }
        } else {
            auto iter = scanLineSwap[edgeNormal].find({a1, step1 - 1});
            scanLineSwap[edgeNormal].erase(iter);
        }
    }


    // EDGE CONFLICT (OVER)

    std::vector<EventEdge> collectEventsOver;
    for (int agent = 0; agent < ln; ++agent) {
        for (int step = 1; step < std::min(horizon, int(paths[agent].size())); ++step) {
            if (paths[agent][step] != paths[agent][step - 1]) {
                int in_time = (step - 1) * agents[agent].speed;
                int out_time = step * agents[agent].speed;
                std::pair<int, int> start = paths[agent][step - 1];
                std::pair<int, int> end = paths[agent][step];

                collectEventsOver.push_back(EventEdge(in_time, agent, start, end, (step - 1), 0));
                collectEventsOver.push_back(EventEdge(out_time, agent, start, end, step, 1));
            }
        }
    }


    std::sort(collectEventsOver.begin(), collectEventsOver.end());
    std::unordered_map<KeyFour, std::queue<std::pair<int, int>>, KeyFourHash, KeyFourEqual> scanLineOver;

    for (auto curEvent : collectEventsOver) {
        std::pair<int, int> start = curEvent.start;
        std::pair<int, int> end = curEvent.end;

        KeyFour edgeNormal = std::make_tuple(start.first, start.second, end.first, end.second);
        KeyFour edgeInvert = std::make_tuple(end.first, end.second, start.first, start.second);

        int a1 = curEvent.agent;
        int step1 = curEvent.step;

        if (curEvent.type == 0) {
            if (scanLineOver.find(edgeNormal) == scanLineOver.end()) {
                scanLineOver[edgeNormal] = std::queue<std::pair<int, int>>();
            }
            scanLineOver[edgeNormal].push({a1, step1});

        // if current event is of type 'out'
        } else {
            // if first element == our agent
            if ((scanLineOver[edgeNormal].front()).first == a1) {
                scanLineOver[edgeNormal].pop();
            } else {
                std::pair<int, int> other = scanLineOver[edgeNormal].front();
                int a2 = other.first;
                int step2 = other.second;

                if (a2 < a1) {
                    std::swap(a1, a2);
                    std::swap(step1, step2);
                }
                allConf.insert(std::make_tuple(a1, a2, step1, step2));
                //Conflict curConflict = Conflict("edge", {a1, a2}, step1, step2, start, end);
            }
        }
    }

    return allConf.size();
}

bool CTNode::operator< (CTNode &other) {
    return cost < other.cost;
}

bool CTNode::operator== (CTNode &other) {
    return cost == other.cost;
}

bool CTNode::operator!= (CTNode &other) {
    return !(*this == other);
}