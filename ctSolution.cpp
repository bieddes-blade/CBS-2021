#include "map.h"
#include "agent.h"
#include "searchNode.h"
#include "search.h"
#include "constr.h"
#include "ctNode.h"
#include "ctSolution.h"

#include <unordered_map>
#include <vector>
#include <tuple>
#include <iostream>
#include <cstdlib>
#include <climits>

CTSolution::CTSolution(Map& map_read, std::vector<Agent>& agents_read, bool useDijkstraPrecalc_read, bool useCAT_read, std::string heuristic_read, bool prioritizeConflicts_read, bool useBypass_read, bool useFocal_read, double omega_read, bool printPaths_read) {
    map = map_read;
    agents = agents_read;
    useDijkstraPrecalc = useDijkstraPrecalc_read;
    useCAT = useCAT_read;
    heuristic = heuristic_read;
    prioritizeConflicts = prioritizeConflicts_read;
    useBypass = useBypass_read;
    useFocal = useFocal_read;
    omega = omega_read;
    printPaths = printPaths_read;
}

Path CTSolution::lowLevelSearch(CTNode node, int i) {
    // create a search object
    // it uses search options (type of search, constraints etc) and returns paths
    // the paths may include conflicts
    Search search(map);

    // precompute perfect heuristic
    if (useDijkstraPrecalc) {
        search.dijkstraPrecalc(map, node.vertexConstr, node.edgeConstr, agents[i]);
    }

    search.startSearch(map, node.vertexConstr, node.edgeConstr, node.conflictAvoidanceTable, agents[i], node.stateAgentMap, useDijkstraPrecalc, useFocal, omega);
    return search.fullPath;
}

std::vector<Path> CTSolution::highLevelSearch() {
    // create the root of the CT with paths made by low-level search
    CTNode root;
    for (int agent = 0; agent < agents.size(); ++agent) {
        root.paths.push_back(lowLevelSearch(root, agent));
    }
    // count the cost of the current solution
    root.countCost(heuristic);

    if (useCAT) {
        root.countCAT();
    }

    // add the root node to the heap
    // nodes are sorted by solution cost
    heap.push(root);

    // while there are nodes to analyze, take the best node and analyze it
    while (!heap.empty()) {
        // extract the best node from the CT
        CTNode best;
        best = heap.top();
        heap.pop();

        // find the best conflict in this node
        Conflict conflict = best.findBestConflict(map, agents, prioritizeConflicts, heuristic);

        // if there are no conflicts, we have found the goal node
        if (conflict.type == "none") {
            std::cout << "Solution Cost: " << best.cost << "\n";
            // if option printPaths is activated, we print out the path of each agent
            if (printPaths) {
                for (int agent = 0; agent < (best.paths).size(); ++agent) {
                    std::cout << "agent " << agent + 1 << ": [ ";
                    for (int i = 0; i < (best.paths[agent]).size(); ++i) {
                        auto currentPair = (best.paths[agent])[i];
                        std::cout << "[" << currentPair.first << ", " << currentPair.second << "] ";
                    }
                    std::cout << "]\n";
                }
            }

            return best.paths;
        }

        // some variables for bypass
        bool bypassFound = false;
        std::vector<CTNode> twoNodes;
        int curConfNum, curPathCost, newConfNum, newPathCost;

        // if there is a conflict, we construct constraints on this conflict
        // two agents take part in a conflict: agent_number == 0 and agent_number == 1
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

            // create a new node
            // it inherits vertex and edge constraints from the best node
            CTNode node;
            node.vertexConstr = best.vertexConstr;
            node.edgeConstr = best.edgeConstr;

            // we are only going to change one agent's path
            // other paths are inherited from the best node
            node.paths = best.paths;
            if (useCAT) {
                node.countCAT();
            }

            if (useBypass) {
                // count old path cost and number of conflicts involving one agent
                curConfNum = node.findNumOfConflicts(map, agents, agent);
                curPathCost = node.countPathCost(agent, heuristic);
            }

            // with a rectangular vertex conflict, we need to insert a barrier constraint
            // we count Rk and Rg and prohibit the agent ak from occupying all locations along the border of the rectangle
            // that is opposite of its start node at the timestep when ak would optimally reach the location
            if (conflict.type == "rectangular") {
                int a1 = (conflict.agents).first;
                int a2 = (conflict.agents).second;

                int start_ix = agents[a1].start_i;
                int start_iy = agents[a1].start_j;
                int goal_ix = agents[a1].fin_i;
                int goal_iy = agents[a1].fin_j;

                int start_jx = agents[a2].start_i;
                int start_jy = agents[a2].start_j;
                int goal_jx = agents[a2].fin_i;
                int goal_jy = agents[a2].fin_j;

                int start_it = 0;
                int goal_it = node.paths[a1].size() - 1;
                int start_jt = 0;
                int goal_jt = node.paths[a2].size() - 1;

                int R_sx, R_gx, R_sy, R_gy, R_ix, R_jx, R_iy, R_jy;
                int R_it, R_jt, R_st, R_gt;

                if (start_ix == goal_ix) {
                    R_sx = start_ix;
                    R_gx = goal_ix;
                } else if (start_ix < goal_ix) {
                    R_sx = std::max(start_ix, start_jx);
                    R_gx = std::min(goal_ix, goal_jx);
                } else {
                    R_sx = std::min(start_ix, start_jx);
                    R_gx = std::max(goal_ix, goal_jx);
                }

                if (start_iy == goal_iy) {
                    R_sy = start_iy;
                    R_gy = goal_iy;
                } else if (start_iy < goal_iy) {
                    R_sy = std::max(start_iy, start_jy);
                    R_gy = std::min(goal_iy, goal_jy);
                } else {
                    R_sy = std::min(start_iy, start_jy);
                    R_gy = std::max(goal_iy, goal_jy);
                }

                if ((start_ix - start_jx) * (start_jx - R_gx) >= 0) {
                    R_ix = R_gx;
                    R_iy = start_iy;
                    R_jx = start_jx;
                    R_jy = R_gy;
                } else {
                    R_ix = start_ix;
                    R_iy = R_gy;
                    R_jx = R_gx;
                    R_jy = start_jy;
                }

                R_it = start_it + std::abs(start_ix - R_ix) + std::abs(start_iy - R_iy);
                R_jt = start_it + std::abs(start_ix - R_jx) + std::abs(start_iy - R_jy);
                R_st = start_it + std::abs(start_ix - R_sx) + std::abs(start_iy - R_sy);
                R_gt = start_it + std::abs(start_ix - R_gx) + std::abs(start_iy - R_gy);

                if (agent == 1) {
                    if (R_ix == R_gx) {
                        for (int n = R_it; n <= R_gt; ++n) {
                            node.vertexConstr[n].insert({R_ix, R_iy + n - R_it});
                        }
                    } else if (R_iy == R_gy) {
                        for (int n = R_it; n <= R_gt; ++n) {
                            node.vertexConstr[n].insert({R_ix + n - R_it, R_iy});
                        }
                    }
                } else {
                    if (R_jx == R_gx) {
                        for (int n = R_jt; n <= R_gt; ++n) {
                            node.vertexConstr[n].insert({R_jx, R_jy + n - R_jt});
                        }
                    } else if (R_jy == R_gy) {
                        for (int n = R_jt; n <= R_gt; ++n) {
                            node.vertexConstr[n].insert({R_jx + n - R_jt, R_jy});
                        }
                    }
                }

                node.vertexConstr[confTime].insert(conflict.v1);



            // with a vertex conflict, we only need to insert a constraint that doesn't allow an agent
            // to be in a certain place at a certain time
            } else if (conflict.type == "vertex") {
                node.vertexConstr[confTime].insert(conflict.v1);
            
            // with an edge conflict, we insert a conflict tuple
            // the pairs correspond to the vertices at the ends of the edge
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

            // we find a new path for this agent
            node.paths[agent] = lowLevelSearch(node, agent);
            // and count the node's cost
            node.countCost(heuristic);

            if (useBypass) {
                newConfNum = node.findNumOfConflicts(map, agents, agent);
                newPathCost = node.countPathCost(agent, heuristic);

                // if the cost does not change and confNum decreases, we have found a helpful bypass!
                // we adopt it and don't split the node
                if (newPathCost == curPathCost && newConfNum < curConfNum) {
                    bypassFound = true;
                    heap.push(node);
                    break;
                // otherwise, store this node in a vector
                } else {
                    twoNodes.push_back(node);
                }
            }

            if (!useBypass) {
                heap.push(node);
            }
        }

        // if there were no helpful bypasses, we just split the node
        if (useBypass && !bypassFound) {
            heap.push(twoNodes[0]);
            heap.push(twoNodes[1]);
        }
    }

    // if we didn't find a solution
    std::cout << "NO SOLUTION\n";
    return root.paths;
}