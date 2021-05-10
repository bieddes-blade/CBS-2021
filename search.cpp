#include "search.h"

#include <set>
#include <list>
#include <cmath>
#include <vector>

std::string curBreakingTies;
bool curUseCAT;

Search::Search(Map& map) {
    partPath = std::vector<SearchNode>();
    fullPath = Path();
    curBreakingTies = "g-max";
    curUseCAT = true;
}

bool CompareAStar(const SearchNode& one, const SearchNode& two) {
    if (one.f == two.f) {
        if (!curUseCAT) {
            if (one.g == two.g) {
                return one < two;
            } else {
                if (curBreakingTies == "g-min") {
                    return one.g < two.g;
                }
                return one.g > two.g;
            }

        } else {
            if (one.numCAT == two.numCAT) {
                if (one.g == two.g) {
                    return one < two;
                } else {
                    if (curBreakingTies == "g-min") {
                        return one.g < two.g;
                    }
                    return one.g > two.g;
                }
            }
            return one.numCAT < two.numCAT;
        }
    }
    return one.f < two.f;
}

bool CompareFocal(const SearchNode& one, const SearchNode& two) {
    return (one.confAgents).size() < (two.confAgents).size();
}

bool Search::checkVertexConstr(int i, int j, int t, VertexConstrStruct& vertexConstr) {
    if (t < vertexConstr.size()) {
        auto result = vertexConstr[t].find({i, j});
        return result != vertexConstr[t].end();
    }
    return false;
}

bool Search::checkEdgeConstr(int i1, int j1, int i2, int j2, int time, EdgeConstrStruct& edgeConstr) {
    if (time < edgeConstr.size()) {
        auto result = edgeConstr[time].find({{i1, j1}, {i2, j2}});
        return result != edgeConstr[time].end();
    }
    return false;
}


double Search::diagonal(Map& map, int i1, int j1, int i2, int j2) {
    double sqrtTwo = 1.41421356237;
    return (std::abs(std::abs(i2 - i1) - std::abs(j2 - j1)) + (std::min(std::abs(i2 - i1), std::abs(j2 - j1))) * sqrtTwo) * map.hweight;
}

double Search::manhattan(Map& map, int i1, int j1, int i2, int j2) {
    return (std::abs(i1 - i2) + std::abs(j1 - j2)) * map.hweight;
}

double Search::euclidean(Map& map, int i1, int j1, int i2, int j2) {
    return std::sqrt((i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2)) * map.hweight;
}

double Search::chebyshev(Map& map, int i1, int j1, int i2, int j2) {
    return std::max(std::abs(i1 - i2), std::abs(j1 - j2)) * map.hweight;
}


double Search::computeHFromCellToCell(Map& map, int i1, int j1, std::vector<int> i2, std::vector<int> j2, int label, bool dijkstra, std::map<pairVert, int, pvCompare>& distMap) {
    double answer = 0.0;
    int done = int(i2.size());

    if (dijkstra) {
        pairVert v;
        v.from = {i1, j1};
        v.to = {i2[label], j2[label]};
        answer += distMap[v];
        for (int goal = label + 1; goal < done; ++goal) {
            v.from = {i2[goal], j2[goal]};
            v.to = {i2[goal - 1], j2[goal - 1]};
            answer += distMap[v];   
        }
        return answer;
    }

    if (map.metricType == "diagonal") {
        answer += diagonal(map, i1, j1, i2[label], j2[label]);
        for (int goal = label + 1; goal < done; ++goal) {
            answer += diagonal(map, i2[goal], j2[goal], i2[goal - 1], j2[goal - 1]);     
        }

    } else if (map.metricType == "manhattan") {
        answer += manhattan(map, i1, j1, i2[label], j2[label]);
        for (int goal = label + 1; goal < done; ++goal) {
            answer += manhattan(map, i2[goal], j2[goal], i2[goal - 1], j2[goal - 1]);     
        }

    } else if (map.metricType == "euclidean") {
        answer += euclidean(map, i1, j1, i2[label], j2[label]);
        for (int goal = label + 1; goal < done; ++goal) {
            answer += euclidean(map, i2[goal], j2[goal], i2[goal - 1], j2[goal - 1]);     
        }

    } else if (map.metricType == "chebyshev") {
        answer += chebyshev(map, i1, j1, i2[label], j2[label]);
        for (int goal = label + 1; goal < done; ++goal) {
            answer += chebyshev(map, i2[goal], j2[goal], i2[goal - 1], j2[goal - 1]);     
        }
    }

    return answer;
}

void Search::startSearch(Map& map, std::map<pairVert, int, pvCompare>& distMap, VertexConstrStruct& vertexConstr, EdgeConstrStruct& edgeConstr, ConfMap& conflictAvoidanceTable, Agent& agent, StateMap& states, bool dijkstra, bool useFocal, double omega) {
    SearchNode startNode(agent.start_i, agent.start_j);
    SearchNode finNode(agent.fin_i[0], agent.fin_j[0]);

    if (curUseCAT) {
        startNode.initCAT(conflictAvoidanceTable);
        finNode.initCAT(conflictAvoidanceTable);
    }
    
    startNode.h = computeHFromCellToCell(map, agent.start_i, agent.start_j, agent.fin_i, agent.fin_j, startNode.label, dijkstra, distMap);
    startNode.f = startNode.g + startNode.h;

    std::set<SearchNode, bool(*)(const SearchNode&, const SearchNode&)> open(&CompareAStar);
    std::set<SearchNode> openCopy;
    std::set<SearchNode> closed;
    std::list<SearchNode> parents;

    bool pathFound = false;

    open.insert(startNode);
    openCopy.insert(startNode);

    while (!pathFound && !open.empty()) {
        auto best = open.begin();
        SearchNode curNode = *best;

        if (useFocal) {
            double fmin = curNode.f;
            std::set<SearchNode, bool(*)(const SearchNode&, const SearchNode&)> focal(&CompareFocal);
            for (auto elem: open) {
                if (elem.f <= omega * fmin) {
                    focal.insert(elem);
                }
            }
            auto best = focal.begin();
            curNode = *best;
        }

        open.erase(curNode);
        openCopy.erase(curNode);

        if (curNode.i == agent.fin_i[curNode.label] && curNode.j == agent.fin_j[curNode.label]) {
            ++curNode.label;
        }

        if (curNode.label == agent.fin_i.size()) {
            finNode = curNode;
            pathFound = true;
        } else {
            closed.insert(curNode);

            parents.push_back(curNode);

            std::list<SearchNode> successors = findSuccessors(curNode, map, vertexConstr, edgeConstr, agent, dijkstra, distMap);
            for (SearchNode& scNode: successors) {
                if (closed.find(scNode) == closed.end() && curNode.g + 1 <= scNode.g) {

                    scNode.g = curNode.g + 1;
                    scNode.h = computeHFromCellToCell(map, scNode.i, scNode.j, agent.fin_i, agent.fin_j, scNode.label, dijkstra, distMap);
                    scNode.f = scNode.g + scNode.h;

                    scNode.t = curNode.t + 1;
                    scNode.parent = &parents.back();
                    scNode.confAgents = (&parents.back())->confAgents;
                    scNode.label = curNode.label;

                    if (useFocal) {
                        KeyThree state = std::make_tuple(scNode.i, scNode.j, scNode.t);
                        if (states.find(state) != states.end()) {
                            for (auto elem : states[state]) {
                                if (elem != agent.agentId) {
                                    (scNode.confAgents).insert(elem);
                                }
                            }
                        }
                    }

                    if (curUseCAT) {
                        scNode.initCAT(conflictAvoidanceTable);
                    }

                    auto check = openCopy.find(scNode);

                    if (check != openCopy.end() && (*check).f > scNode.f) {
                        open.erase(*check);
                        openCopy.erase(check);

                        open.insert(scNode);
                        openCopy.insert(scNode);

                    }

                    if (check == openCopy.end()) {
                        open.insert(scNode);
                        openCopy.insert(scNode);
                    }
                }
            }
        }
    }

    if (pathFound) {
        makePartPath(finNode, startNode);
        makeFullPath();
    }
}

std::list<SearchNode> Search::findSuccessors(SearchNode& curNode, Map& map, VertexConstrStruct& vertexConstr, EdgeConstrStruct& edgeConstr, Agent& agent, bool dijkstra, std::map<pairVert, int, pvCompare>& distMap) {
    std::list<SearchNode> successors;
    int cur_i = curNode.i;
    int cur_j = curNode.j;

    // cycle through the neighbours
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if ((i != 0 || j != 0) && !(i != 0 && j != 0) && map.cellOnGrid(cur_i + i, cur_j + j) && map.cellIsTraversable(cur_i + i, cur_j + j)) {
                // create a 'neighbour state', calculate its g, h, f
                SearchNode scNode(cur_i + i, cur_j + j);

                scNode.g = curNode.g + 1;
                scNode.h = computeHFromCellToCell(map, scNode.i, scNode.j, agent.fin_i, agent.fin_j, scNode.label, dijkstra, distMap);
                scNode.f = scNode.g + scNode.h;
                scNode.t = curNode.t + 1;
                scNode.label = curNode.label;

                if (!checkVertexConstr(scNode.i, scNode.j, scNode.t, vertexConstr)) {
                    if (!checkEdgeConstr(curNode.i, curNode.j, scNode.i, scNode.j, curNode.t, edgeConstr)) {
                        successors.push_back(scNode);
                    }
                }
            }
        }
    }

    // create a 'neighbour state' for waiting
    SearchNode scNode(cur_i, cur_j);

    scNode.g = curNode.g + 1;
    scNode.h = curNode.h;
    scNode.f = scNode.g + scNode.h;
    scNode.t = curNode.t + 1;
    scNode.label = curNode.label;

    if (!checkVertexConstr(scNode.i, scNode.j, scNode.t, vertexConstr)) {
        if (!checkEdgeConstr(curNode.i, curNode.j, scNode.i, scNode.j, curNode.t, edgeConstr)) {
            successors.push_back(scNode);
        }
    }

    return successors;
}

void Search::makePartPath(SearchNode curNode, SearchNode startNode) {
    partPath.clear();
    while (curNode.parent != nullptr) {
        partPath.push_back(curNode);
        curNode = *curNode.parent;
    }

    partPath.push_back(curNode);
    reverse(partPath.begin(), partPath.end());
}

void Search::makeFullPath() {
    fullPath.clear();
    int ln = partPath.size();
    for (int node  = 0; node < ln; ++node) {
        if (node + 1 < ln) {
            for (int time = 0; time < partPath[node + 1].t - partPath[node].t; ++time) {
                fullPath.push_back({partPath[node].i, partPath[node].j});
            }
        } else {
            fullPath.push_back({partPath[node].i, partPath[node].j});
        }
    }
}