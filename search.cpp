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
    dist = std::vector<std::vector<double>>(map.height, std::vector<double>(map.width, -1));
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

bool CompareDijkstra(const std::pair<double, SearchNode>& one, const std::pair<double, SearchNode>& two) {
    return one.first < two.first;
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

double Search::computeHFromCellToCell(Map& map, int i1, int j1, int i2, int j2) {
    double sqrtTwo = 1.41421356237;
    if (map.metricType == "diagonal") {
        return (std::abs(std::abs(i2 - i1) - std::abs(j2 - j1)) + (std::min(std::abs(i2 - i1), std::abs(j2 - j1))) * sqrtTwo) * map.hweight;
    } else if (map.metricType == "manhattan") {
        return std::abs(i1 - i2) * map.hweight + std::abs(j1 - j2);
    } else if (map.metricType == "euclidean") {
        return std::sqrt((i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2)) * map.hweight;
    } else if (map.metricType == "chebyshev") {
        return std::max(std::abs(i1 - i2), std::abs(j1 - j2)) * map.hweight;
    } else {
        return 0;
    }
}

void Search::dijkstraPrecalc(Map& map, VertexConstrStruct& vertexConstr, EdgeConstrStruct& edgeConstr, Agent& agent) {
    SearchNode startNode(agent.start_i, agent.start_j);
    SearchNode finNode(agent.fin_i, agent.fin_j);

    bool pathFound = false;

    std::set<std::pair<double, SearchNode>, bool(*)(const std::pair<double, SearchNode>&, const std::pair<double, SearchNode>&)> heap(&CompareDijkstra);

    dist[finNode.i][finNode.j] = 0;
    heap.insert({dist[finNode.i][finNode.j], finNode});

    while (!pathFound && !heap.empty()) {
        auto best = heap.begin();
        std::pair<double, SearchNode> curBest = *best;
        heap.erase(curBest);
        SearchNode curNode = curBest.second;

        if (curNode.i == startNode.i && curNode.j == startNode.j) {
            pathFound = true;
        } else {
            std::list<SearchNode> successors = findSuccessors(curNode, map, vertexConstr, edgeConstr, agent, true);
            for (SearchNode scNode : successors) {
                double length = computeHFromCellToCell(map, scNode.i, scNode.j, curNode.i, curNode.j);
                double curDist = dist[curNode.i][curNode.j];

                if (dist[scNode.i][scNode.j] != -1) {
                    if (curDist + length < dist[scNode.i][scNode.j]) {
                        heap.erase({dist[scNode.i][scNode.j], scNode});
                        dist[scNode.i][scNode.j] = curDist + length;
                        heap.insert({dist[scNode.i][scNode.j], scNode});
                    }
                } else {
                    dist[scNode.i][scNode.j] = curDist + length;
                    heap.insert({dist[scNode.i][scNode.j], scNode});
                }
            }
        }
    }
}

void Search::startSearch(Map& map, VertexConstrStruct& vertexConstr, EdgeConstrStruct& edgeConstr, ConfMap& conflictAvoidanceTable, Agent& agent, StateMap& states, bool useDijkstra, bool useFocal, double omega) {
    SearchNode startNode(agent.start_i, agent.start_j);
    SearchNode finNode(agent.fin_i, agent.fin_j);

    if (curUseCAT) {
        startNode.initCAT(conflictAvoidanceTable);
        finNode.initCAT(conflictAvoidanceTable);
    }

    if (dist[startNode.i][startNode.j] != -1) {
        startNode.h = dist[startNode.i][startNode.j];
    } else {
        startNode.h = computeHFromCellToCell(map, agent.start_i, agent.start_j, agent.fin_i, agent.fin_j);
    }
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

        if (curNode.i == finNode.i && curNode.j == finNode.j) {
            finNode = curNode;
            pathFound = true;
        } else {
            open.erase(curNode);
            openCopy.erase(curNode);
            closed.insert(curNode);

            parents.push_back(curNode);

            std::list<SearchNode> successors = findSuccessors(curNode, map, vertexConstr, edgeConstr, agent, false);
            for (SearchNode& scNode: successors) {
                if (closed.find(scNode) == closed.end() && curNode.g + 1 <= scNode.g) {

                    scNode.g = curNode.g + 1;

                    if (dist[scNode.i][scNode.j] != -1) {
                        scNode.h = dist[scNode.i][scNode.j];
                    } else {
                        scNode.h = computeHFromCellToCell(map, scNode.i, scNode.j, agent.fin_i, agent.fin_j);
                    }

                    scNode.f = scNode.g + scNode.h;
                    scNode.t = curNode.t + 1;
                    scNode.parent = &parents.back();
                    scNode.confAgents = (&parents.back())->confAgents;

                    KeyThree state = std::make_tuple(scNode.i, scNode.j, scNode.t);
                    if (states.find(state) != states.end()) {
                        for (auto elem : states[state]) {
                            if (elem != agent.agentId) {
                                (scNode.confAgents).insert(elem);
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

std::list<SearchNode> Search::findSuccessors(SearchNode& curNode, Map& map, VertexConstrStruct& vertexConstr, EdgeConstrStruct& edgeConstr, Agent& agent, bool dijkstra) {
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
                scNode.h = computeHFromCellToCell(map, scNode.i, scNode.j, agent.fin_i, agent.fin_j);
                scNode.f = scNode.g + scNode.h;
                scNode.t = curNode.t + 1;

                if (!dijkstra) {
                    if (!checkVertexConstr(scNode.i, scNode.j, scNode.t, vertexConstr)) {
                        if (!checkEdgeConstr(curNode.i, curNode.j, scNode.i, scNode.j, curNode.t, edgeConstr)) {
                            successors.push_back(scNode);
                        }
                    }
                } else {
                    successors.push_back(scNode);
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

    if (!dijkstra) {
        if (!checkVertexConstr(scNode.i, scNode.j, scNode.t, vertexConstr)) {
            if (!checkEdgeConstr(curNode.i, curNode.j, scNode.i, scNode.j, curNode.t, edgeConstr)) {
                successors.push_back(scNode);
            }
        }
    } else {
        successors.push_back(scNode);
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