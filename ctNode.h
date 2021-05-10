#ifndef CTNODE_H
#define CTNODE_H

#include "map.h"
#include "agent.h"
#include "searchNode.h"
#include "search.h"
#include "constr.h"
#include "pairVert.h"

#include <map>
#include <vector>
#include <unordered_map>

// a data structure for vertex constraints
typedef std::vector<std::set<std::pair<int, int>>> VertexConstrStruct;
// a data structure for edge constraints
typedef std::vector<std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>> EdgeConstrStruct;
// a data structure for paths (vector if pairs (i, j))
typedef std::vector<std::pair<int, int>> Path;
// a data structure for pairs (state, set of agents that visited it at some point)
typedef std::unordered_map<KeyThree, std::set<int>, KeyHash, KeyEqual> StateMap;

typedef std::tuple<int, int, int, int> KeyFour;

struct KeyFourHash : public std::unary_function<KeyFour, size_t> {
    size_t operator()(const KeyFour& k) const {
        return std::get<0>(k) ^ std::get<1>(k) ^ std::get<2>(k) ^ std::get<3>(k);
    }
};

struct KeyFourEqual : public std::binary_function<KeyFour, KeyFour, bool> {
    bool operator()(const KeyFour& v0, const KeyFour& v1) const {
        return std::get<0>(v0) == std::get<0>(v1) && std::get<1>(v0) == std::get<1>(v1) && std::get<2>(v0) == std::get<2>(v1) && std::get<3>(v0) == std::get<3>(v1);
    }
};

struct pair_hash final {
    template<class TFirst, class TSecond>
    size_t operator()(const std::pair<TFirst, TSecond>& p) const noexcept {
        uintmax_t hash = std::hash<TFirst>{}(p.first);
        hash <<= sizeof(uintmax_t) * 4;
        hash ^= std::hash<TSecond>{}(p.second);
        return std::hash<uintmax_t>{}(hash);
    }
};

class CTNode {
    public:
        std::vector<Path> paths;
        int cost;
        int maxTime;
        VertexConstrStruct vertexConstr;
        EdgeConstrStruct edgeConstr;
        ConfMap conflictAvoidanceTable;
        StateMap stateAgentMap;
        bool** graph;

        CTNode();

        bool isCover(int V, int k, int E);
        int findMinCover(int n, int m);
        void insertEdge(int u, int v);
        void countCost(std::string heuristic);
        void countCAT();
        int countPathCost(int i, std::string heuristic);
        std::string findConflictType(Map& map, bool dijkstra, std::map<pairVert, int, pvCompare>& distMap, std::vector<Agent>& agents, Conflict& conflict, std::string heuristic);
        Conflict findBestConflict(Map& map, bool dijkstra, std::map<pairVert, int, pvCompare>& distMap, std::vector<Agent>& agents, bool prioritizeConflicts, bool useSymmetry, std::string heuristic, int horizon);
        int findNumOfConflicts(Map& map, std::vector<Agent>& agents, int agentCheck, int horizon);
        bool operator< (CTNode &other);
        bool operator== (CTNode &other);
        bool operator!= (CTNode &other);
};

struct EventVertex {
    int time;
    int agent;
    int v;
    int step;
    int type;

    EventVertex(int time, int agent, int v, int step, int type)
        : time(time)
        , agent(agent)
        , v(v)
        , step(step)
        , type(type)
    { }

    bool operator< (const EventVertex& other) const {
        if (time == other.time) {
            return type < other.type;
        }
        return time < other.time;
    }

    bool operator== (const EventVertex& other) const {
        return time == other.time && type == other.type;
    }

    bool operator!= (const EventVertex& other) const {
        return !(*this == other);
    }
};

struct EventEdge {
    int time;
    int agent;
    std::pair<int, int> start;
    std::pair<int, int> end;
    int step;
    bool type;

    EventEdge(int time, int agent, std::pair<int, int> start, std::pair<int, int> end, int step, int type)
        : time(time)
        , agent(agent)
        , start(start)
        , end(end)
        , step(step)
        , type(type)
    { }

    bool operator< (const EventEdge& other) const {
        if (time == other.time) {
            return type < other.type;
        }
        return time < other.time;
    }

    bool operator== (const EventEdge& other) const {
        return time == other.time && type == other.type;
    }

    bool operator!= (const EventEdge& other) const {
        return !(*this == other);
    }
};

#endif