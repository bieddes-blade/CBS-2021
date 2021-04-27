#ifndef SEARCH_NODE_H
#define SEARCH_NODE_H

#include <set>
#include <tuple>
#include <unordered_map>

typedef std::tuple<int, int, int> KeyThree;

struct KeyHash : public std::unary_function<KeyThree, size_t> {
    size_t operator()(const KeyThree& k) const {
        return std::get<0>(k) ^ std::get<1>(k) ^ std::get<2>(k);
    }
};

struct KeyEqual : public std::binary_function<KeyThree, KeyThree, bool> {
    bool operator()(const KeyThree& v0, const KeyThree& v1) const {
        return std::get<0>(v0) == std::get<0>(v1) && std::get<1>(v0) == std::get<1>(v1) && std::get<2>(v0) == std::get<2>(v1);
    }
};

typedef std::unordered_map<KeyThree, int, KeyHash, KeyEqual> ConfMap;
typedef std::tuple<int, int, int> KeyThree;

struct SearchNode {
    int i, j, t;
    int numCAT;
    std::set<int> confAgents;
    double f, g, h;
    SearchNode *parent;

    SearchNode(int i, int j)
        : i(i)
        , j(j)
        , t(0)
        , f(0)
        , g(0)
        , h(0)
        , parent(nullptr)
        , numCAT(0)
        , confAgents(std::set<int>())
    { }

    void initCAT(ConfMap& confMap) {
        auto state = std::make_tuple(i, j, t);
        if (confMap.find(state) != confMap.end()) {
            numCAT = confMap[state];
        } else {
            numCAT = 0;
        }
    }

    bool operator< (const SearchNode& other) const {
        if (i == other.i) {
            return j < other.j;
        }
        return i < other.i;
    }

    bool operator== (const SearchNode& other) const {
        return i == other.i && j == other.j && t == other.t;
    }

    bool operator!= (const SearchNode& other) const {
        return !(*this == other);
    }
};

#endif