#ifndef CTSOLUTION_H
#define CTSOLUTION_H

#include "search.h"
#include "ctNode.h"
#include "pairVert.h"

#include <queue>
#include <vector>

// a comparator for comparing constraint tree nodes
struct CompareCBS {
    bool operator()(CTNode& one, CTNode& two) {
        return one.cost < two.cost;
    }
};

// an object for storing a CBS problem and its solution
class CTSolution {
    public:
        std::priority_queue<CTNode, std::vector<CTNode>, CompareCBS> heap;
        Map map;
        std::vector<Agent> agents;
        std::map<pairVert, int, pvCompare> distMap;
        bool useDijkstraPrecalc;
        bool useCAT;
        std::string heuristic;
        bool prioritizeConflicts;
        bool useBypass;
        bool useFocal;
        double omega;
        bool useSymmetry;
        bool online;
        std::vector<std::vector<std::pair<int, int>>> goalLocs;
        int horizon;
        int replanning;
        bool printPaths;

        CTSolution(Map& map_read, std::vector<Agent>& agents_read, bool useDijkstraPrecalc_read, bool useCAT_read, std::string heuristic_read, bool prioritizeConflicts_read, bool useBypass_read, bool useFocal_read, double omega_read, bool useSymmetry_read, bool online_read, std::vector<std::vector<std::pair<int, int>>>& goalLocs_read, int horizon, int replanning, bool printPaths_read);
        
        std::map<pairVert, int, pvCompare> dijkstraPrecalc(Map& map);
        Path lowLevelSearch(CTNode node, int i);
        std::vector<Path> highLevelSearch();
        void solve();
};

#endif