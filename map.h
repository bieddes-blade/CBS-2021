#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <fstream>

class Map {
    public:  
        int height;
        int width;
        int** grid;
        std::string metricType;
        int hweight;

        Map();

        void getMapOptions(std::ifstream& mapFile);
        void getMapGrid(std::ifstream& mapFile);
        bool cellOnGrid(int i, int j) const;
        bool cellIsTraversable(int i, int j) const;
};

#endif