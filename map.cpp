#include "map.h"

#include <iostream>
#include <fstream>

Map::Map() {
    height = 0;
    width = 0;
    grid = NULL;
    metricType = "manhattan";
    hweight = 1;
}

void Map::getMapOptions(std::ifstream& mapFile) {
    std::string word;
    mapFile >> word >> word;
    mapFile >> word >> height;
    mapFile >> word >> width;
}

void Map::getMapGrid(std::ifstream& mapFile) {
    std::string word;
    mapFile >> word;

    grid = new int *[height];
    for (int i = 0; i < height; ++i) {
        grid[i] = new int[width];
    }

    for (int line = 0; line < height; ++line) {
        std::string curLine;
        mapFile >> curLine;
        for (int place = 0; place < width; ++place) {
            char letter = curLine[place];

            if (letter == '.' || letter == 'G' || letter == 'S' || letter == 'W') {
                grid[line][place] = 0;
            } else {
                grid[line][place] = 1;
            }
        }
    }
}

bool Map::cellOnGrid(int i, int j) const {
    return i >= 0 && i < height && j >= 0 && j < width;
}

bool Map::cellIsTraversable(int i, int j) const {
    return cellOnGrid(i, j) && grid[i][j] == 0;
}