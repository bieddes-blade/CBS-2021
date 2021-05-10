#ifndef PAIR_VERT_H
#define PAIR_VERT_H

// a data structure for pairs of pairs
struct pairVert {
    std::pair<int, int> from;
    std::pair<int, int> to;
};

// a comparator for this struct
struct pvCompare {
    bool operator() (const pairVert& lhs, const pairVert& rhs) const {
        if (lhs.from.first == rhs.from.first) {
            if (lhs.from.second == rhs.from.second) {
                if (lhs.to.first == rhs.to.first) {
                    return lhs.to.second < rhs.to.second;
                }
                return lhs.to.first < rhs.to.first;
            }
            return lhs.from.second < rhs.from.second;
        }
        return lhs.from.first < rhs.from.first;
    }
};

#endif