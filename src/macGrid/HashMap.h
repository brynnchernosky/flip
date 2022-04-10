#ifndef HASHMAP_H
#define HASHMAP_H

#include <algorithm>
#include <unordered_map>
#include <Eigen/Dense>
#include <iostream>

class HashFunction {
public:
    size_t operator() (const Eigen::Vector3i vec) const {
        return 541*vec[0]+79*vec[1]+31*vec[2];
    }
};

#endif // HASHMAP_H
