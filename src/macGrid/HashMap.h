#ifndef HASHMAP_H
#define HASHMAP_H

#include <algorithm>
#include <Eigen/Dense>
#include <iostream>
#include <unordered_map>

class HashFunction
{
  public:

    size_t operator() (const Eigen::Vector3i &v) const
    {
      return 541 * v[0] + 79 * v[1] + 31 * v[2];
    }
};

#endif // HASHMAP_H
