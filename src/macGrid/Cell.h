#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>
#include <set>

#include "src/macGrid/Material.h"

struct Particle;

struct Cell
{
  Cell() {};
  Cell(Material _m, Eigen::Vector3i _c) : material(_m), cellIndices(_c) {};

  Material material = Material::Air;
  Eigen::Vector3i cellIndices = Eigen::Vector3i::Zero();

  // Velocity field
  Eigen::Vector3f u    = Eigen::Vector3f::Zero();
  Eigen::Vector3f oldU = Eigen::Vector3f::Zero();

  // Only valid for fluid cells
  std::vector<Particle *> particles;

  // Only valid for non-boundary solid cells
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();

  // Temporary. Only valid when set in the same function
  Eigen::Vector3f uWeights = Eigen::Vector3f::Zero();
  int layer = -1;
  int index = -1;
};

#endif // CELL_H
