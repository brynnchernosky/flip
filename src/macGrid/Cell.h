#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>
#include <set>

#include "src/macGrid/Material.h"

struct Particle;

struct Cell
{
  float pseudoPressure = 0;

  Eigen::Vector3f u = Eigen::Vector3f::Zero();
  Eigen::Vector3f oldU = Eigen::Vector3f::Zero();

  Eigen::Vector3f uWeights = Eigen::Vector3f::Zero();

  Eigen::Vector3i cellIndices = Eigen::Vector3i::Zero();

  Material material = Material::Air;
  int layer = -1;

  std::vector<Particle *> particles;

  // Temporary variables valid only within transferParticleToGrid()
  Eigen::Vector3f temp_avgParticleV = Eigen::Vector3f::Zero();
  Eigen::Vector3i temp_particleNums = Eigen::Vector3i::Zero();

  // Temporary variables valid only within pseudopressure calculation
  int index = -1;
};

#endif // CELL_H
