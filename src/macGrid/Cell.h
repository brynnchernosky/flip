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

  Eigen::Vector3f old_u = Eigen::Vector3f::Zero();

  Eigen::Vector3f avgParticleV = Eigen::Vector3f::Zero();
  int particleNums = -1;


  Eigen::Vector3i cellIndices = Eigen::Vector3i::Zero();

  int index = -1;

  Material material = Material::Air;
  int layer = -1;
};

#endif // CELL_H
