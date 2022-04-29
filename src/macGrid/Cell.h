#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>
#include <set>

#include "src/macGrid/Material.h"

struct Particle;

struct Cell
{
  float pseudoPressure = 0;

  float ux = 0;
  float uy = 0;
  float uz = 0;

  float oldUX = 0;
  float oldUY = 0;
  float oldUZ = 0;

  Eigen::Vector3f avgParticleV = Eigen::Vector3f::Zero();
  int particleNums = -1;


  Eigen::Vector3i cellIndices = Eigen::Vector3i::Zero();

  int index = -1;

  Material material = Material::Air;
  int layer = -1;
};

#endif // CELL_H
