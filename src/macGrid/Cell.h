#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>
#include <set>

#include "src/macGrid/Material.h"

struct Particle;

struct Cell
{
  float pseudoPressure = 0;

  float ux;
  float uy;
  float uz;

  float oldUX;
  float oldUY;
  float oldUZ;

  Eigen::Vector3f avgParticleV = Eigen::Vector3f(0,0,0);
  int particleNums = 0;


  Eigen::Vector3i cellIndices;

  int index;

  Material material = Material::Air;
  int layer = -1;
};

#endif // CELL_H
