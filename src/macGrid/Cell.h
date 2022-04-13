#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>
#include <set>

#include "src/macGrid/Material.h"

struct Particle;

struct Cell
{
  float ux;
  float uy;
  float uz;
  float p;

  Material material = Material::Air;
  int layer = -1;

  std::set<Particle *> particles;
};

#endif // CELL_H
