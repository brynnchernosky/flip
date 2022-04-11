#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>
#include <vector>

enum Material
{
  Solid,
  Fluid,
  Air
};

struct Particle;

struct Cell
{
  float ux;
  float uy;
  float uz;
  float p;

  Material material = Material::Air;
  int layer = -1;

  std::vector<Particle *> particles;
};

#endif // CELL_H
