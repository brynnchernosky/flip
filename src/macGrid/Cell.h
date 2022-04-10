#ifndef CELL_H
#define CELL_H

#include <Eigen/Dense>

struct HalfFace;

struct Cell
{
  HalfFace *top;
  HalfFace *bottom;
  HalfFace *north;
  HalfFace *south;
  HalfFace *east;
  HalfFace *west;

  float pseudoPressure;
};

#endif // CELL_H
