#ifndef FACE_H
#define FACE_H

#include <Eigen/Dense>

struct Cell;

struct Face
{
  Cell *cellA; // cellA = lower in x, y, or z
  Cell *cellB;
  
  float velocity;
};

#endif // FACE_H
