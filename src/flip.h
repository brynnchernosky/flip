#ifndef FLIP_H
#define FLIP_H

#include "graphics/shape.h"

#include <Eigen/StdList>
#include <Eigen/StdVector>

class Flip
{
  public:

    Flip();
    ~Flip();

    void init();
    void draw() const;

  private:

    Shape m_shape;
    // HalfFaceGrid m_grid;
};

#endif // FLIP_H
