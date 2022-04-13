#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Dense>

struct Cell;

struct Particle
{
  Cell * cell;

  Eigen::Vector3f position;
  Eigen::Vector3f velocity;

  // Eigen::Vector3f picVelocity;
  // Eigen::Vector3f flipVelocity;
};

#endif // PARTICLE_H
