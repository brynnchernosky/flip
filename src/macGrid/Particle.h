#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Dense>
#include <iostream>

using namespace std;

struct Cell;

struct Particle
{
  Particle() {};
  Particle(Eigen::Vector3f _p) : position(_p) {};
  Particle(Eigen::Vector3f _p, Eigen::Vector3f _v) : position(_p), velocity(_v) {};

  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

  Eigen::Vector3f oldPosition = Eigen::Vector3f::Zero();
  Eigen::Vector3f oldVelocity = Eigen::Vector3f::Zero();

  bool foamParticle = false;
};

#endif // PARTICLE_H
