#ifndef DEBUG_H
#define DEBUG_H

#include <Eigen/Dense>
#include <string>

#include "src/macGrid/Material.h"
#include "src/macGrid/Cell.h"
#include "src/macGrid/Particle.h"

namespace Debug {

  std::string vectorToString(const Eigen::Vector3f &v)
  {
    return std::string("<")
           + std::to_string(v[0]) + ","
           + std::to_string(v[1]) + ","
           + std::to_string(v[2]) + ">";
  }

  std::string vectorToString(const Eigen::Vector3i &v)
  {
    return std::string("<")
           + std::to_string(v[0]) + ","
           + std::to_string(v[1]) + ","
           + std::to_string(v[2]) + ">";
  }

  std::string materialToString(const Material m)
  {
    switch (m) {
      case Material::Solid: return "solid";
      case Material::Fluid: return "fluid";
      case Material::Air:   return "air";
    }
    return "invalid material";
  }

  std::string cellToString(Cell * const c)
  {
    return vectorToString(c->cellIndices)
           + ": cell with material "
           + materialToString(c->material)
           + ", layer "
           + std::to_string(c->layer)
           + ", and velocity "
           + vectorToString(Eigen::Vector3f(c->u[0], c->u[1], c->u[2]));
  }


  std::string particleToString(Particle * const p)
  {
    return std::string("Particle with position ")
           + vectorToString(p->position)
           + " and velocity "
           + vectorToString(p->velocity);
  }

}


#endif // DEBUG_H
