#ifndef MACGRID_H
#define MACGRID_H

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <vector>

#include "HashMap.h"
#include "Cell.h"
#include "Particle.h"
#include "Material.h"

class MacGrid
{
  public:

    MacGrid(float cellWidth, const Eigen::Vector3i cellCount, const Eigen::Vector3f cornerPosition);
    ~MacGrid();

    void validate();
    void init();
    void simulate();
    void updateGrid();

    // Debugging
    void addParticle(Eigen::Vector3f position, Eigen::Vector3f velocity);
    void printGrid() const;

  private:

    const float m_cellWidth;
    const Eigen::Vector3i m_cellCount;
    const Eigen::Vector3f m_cornerPosition;
    std::unordered_map<Eigen::Vector3i, Cell *, HashFunction> m_cells;
    std::vector<Particle *> m_particles;


    // Initialization Helpers

    std::string m_fluidMeshFilepath;
    std::string m_solidMeshFilepath;
    std::vector<Particle *> m_surfaceParticles; // this is used only temporarily for loading in
                                                // meshes; perhaps we could use the velocity field
                                                // to indicate the inward direction of the mesh?
    void meshToSurfaceParticles(const std::string meshFilepath);
    void updateGridFromSurfaceParticles(Material material);
    void fillGridFromInternalPosition(Material material, const Vector3f &internalPosition);

    // Simulation Helpers

    float m_kCFL = 2;                           // IDK lol
    float m_simulationTime;                     // total time for simulation
    Eigen::Vector3f m_gravityVector;            // acceleration vector due to gravity
    float m_interpolationCoefficient;           // for interpolating between PIC and FLIP
    void applyExternalForces(float deltaTime);
    void enforceDirichletBC();
    void classifyPseudoPressureGradient();
    void updateParticleVelocities();
    void updateParticlePositions();

    // Miscellaneous Helpers

    void assignParticleCellMaterials(Material material, std::vector<Particle *> &particles);
    const Eigen::Vector3i positionToIndices(const Eigen::Vector3f &position) const;
    bool withinBounds(const Eigen::Vector3i &cellIndices) const;
};

#endif // MACGRID_H
