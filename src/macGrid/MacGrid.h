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

    MacGrid();
    ~MacGrid();

    void validate();
    void init();
    void simulate();
    void createBufferZone();

    // Debugging
    void setGridCellVelocity(Eigen::Vector3i cellIndices, Eigen::Vector3f velocity1, Eigen::Vector3f velocity2);
    void addParticle(Eigen::Vector3f position, Eigen::Vector3f velocity);
    void printGrid() const;

    // Current unit-testing target
    void classifyPseudoPressureGradient();

  private:

    float m_cellWidth;
    float m_maxAverageSurfaceParticlesPerCellFaceArea; // surfaceParticles.size() <= this * surfaceArea
    float m_maxAverageSurfaceParticlesPerArea;
    Eigen::Vector3i m_cellCount;
    Eigen::Vector3f m_cornerPosition;
    std::unordered_map<Eigen::Vector3i, Cell *, HashFunction> m_cells;
    std::vector<Particle *> m_particles;

    // Initialization Helpers

    std::string             m_solidMeshFilepath;
    std::vector<Particle *> m_solidSurfaceParticles;

    std::string             m_fluidMeshFilepath;
    std::vector<Particle *> m_fluidSurfaceParticles;
    Eigen::Vector3f         m_fluidInternalPosition;

    void meshToSurfaceParticles(std::vector<Particle *> &surfaceParticles, std::string meshFilepath);
    void fillGridCellsFromInternalPosition(Material material, const Eigen::Vector3f &internalPosition);
    void fillGridCellsRecursive           (Material material, int layerNumber, const Eigen::Vector3i &cellPosition);
    void addParticlesToCells(Material material);

    // Simulation Helpers

    float m_kCFL = 2;                           // IDK lol
    float m_simulationTime;                     // total time for simulation
    Eigen::Vector3f m_gravityVector;            // acceleration vector due to gravity
    float m_interpolationCoefficient;           // for interpolating between PIC and FLIP
    void applyExternalForces(float deltaTime);
    void enforceDirichletBC();
    void updateParticleVelocities();
    void updateParticlePositions(float deltaTime);
    float calculateDeltaTime();

    // Miscellaneous Helpers

    void assignParticleCellMaterials(Material material, std::vector<Particle *> &particles);

    const Eigen::Vector3i positionToIndices(const Eigen::Vector3f &position) const;

    bool withinBounds(const Eigen::Vector3i &cellIndices) const;
};

#endif // MACGRID_H
