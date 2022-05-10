#ifndef MACGRID_H
#define MACGRID_H

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <QtConcurrent>
#include <QSettings>
#include <math.h>
#include <vector>

#include "HashMap.h"
#include "Cell.h"
#include "Particle.h"
#include "Material.h"

class MacGrid
{
  public:

    MacGrid(std::string folder);
    ~MacGrid();

    void init();

    void simulate();

    void setCellAndParticleRelationships();

    void createBufferZone();

    // For Debugging
    void printGrid()      const;
    void printParticles() const;

  private:

    std::string m_outputFolder;
    float m_cellWidth;
    int   m_strata;
    float m_maxAverageSurfaceParticlesPerCellFaceArea; // surfaceParticles.size() <= this * surfaceArea
    float m_maxAverageSurfaceParticlesPerArea;
    Eigen::Vector3i m_cellCount;
    Eigen::Vector3f m_cornerPosition;
    Eigen::Vector3f m_otherCornerPosition;
    std::unordered_map<Eigen::Vector3i, Cell *, HashFunction> m_cells;
    std::vector<Particle *> m_particles;

    // ================== Initialization Helpers

    std::string m_solidMeshFilepath;
    std::string m_fluidMeshFilepath;
    std::vector<Particle *> m_solidSurfaceParticles;
    std::vector<Particle *> m_fluidSurfaceParticles;
    Eigen::Vector3f m_solidInternalPosition;
    Eigen::Vector3f m_fluidInternalPosition;
    Eigen::Affine3f m_solidTransformation;
    Eigen::Affine3f m_fluidTransformation;

    void getSurfaceParticlesFromMesh(std::vector<Particle *> &surfaceParticles, std::string meshFilepath, const Eigen::Affine3f &transformation);

    void fillCellsFromInternalPosition(const Material material, const Eigen::Vector3f &internalPosition);

    void propagateSolidNormals();
    
    void spawnParticlesInFluidCells();

    // ================== Simulation Helpers

    float m_framePeriod;              // time (in seconds) in between frames
    float m_minCFLTime;               // the minimum timestep that calculateCFLTime() can return
    float m_maxCFLTime;               // the maximum timestep that calculateCFLTime() can return
    float m_simulationTime;           // total time for simulation

    int m_fluidAddCounter = 0;        // number of times fluid has been added to simulation
    float m_fluidVelocityZ = -0.1;    // z velocity of fluid being added

    Eigen::Vector3f m_gravityVector;  // acceleration vector due to gravity
    float m_interpolationCoefficient; // for interpolating between PIC and FLIP
    float m_foamParticleBoundary;     // for adding foam particles

    // Calculate the timestep that can be taken while obeying the CFL condition
    float calculateCFLTime() const;

    // Update the dynamic grid
    void updateGridExcludingVelocity();

    // Update the velocity field (grid cell velocities)
    void updateGridVelocity();
    void contributeToCells(const Eigen::Vector3f &velocity, const Eigen::Vector3f &xyz, int index) const;

    // Save a copy of the velocity field for FLIP calculations
    void saveCopyOfGridVelocity();

    // Apply external forces to the velocity field
    void applyExternalForces(const float deltaTime);

    // Enforce the Neumann boundary condition to prevent flow from air/fluid cells into solid cells
    void enforceBoundaryConditions();

    // Solve for pressure and remove divergence from the velocity field
    void updateGridVelocityByRemovingDivergence();

    // Update particle positions with RK2
    void updateParticlePositions(float deltaTime, const std::vector<Particle *> &particles);
    void updateParticleVelocities(const std::vector<Particle *> &particles);
    std::pair<float, float> getInterpolatedPICAndFLIP(const Eigen::Vector3f &xyz, const int index) const;

    // Helpers to resolve particle collisions
    void resolveParticleCollisions(const std::vector<Particle *> &particles);
    void resolveParticleOutOfBoundsHelper(Particle * particle, const int index);
    void resolveParticleInSolidHelper(Particle * particle, Cell * const cell);

    // ================== Common (Initialization and Simulation) Helpers

    void setCellsBasedOnParticles(const Material material, const std::vector<Particle *> &particles, const bool preventBadPositions);

    void setCellLayerBasedOnMaterial();

    // ================== Positional Helpers

    const Eigen::Vector3f toRegularizedPosition  (const Eigen::Vector3f &position)    const;
    const Eigen::Vector3i positionToIndices      (const Eigen::Vector3f &position)    const;
    const Eigen::Vector3f indicesToBasePosition  (const Eigen::Vector3i &cellIndices) const;
    const Eigen::Vector3f indicesToCenterPosition(const Eigen::Vector3i &cellIndices) const;

    // ================== Miscellaneous Helpers

    QFuture<void> saveParticlesToFile(const float time, bool saveParticle) const;

    bool withinBounds(const Eigen::Vector3i &cellIndices) const;

    // ================== Extension Helpers

    std::vector<Particle *> addParticlesToCell(int x, int y, int z);
    void addFluid(int x, int y, int z, int sideLength, const float time);
    void removeFluid(int x, int y, int z, int sideLength);
    void addFoamParticles();
};

#endif // MACGRID_H
