#ifndef MACGRID_H
#define MACGRID_H

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <vector>

#include <QSettings>

#include "HashMap.h"
#include "Cell.h"
#include "Particle.h"
#include "Material.h"

class MacGrid
{
  public:

    MacGrid(std::string folder);
    ~MacGrid();

    void validate();
    void init();
    void simulate();
    void setCellAndParticleRelationships();
    void createBufferZone();

    // Debugging
    void setGridCellVelocity(const Eigen::Vector3i cellIndices, const Eigen::Vector3f velocity1, const Eigen::Vector3f velocity2);
    void printParticles(std::string output_filepath);
    void addParticle(const Eigen::Vector3f position, const Eigen::Vector3f velocity);
    void printGrid() const;

    // Current unit-testing target
    void updateVelocityFieldByRemovingDivergence();

  private:

    std::string m_outputFolder;

    float m_cellWidth;
    int   m_strata;
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

    void fillGridCellsFromInternalPosition(const Material material, const Eigen::Vector3f &internalPosition);
    void fillGridCellsRecursive           (const Material material, const int layerNumber, const Eigen::Vector3i &cellPosition);

    void addParticlesToCells              (const Material material);

    // Simulation Helpers

    float m_kCFL = 2;                           // IDK lol
    float m_simulationTime;                     // total time for simulation
    Eigen::Vector3f m_gravityVector;            // acceleration vector due to gravity
    float m_interpolationCoefficient;           // for interpolating between PIC and FLIP

    float calculateDeltaTime();
    void  applyExternalForces(const float deltaTime);
    void  enforceDirichletBC();
    void  transferParticlesToGrid();
    void  updateParticleVelocities();
    std::pair<float, float> getInterpolatedPICAndFLIP(const Eigen::Vector3f &xyz, const int index) const;
    void updateParticlePositions(float deltaTime);
    void resolveParticlePenetratingSolid();

    // Positional Helpers

    const Eigen::Vector3f toRegularizedPosition  (const Eigen::Vector3f &position)    const;
    const Eigen::Vector3i positionToIndices      (const Eigen::Vector3f &position)    const;
    const Eigen::Vector3f indicesToBasePosition  (const Eigen::Vector3i &cellIndices) const;
    const Eigen::Vector3f indicesToCenterPosition(const Eigen::Vector3i &cellIndices) const;

    // Miscellaneous Helpers

    void assignParticleCellMaterials(const Material material, const std::vector<Particle *> &particles);
    void setCellMaterialLayers(const Material material, const int layer);

    bool withinBounds(const Eigen::Vector3i &cellIndices) const;
};

#endif // MACGRID_H
