#ifndef MACGRID_H
#define MACGRID_H

#include <Eigen/Dense>
#include <vector>

#include "HashMap.h"
#include "Cell.h"
#include "Particle.h"

class MacGrid
{
  public:

    MacGrid(float cellWidth, const Eigen::Vector3i cellCount, const Eigen::Vector3f cornerPosition);
    ~MacGrid();

    void validate();

    void simulate();

    const Eigen::Vector3i positionToIndices(const Eigen::Vector3f position) const;

    const Eigen::Vector3f getInterpolatedVelocity(const Eigen::Vector3f position) const;

    void updateGrid();

  private:

    float m_kCFL = 2;
    float m_cellWidth;

    Eigen::Vector3i m_cellCount;
    Eigen::Vector3f m_cornerPosition;
    Eigen::Vector3f m_otherCornerPosition;
    
    int m_cellsPerLayer;
    int m_cellsPerRow;

    std::unordered_map<Eigen::Vector3i, Cell *, HashFunction> m_cells;

    std::vector<Particle *> m_particles;

    float m_simulationTime; //total time to run simulation
    float m_timestep; //current timestep
    Eigen::Vector3f m_gravityVector;
    float m_interpolationCoefficient;

    std::string m_fluidMesh;
    std::string m_solidMesh;

    // ================== Helpers

    void applyExternalForces();
    void checkForCollisions();
    void classifyAsFluidSolidAir();
    void classifyPseudoPressureGradient();
    void updateParticleVelocities();
    void updateParticlePositions();
    void convertFromMeshToParticles(Material mat, std::string mesh);

    bool withinBounds(Eigen::Vector3i cellIndices);
};

#endif // MACGRID_H
