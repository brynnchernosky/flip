#ifndef MACGRID_H
#define MACGRID_H

#include <Eigen/Dense>
#include <vector>

#include "Cell.h"
#include "Face.h"

class MacGrid
{
  public:

    MacGrid(float cellWidth, const Eigen::Vector3i cellCount, const Eigen::Vector3f cornerPosition);
    ~MacGrid();

    void validate();

    const Eigen::Vector3i getCellCoordinates(const Eigen::Vector3f position) const;

    const Eigen::Vector3f getInterpolatedVelocity(const Eigen::Vector3f position) const;

    Cell *getCell(const Eigen::Vector3i cellCoordinates) const;
    Cell *getCell(const Eigen::Vector3f position)        const;

  private:
  
    float           m_cellWidth;
    Eigen::Vector3i m_cellCount;
    Eigen::Vector3f m_cornerPosition;
    
    int m_cellsPerLayer;
    int m_cellsPerRow;
    Eigen::Vector3f m_otherCornerPosition;

    std::vector<Cell *> m_cells;
    std::vector<Particle *> m_particles;

    unsigned int    coordinateToIndex(Eigen::Vector3i cellCoordinates) const;
    Eigen::Vector3i indexToCoordinate(unsigned int    cellIndex)       const;
    void simulate();
    void applyExternalForces();
    void checkForCollisions();
    void classifyAsFluidSolidAir();
    void classifyPseudoPressureGradient();
    void updateParticleVelocities();
    void updateParticlePositions();

};

#endif // MACGRID_H
