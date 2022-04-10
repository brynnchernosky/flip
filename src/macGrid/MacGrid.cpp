#include "MacGrid.h"

#include <iostream>
#include <QSettings>
#include <QFile>

using namespace std;
using namespace Eigen;

// ================== Sanity Checks

#define SANITY_CHECKS true

#if SANITY_CHECKS

inline void assertCellWithinBounds(const Vector3i cellCoordinates, const Vector3i cellCount)
{
  assert(0 <= cellCoordinates[0] && cellCoordinates[0] < cellCount[0]);
  assert(0 <= cellCoordinates[1] && cellCoordinates[1] < cellCount[1]);
  assert(0 <= cellCoordinates[2] && cellCoordinates[2] < cellCount[2]);
}


inline void assertCellWithinBounds(const Vector3f position, const Vector3f corner, const Vector3f otherCorner)
{
  assert(corner[0] <= position[0] && position[0] < otherCorner[0]);
  assert(corner[1] <= position[1] && position[1] < otherCorner[1]);
  assert(corner[2] <= position[2] && position[2] < otherCorner[2]);
}

#endif

// ================== Constructor

// Todo
MacGrid::MacGrid(float cellWidth,
                 Vector3i cellCount,
                 Vector3f cornerPosition) :
  m_cellWidth(cellWidth),
  m_cellCount(cellCount),
  m_cornerPosition(cornerPosition),
  m_cellsPerLayer(cellCount[0] * cellCount[1]),
  m_cellsPerRow(cellCount[0]),
  m_otherCornerPosition(cornerPosition + cellCount.cast<float>() * cellWidth)
{
#if SANITY_CHECKS
  assert(0 < cellWidth);
  assert(0 < cellCount[0]);
  assert(0 < cellCount[1]);
  assert(0 < cellCount[2]);
#endif
}

// ================== Destructor

// Todo
MacGrid::~MacGrid() {}

// ================== Validator

// Todo
void MacGrid::validate()
{
  for (Cell * const cell : m_cells) {
    assert(cell != nullptr);
  }
}

const Vector3i MacGrid::getCellCoordinates(const Vector3f position) const
{
#if SANITY_CHECKS
  assertCellWithinBounds(position, m_cornerPosition, m_otherCornerPosition);
#endif

  const Vector3f regularizedPosition = (position - m_cornerPosition) / m_cellWidth;

  return Vector3i(floor(regularizedPosition[0]),
                  floor(regularizedPosition[1]),
                  floor(regularizedPosition[2]));
}

// Todo
const Vector3f MacGrid::getInterpolatedVelocity(const Vector3f position) const
{
#if SANITY_CHECKS
  assertCellWithinBounds(position, m_cornerPosition, m_otherCornerPosition);
#endif

  Vector3f velocity;

  // const Vector3i cellCoordinates = getCellCoordinates(position);
  // Cell * const cell = getCell(cellCoordinates);

  // velocity[0] = getInterpolatedValue();
  // velocity[1] = getInterpolatedValue();
  // velocity[2] = getInterpolatedValue();
  
  return velocity;
}

Cell *MacGrid::getCell(const Vector3i cellCoordinates) const
{
#if SANITY_CHECKS
  assertCellWithinBounds(cellCoordinates, m_cellCount);
#endif

  return m_cells[coordinateToIndex(cellCoordinates)];
}

Cell *MacGrid::getCell(const Vector3f position) const
{
#if SANITY_CHECKS
  assertCellWithinBounds(position, m_cornerPosition, m_otherCornerPosition);
#endif

  return getCell(getCellCoordinates(position));
}

// ================== Helpers

unsigned int MacGrid::coordinateToIndex(const Vector3i cellCoordinates) const
{
#if SANITY_CHECKS
  assertCellWithinBounds(cellCoordinates, m_cellCount);
#endif

  return cellCoordinates[2] * m_cellsPerLayer +
         cellCoordinates[1] * m_cellsPerRow +
         cellCoordinates[0];
}

Vector3i MacGrid::indexToCoordinate(unsigned int cellIndex) const
{
#if SANITY_CHECKS
  assert(0 <= cellIndex && cellIndex < m_cells.size());
#endif

  const int z = cellIndex % m_cellsPerLayer;
  const int temp = cellIndex - z * m_cellsPerLayer;
  const int y = temp % m_cellsPerRow;
  const int x = (temp - y * m_cellsPerRow);
  return Vector3i(x, y, z);
}

void MacGrid::simulate() {
    QSettings settings("src/config.ini", QSettings::IniFormat);
    int numTimesteps = settings.value(QString("numTimesteps")).toInt();
    for (int i = 0; i < numTimesteps; i++) {
        applyExternalForces();
        checkForCollisions();
        classifyAsFluidSolidAir();
        classifyPseudoPressureGradient();
        updateParticleVelocities();
        //CFL condition?
        updateParticlePositions();
    }
}

void MacGrid::applyExternalForces() {
    #pragma omp parallel for
    for (int i = 0; i < m_particles.size(); i++) {
        QSettings settings("src/config.ini", QSettings::IniFormat);
        Eigen::Vector3f gravity = Eigen::Vector3d(0,settings.value(QString("gravity")).toDouble(),0);
        //TO DO, add force of gravity to each particle
    }
}

void MacGrid::checkForCollisions() {
    #pragma omp parallel for
    //TO DO
    //iterate through all cells
        //for each direction, check if neighboring cell is solid
        //if neighboring solid cell, set velocity in that direction to 0
}

void MacGrid::classifyAsFluidSolidAir() {
    #pragma omp parallel for
    //TO DO
    //iterate through all cells
        //if non solid
            //if one or more particles in cell set to fluid
            //else set to air
}

void MacGrid::classifyPseudoPressureGradient() {
    //TO DO
    //define solver, recommended preconditioned conjugate gradient method with a preconditioner of the modified incomplete cholesky factorization type
    //sparse matrix A = divergence of velocity field using equation 7
    //sparse matrix b =
    // for every neighboring cell
        //if solid neighboring cell, solid coefficient 0, increase cental coefficient by 1
        //else, use equation 8 to get coefficient
    //solve Ax=b to get scalar vield
    //subtract scalar field from velocities to get divergence free velocity and enforce conservation fo mass
}

void MacGrid::updateParticleVelocities() {
    //TO DO
    for (int i = 0; i < m_particles.size(); i++) {
         //calculate FLIP particle velocity
         //calculate PIC particle velocity
        QSettings settings("src/config.ini", QSettings::IniFormat);
        double interpolationCoefficient = settings.value(QString("interpolationCoefficient")).toDouble();
        //update particle with interpolared PIC/FLIP velocities
    }
}

void MacGrid::updateParticlePositions() {
    for (int i = 0; i < m_particles.size(); i++) {
        //TO DO, use Runge Kutta 2 ODE solver
    }
}
