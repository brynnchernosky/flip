#include "MacGrid.h"

#include <iostream>
#include <QSettings>
#include <QFile>
#include "../graphics/MeshLoader.h"



using namespace std;
using namespace Eigen;

// ================== Sanity Checks

#define SANITY_CHECKS false

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
  m_kCFL(2),
  m_cellWidth(cellWidth),
  m_cellCount(cellCount),
  m_cornerPosition     (cornerPosition),
  m_otherCornerPosition(cornerPosition + cellCount.cast<float>() * cellWidth),
  m_cellsPerLayer(cellCount[0] * cellCount[1]),
  m_cellsPerRow  (cellCount[0])
{
#if SANITY_CHECKS
  assert(0 < cellWidth);
  assert(0 < cellCount[0]);
  assert(0 < cellCount[1]);
  assert(0 < cellCount[2]);
#endif

  // Reading from ini file
  QSettings settings("src/config.ini", QSettings::IniFormat);
  m_simulationTime = settings.value(QString("simulationTime")).toInt();
  m_gravityVector  = Vector3f(0, settings.value(QString("gravity")).toFloat(), 0);
  m_interpolationCoefficient = settings.value(QString("interpolationCoefficient")).toFloat();
  m_fluidMesh = settings.value(QString("fluidMesh")).toString().toStdString();
  m_solidMesh = settings.value(QString("solidMesh")).toString().toStdString();

  convertFromMeshToParticles(Fluid, m_fluidMesh);
  convertFromMeshToParticles(Solid, m_solidMesh);
  simulate();
}

// ================== Destructor

// Todo
MacGrid::~MacGrid() {}

// ================== Validator

void MacGrid::validate()
{
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    assert(kv->second != nullptr);
  }
}

// ================== Simulator

void MacGrid::simulate()
{
  float time = 0.0f;
  while (time < m_simulationTime) {

    applyExternalForces();
    updateGrid();
    classifyAsFluidSolidAir();
    classifyPseudoPressureGradient();
    updateParticleVelocities();
    updateParticlePositions();

    // Increment time
    const float stepTime = 1.0f;
    time += stepTime;
  }
}

const Vector3i MacGrid::positionToIndices(const Vector3f position) const
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

  // const Vector3i cellCoordinates = positionToIndices(position);
  // Cell * const cell = getCell(cellCoordinates);

  // velocity[0] = getInterpolatedValue();
  // velocity[1] = getInterpolatedValue();
  // velocity[2] = getInterpolatedValue();
  
  return velocity;
}

// ================== Update Grid

const vector<Vector3i> NEIGHBOR_OFFSETS = {
  { 1,  0,  0},
  {-1,  0,  0},
  { 0,  1,  0},
  { 0, -1,  0},
  { 0,  0,  1},
  { 0,  0, -1},
};

// Given particle positions, update the dynamic grid
void MacGrid::updateGrid()
{
  // Set layer field of all cells to −1
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    kv->second->layer = -1;
  }

  // Update cells that currently have fluid in them
  for (Particle * particle : m_particles) {

    const Vector3i cellIndices = positionToIndices(particle->position);
    auto kv = m_cells.find(cellIndices);

    // If cell does not exist
    if (kv == m_cells.end()) {
      
      // If cell is within simulation bounds
      if (withinBounds(cellIndices)) {

        // Create the cell and put it in the hash table
        Cell * newCell = new Cell{};
        m_cells.insert({cellIndices, newCell});

        // Set its material and layer
        newCell->material = Material::Fluid;
        newCell->layer = 0;
      }

      continue;
    }

    // If cell does exist, and is not solid
    Cell * cell = kv->second;
    if (cell->material != Material::Solid) {
      cell->material = Material::Fluid;
      cell->layer = 0;
    }
  }

  // Create a buffer zone around the fluid
  for (int bufferLayer = 1; bufferLayer < max(2, (int) ceil(m_kCFL)); ++bufferLayer) {

    // For each cell
    for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
      const Vector3i cellIndices = kv->first;
      Cell * cell = kv->second;

      // Skip if it's not a fluid/air cell with layer = bufferLayer - 1
      if (cell->material == Material::Solid || cell->layer != bufferLayer - 1) continue;

      // For each of its six neighbor cells
      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
        const Vector3i neighborIndices = cellIndices + neighborOffset;
        auto kv = m_cells.find(cellIndices);

        // If the neighbor cell does not exist
        if (kv == m_cells.end()) {

          // Create the neighbor cell and put it in the hash table
          Cell * newNeighbor = new Cell{};
          m_cells.insert({neighborIndices, newNeighbor});
          
          // Set its material and layer
          newNeighbor->material = withinBounds(cellIndices) ? Material::Air : Material::Solid;
          newNeighbor->layer = bufferLayer;

          continue;
        }

        // If the neighbor cell does exist, is not solid, and has layer = -1
        Cell * neighbor = kv->second;
        if (neighbor->material != Material::Solid && neighbor->layer == -1) {
          neighbor->material = Material::Air;
          neighbor->layer = bufferLayer;
        }
      }
    }
  }
}

// ================== Helpers

void MacGrid::applyExternalForces()
{
#pragma omp parallel for
  for (int i = 0; i < m_particles.size(); i++) {
    Particle * particle = m_particles[i];
    particle->velocity += m_gravityVector*m_timestep;
  }
}

void MacGrid::checkForCollisions()
{
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
      if (m_cells[i->first + Eigen::Vector3i(1,0,0)]->material == Solid) {
          i->second->ux = std::min(0.f,i->second->ux);
      }
      if (m_cells[i->first + Eigen::Vector3i(-1,0,0)]->material == Solid) {
          i->second->ux = std::max(0.f,i->second->ux);
      }
      if (m_cells[i->first + Eigen::Vector3i(0,1,0)]->material == Solid) {
          i->second->uy = std::min(0.f,i->second->uy);
      }
      if (m_cells[i->first + Eigen::Vector3i(0,-1,0)]->material == Solid) {
          i->second->uy = std::max(0.f,i->second->uy);
      }
      if (m_cells[i->first + Eigen::Vector3i(0,0,1)]->material == Solid) {
          i->second->uz = std::min(0.f,i->second->uz);
      }
      if (m_cells[i->first + Eigen::Vector3i(0,0,-1)]->material == Solid) {
          i->second->uz = std::max(0.f,i->second->uz);
      }
  }
}

void MacGrid::classifyPseudoPressureGradient()
{
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

void MacGrid::updateParticleVelocities()
{
#pragma omp parallel for
  for (int i = 0; i < m_particles.size(); i++) {
    Particle * particle = m_particles[i];

    // Calculate FLIP particle velocity
    
    // Calculate PIC particle velocity
    // Update particle with interpolared PIC/FLIP velocities
  }
}

void MacGrid::updateParticlePositions()
{
#pragma omp parallel for
  for (int i = 0; i < m_particles.size(); i++) {
    Particle * particle = m_particles[i];

    // TO DO, use Runge Kutta 2 ODE solver
  }
}

void convertFromMeshToParticles(Material mat, std::string mesh) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3i> faces;
    std::vector<Cell> cells;
    if (MeshLoader::loadTriMesh(mesh,vertices,normals,faces)) {
        //make each vertex a particle
        //set a number of randomly selected points on each face to particles
    }

    //zack has helper functions for this
    for (int i = 0; i < cells.size(); i++) {
        std::vector<Cell> path;
        //step along x,y,z directions in turn, add cell to path
        //terminate path if exit mesh
        //if find voxel with same cell type, set cells on path to correct voxel type, add particle(s) to each cell
    }
}

bool MacGrid::withinBounds(Eigen::Vector3i cellIndices)
{
  // Todo
  return true;
}
