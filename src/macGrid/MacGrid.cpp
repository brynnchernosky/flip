#include <iostream>
#include <QSettings>
#include <QFile>

#include "src/macGrid/MacGrid.h"
#include "src/graphics/MeshLoader.h"
#include "src/Debug.h"
#include <random>
typedef Eigen::Triplet<float> T;

using namespace std;
using namespace Eigen;

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

// Todo: @brynn ideally, this should take in a config file filepath, and read 
//       everything from the file; is it possible to read vectors from the config file?
MacGrid::MacGrid(float cellWidth,
                 const Vector3i cellCount,
                 const Vector3f cornerPosition) :
  m_cellWidth(cellWidth),
  m_cellCount(cellCount),
  m_cornerPosition(cornerPosition)
{
#if SANITY_CHECKS
  assert(0 < cellWidth);
  assert(0 < cellCount[0]);
  assert(0 < cellCount[1]);
  assert(0 < cellCount[2]);
#endif

  // Read remaining fields from ini file
  QSettings settings("src/config.ini", QSettings::IniFormat);
  m_fluidMeshFilepath        = settings.value(QString("fluidMeshFilepath")).toString().toStdString();
  m_solidMeshFilepath        = settings.value(QString("solidMeshFilepath")).toString().toStdString();
  m_simulationTime           = settings.value(QString("simulationTime")).toInt();
  m_gravityVector            = Vector3f(0, settings.value(QString("gravity")).toFloat(), 0);
  m_interpolationCoefficient = settings.value(QString("interpolationCoefficient")).toFloat();
}

MacGrid::~MacGrid()
{
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) delete kv->second;
  for (Particle * particle : m_particles)                    delete particle;
  for (Particle * surfaceParticle : m_surfaceParticles)      delete surfaceParticle;
}

void MacGrid::validate()
{
  // Check that all particles and cells are defined
  for (Particle * const particle : m_particles)              assert(particle  != nullptr);
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) assert(kv->second != nullptr);

  // Check that every particle:
  for (Particle * const particle : m_particles) {
    const Vector3i cellIndices = positionToIndices(particle->position);
    const auto kv = m_cells.find(cellIndices);
    assert(kv != m_cells.end()); // is in a cell which exists,
    assert(kv->second == particle->cell); // is in the correct cell for its position, and
    assert(kv->second->particles.find(particle) != kv->second->particles.end()); // is accounted for by that cell
  }
}

void MacGrid::init()
{
  meshToSurfaceParticles(m_solidMeshFilepath);
  updateGridFromSurfaceParticles(Material::Solid, false);
  meshToSurfaceParticles(m_fluidMeshFilepath);
  updateGridFromSurfaceParticles(Material::Fluid, true);
}

void MacGrid::simulate()
{
  float time = 0.0f;
  while (time < m_simulationTime) {

    // Compute deltaTime or something
    const float deltaTime = 1.0f;

    applyExternalForces(deltaTime);
    updateGrid();
    classifyPseudoPressureGradient();
    updateParticleVelocities();
    updateParticlePositions();

    time += deltaTime;
  }
}

// Updates the dynamic grid, assuming particle positions are correct (todo: set cells' and particles' relationships)
const vector<Vector3i> NEIGHBOR_OFFSETS = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
void MacGrid::updateGrid()
{
  // Set layer field of all cells to −1
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    kv->second->layer = -1;
  }

  // Update grid cells that currently have fluid in them
  assignParticleCellMaterials(Material::Fluid, m_particles);

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
        auto kv = m_cells.find(neighborIndices);

        // If the neighbor cell does not exist
        if (kv == m_cells.end()) {

          // Create the neighbor cell and put it in the hash table
          Cell * newNeighbor = new Cell{};
          m_cells.insert({neighborIndices, newNeighbor});
          
          // Set its material and layer
          newNeighbor->material = withinBounds(neighborIndices) ? Material::Air : Material::Solid;
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

// ================== Debugging

// Debugging only: adds a particle to the system (does not set cells' and particles' relationships)
void MacGrid::addParticle(Vector3f position, Vector3f velocity)
{
  Particle * newParticle = new Particle{nullptr, position, velocity};
  m_particles.push_back(newParticle);
}

// Debugging only: prints the grid
void MacGrid::printGrid() const
{
  if (m_cells.size() == 0) {
    cout << "No cells in hashmap." << endl;
    return;
  }
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    cout << Debug::cellToString(kv->second) << endl;
  }
}

// ================== Initialization Helpers

void MacGrid::meshToSurfaceParticles(string meshFilepath)
{
    vector<Vector3f> vertices, normals;
    vector<Vector3i> faces;
    vector<Cell> cells;

    // Load mesh (panic if failed)
    if (!MeshLoader::loadTriMesh(meshFilepath, vertices, normals, faces)) {
      cout << "MacGrid::convertFromMeshToParticles() failed to load mesh. Exiting!" << endl;
      exit(1);
    }

    // Spawn particles on the surface of the mesh
    for (unsigned int i = 0; i < vertices.size(); ++i) {
        //create particle at vertices[i]
    }
    for (unsigned int i = 0; i < faces.size(); ++i) {
        int numParticlesPerFace = 5;
        for (unsigned int j = 0; j < numParticlesPerFace; j++) {
            float alpha = (static_cast<float>(random())/RAND_MAX);
            float beta = (static_cast<float>(random())/RAND_MAX);
            float gamma = (static_cast<float>(random())/RAND_MAX);
            float normalizationFactor = alpha+beta+gamma;
            alpha /= normalizationFactor;
            beta /= normalizationFactor;
            gamma /= normalizationFactor;
            //create particle at alpha*vertices[faces[i][0]] + beta*vertices[faces[i][1]] + gamma*vertices[faces[i][2]]
        }
    }

    for (unsigned int i = 0; i < m_cells.size(); ++i) {
        vector<Cell> path;
        //step along x,y,z directions in turn, add cell to path
        //terminate path if exit mesh
        //if find voxel with same cell type, set cells on path to correct voxel type, add particle(s) to each cell
    }
}

void MacGrid::updateGridFromSurfaceParticles(Material material, bool fillInnerSpace)
{
  // Update grid cells
  assignParticleCellMaterials(material, m_surfaceParticles);

  // Fill inner space
  if (!fillInnerSpace) return;
  assignInnerCellMaterials(material);
}

// ================== Simulation Helpers

void MacGrid::applyExternalForces(float deltaTime)
{
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    m_particles[i]->velocity += m_gravityVector * deltaTime;
  }
}

// Sets the fluid velocity into solid cells to zero 
void MacGrid::enforceDirichletBC()
{
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    Cell * cell = kv->second;

    // Skip air cells
    if (cell->material == Material::Air) continue;

    // Set solid cells' velocities to zero
    if (cell->material == Material::Solid) {
      cell->ux = 0;
      cell->uy = 0;
      cell->uz = 0;
    }

    // Set fluid-solid boundary velocities to zero (this is safe due to the buffer layer around the fluid)
    if (m_cells[kv->first + Vector3i(-1, 0, 0)]->material == Solid) kv->second->ux = 0;
    if (m_cells[kv->first + Vector3i(0, -1, 0)]->material == Solid) kv->second->uy = 0;
    if (m_cells[kv->first + Vector3i(0, 0, -1)]->material == Solid) kv->second->uz = 0;
  }
}

void MacGrid::classifyPseudoPressureGradient()
{
  Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,Lower|Upper,Eigen::IncompleteCholesky<float>> m_solver;

  Eigen::SparseMatrix<float> A; //coefficient matrix
  A.resize(m_cells.size(),m_cells.size());
  std::vector<T> coefficients;

  Eigen::Matrix3f b; //divergence of velocity field
  b.resize(m_cells.size(),1);

  int matrixIndexCounter = 0;
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
      if (i->second->material == Fluid) {
          i->second->index = matrixIndexCounter;
          matrixIndexCounter++;
      }
  }

  #pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
      if (i->second->material == Fluid) {
          float centerCoefficient = -6;
          if (m_cells[i->first + Vector3i(1, 0, 0)]->material == Solid) {
              centerCoefficient += 1;
          } else if (m_cells[i->first + Vector3i(1, 0, 0)]->material == Fluid) {
              coefficients.push_back(T(i->second->index,m_cells[i->first + Vector3i(1, 0, 0)]->index,1));
          }
          if (m_cells[i->first + Vector3i(-1, 0, 0)]->material == Solid) {
              centerCoefficient += 1;
          } else if (m_cells[i->first + Vector3i(-1, 0, 0)]->material == Fluid) {
              coefficients.push_back(T(i->second->index,m_cells[i->first + Vector3i(-1, 0, 0)]->index,1));

          }
          if (m_cells[i->first + Vector3i(0, 1, 0)]->material == Solid) {
              centerCoefficient += 1;
          } else if (m_cells[i->first + Vector3i(0, 1, 0)]->material == Fluid) {
              coefficients.push_back(T(i->second->index,m_cells[i->first + Vector3i(0, 1, 0)]->index,1));
          }
          if (m_cells[i->first + Vector3i(0, -1, 0)]->material == Solid) {
              centerCoefficient += 1;
          } else if (m_cells[i->first + Vector3i(0, -1, 0)]->material == Fluid) {
              coefficients.push_back(T(i->second->index,m_cells[i->first + Vector3i(0, -1, 0)]->index,1));
          }
          if (m_cells[i->first + Vector3i(0, 0, 1)]->material == Solid) {
              centerCoefficient += 1;
          } else if (m_cells[i->first + Vector3i(0, 0, 1)]->material == Fluid) {
              coefficients.push_back(T(i->second->index,m_cells[i->first + Vector3i(0, 0, 1)]->index,1));
          }
          if (m_cells[i->first + Vector3i(0, 0, -1)]->material == Solid) {
              centerCoefficient += 1;
          } else if (m_cells[i->first + Vector3i(0, 0, -1)]->material == Fluid) {
              coefficients.push_back(T(i->second->index,m_cells[i->first + Vector3i(0, 0, -1)]->index,1));
          }
          coefficients.push_back(T(i->second->index,i->second->index,centerCoefficient));
          //assume ux,uy,uz in negative direction
          //TO DO second paper mentions doing something different for air neighboring cells
          float divergence = ((i->second->ux)-(m_cells[i->first+Eigen::Vector3i(1,0,0)]->ux))/(m_cellWidth*m_cellWidth)
                  + ((i->second->uy)-(m_cells[i->first+Eigen::Vector3i(0,1,0)]->uy))/(m_cellWidth*m_cellWidth)
                  + ((i->second->uz)-(m_cells[i->first+Eigen::Vector3i(0,0,1)]->uz))/(m_cellWidth*m_cellWidth);
          b(matrixIndexCounter,0)= divergence;
      }
  }
  A.setFromTriplets(coefficients.begin(), coefficients.end());

  Eigen::Matrix3f scalarField;
  scalarField.resize(m_cells.size(),1);
  m_solver.compute(A);
  scalarField = m_solver.solve(b);

  #pragma omp parallel for
  matrixIndexCounter = 0;
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
      if (i->second->material == Fluid) {
          float xGradient = (scalarField[m_cells[i->first+Eigen::Vector3i(1,0,0)]->index]-scalarField[i->second->index])/(m_cellWidth*m_cellWidth);
          float yGradient = (scalarField[m_cells[i->first+Eigen::Vector3i(0,1,0)]->index]-scalarField[i->second->index])/(m_cellWidth*m_cellWidth);
          float zGradient = (scalarField[m_cells[i->first+Eigen::Vector3i(0,0,1)]->index]-scalarField[i->second->index])/(m_cellWidth*m_cellWidth);
          i->second->ux -= xGradient;
          i->second->uy -= yGradient;
          i->second->uz -= zGradient;
      }
      //TO DO second paper mentions doing something different for air neighboring cells
  }
}

void MacGrid::updateParticleVelocities()
{
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];
    Vector3f particlePos = particle->position;
    Vector3i gridIdx = positionToIndices(particle->position);
    Vector3i idx;
    idx[0] = floor(particlePos[0]);//l
    idx[1] = floor(particlePos[1]);//m
    idx[2] = floor(particlePos[2]);//n
    Vector3f weights = Vector3f(particlePos[0]+1-particlePos[0], particlePos[1]+1-particlePos[1], particlePos[2]+1-particlePos[2]);

    // Calculate FLIP particle velocity
//    Vector3i pariticlePosition = positionToIndices(particle->position);
    for(int l = 0; l < 2; l++){
        for(int m = 0; m < 2; m++){
            for(int n = 0; n < 2; n++){

            }
        }
    }
//    //#1
//    float wx = l+1-particlePos[0];
//    float wy = m+1-particlePos[1];
//    float wz = n+1-particlePos[2];
//    float vx = wx*m_cells[gridIdx]->ux;
//    float vy = wy*m_cells[gridIdx]->uy;
//    float vz = wz*m_cells[gridIdx]->uz;
//    //#2
//    Vector3i offset = Vector3i(1,0,0);
//    wx = particlePos[0]-l;
//    vx = vx + wx*m_cells[gridIdx+offset]->ux;
//    vy = vy + wy*m_cells[gridIdx+offset]->uy;
//    vz = vz + wz*m_cells[gridIdx+offset]->uz;
//    //#3
//    offset = Vector3i(0,1,0);
//    wx = l+1-particlePos[0];
//    wy = particlePos[1] - m;
//    vx = vx + wx*m_cells[gridIdx+offset]->ux;
//    vy = vy + wy*m_cells[gridIdx+offset]->uy;
//    vz = vz + wz*m_cells[gridIdx+offset]->uz;
//    //#4
//    offset = Vector3i(1,1,0);
//    wx = particlePos[0]-l;
//    vx = vx + wx*m_cells[gridIdx+offset]->ux;
//    vy = vy + wy*m_cells[gridIdx+offset]->uy;
//    vz = vz + wz*m_cells[gridIdx+offset]->uz;
//    //#5
//    offset = Vector3i(1,1,0);
//    wx = particlePos[0]-l;
//    vx = vx + wx*m_cells[gridIdx+offset]->ux;
//    vy = vy + wy*m_cells[gridIdx+offset]->uy;
//    vz = vz + wz*m_cells[gridIdx+offset]->uz;


    // Calculate PIC particle velocity

    // Update particle with interpolated PIC/FLIP velocities
  }
}

void MacGrid::updateParticlePositions()
{
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];

    // Runge-Kutta 2 ODE solver
  }
}

// ================== Miscellaneous Helpers

// Assigns the materials of cells which themselves contain particles,
void MacGrid::assignParticleCellMaterials(Material material, vector<Particle *> &particles)
{
  // Iterate through particles
  for (Particle * const particle : particles) {

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
        newCell->material = material;
        newCell->layer = 0;
      }

      continue;
    }

    // If cell does exist, and is not solid
    Cell * cell = kv->second;
    if (cell->material != Material::Solid) {
      cell->material = material;
      cell->layer = 0;
    }
  }
}

// Assigns the materials of cells contained within the surface particles, by using a fill method
void MacGrid::assignInnerCellMaterials(Material material)
{
  // Todo
}

// Converts a given position to the indices of the cell which would contain it
const Vector3i MacGrid::positionToIndices(const Vector3f &position) const
{
#if SANITY_CHECKS
  assertCellWithinBounds(position, m_cornerPosition, m_otherCornerPosition);
#endif

  const Vector3f regularizedPosition = (position - m_cornerPosition) / m_cellWidth;

  return Vector3i(floor(regularizedPosition[0]),
                  floor(regularizedPosition[1]),
                  floor(regularizedPosition[2]));
}

// Checks if a given cell falls within the simulation's grid bounds
bool MacGrid::withinBounds(const Vector3i &cellIndices) const
{
  return 0 <= cellIndices[0] && cellIndices[0] < m_cellCount[0] &&
         0 <= cellIndices[1] && cellIndices[1] < m_cellCount[1] &&
         0 <= cellIndices[2] && cellIndices[2] < m_cellCount[2];
}
