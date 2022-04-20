#include <iostream>
#include <QSettings>
#include <QFile>
#include <random>

#include "src/macGrid/MacGrid.h"
#include "src/graphics/MeshLoader.h"
#include "src/Debug.h"

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
MacGrid::MacGrid()
{
#if SANITY_CHECKS
  assert(0 < cellWidth);
  assert(0 < cellCount[0]);
  assert(0 < cellCount[1]);
  assert(0 < cellCount[2]);
#endif

  QSettings settings("src/config.ini", QSettings::IniFormat);

  m_cellWidth = settings.value(QString("cellWidth")).toFloat();
  m_maxAverageSurfaceParticlesPerCellFaceArea = settings.value(QString("maxAverageSurfaceParticlesPerCellFaceArea")).toFloat();
  m_maxAverageSurfaceParticlesPerArea = m_maxAverageSurfaceParticlesPerCellFaceArea / m_cellWidth / m_cellWidth;
  // cout << "m_maxAverageSurfaceParticlesPerArea = " << m_maxAverageSurfaceParticlesPerArea << endl;

  m_cellCount = Vector3i(settings.value(QString("cellCountX")).toInt(),
                         settings.value(QString("cellCountY")).toInt(),
                         settings.value(QString("cellCountZ")).toInt());

  m_cornerPosition = Vector3f(settings.value(QString("cornerPositionX")).toFloat(),
                              settings.value(QString("cornerPositionY")).toFloat(),
                              settings.value(QString("cornerPositionZ")).toFloat());

  m_solidMeshFilepath = settings.value(QString("solidMeshFilepath")).toString().toStdString();
  m_fluidMeshFilepath = settings.value(QString("fluidMeshFilepath")).toString().toStdString();
  m_fluidInternalPosition = Vector3f(settings.value(QString("fluidInternalPositionX")).toFloat(),
                                     settings.value(QString("fluidInternalPositionY")).toFloat(),
                                     settings.value(QString("fluidInternalPositionZ")).toFloat());

  m_simulationTime = settings.value(QString("simulationTime")).toInt();

  m_gravityVector = Vector3f(settings.value(QString("gravityX")).toFloat(),
                             settings.value(QString("gravityY")).toFloat(),
                             settings.value(QString("gravityZ")).toFloat());

  m_interpolationCoefficient = settings.value(QString("interpolationCoefficient")).toFloat();
}

MacGrid::~MacGrid()
{
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) delete kv->second;
  for (Particle * particle : m_particles)                    delete particle;
  for (Particle * particle : m_solidSurfaceParticles)        delete particle;
  for (Particle * particle : m_fluidSurfaceParticles)        delete particle;
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
  // Solid
  // meshToSurfaceParticles(m_solidSurfaceParticles, m_solidMeshFilepath);
  // assignParticleCellMaterials(Material::Solid, m_solidSurfaceParticles);

  // Fluid
  meshToSurfaceParticles(m_fluidSurfaceParticles, m_fluidMeshFilepath);
  assignParticleCellMaterials(Material::Fluid, m_fluidSurfaceParticles);
  fillGridCellsFromInternalPosition(Material::Fluid, m_fluidInternalPosition);
  // addParticlesToCells(Material::Fluid);
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
    cout << Debug::vectorToString(kv->first) << ": " << Debug::cellToString(kv->second) << endl;
  }
  cout << "Total cells = " << m_cells.size() << endl;
}

// ================== Initialization Helpers

inline float getRandomFloat()
{
  return static_cast <float> (arc4random()) / static_cast <float> (UINT32_MAX);
}

const Vector3f getRandomPositionOnTriangle(const Vector3f &a, const Vector3f &ab, const Vector3f &ac)
{
  float R = getRandomFloat();
  float S = getRandomFloat();
  if (R + S > 1) {
    R = 1 - R;
    S = 1 - S;
  }
  return a + R * ab + S * ac;
}

// Fills the given vector with particles derived from the surface of the given mesh
void MacGrid::meshToSurfaceParticles(vector<Particle *> &surfaceParticles, string meshFilepath)
{
  vector<Vector3f> vertices, normals;
  vector<Vector3i> faces;
  vector<Cell> cells;

  // Load mesh (panic if failed)
  if (!MeshLoader::loadTriMesh(meshFilepath, vertices, normals, faces)) {
    cout << "MacGrid::convertFromMeshToParticles() failed to load mesh. Exiting!" << endl;
    exit(1);
  }

  // Spawn particles on the mesh's vertices
  for (const Vector3f &vertexPosition : vertices) {
    surfaceParticles.push_back(new Particle{nullptr, vertexPosition, Vector3f::Zero()});
  }

  float area = 0;
  
  // Spawn particles on the mesh's faces
  for (const Vector3i &face : faces) {

    const Vector3f &a = vertices[face[0]];
    const Vector3f ab = vertices[face[1]] - a;
    const Vector3f ac = vertices[face[2]] - a;
    const float abNorm = ab.norm();
    const float acNorm = ac.norm();

    const float faceArea = ab.cross(ac).norm() / 2;
    area += faceArea;

    const float numParticles = faceArea * m_maxAverageSurfaceParticlesPerArea;
    const float acAbRatio = acNorm / abNorm;
    const float temp = sqrt(numParticles / acAbRatio);
    const int abStrata = (int) (temp);
    const int acStrata = (int) (acAbRatio * temp);

    // Use stratified sampling
    for (int abStratum = 0; abStratum < abStrata; ++abStratum) {
      for (int acStratum = 0; acStratum < acStrata; ++acStratum) {

        float abWeight = (abStratum + getRandomFloat()) / abStrata;
        float acWeight = (acStratum + getRandomFloat()) / acStrata;
        if (abWeight + acWeight > 1) {
          abWeight = 1 - abWeight;
          acWeight = 1 - acWeight;
        }
        const Vector3f surfacePosition = a + abWeight * ab + acWeight * ac;
        
        surfaceParticles.push_back(new Particle{nullptr, surfacePosition, Vector3f::Zero()});
      }
    }
  }

  cout << "surface particles (not counting vertices)  = " << surfaceParticles.size() - vertices.size() << endl;
  cout << "surface area                               = " << area << endl;
  cout << "surface area (multiples of cell face area) = " << area / m_cellWidth / m_cellWidth << endl;
}

// Densely fills grid cells with the given material, starting from a given internal position
void MacGrid::fillGridCellsFromInternalPosition(Material material, const Eigen::Vector3f &internalPosition)
{
  const Vector3i cellIndices = positionToIndices(internalPosition);
  const int layerNumber = material == Material::Fluid ? 0 : 100;

#if SANITY_CHECKS
  assert(withinBounds(cellIndices));
#endif

  // Check that it doesn't already exist
  if (m_cells.find(cellIndices) != m_cells.end()) {
    assert(m_cells.find(cellIndices)->second->material == Material::Fluid);
    return;
  }

  // Create a cell here
  Cell * newCell = new Cell{};
  newCell->material = material;
  newCell->layer = layerNumber;
  m_cells.insert({cellIndices, newCell});

  // Recursively fill neighbors
  fillGridCellsRecursive(material, layerNumber, cellIndices);
}

// Recursive helper function for the above
void MacGrid::fillGridCellsRecursive(Material material, int layerNumber, const Eigen::Vector3i &cellIndices) {

  // For each neighbor
  for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
    const Vector3i neighborIndices = cellIndices + neighborOffset;

    // Move on if it already exists or is outside the bounds
    if (m_cells.find(neighborIndices) != m_cells.end() || !withinBounds(neighborIndices)) continue;

    // Create a cell here
    Cell * newCell = new Cell{};
    newCell->material = material;
    newCell->layer = layerNumber;
    m_cells.insert({neighborIndices, newCell});

    // Recursively fill neighbors
    fillGridCellsRecursive(material, layerNumber, neighborIndices);
  }
}

// Adds particles to the system where cells of the given material are located, using stratified sampling (does not affect cell/particle relationship)
void MacGrid::addParticlesToCells(Material material) {
  int strata = 3; //number of subdivisions per side, total number of subcells is strata**3
  int samplesPerStrata = 1; //number of particles per subcell
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    if (i->second->material == material) {
      for (int x = 0; x < strata; x++) {
        for (int y = 0; y < strata; y++) {
          for (int z = 0; z < strata; z++) {
            float alpha = (static_cast<float>(random())/RAND_MAX);
            float beta = (static_cast<float>(random())/RAND_MAX);
            float gamma = (static_cast<float>(random())/RAND_MAX);
            Vector3f position(m_cellWidth*i->first[0],m_cellWidth*i->first[1],m_cellWidth*i->first[2]);
            position += Vector3f(x*m_cellWidth/strata,y*m_cellWidth/strata,z*m_cellWidth/strata);
            position += Vector3f(alpha*m_cellWidth/strata,beta*m_cellWidth/strata,gamma*m_cellWidth/strata);
            Particle * newParticle = new Particle{nullptr, position, Vector3f(0,0,0)};
            m_particles.push_back(newParticle);
          }
        }
      }
    }
  }
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
    //debug solver setup
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
      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
        if (m_cells[i->first + neighborOffset]->material == Solid) {
          centerCoefficient += 1;
        } else if (m_cells[i->first + neighborOffset]->material == Fluid) {
          coefficients.push_back(T(i->second->index,m_cells[i->first + neighborOffset]->index,1));
        }
      }
      coefficients.push_back(T(i->second->index,i->second->index,centerCoefficient));
      //assume ux,uy,uz in negative direction
      float divergence = ((i->second->ux)-(m_cells[i->first+Eigen::Vector3i(1,0,0)]->ux))/(m_cellWidth*m_cellWidth)
          + ((i->second->uy)-(m_cells[i->first+Eigen::Vector3i(0,1,0)]->uy))/(m_cellWidth*m_cellWidth)
          + ((i->second->uz)-(m_cells[i->first+Eigen::Vector3i(0,0,1)]->uz))/(m_cellWidth*m_cellWidth);
      //second paper subtracts number of air cells, first paper does not
      b(matrixIndexCounter,0)= divergence;
    }
  }
  A.setFromTriplets(coefficients.begin(), coefficients.end());

  Eigen::Matrix3f scalarField;
  scalarField.resize(m_cells.size(),1);
  //m_solver.compute(A);
  scalarField = m_solver.solve(b);

#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    if (i->second->material == Fluid) {
      //          float xGradient = (scalarField[m_cells[i->first+Eigen::Vector3i(1,0,0)]->index]-scalarField[i->second->index])/(m_cellWidth*m_cellWidth);
      //          float yGradient = (scalarField[m_cells[i->first+Eigen::Vector3i(0,1,0)]->index]-scalarField[i->second->index])/(m_cellWidth*m_cellWidth);
      //          float zGradient = (scalarField[m_cells[i->first+Eigen::Vector3i(0,0,1)]->index]-scalarField[i->second->index])/(m_cellWidth*m_cellWidth);
      //          i->second->ux -= xGradient;
      //          i->second->uy -= yGradient;
      //          i->second->uz -= zGradient;
    }
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

    Vector3f weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    float picx, picy, picz = 0;
    float flipx, flipy, flipz = 0;
    for(int l = 0; l < 2; l++){
        for(int m = 0; m < 2; m++){
            for(int n = 0; n < 2; n++){
                if (l == 1){weights[0] = particlePos[0] - idx[0];}
                if (m == 1){weights[1] = particlePos[1] - idx[1];}
                if (n == 1){weights[2] = particlePos[2] - idx[2];}
                Vector3i offset = Vector3i(l, m, n);
                // Calculate PIC particle velocity
                picx = picx + weights[0]*m_cells[gridIdx+offset]->ux;
                picy = picy + weights[1]*m_cells[gridIdx+offset]->uy;
                picz = picz + weights[2]*m_cells[gridIdx+offset]->uz;
                // Calculate FLIP particle velocity
                flipx = flipx + weights[0]*(m_cells[gridIdx+offset]->ux - particle->velocity[0]);
                flipy = flipy + weights[1]*(m_cells[gridIdx+offset]->ux - particle->velocity[1]);
                flipz = flipz + weights[2]*(m_cells[gridIdx+offset]->ux - particle->velocity[2]);
            }
        }
    }



    Vector3f pic = Vector3f(picx, picy, picz);
    Vector3f flip = Vector3f(flipx, flipy, flipz);

    // Update particle with interpolated PIC/FLIP velocities
    particle->velocity = m_interpolationCoefficient*pic + (1-m_interpolationCoefficient)*(particle->velocity+flip);
  }
}

void MacGrid::updateParticlePositions()
{
#pragma omp parallel for
    // Runge-Kutta 2 ODE solver
    float timestep;
    float maxV = 0;
    for (unsigned int i = 0; i < m_particles.size(); ++i) {
        Particle * particle = m_particles[i];
        if(m_particles[i]->velocity.norm()>=maxV){maxV = m_particles[i]->velocity.norm();}
    }
    timestep = m_cellWidth/maxV;
    std::vector<Vector3f> oldPos;
    updateParticleVelocities();
    for (unsigned int i = 0; i < m_particles.size(); ++i) {
        Particle * particle = m_particles[i];
        oldPos.push_back(particle->position);
        particle->position = particle->position + particle->velocity*timestep*0.5;
        //TODO: fix problem of particle penetrating the solid cell
        Vector3i gridIdx = positionToIndices(particle->position);
        if(m_cells[gridIdx]->material == Material::Solid){
            Vector3i oldIdx = positionToIndices(oldPos[i]);
            Vector3i movement = oldIdx-gridIdx;
            Vector3i offset = Vector3i(0,0,0);
            for(int j = 0; j<3; j++){
                offset[j] = movement[j];
                if(movement[j] == 0 && m_cells[gridIdx + offset]->material!=Material::Fluid){
                    offset[j] = 0;
                }
            }
            particle->position = Vector3f(gridIdx[0] + offset[0] +0.5,gridIdx[1] + offset[1] + 0.5,gridIdx[2] + offset[2] +0.5)*m_cellWidth+m_cornerPosition;
        }
    }
    updateGrid();
    updateParticleVelocities();
    for (unsigned int i = 0; i < m_particles.size(); ++i) {
        Particle * particle = m_particles[i];
        particle->position = oldPos[i] + particle->velocity*timestep;
    }
    m_simulationTime = m_simulationTime + timestep;
}

// ================== Miscellaneous Helpers

// Assigns the materials of cells which themselves contain particles,
void MacGrid::assignParticleCellMaterials(Material material, vector<Particle *> &particles)
{
  const int layerNumber = material == Material::Fluid ? 0 : 100;

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
        newCell->layer = layerNumber;
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

// Converts a given position to the indices of the cell which would contain it
const Vector3i MacGrid::positionToIndices(const Vector3f &position) const
{
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
