#include <iostream>
#include <fstream>
#include <QFile>
#include <QtConcurrent>
#include <random>

#include "src/macGrid/MacGrid.h"
#include "src/graphics/MeshLoader.h"
#include "src/Debug.h"

typedef Eigen::Triplet<float> T;

using namespace std;
using namespace Eigen;

const vector<Vector3i> NEIGHBOR_OFFSETS = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

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


MacGrid::MacGrid(QSettings &settings, std::string folder) {
#if SANITY_CHECKS
  assert(0 < cellWidth);
  assert(0 < cellCount[0]);
  assert(0 < cellCount[1]);
  assert(0 < cellCount[2]);
#endif

    settings.beginGroup("/Simulation");
    m_cellWidth = settings.value(QString("cellWidth")).toFloat();
    m_maxAverageSurfaceParticlesPerCellFaceArea = settings.value(QString("maxAverageSurfaceParticlesPerCellFaceArea")).toFloat();
    m_maxAverageSurfaceParticlesPerArea = m_maxAverageSurfaceParticlesPerCellFaceArea / m_cellWidth / m_cellWidth;

    m_cellCount = Vector3i(settings.value(QString("cellCountX")).toInt(),
                           settings.value(QString("cellCountY")).toInt(),
                           settings.value(QString("cellCountZ")).toInt());

    m_cornerPosition = Vector3f(settings.value(QString("cornerPositionX")).toFloat(),
                                settings.value(QString("cornerPositionY")).toFloat(),
                                settings.value(QString("cornerPositionZ")).toFloat());

    m_solidMeshFilepath = folder + "/solid.obj";
    m_fluidMeshFilepath = folder + "/fluid.obj";

    m_fluidInternalPosition = Vector3f(settings.value(QString("fluidInternalPositionX")).toFloat(),
                                       settings.value(QString("fluidInternalPositionY")).toFloat(),
                                       settings.value(QString("fluidInternalPositionZ")).toFloat());

    m_simulationTime = settings.value(QString("simulationTime")).toInt();

    m_gravityVector = Vector3f(settings.value(QString("gravityX")).toFloat(),
                               settings.value(QString("gravityY")).toFloat(),
                               settings.value(QString("gravityZ")).toFloat());

    m_interpolationCoefficient = settings.value(QString("interpolationCoefficient")).toFloat();
    settings.endGroup();
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
    assert(kv != m_cells.end()); // is in a cell which exists, and
    assert(kv->second == particle->cell); // is in the correct cell for its position
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
  // fillGridCellsFromInternalPosition(Material::Fluid, m_fluidInternalPosition);
  addParticlesToCells(Material::Fluid);
}

extern void saveParticles(std::string filepath, std::vector<Eigen::Vector3f> particle_positions) {
    fstream fout;
    fout.open(filepath, ios::out);
    if(!fout.good()) {
        std::cerr << filepath << " could not be opened" << std::endl;
        return;
    }

    for (Eigen::Vector3f particle : particle_positions) {
        std::string toWrite =
                std::to_string(particle[0]) + ", " +
                std::to_string(particle[1]) + ", " +
                std::to_string(particle[2]);
        fout << toWrite << std::endl;
    }

    fout.close();
}

void MacGrid::simulate(std::string output_folder)
{
  float time = 0.0f;

  float framePeriod = 0.166666666f;

  std::vector<QFuture<void>> futures;
  while (time < m_simulationTime) {
    // Compute deltaTime or something
    // const float deltaTime = calculateDeltaTime();
    const float deltaTime = 1.0f;

    // Given particle velocities, update grid velocities

    // Calculate and apply external forces
    applyExternalForces(deltaTime);

    // Enforce DBC
    enforceDirichletBC();

    // Given particle positions, update cell materials and neighbors
    createBufferZone();

    // Given cells, neighbors, and cell velocities, update velocity by removing divergence
    classifyPseudoPressureGradient();

    // Given old and new grid velocities, updates particle positions using RK2
    updateParticlePositions(deltaTime);

    time += deltaTime;

    // Todo: if some conditional is met spit out the particles
    if (true) {
        std::string output_filepath = output_folder + "/" + std::to_string(time) + ".csv";
        std::vector<Eigen::Vector3f> particles;
        for (Particle * particle : m_particles) {
            Eigen::Vector3f position = particle->position;
            particles.emplace_back(position[0], position[1], position[2]);
        }
        QFuture<void> future = QtConcurrent::run(saveParticles, output_filepath, particles);
        futures.push_back(future);
    }
  }

  std::cout << "Waiting for threads  to finish writing particles" << std::endl;
  for (QFuture<void> future: futures) {
      future.waitForFinished();
  }
}

// Sets up fluid cells and a buffer zone around them, based on the particles' positions
void MacGrid::createBufferZone()
{
  // Set layer field of all cells to −1
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) kv->second->layer = -1;

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
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); i++) {
      m_particles[i]->cell = m_cells[positionToIndices(m_particles[i]->position)];
  }
}

// ================== Debugging

// Debugging only: sets the 6 adjacent velocity values for a grid cell with the given indices
void MacGrid::setGridCellVelocity(const Vector3i cellIndices, const Vector3f velocity1, const Vector3f velocity2)
{
  Cell *gridCell = m_cells[cellIndices];

  assert(gridCell != nullptr);
  assert(gridCell->material == Material::Fluid);

  gridCell->ux = velocity1[0];
  gridCell->uy = velocity1[0];
  gridCell->uz = velocity1[0];

  m_cells[cellIndices + Vector3i(1, 0, 0)]->ux = -velocity2[0];
  m_cells[cellIndices + Vector3i(0, 1, 0)]->uy = -velocity2[1];
  m_cells[cellIndices + Vector3i(0, 0, 1)]->uz = -velocity2[2];
}

// Debugging only: adds a particle to the system (does not set cells' and particles' relationships)
void MacGrid::addParticle(const Vector3f position, const Vector3f velocity)
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

// Helper: produces a random float in the range [0, 1]
inline float getRandomFloat()
{
  return static_cast <float> (arc4random()) / static_cast <float> (UINT32_MAX);
}

// Helper: gets a uniformly random position on a given triangle defined by a point and two vectors
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
void MacGrid::meshToSurfaceParticles(vector<Particle *> &surfaceParticles, const string meshFilepath)
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
void MacGrid::fillGridCellsFromInternalPosition(const Material material, const Eigen::Vector3f &internalPosition)
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
void MacGrid::fillGridCellsRecursive(const Material material, const int layerNumber, const Eigen::Vector3i &cellIndices) {

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

// Given a material, populates all cells with that material with particles using stratified sampling
void MacGrid::addParticlesToCells(const Material material) {
  const int strata = 3;           // number of subdivisions per side, such that there are strata^3 subcells per cell
  const int samplesPerStrata = 1; // number of particles per subcell
  const float strataWidth = m_cellWidth / strata;
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    if (i->second->material != material) {
      continue;
    }
    for (int x = 0; x < strata; x++) {
      for (int y = 0; y < strata; y++) {
        for (int z = 0; z < strata; z++) {
          for (int sample = 0; sample < samplesPerStrata; ++sample) {
            const float a = getRandomFloat();
            const float b = getRandomFloat();
            const float c = getRandomFloat();
            Vector3f position = indicesToBasePosition(i->first) + Vector3f(x+a, y+b, z+c) * strataWidth;
            Particle * newParticle = new Particle{nullptr, position, Vector3f::Zero()};
            assert(positionToIndices(position) == i->first);
            m_particles.push_back(newParticle);
          }
        }
      }
    }
  }
}

// ================== Simulation Helpers

// Todo
float MacGrid::calculateDeltaTime()
{
  float maxV = 0;
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
      Particle * particle = m_particles[i];
      if(m_particles[i]->velocity.norm()>=maxV) {maxV = m_particles[i]->velocity.norm();}
  }
  return m_cellWidth / maxV;

}

// Applies external forces to the velocity field
void MacGrid::applyExternalForces(const float deltaTime)
{
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    m_particles[i]->velocity += m_gravityVector * deltaTime;
  }
}

// Sets the velocity field into and out of solid cells to zero 
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
      continue;
    }

    // Set fluid-solid boundary velocities to zero (this is safe due to the buffer layer around the fluid)
    if (m_cells[kv->first + Vector3i(-1, 0, 0)]->material == Solid) kv->second->ux = 0;
    if (m_cells[kv->first + Vector3i(0, -1, 0)]->material == Solid) kv->second->uy = 0;
    if (m_cells[kv->first + Vector3i(0, 0, -1)]->material == Solid) kv->second->uz = 0;
  }
}

// Sets the diverging component of the velocity field to zero
// Todo: save space by only having rows/cols for fluid cells
void MacGrid::classifyPseudoPressureGradient()
{
  Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,Lower|Upper> m_solver;
  int numFluidCells = 0;
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    if (i->second->material == Fluid) {
        i->second->index = numFluidCells;
        numFluidCells++;

    }
  }

  //,Eigen::IncompleteCholesky<float>
  Eigen::SparseMatrix<float> A(numFluidCells, numFluidCells);
  std::vector<T> coefficients;

  VectorXf b(numFluidCells);

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
      float divergence = 0;
      //assume velocity on solid always 0
      divergence += (m_cells[i->first+Eigen::Vector3i(1,0,0)]->ux)-(i->second->ux);
      divergence += (m_cells[i->first+Eigen::Vector3i(0,1,0)]->uy)-(i->second->uy);
      divergence += (m_cells[i->first+Eigen::Vector3i(0,0,1)]->uz)-(i->second->uz);
      b(i->second->index,0) = divergence;
    } else {
      b(i->second->index,0) = 0;
    }
  }
  A.setFromTriplets(coefficients.begin(), coefficients.end());

  VectorXf scalarField(m_cells.size());
  m_solver.compute(A);
  scalarField = m_solver.solve(b);
  cout << "mat A" << A << endl;
  cout << "mat b" << b << endl;
  cout << "scalar field " << scalarField << endl;

#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    if (i->second->material == Fluid) {
      float xGradient = scalarField[i->second->index];
      if (m_cells[i->first+Eigen::Vector3i(-1,0,0)]->material == Fluid) {
          xGradient -= scalarField[m_cells[i->first+Eigen::Vector3i(-1,0,0)]->index];
      }
      float yGradient = scalarField[i->second->index];
      if (m_cells[i->first+Eigen::Vector3i(0,-1,0)]->material == Fluid) {
          yGradient -= scalarField[m_cells[i->first+Eigen::Vector3i(0,-1,0)]->index];
      }
      float zGradient = scalarField[i->second->index];
      if (m_cells[i->first+Eigen::Vector3i(0,0,-1)]->material == Fluid) {
          zGradient -= scalarField[m_cells[i->first+Eigen::Vector3i(0,0,-1)]->index];
      }
      i->second->ux -= xGradient;
      i->second->uy -= yGradient;
      i->second->uz -= zGradient;
    }
  }
}

// Sets the velocity field based on the particles' positions and velocities
void MacGrid::transferParticlesToGrid()
{
  // Todo
}

// Sets particle velocities based on their positions and the velocity field
// Step 1: Given new and old grid velocities produce new FLIP velocities per particle
// Step 2: Given new grid velocities produce new PIC velocities per particle
// Step 3: Interpolate PIC and FLIP velocities to get new particle velocities
void MacGrid::updateParticleVelocities()
{
  // For every particle
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];
    Vector3f particlePos_original = particle->position;
    Vector3f particlePos = particlePos_original;
    Vector3i gridIdx = positionToIndices(particle->position);
    Vector3i idx;
    float picx, picy, picz = 0;
    float flipx, flipy, flipz = 0;

    particlePos = particlePos_original-Vector3f(0,0.5,0.5);
    idx[0] = floor(particlePos[0]);//l
    idx[1] = floor(particlePos[1]);//m
    idx[2] = floor(particlePos[2]);//n
    Vector3f weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    // For 2x2 cell neighborhood
    for(int l = 0; l < 2; l++){
        for(int m = 0; m < 2; m++){
            for(int n = 0; n < 2; n++){
                if (l == 1){weights[0] = particlePos[0] - idx[0];}
                if (m == 1){weights[1] = particlePos[1] - idx[1];}
                if (n == 1){weights[2] = particlePos[2] - idx[2];}
                Vector3i offset = Vector3i(l, m, n);
                // Calculate PIC particle velocity
                picx = picx + weights[0]*weights[2]*weights[3]*m_cells[gridIdx+offset]->ux;
//                picy = picy + weights[1]*m_cells[gridIdx+offset]->uy;
//                picz = picz + weights[2]*m_cells[gridIdx+offset]->uz;
                // Calculate FLIP particle velocity
                flipx = flipx + weights[0]*weights[2]*weights[3]*(m_cells[gridIdx+offset]->ux - m_cells[gridIdx+offset]->oldUX);
//                flipy = flipy + weights[1]*(m_cells[gridIdx+offset]->ux - particle->velocity[1]);
//                flipz = flipz + weights[2]*(m_cells[gridIdx+offset]->ux - particle->velocity[2]);
            }
        }
    }

    particlePos = particlePos_original-Vector3f(0.5,0,0.5);
    idx[0] = floor(particlePos[0]);//l
    idx[1] = floor(particlePos[1]);//m
    idx[2] = floor(particlePos[2]);//n

    weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    // For 2x2 cell neighborhood
    for(int l = 0; l < 2; l++){
        for(int m = 0; m < 2; m++){
            for(int n = 0; n < 2; n++){
                if (l == 1){weights[0] = particlePos[0] - idx[0];}
                if (m == 1){weights[1] = particlePos[1] - idx[1];}
                if (n == 1){weights[2] = particlePos[2] - idx[2];}
                Vector3i offset = Vector3i(l, m, n);
                // Calculate PIC particle velocity
                picy = picy + weights[0]*weights[2]*weights[3]*m_cells[gridIdx+offset]->uy;
                // Calculate FLIP particle velocity
                flipy = flipy + weights[0]*weights[2]*weights[3]*(m_cells[gridIdx+offset]->uy - m_cells[gridIdx+offset]->oldUY);
            }
        }
    }

    particlePos = particlePos_original-Vector3f(0.5,0.5,0);
    idx[0] = floor(particlePos[0]);//l
    idx[1] = floor(particlePos[1]);//m
    idx[2] = floor(particlePos[2]);//n

    weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    // For 2x2 cell neighborhood
    for(int l = 0; l < 2; l++){
        for(int m = 0; m < 2; m++){
            for(int n = 0; n < 2; n++){
                if (l == 1){weights[0] = particlePos[0] - idx[0];}
                if (m == 1){weights[1] = particlePos[1] - idx[1];}
                if (n == 1){weights[2] = particlePos[2] - idx[2];}
                Vector3i offset = Vector3i(l, m, n);
                // Calculate PIC particle velocity
                picz = picz + weights[0]*weights[2]*weights[3]*m_cells[gridIdx+offset]->uz;
                // Calculate FLIP particle velocity
                flipz = flipz + weights[0]*weights[2]*weights[3]*(m_cells[gridIdx+offset]->uz - m_cells[gridIdx+offset]->oldUZ);
            }
        }
    }




    Vector3f pic = Vector3f(picx, picy, picz);
    Vector3f flip = Vector3f(flipx, flipy, flipz);

    // Update particle with interpolated PIC/FLIP velocities
    particle->velocity = m_interpolationCoefficient*pic + (1-m_interpolationCoefficient)*(particle->velocity+flip);
  }
}

// Todo: move particles, use oldPosition, fix collisions
void MacGrid::updateParticlePositions(const float deltaTime)
{
  // Runge-Kutta 2 ODE solver
  updateParticleVelocities();

#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];
    particle->oldPosition = particle->position;
    // Move particles half step
    particle->position = particle->position + particle->velocity*deltaTime*0.5;
    // TODO: fix problem of particle penetrating the solid cell
    Vector3i gridIdx = positionToIndices(particle->position);
    if(m_cells[gridIdx]->material == Material::Solid){
          Vector3i oldIdx = positionToIndices(particle->oldPosition);
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
  updateParticleVelocities();
  // Final position update
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];
//    particle->position = particle->oldPosition + particle->velocity*deltaTime;
  }
}

// ================== Positional Helpers

// Given a position, returns the indices of the cell which would contain it
const Vector3i MacGrid::positionToIndices(const Vector3f &position) const
{
  const Vector3f regularizedPosition = (position - m_cornerPosition) / m_cellWidth;

  return Vector3i(floor(regularizedPosition[0]),
      floor(regularizedPosition[1]),
      floor(regularizedPosition[2]));
}

// Given a cell's indices, returns the position of its lowest x, y, z corner
const Vector3f MacGrid::indicesToBasePosition(const Vector3i &cellIndices) const
{
  return cellIndices.cast<float>() * m_cellWidth + m_cornerPosition;
}

// Given a cell's indices, returns the position of its center
const Vector3f MacGrid::indicesToCenterPosition(const Vector3i &cellIndices) const
{
  return (cellIndices.cast<float>() + Vector3f::Ones() * 0.5f) * m_cellWidth + m_cornerPosition;
}

// ================== Miscellaneous Helpers

// Given a material and a vector of particles, sets all cells containing those particles to that material
void MacGrid::assignParticleCellMaterials(const Material material, const vector<Particle *> &particles)
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
      cell->layer = layerNumber;
    }
  }
}

// Given a cell's indices, returns whether it falls within the simulation grid's bounds
bool MacGrid::withinBounds(const Vector3i &cellIndices) const
{
  return 0 <= cellIndices[0] && cellIndices[0] < m_cellCount[0] &&
      0 <= cellIndices[1] && cellIndices[1] < m_cellCount[1] &&
      0 <= cellIndices[2] && cellIndices[2] < m_cellCount[2];
}
