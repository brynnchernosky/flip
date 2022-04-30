#include <iostream>
#include <fstream>
#include <QFile>
#include <QtConcurrent>
#include <random>

#include "src/macGrid/MacGrid.h"
#include "src/graphics/MeshLoader.h"
#include "src/Debug.h"

using namespace std;
using namespace Eigen;

typedef Triplet<float> T;

const vector<const Vector3i> NEIGHBOR_OFFSETS = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
const vector<const Vector3f> INTERP_OFFSETS = {{0, 0.5, 0.5}, {0.5, 0, 0.5}, {0.5, 0.5, 0}};

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

MacGrid::MacGrid(string folder)
{
#if SANITY_CHECKS
  assert(0 < cellWidth && 0 < cellCount[0] && 0 < cellCount[1] && 0 < cellCount[2]);
#endif

  QSettings settings(QString::fromStdString(folder + "/config.ini"), QSettings::IniFormat);
  
  settings.beginGroup("/Simulation");

  m_outputFolder = folder + "/particles";

  m_cellWidth = settings.value(QString("cellWidth")).toFloat();
  m_strata = settings.value(QString("strata")).toFloat();

  m_maxAverageSurfaceParticlesPerCellFaceArea = settings.value(QString("maxAverageSurfaceParticlesPerCellFaceArea")).toFloat();
  m_maxAverageSurfaceParticlesPerArea         = m_maxAverageSurfaceParticlesPerCellFaceArea / m_cellWidth / m_cellWidth;

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

// Function for QtConcurrent to save files asynchronously
extern void saveParticles(const string filepath, const vector<const Vector3f> particlePositions) {
  fstream fout;
  fout.open(filepath, ios::out);

  if (!fout.good()) {
    cerr << filepath << " could not be opened" << endl;
    return;
  }

  for (const Vector3f &particlePosition : particlePositions) {
    string toWrite =
        to_string(particlePosition[0]) + ", " +
        to_string(particlePosition[1]) + ", " +
        to_string(particlePosition[2]);
    fout << toWrite << endl;
  }

  fout.close();
}

void MacGrid::simulate()
{
  float time = 0.0f;

  // const float framePeriod = 1 / 60.f;

  // Prepare to asynchronously save files
  vector<QFuture<void>> futures;

  int i = 0;
  
  // Start simulation loop
  while (time < m_simulationTime) {

    cout << "================== Starting loop " << i << endl;
    ++i;

    // Compute deltaTime or something
    const float deltaTime = calculateDeltaTime();
    cout << "∟ calculated deltaTime to be " << deltaTime << endl;

    // Given particle positions, update cell materials, neighbors, and layers, then create buffer zone
    createBufferZone();
    cout << "∟ created buffer zone, now have " << m_cells.size() << " cells"  << endl;

    // Given particle velocities, update velocity field (adjacent grid cells' velocities)
    transferParticlesToGrid();
    cout << "∟ " << m_particles.size() << " particles transferred to grid" << endl;

    // Calculate and apply external forces to velocity field
    applyExternalForces(deltaTime);
    cout << "∟ applied external forces to " << m_cells.size() << " cells" << endl;

    printGrid();

    // Given cells, neighbors, and cell velocities, update the velocity field by removing divergence
    updateVelocityFieldByRemovingDivergence();
    cout << "∟ updated velocity field by removing divergence" << endl;

    printGrid();

    // Enforce DBC
    enforceDirichletBC();
    cout << "∟ enforced dirichlet boundary condition" << endl;

    printGrid();

    // Given old and new grid velocities, update particle positions using RK2
    updateParticlePositions(deltaTime);
    cout << "∟ updated particle positions" << endl;
    
    // Increment time
    time += deltaTime;

    // Todo: if some conditional is met, spit out the particles
    if (true) {
      const string filepath = m_outputFolder + "/" + to_string(time) + ".csv";
      vector<const Vector3f> particlePositions;
      for (Particle * const particle : m_particles) particlePositions.push_back(particle->position);
      QFuture<void> future = QtConcurrent::run(saveParticles, filepath, particlePositions);
      futures.push_back(future);
      
      cout << "∟ started saving particles" << endl;
    }
  }

  // Wait for threads to finish writing particles
  for (QFuture<void> future : futures) future.waitForFinished();
}

// Sets up fluid cells and a buffer zone around them, based on the particles' positions
void MacGrid::createBufferZone()
{
  // Set all fluid cells to air
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    Cell * cell = kv->second;
    if (cell->material == Material::Fluid) cell->material = Material::Air;
  }

  // Update grid cells that currently have fluid in them
  assignParticleCellMaterials(Material::Fluid, m_particles);

  // Set layer field of non-fluid cells to -1 and fluid cells to 0
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) kv->second->layer = -1;
  setCellMaterialLayers(Material::Fluid, 0);

  // Create a buffer zone around the fluid
  for (int bufferLayer = 1; bufferLayer < max(4, (int) ceil(m_kCFL)); ++bufferLayer) {

    // For each cell
    for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
      const Vector3i cellIndices = kv->first;
      Cell * cell = kv->second;

      // Skip if its layer != bufferLayer - 1
      if (cell->layer != bufferLayer - 1) continue;

      // If it's a solid cell, set its layer and move on
      if (cell->material == Material::Solid) {
        cell->layer = bufferLayer;
        continue;
      }

      // For each of its six neighbor cells
      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
        const Vector3i neighborIndices = cellIndices + neighborOffset;
        auto neighborKV = m_cells.find(neighborIndices);

        // If the neighbor cell does not exist
        if (neighborKV == m_cells.end()) {

          // Create the neighbor cell and put it in the hash table
          Cell * newNeighbor = new Cell{};
          newNeighbor->cellIndices = neighborIndices;
          m_cells.insert({neighborIndices, newNeighbor});
          
          // Set its material and layer
          newNeighbor->material = withinBounds(neighborIndices) ? Material::Air : Material::Solid;
          newNeighbor->layer = bufferLayer;

          continue;
        }

        // If the neighbor cell does exist
        Cell * neighbor = neighborKV->second;

        // If the neighbor's layer and material haven't been set yet
        if (neighbor->layer == -1) {
          neighbor->layer = bufferLayer;
          if (neighbor->material != Material::Solid) neighbor->material = Material::Air;
        } 
      }
    }
  }

  // Set particles' cells
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    m_particles[i]->cell = m_cells[positionToIndices(m_particles[i]->position)];
  }
  
  // Delete all cells with layer == -1
  auto kv = m_cells.cbegin();
  while (kv != m_cells.cend()) {
    if (kv->second->layer == -1) {
      delete kv->second;
      kv = m_cells.erase(kv);
    } else {
      ++kv;
    }
  }
}

// ================== Debugging

// Debugging only: sets the 6 adjacent velocity values for a grid cell with the given indices
void MacGrid::setGridCellVelocity(const Vector3i cellIndices, const Vector3f velocity1, const Vector3f velocity2)
{
  Cell *gridCell = m_cells[cellIndices];

  assert(gridCell != nullptr);
  assert(gridCell->material == Material::Fluid);

  gridCell->u[0] = velocity1[0];
  gridCell->u[1] = velocity1[0];
  gridCell->u[2] = velocity1[0];

  m_cells[cellIndices + Vector3i(1, 0, 0)]->u[0] = -velocity2[0];
  m_cells[cellIndices + Vector3i(0, 1, 0)]->u[1] = -velocity2[1];
  m_cells[cellIndices + Vector3i(0, 0, 1)]->u[2] = -velocity2[2];
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

void MacGrid::printParticles(string output_filepath) {
  fstream fout;
  fout.open(output_filepath, ios::out);

  if (!fout.good()) {
    cerr << output_filepath << " could not be opened" << endl;
    return;
  }

  for (Particle * const particle : m_particles) {
    const Vector3f particlePosition = particle->position;
    string toWrite =
      to_string(particlePosition[0]) + ", " +
      to_string(particlePosition[1]) + ", " +
      to_string(particlePosition[2]);
    fout << toWrite << endl;
  }

  cout << "Wrote particles to " << output_filepath << endl;
  fout.close();
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
  cout << "Successfully loaded mesh: \"" << meshFilepath << "\"" << endl;

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
void MacGrid::fillGridCellsFromInternalPosition(const Material material, const Vector3f &internalPosition)
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
  newCell->cellIndices = cellIndices;
  newCell->material = material;
  newCell->layer = layerNumber;
  m_cells.insert({cellIndices, newCell});

  // Recursively fill neighbors
  fillGridCellsRecursive(material, layerNumber, cellIndices);
}

// Recursive helper function for the above
void MacGrid::fillGridCellsRecursive(const Material material, const int layerNumber, const Vector3i &cellIndices) {

  // For each neighbor
  for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
    const Vector3i neighborIndices = cellIndices + neighborOffset;

    // Move on if it already exists or is outside the bounds
    if (m_cells.find(neighborIndices) != m_cells.end() || !withinBounds(neighborIndices)) continue;

    // Create a cell here
    Cell * newCell = new Cell{};
    newCell->cellIndices = cellIndices;
    newCell->material = material;
    newCell->layer = layerNumber;
    m_cells.insert({neighborIndices, newCell});

    // Recursively fill neighbors
    fillGridCellsRecursive(material, layerNumber, neighborIndices);
  }
}

// Given a material, populates all cells with that material with particles using stratified sampling
void MacGrid::addParticlesToCells(const Material material) {

  // Note: m_strata is the number of subdivisions per side, such that there are strata^3 subcells per cell
  const float strataWidth = m_cellWidth / m_strata;

  // For each cell
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {

    const Vector3i cellIndices = i->first;
    Cell * cell = i->second;

    // Skip if the cell is of the wrong material
    if (cell->material != material) continue;

    // Do stratified sampling
    for (int x = 0; x < m_strata; ++x) {
      for (int y = 0; y < m_strata; ++y) {
        for (int z = 0; z < m_strata; ++z) {

          // Get offsets
          const float a = getRandomFloat(), b = getRandomFloat(), c = getRandomFloat();

          // Create a new particle
          Particle * newParticle = new Particle{};
          newParticle->position = indicesToBasePosition(cellIndices) + Vector3f(x+a, y+b, z+c) * strataWidth;
          assert(positionToIndices(newParticle->position) == cellIndices);
          m_particles.push_back(newParticle);
        }
      }
    }
  }
}

// ================== Simulation Helpers

// Calculates timestep so CFL condition met
float MacGrid::calculateDeltaTime()
{
  float maxV = 0;
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * const particle = m_particles[i];
    if (particle->velocity.norm() > maxV) {
      maxV = particle->velocity.norm();
    }
  }
  maxV += __FLT_EPSILON__;
  return max(min(m_cellWidth / maxV, 1.f), 0.1f);
}

// Applies external forces to the velocity field
void MacGrid::applyExternalForces(const float deltaTime)
{
  const Vector3f gravityDeltaV = m_gravityVector * deltaTime;
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    i->second->u += gravityDeltaV;
  }
}

// Sets the velocity field into solid cells to zero 
void MacGrid::enforceDirichletBC()
{
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    const Vector3i cellIndices = kv->first;
    Cell * cell = kv->second;

    // Skip if cell is not solid
    if (cell->material != Material::Solid) continue;

    // Prevent flow into solid

    cell->u = Vector3f(min(0.f, cell->u[0]),
                       min(0.f, cell->u[1]),
                       min(0.f, cell->u[2]));

    auto xMaxKV = m_cells.find(cellIndices + Vector3i(1, 0, 0));
    if (xMaxKV != m_cells.end()) xMaxKV->second->u[0] = max(0.f, xMaxKV->second->u[0]);

    auto yMaxKV = m_cells.find(cellIndices + Vector3i(0, 1, 0));
    if (yMaxKV != m_cells.end()) yMaxKV->second->u[1] = max(0.f, yMaxKV->second->u[1]);

    auto zMaxKV = m_cells.find(cellIndices + Vector3i(0, 0, 1));
    if (zMaxKV != m_cells.end()) zMaxKV->second->u[2] = max(0.f, zMaxKV->second->u[2]);
  }
}

// Sets the diverging component of the velocity field to zero
void MacGrid::updateVelocityFieldByRemovingDivergence()
{
  // Set up solver with preconditioning
  ConjugateGradient<SparseMatrix<float>, Lower|Upper, IncompleteCholesky<float>> m_solver;

  // Number all fluid cells
  int numFluidCells = 0;
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    if (i->second->material == Fluid) {
      i->second->index = numFluidCells;
      numFluidCells++;
    } else {
      i->second->index = -1;
    }
  }

  // Create A and b arrays
  SparseMatrix<float> A(numFluidCells, numFluidCells);
  vector<T> coefficients;
  VectorXf b(numFluidCells);

  // Fill A and b arrays by iterating over all fluid cells
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {

    const Vector3i cellIndices = i->first;
    const Cell * cell = i->second;

    // Skip if the cell is non-fluid
    if (cell->material != Fluid) continue;

    // Fill this row of the A matrix (coefficients)
    float centerCoefficient = -6;
    for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
      if (m_cells[cellIndices + neighborOffset]->material == Solid) {
        centerCoefficient += 1;
      } else if (m_cells[cellIndices + neighborOffset]->material == Fluid) {
        coefficients.push_back(T(cell->index, m_cells[cellIndices + neighborOffset]->index, 1));
      }
    }
    coefficients.push_back(T(cell->index, cell->index, centerCoefficient));

    // Fill this row of the b matrix (divergences) (at this point, velocities in/out of solids should be 0)
    float divergence = 0;
    divergence += (m_cells[cellIndices + Vector3i(1, 0, 0)]->u[0]) - cell->u[0];
    divergence += (m_cells[cellIndices + Vector3i(0, 1, 0)]->u[1]) - cell->u[1];
    divergence += (m_cells[cellIndices + Vector3i(0, 0, 1)]->u[2]) - cell->u[2];
    b(cell->index, 0) = divergence;
  }
  A.setFromTriplets(coefficients.begin(), coefficients.end());

  // Solve for pseudo-pressures
  VectorXf pseudoPressures(m_cells.size());
  m_solver.compute(A);
  pseudoPressures = m_solver.solve(b);

  // Use pseudo-pressures to correct velocities
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {

    const Vector3i cellIndices = i->first;
    Cell * cell = i->second;

    // Skip if the cell is non-fluid
    if (cell->material != Fluid) continue;

    // Update velocity based on pseudopressure
    float xGradient = pseudoPressures[cell->index];
    if (m_cells[cellIndices + Vector3i(-1, 0, 0)]->material == Fluid) {
      xGradient -= pseudoPressures[m_cells[cellIndices + Vector3i(-1, 0, 0)]->index];
    }
    cell->u[0] -= xGradient;

    float yGradient = pseudoPressures[cell->index];
    if (m_cells[cellIndices + Vector3i(0, -1, 0)]->material == Fluid) {
      yGradient -= pseudoPressures[m_cells[cellIndices + Vector3i(0, -1, 0)]->index];
    }
    cell->u[1] -= yGradient;

    float zGradient = pseudoPressures[cell->index];
    if (m_cells[cellIndices + Vector3i(0, 0, -1)]->material == Fluid) {
      zGradient -= pseudoPressures[m_cells[cellIndices + Vector3i(0, 0, -1)]->index];
    }
    cell->u[2] -= zGradient;
  }

  // ================== Section 3f

  // Set layer field of non-fluid cells to -1 and fluid cells to 0
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) kv->second->layer = -1;
  setCellMaterialLayers(Material::Fluid, 0);

  // Iterate to extrapolate velocities
  for (int bufferLayer = 1; bufferLayer < max(4, (int) ceil(m_kCFL)); ++bufferLayer) {
    
    // For each cell
#pragma omp parallel for
    for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
      
      // Skip if layer != -1
      const Vector3i cellIndices = i->first;
      Cell * cell = i->second;
      if (cell->layer != -1) continue;

      // If cell has a neighbor of one lower layer, set layer to bufferLayer and set its velocities
      int expectedLayer = bufferLayer - 1;
      int count = 0;
      float averageX = 0;
      float averageY = 0;
      float averageZ = 0;

      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {

        // Skip if neighbor does not exist
        if (m_cells.find(cellIndices + neighborOffset) == m_cells.end()) continue;

        // Skip if neighbor is not of expected layer
        const Cell * neighbor = m_cells[cellIndices + neighborOffset];
        if (neighbor->layer != expectedLayer) continue;
        
        // Capture info
        count++;
        averageX += neighbor->u[0];
        averageY += neighbor->u[1];
        averageZ += neighbor->u[2];
      }

      // Has neighbors of one lower layer
      if (count > 0) {
        averageX /= count;
        averageY /= count;
        averageZ /= count;

        // Set stuff
        cell->layer = bufferLayer;

        if (m_cells.find(cellIndices + Vector3i(-1, 0, 0)) != m_cells.end() && m_cells[cellIndices + Vector3i(-1, 0, 0)]->material == Material::Fluid)
          cell->u[0] = averageX;
        
        if (m_cells.find(cellIndices + Vector3i(0, -1, 0)) != m_cells.end() && m_cells[cellIndices + Vector3i(0, -1, 0)]->material == Material::Fluid)
          cell->u[1] = averageY;
        
        if (m_cells.find(cellIndices + Vector3i(0, 0, -1)) != m_cells.end() && m_cells[cellIndices + Vector3i(0, 0, -1)]->material == Material::Fluid)
          cell->u[2] = averageZ;
      }
    }
  }
}

// Sets the velocity field based on the particles' positions and velocities
void MacGrid::transferParticlesToGrid()
{
  // Zero out cells' particle numbers and average velocities
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    Cell * cell = i->second;
    cell->temp_avgParticleV = Vector3f::Zero();
    cell->temp_particleNums = 0;
  }
  
  // Accumulate cells' particle numbers and velocities
  const Vector3f offset = Vector3f(0.5, 0.5, 0.5);
  for (Particle * const particle : m_particles) {
    const Vector3i belongingCell = positionToIndices(particle->position + offset);
#if SANITY_CHECKS
    assert(m_cells.find(belongingCell) != m_cells.end());
#endif
    m_cells[belongingCell]->temp_avgParticleV += particle->velocity;
    m_cells[belongingCell]->temp_particleNums += 1;
  }
  
  // Average velocities
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    Cell * cell = i->second;
    if (cell->temp_particleNums != 0) cell->temp_avgParticleV /= cell->temp_particleNums;
  }
  
  // Do trilinear interpolation
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    
    const Vector3i cellIndices = i->first;
    Cell * cell = i->second;
    
    // Skip if the cell is non-fluid
    if (cell->material != Material::Fluid) continue;

    Vector3i offset = Vector3i::Zero();
    Vector3f gridV  = Vector3f::Zero();

    // Transfer particle velocity to grid
    for (int m = 0; m < 2; ++m) {
      for (int n = 0; n < 2; ++n) {
        for (int l = 0; l < 2; ++l) {
          offset = Vector3i(m, n, l);
          gridV += 0.125 * m_cells[cellIndices + offset]->temp_avgParticleV;
        }
      }
    }

    // Save cell's velocities
    cell->u    = gridV;
    cell->oldU = gridV;
  }
}

// Sets particle velocities based on their positions and the velocity field
// Step 1: Given new and old grid velocities produce new FLIP velocities per particle
// Step 2: Given new grid velocities produce new PIC velocities per particle
// Step 3: Interpolate PIC and FLIP velocities to get new particle velocities
void MacGrid::updateParticleVelocities()
{
  // For every particle
// #pragma omp parallel for
  for (unsigned int particleIndex = 0; particleIndex < m_particles.size(); ++particleIndex) {

    Particle * particle = m_particles[particleIndex];

    cout << "Before: " << Debug::particleToString(particle) << endl;

    Vector3f pic  = Vector3f::Zero();
    Vector3f flip = Vector3f::Zero();

    const Vector3f position = particle->position;
    const Vector3f regularizedPosition = toRegularizedPosition(position);

    // For each component of velocity
    for (int index = 0; index < 3; ++index) {

      // Get the regularized position to interpolate this component of velocity from
      const Vector3f xyz = regularizedPosition - INTERP_OFFSETS[index];

      // Get PIC and FLIP values from grid
      pair<float, float> pf = getInterpolatedPICAndFLIP(xyz, index);
      pic[index]  = pf.first;
      flip[index] = pf.second;
    }

    cout << "pic " << Debug::vectorToString(pic) << ", flip " << Debug::vectorToString(flip) << endl;

    // Update particle with interpolated PIC/FLIP velocities
    particle->velocity = pic; // m_interpolationCoefficient * pic + (1 - m_interpolationCoefficient) * (particle->velocity + flip);

    cout << "After: " << Debug::particleToString(particle) << endl;
  }
}

// Helper for the above
pair<float, float> MacGrid::getInterpolatedPICAndFLIP(const Vector3f &xyz, const int index) const
{
  float p = 0, f = 0;

  // Get stuff for weight calculation
  const Vector3i ijk = Vector3i(floor(xyz[0]), floor(xyz[1]), floor(xyz[2]));

  // For a 2x2 cell neighborhood around xyz
  for (int l = 0; l < 2; ++l) {
    for (int m = 0; m < 2; ++m) {
      for (int n = 0; n < 2; ++n) {

        // Get cell
        const Vector3i offset(l, m, n);
        const Vector3i sum = ijk + offset;
        // cout << "- ijk    = " << Debug::vectorToString(ijk) << endl;
        // cout << "- offset = " << Debug::vectorToString(offset) << endl;
        cout << "- sum    = " << Debug::vectorToString(sum) << endl;
        assert(m_cells.find(sum) != m_cells.end());
        Cell * const cell = m_cells.at(sum);

        // Get weights
        Vector3f weights = ijk.cast<float>() + Vector3f::Ones() - xyz;
        if (l == 1) weights[0] = xyz[0] - ijk[0];
        if (m == 1) weights[1] = xyz[1] - ijk[1];
        if (n == 1) weights[2] = xyz[2] - ijk[2];

        // Add to PIC and FLIP particle velocities
        p += weights[0] * weights[1] * weights[2] * cell->u[index];
        f += weights[0] * weights[1] * weights[2] * (cell->u[index] - cell->oldU[index]);
      }
    }
  }

  return {p, f};
}

// Sets particle positions based on their velocities
void MacGrid::updateParticlePositions(const float deltaTime)
{
  // Get their velocities
  updateParticleVelocities();

  // Take a half step
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle   = m_particles[i];
    particle->oldPosition = particle->position;
    particle->position    = particle->position + particle->velocity*deltaTime*0.5;

    // Resolve collisions by moving the particle out of the solid cell
    Vector3i newIndices = positionToIndices(particle->position);
    if (m_cells[newIndices]->material == Material::Solid) {
      const Vector3i oldIndices = positionToIndices(particle->oldPosition);
      const Vector3i newToOld = oldIndices - newIndices;
      Vector3i normalEstimate = newToOld;
      for (int j = 0; j < 3; ++j) {
        if (m_cells[newIndices + normalEstimate]->material != Material::Fluid) {
          normalEstimate[j] = 0;
        }
      }
      particle->position = indicesToCenterPosition(newIndices + normalEstimate);
    }
  }

  // Get their velocities again
  updateParticleVelocities();

  // Take a full step from their original positions
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];
    particle->position  = particle->oldPosition + particle->velocity * deltaTime;
  }
}

// ================== Positional Helpers

// Given a position, regularizes it in the grid space
const Vector3f MacGrid::toRegularizedPosition(const Vector3f &position) const
{
  return (position - m_cornerPosition) / m_cellWidth;
}

// Given a position, returns the indices of the cell which would contain it
const Vector3i MacGrid::positionToIndices(const Vector3f &position) const
{
  const Vector3f regularizedPosition = toRegularizedPosition(position);

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
  // Iterate through particles
  for (Particle * const particle : particles) {

    const Vector3i cellIndices = positionToIndices(particle->position);
    auto kv = m_cells.find(cellIndices);

    // If cell does not exist
    if (kv == m_cells.end()) {
      
      // If cell is outside simulation bounds
      if (!withinBounds(cellIndices)) {
        cout << "particle outside bounds!" << endl;
        continue;
      }

      // Create the cell and put it in the hash table
      Cell * newCell = new Cell{};
      newCell->cellIndices = cellIndices;
      m_cells.insert({cellIndices, newCell});

      // Set its material
      newCell->material = material;

      continue;
    }

    // If cell does exist, and is not solid
    Cell * cell = kv->second;
    if (cell->material != Material::Solid) {
      cell->material = material;
    } else {
      cout << "particle in solid!" << endl;
    }
  }
}

void MacGrid::setCellMaterialLayers(const Material material, const int layer)
{
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    
    const Vector3i cellIndices = i->first;
    Cell * cell = i->second;

    // Skip if the cell is of the wrong material
    if (cell->material != material) continue;

    // Set layer
    cell->layer = layer;
  }
}

// Given a cell's indices, returns whether it falls within the simulation grid's bounds
bool MacGrid::withinBounds(const Vector3i &cellIndices) const
{
  return 0 <= cellIndices[0] && cellIndices[0] < m_cellCount[0] &&
      0 <= cellIndices[1] && cellIndices[1] < m_cellCount[1] &&
      0 <= cellIndices[2] && cellIndices[2] < m_cellCount[2];
}
