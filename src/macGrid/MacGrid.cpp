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

MacGrid::MacGrid(string folder)
{
#if SANITY_CHECKS
  assert(0 < cellWidth && 0 < cellCount[0] && 0 < cellCount[1] && 0 < cellCount[2]);
#endif

  QSettings settings(QString::fromStdString(folder + "/config.ini"), QSettings::IniFormat);
  
  settings.beginGroup("/Simulation");

  m_outputFolder = folder + "/particles";

  m_cellWidth = settings.value(QString("cellWidth")).toFloat();

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
   fillGridCellsFromInternalPosition(Material::Fluid, m_fluidInternalPosition);
  addParticlesToCells(Material::Fluid);
}

void MacGrid::simulate()
{
  float time = 0.0f;

  // const float framePeriod = 1 / 60.f;

  // Prepare to asynchronously save files
  vector<QFuture<void>> futures;

  // Start simulation loop
  while (time < m_simulationTime) {

    // Compute deltaTime or something
    const float deltaTime = calculateDeltaTime();

    // Given particle velocities, update grid velocities
    transferParticlesToGrid();

    // Calculate and apply external forces
    applyExternalForces(deltaTime);

    // Given particle positions, update cell materials, neighbors, and layers, then create buffer zone
    createBufferZone();

    // Given cells, neighbors, and cell velocities, update the velocity field by removing divergence
    updateVelocityFieldByRemovingDivergence();

    // Enforce DBC
    enforceDirichletBC();

    // Given old and new grid velocities, update particle positions using RK2
    updateParticlePositions(deltaTime);

    // Increment time
    time += deltaTime;

    // Todo: if some conditional is met, spit out the particles
    if (true) {
      const string filepath = m_outputFolder + "/" + to_string(time) + ".csv";
      vector<const Vector3f> particlePositions;
      for (Particle * const particle : m_particles) particlePositions.push_back(particle->position);
      QFuture<void> future = QtConcurrent::run(saveParticles, filepath, particlePositions);
      futures.push_back(future);
    }
  }

  // Wait for threads to finish writing particles
  for (QFuture<void> future : futures) future.waitForFinished();
}

// Sets up fluid cells and a buffer zone around them, based on the particles' positions
void MacGrid::createBufferZone()
{
  // Update grid cells that currently have fluid in them
  assignParticleCellMaterials(Material::Fluid, m_particles);

  // Set layer field of non-fluid cells to -1 and fluid cells to 0
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) kv->second->layer = -1;
  setCellMaterialLayers(Material::Fluid, 0);

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
        auto neighborKey = m_cells.find(neighborIndices);

        // If the neighbor cell does not exist
        if (neighborKey == m_cells.end()) {

          // Create the neighbor cell and put it in the hash table
          Cell * newNeighbor = new Cell{};
          newNeighbor->cellIndices = neighborIndices;
          m_cells.insert({neighborIndices, newNeighbor});
          
          // Set its material and layer
          newNeighbor->material = withinBounds(neighborIndices) ? Material::Air : Material::Solid;
          newNeighbor->layer = bufferLayer;

          continue;
        }

        // If the neighbor cell does exist, is not solid, and has layer = -1
        Cell * neighbor = neighborKey->second;
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

void MacGrid::printParticles(std::string output_filepath) {
    fstream fout;
    fout.open(output_filepath, ios::out);

    if (!fout.good()) {
      cerr << output_filepath << " could not be opened" << endl;
      return;
    }

    for (Particle * particle : m_particles) {
        Eigen::Vector3f particlePosition = particle->position;
        string toWrite =
            to_string(particlePosition[0]) + ", " +
            to_string(particlePosition[1]) + ", " +
            to_string(particlePosition[2]);
        fout << toWrite << endl;
    }

    std::cout << "Wrote to " << output_filepath << std::endl;
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
    if (particle->velocity.norm()>=maxV) {
      maxV = particle->velocity.norm();
    }
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

    // Set solid cells' velocities to zero
    if (cell->material == Material::Solid) {
      continue;
    }

    // Set fluid-solid boundary velocities to zero (this is safe due to the buffer layer around the fluid)
    if (m_cells[kv->first + Vector3i(-1, 0, 0)]->material == Solid) kv->second->ux = 0;
    if (m_cells[kv->first + Vector3i(0, -1, 0)]->material == Solid) kv->second->uy = 0;
    if (m_cells[kv->first + Vector3i(0, 0, -1)]->material == Solid) kv->second->uz = 0;
  }
}

// Sets the diverging component of the velocity field to zero
void MacGrid::updateVelocityFieldByRemovingDivergence()
{
  // Set up solver with preconditioning
  ConjugateGradient<SparseMatrix<float>, Lower|Upper, IncompleteCholesky<float>> m_solver;

  // Number all fluid cells
  int numFluidCells = 0;
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    if (i->second->material == Fluid) {
      i->second->index = numFluidCells;
      numFluidCells++;
    } else {
      i->second->index = -1;
    }
  }

  cout << "1" << endl;

  // Create A and b arrays
  SparseMatrix<float> A(numFluidCells, numFluidCells);
  vector<T> coefficients;
  VectorXf b(numFluidCells);

  // Fill A and b arrays by iterating over all fluid cells
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {

    // Skip if non-fluid
    const Vector3i cellIndices = i->first;
    const Cell * cell = i->second;
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
    divergence += (m_cells[cellIndices + Vector3i(1, 0, 0)]->ux) - cell->ux;
    divergence += (m_cells[cellIndices + Vector3i(0, 1, 0)]->uy) - cell->uy;
    divergence += (m_cells[cellIndices + Vector3i(0, 0, 1)]->uz) - cell->uz;
    b(cell->index, 0) = divergence;
  }
  A.setFromTriplets(coefficients.begin(), coefficients.end());


  cout << "2" << endl;

  // Solve for pseudo-pressures
  VectorXf pseudoPressures(m_cells.size());
  m_solver.compute(A);
  pseudoPressures = m_solver.solve(b);


  cout << "3" << endl;

  // Use pseudo-pressures to correct velocities
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {

    // Skip if non-fluid
    const Vector3i cellIndices = i->first;
    Cell * cell = i->second;
    if (cell->material != Fluid) continue;

    // Update velocity based on pseudopressure
    float xGradient = pseudoPressures[cell->index];
    if (m_cells[cellIndices + Vector3i(-1, 0, 0)]->material == Fluid) {
      xGradient -= pseudoPressures[m_cells[cellIndices + Vector3i(-1, 0, 0)]->index];
    }
    cell->ux -= xGradient;

    float yGradient = pseudoPressures[cell->index];
    if (m_cells[cellIndices + Vector3i(0, -1, 0)]->material == Fluid) {
      yGradient -= pseudoPressures[m_cells[cellIndices + Vector3i(0, -1, 0)]->index];
    }
    cell->uy -= yGradient;

    float zGradient = pseudoPressures[cell->index];
    if (m_cells[cellIndices + Vector3i(0, 0, -1)]->material == Fluid) {
      zGradient -= pseudoPressures[m_cells[cellIndices + Vector3i(0, 0, -1)]->index];
    }
    cell->uz -= zGradient;
  }


  cout << "4" << endl;

  // ================== Section 3f

  // Set layer field of non-fluid cells to -1 and fluid cells to 0
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) kv->second->layer = -1;
  setCellMaterialLayers(Material::Fluid, 0);


  cout << "5" << endl;

  // Iterate to extrapolate velocities
  for (int bufferLayer = 1; bufferLayer < max(2, (int) ceil(m_kCFL)); ++bufferLayer) {
    
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
        averageX += neighbor->ux;
        averageY += neighbor->uy;
        averageZ += neighbor->uz;
      }

      // Has neighbors of one lower layer
      if (count > 0) {
        averageX /= count;
        averageY /= count;
        averageZ /= count;

        // Set stuff
        cell->layer = bufferLayer;

        if (m_cells.find(cellIndices + Vector3i(-1, 0, 0)) != m_cells.end() && m_cells[cellIndices + Vector3i(-1, 0, 0)]->material == Material::Fluid)
          cell->ux = averageX;
        
        if (m_cells.find(cellIndices + Vector3i(0, -1, 0)) != m_cells.end() && m_cells[cellIndices + Vector3i(0, -1, 0)]->material == Material::Fluid)
          cell->uy = averageY;
        
        if (m_cells.find(cellIndices + Vector3i(0, 0, -1)) != m_cells.end() && m_cells[cellIndices + Vector3i(0, 0, -1)]->material == Material::Fluid)
          cell->uz = averageZ;
      }
    }
  }
}

// Sets the velocity field based on the particles' positions and velocities
void MacGrid::transferParticlesToGrid()
{
  // Zero out cells' particle numbers and average velocities
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    Cell * cell = i->second;
    cell->avgParticleV = Vector3f::Zero();
    cell->particleNums = 0;
  }

  // Accumulate cells' particle numbers and average velocities
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * particle = m_particles[i];
    Vector3f offset = Vector3f(0.5, 0.5, 0.5);
    Vector3i belongingCell = positionToIndices(particle->position+offset);
    m_cells[belongingCell]->avgParticleV = m_cells[belongingCell]->avgParticleV + particle->velocity;
    m_cells[belongingCell]->particleNums = m_cells[belongingCell]->particleNums + 1;
  }

  // Average velocities
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    Cell * cell = i->second;
    if (cell->particleNums > 0) {
      cell->avgParticleV /= cell->particleNums;
    }
  }
  
  // Do trilinear interpolation
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {

    // Skip non-fluid cells
    Cell * cell = i->second;
    if (cell->material != Material::Fluid) continue;

    Vector3i offset;
    Vector3f gridV;

    // Transfer particle velocity to grid
    for (int m = 0; m < 2; ++m) {
      for (int n = 0; n < 2; ++n) {
        for (int l = 0; l < 2; l++) {
          offset = Vector3i(m, n, l);
          gridV = gridV + 0.125 * m_cells[cell->cellIndices + offset]->avgParticleV;
        }
      }
    }

    // Save cell's velocities
    cell->ux = gridV[0];
    cell->uy = gridV[1];
    cell->uz = gridV[2];
    cell->oldUX = gridV[0];
    cell->oldUY = gridV[1];
    cell->oldUZ = gridV[2];
  }

  // For each non fluid cell
  // If cell has at least one fluid neighbor
  // Set velocity components of cell that are not bordering fluid cell to average of fluid cell neighbor velocities
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
    const Vector3f particlePos_original = particle->position;
    const Vector3i gridIdx = positionToIndices(particle->position);
    Vector3f particlePos = particlePos_original;
    Vector3i idx;
    float picX, picY, picZ = 0;
    float flipX, flipY, flipZ = 0;

    // Interpolate on X
    particlePos = particlePos_original - Vector3f(0, 0.5, 0.5);
    idx[0] = floor(particlePos[0]); // l
    idx[1] = floor(particlePos[1]); // m
    idx[2] = floor(particlePos[2]); // n
    Vector3f weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    // For 2x2 cell neighborhood
    for (int l = 0; l < 2; l++) {
      for (int m = 0; m < 2; m++) {
        for (int n = 0; n < 2; n++) {
          if (l == 1) weights[0] = particlePos[0] - idx[0];
          if (m == 1) weights[1] = particlePos[1] - idx[1];
          if (n == 1) weights[2] = particlePos[2] - idx[2];
          Vector3i offset = Vector3i(l, m, n);
          // Calculate PIC particle velocity
          picX = picX + weights[0] * weights[1] * weights[2] * m_cells[gridIdx+offset]->ux;
          // Calculate FLIP particle velocity
          flipX = flipX + weights[0] * weights[1] * weights[2] * (m_cells[gridIdx+offset]->ux - m_cells[gridIdx+offset]->oldUX);
        }
      }
    }

    // Interpolate on Y
    particlePos = particlePos_original-Vector3f(0.5, 0, 0.5);
    idx[0] = floor(particlePos[0]); // l
    idx[1] = floor(particlePos[1]); // m
    idx[2] = floor(particlePos[2]); // n
    weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    // For 2x2 cell neighborhood
    for (int l = 0; l < 2; l++) {
      for (int m = 0; m < 2; m++) {
        for (int n = 0; n < 2; n++) {
          if (l == 1) weights[0] = particlePos[0] - idx[0];
          if (m == 1) weights[1] = particlePos[1] - idx[1];
          if (n == 1) weights[2] = particlePos[2] - idx[2];
          Vector3i offset = Vector3i(l, m, n);
          // Calculate PIC particle velocity
          picY = picY + weights[0] * weights[2] * weights[3] * m_cells[gridIdx+offset]->uy;
          // Calculate FLIP particle velocity
          flipY = flipY + weights[0] * weights[2] * weights[3] * (m_cells[gridIdx+offset]->uy - m_cells[gridIdx+offset]->oldUY);
        }
      }
    }

    // Interpolate on Z
    particlePos = particlePos_original-Vector3f(0.5, 0.5, 0);
    idx[0] = floor(particlePos[0]); // l
    idx[1] = floor(particlePos[1]); // m
    idx[2] = floor(particlePos[2]); // n
    weights = Vector3f(idx[0]+1-particlePos[0], idx[1]+1-particlePos[1], idx[2]+1-particlePos[2]);
    // For 2x2 cell neighborhood
    for (int l = 0; l < 2; l++) {
      for (int m = 0; m < 2; m++) {
        for (int n = 0; n < 2; n++) {
          if (l == 1) weights[0] = particlePos[0] - idx[0];
          if (m == 1) weights[1] = particlePos[1] - idx[1];
          if (n == 1) weights[2] = particlePos[2] - idx[2];
          Vector3i offset = Vector3i(l, m, n);
          // Calculate PIC particle velocity
          picZ = picZ + weights[0]*weights[2]*weights[3]*m_cells[gridIdx+offset]->uz;
          // Calculate FLIP particle velocity
          flipZ = flipZ + weights[0]*weights[2]*weights[3]*(m_cells[gridIdx+offset]->uz - m_cells[gridIdx+offset]->oldUZ);
        }
      }
    }

    const Vector3f pic = Vector3f(picX, picY, picZ);
    const Vector3f flip = Vector3f(flipX, flipY, flipZ);

    // Update particle with interpolated PIC/FLIP velocities
    particle->velocity = m_interpolationCoefficient * pic + (1 - m_interpolationCoefficient) * (particle->velocity + flip);
  }
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

    // Todo: Resolve collisions by moving the particle out of the solid cell
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
        newCell->cellIndices = cellIndices;
        m_cells.insert({cellIndices, newCell});

        // Set its material and layer
        newCell->material = material;
      }

      continue;
    }

    // If cell does exist, and is not solid
    Cell * cell = kv->second;
    if (cell->material != Material::Solid) {
      cell->material = material;
    }
  }
}

void MacGrid::setCellMaterialLayers(const Material material, const int layer)
{
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); i++) {
    
    // Skip if incorrect material
    const Vector3i cellIndices = i->first;
    Cell * cell = i->second;
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
