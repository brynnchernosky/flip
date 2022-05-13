#include <iostream>
#include <fstream>
#include <QFile>
#include <random>
#include <set>

#include "src/macGrid/MacGrid.h"
#include "src/graphics/MeshLoader.h"
#include "src/Debug.h"

using namespace std;
using namespace Eigen;

// ================== Local Helpers

typedef Triplet<float> T;

const vector<const Vector3i> NEIGHBOR_OFFSETS = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
const vector<const Vector3f> INTERP_OFFSETS = {{0, 0.5, 0.5}, {0.5, 0, 0.5}, {0.5, 0.5, 0}};
const int BUFFER_LAYERS = 3;

// Produces a random float in the range [0, 1]
inline float getRandomFloat()
{
  return static_cast <float> (arc4random()) / static_cast <float> (UINT32_MAX);
}

// Gets a uniformly random position on a given triangle defined by a point and two vectors
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

// ================== Constructor

MacGrid::MacGrid(string folder)
{
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
  m_otherCornerPosition = m_cornerPosition + m_cellCount.cast<float>() * m_cellWidth;

  m_solidMeshFilepath = folder + "/solid.obj";
  m_fluidMeshFilepath = folder + "/fluid.obj";

  m_solidInternalPosition = Vector3f(settings.value(QString("solidInternalPositionX")).toFloat(),
                                     settings.value(QString("solidInternalPositionY")).toFloat(),
                                     settings.value(QString("solidInternalPositionZ")).toFloat());
  m_fluidInternalPosition = Vector3f(settings.value(QString("fluidInternalPositionX")).toFloat(),
                                     settings.value(QString("fluidInternalPositionY")).toFloat(),
                                     settings.value(QString("fluidInternalPositionZ")).toFloat());

  m_solidTransformation = Affine3f(
                            Translation3f(settings.value(QString("solidTranslationX")).toFloat(),
                                          settings.value(QString("solidTranslationY")).toFloat(),
                                          settings.value(QString("solidTranslationZ")).toFloat()) *
                            Scaling      (settings.value(QString("solidScaleX")).toFloat(),
                                          settings.value(QString("solidScaleY")).toFloat(),
                                          settings.value(QString("solidScaleZ")).toFloat()) *
                            AngleAxis<float>(settings.value(QString("solidRotationX")).toFloat() / 180.0f * M_PI, Vector3f::UnitX()) *
                            AngleAxis<float>(settings.value(QString("solidRotationY")).toFloat() / 180.0f * M_PI, Vector3f::UnitY()) *
                            AngleAxis<float>(settings.value(QString("solidRotationZ")).toFloat() / 180.0f * M_PI, Vector3f::UnitZ()));
  m_fluidTransformation = Affine3f(
                            Translation3f(settings.value(QString("fluidTranslationX")).toFloat(),
                                          settings.value(QString("fluidTranslationY")).toFloat(),
                                          settings.value(QString("fluidTranslationZ")).toFloat()) *
                            Scaling      (settings.value(QString("fluidScaleX")).toFloat(),
                                          settings.value(QString("fluidScaleY")).toFloat(),
                                          settings.value(QString("fluidScaleZ")).toFloat()) *
                            AngleAxis<float>(settings.value(QString("fluidRotationX")).toFloat() / 180.0f * M_PI, Vector3f::UnitX()) *
                            AngleAxis<float>(settings.value(QString("fluidRotationY")).toFloat() / 180.0f * M_PI, Vector3f::UnitY()) *
                            AngleAxis<float>(settings.value(QString("fluidRotationZ")).toFloat() / 180.0f * M_PI, Vector3f::UnitZ()));

  m_framePeriod = settings.value(QString("framePeriod")).toFloat();
  m_minCFLTime = m_framePeriod * 0.011;
  m_maxCFLTime = m_framePeriod * 0.501;

  m_simulationTime = settings.value(QString("simulationTime")).toFloat();
  m_gravityVector = Vector3f(settings.value(QString("gravityX")).toFloat(),
                             settings.value(QString("gravityY")).toFloat(),
                             settings.value(QString("gravityZ")).toFloat());

  m_interpolationCoefficient = settings.value(QString("interpolationCoefficient")).toFloat();

  // Extension settings

  m_addFluid = settings.value(QString("addFluid"), false).toBool();
  m_fluidVelocity = Vector3f(settings.value(QString("fluidVelocityX")).toFloat(),
                             settings.value(QString("fluidVelocityY")).toFloat(),
                             settings.value(QString("fluidVelocityZ")).toFloat());
  m_fluidIndices = Vector3i(settings.value(QString("fluidX")).toInt(),
                         settings.value(QString("fluidY")).toInt(),
                         settings.value(QString("fluidZ")).toInt());
  m_fluidSize = settings.value(QString("fluidSize")).toInt();

  m_foamParticleBoundary = settings.value(QString("foamParticleBoundary")).toFloat();
  
  settings.endGroup();
}

// ================== Destructor

MacGrid::~MacGrid()
{
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) delete kv->second;
  for (Particle * particle : m_particles)                    delete particle;
  for (Particle * particle : m_solidSurfaceParticles)        delete particle;
  for (Particle * particle : m_fluidSurfaceParticles)        delete particle;
}

// ================== Initialization

void MacGrid::init()
{
  if (m_addFluid) {
    cout << "Skipping initialization" << endl;
    return;
  }

  cout << "Starting initialization" << endl;

  // Solid
  getSurfaceParticlesFromMesh(m_solidSurfaceParticles, m_solidMeshFilepath, m_solidTransformation);
  setCellsBasedOnParticles(Material::Solid, m_solidSurfaceParticles, false);
  setCellLayerBasedOnMaterial(); // By doing this first, only these solid cells will have layer = -2
  fillCellsFromInternalPosition(Material::Solid, m_solidTransformation * m_solidInternalPosition);
  propagateSolidNormals();

  unsigned int count = 0, count2 = 0;
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    if (!withinBounds(kv->first)) continue;
    // cout << Debug::cellToString(kv->second) << endl;
    if (kv->second->material == Material::Solid) {
      ++count;
      if (kv->second->layer == -2) ++count2;
    }
  }
  cout << count << " solid cells, of which " << count2 << " have layer = -2" << endl;

  // Fluid
  getSurfaceParticlesFromMesh(m_fluidSurfaceParticles, m_fluidMeshFilepath, m_fluidTransformation);
  setCellsBasedOnParticles(Material::Fluid, m_fluidSurfaceParticles, false);
  fillCellsFromInternalPosition(Material::Fluid, m_fluidTransformation * m_fluidInternalPosition);
  spawnParticlesInFluidCells();
}

// ================== Simulation

void MacGrid::simulate()
{
  cout << "Starting simulation for " << m_simulationTime << " seconds" << endl;

  // Prepare to asynchronously save files
  vector<QFuture<void>> futures;

  // Count iterations
  float time = 0.f, nextSaveTime = m_framePeriod;
  int saveNumber = 0, loopNumber = 0;
  bool mustSave = false;

  // Given particle positions, update the dynamic grid
  updateGridExcludingVelocity();
  cout << "∟ updated grid (" << m_cells.size() << " cells)" << endl;

  // Save original state
  futures.push_back(saveParticlesToFile(time, false));
  futures.push_back(saveParticlesToFile(time, true));

  // Start simulation loop
  while (time < m_simulationTime) {

    cout << "================== Starting loop " << loopNumber << " at time = " << time << endl;

    // Given particle velocities, calculate the timestep that can be taken while obeying the CFL condition
    float deltaTime = calculateCFLTime();
    cout << "∟ calculated timestep of " << deltaTime << endl;
    if (deltaTime + time > nextSaveTime) {
      deltaTime = nextSaveTime - time + __FLT_EPSILON__ + __FLT_EPSILON__ + __FLT_EPSILON__ + __FLT_EPSILON__;
      mustSave = true;
      cout << "∟ set timestep to " << deltaTime << endl;
    }

    // Given particle velocities, update the velocity field (grid cell velocities)
    updateGridVelocity();
    cout << "∟ updated velocity field (from " << m_particles.size() << " particles)" << endl;

    // Given grid cells' velocities, save a copy of the velocity field for FLIP calculations
    saveCopyOfGridVelocity();
    cout << "∟ saved copy of velocity field" << endl;

    // Given grid cells' velocities, apply external forces to the velocity field
    applyExternalForces(deltaTime);
    cout << "∟ applied external forces" << endl;

    // Given grid cells' velocities, enforce the Neumann boundary condition to prevent flow from air/fluid cells into solid cells
    enforceBoundaryConditions();
    cout << "∟ enforced boundary conditions" << endl;

    // Given grid cells' velocities, solve for pressure and remove divergence from the velocity field
    updateGridVelocityByRemovingDivergence();
    cout << "∟ updated velocity field by removing divergence" << endl;

    // Given grid cells' velocities (old and new), particle positions, and particle velocities, update particle positions with RK2
    updateParticlePositions(deltaTime, m_particles);
    cout << "∟ updated particle positions" << endl;

    // Given particle positions, update the dynamic grid
    updateGridExcludingVelocity();
    cout << "∟ updated grid (" << m_cells.size() << " cells)" << endl;

    // Increment time
    if (mustSave) {
      mustSave = false;

      // // Add additional fluid to simulation at specified position and size
      // if (m_addFluid) {
      //     addFluid(m_fluidIndices[0], m_fluidIndices[1], m_fluidIndices[2], m_fluidSize);
      //     addFluid(m_fluidIndices[0]+10, m_fluidIndices[1]+10, m_fluidIndices[2], m_fluidSize-2);
      //     addFluid(m_fluidIndices[0]-10, m_fluidIndices[1]+10, m_fluidIndices[2], m_fluidSize-2);
      //     addFluid(m_fluidIndices[0]+10, m_fluidIndices[1]-10, m_fluidIndices[2], m_fluidSize-2);
      //     addFluid(m_fluidIndices[0]-10, m_fluidIndices[1]-10, m_fluidIndices[2], m_fluidSize-2);
      // }
      // if (saveNumber == 8*10) {
      //     convertToFluid(0,49,0,49,0,49);
      // }

      // Set fluid particles to foam particles in voxels where curl is above m_foamParticleBoundary, only relevant for visualization
      assignFoamParticles();

      // Set time to current nextSaveTime
      time = nextSaveTime;

      // Save the particles to a file
      futures.push_back(saveParticlesToFile(time, false));
      futures.push_back(saveParticlesToFile(time, true));
      cout << "∟ started saving " << m_particles.size() << " particles (frame number " << saveNumber << ")" << endl;

      // Increase nextSaveTime
      ++saveNumber;
      nextSaveTime = saveNumber * m_framePeriod;
    } else {
      time += deltaTime;
    }

    // Increment the loop number
    ++loopNumber;
  }

  // Wait for threads to finish writing particles
  for (QFuture<void> future : futures) future.waitForFinished();
}

// ================== Debugging

// Debugging only: prints the grid
void MacGrid::printGrid() const
{
  if (m_cells.size() == 0) {
    cerr << "No cells." << endl;
    return;
  }
  int fluidCount = 0;
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    if (kv->second->material == Material::Fluid) fluidCount++;
    cout << Debug::cellToString(kv->second) << endl;
  }
  cout << "Total cells = " << m_cells.size() << ", of which " << fluidCount << " are fluid" << endl;
}

// Debugging only: prints the particles
void MacGrid::printParticles() const
{
  if (m_particles.size() == 0) {
    cerr << "No particles." << endl;
    return;
  }
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    cout << i << ": "<< Debug::particleToString(m_particles[i]) << endl;
  }
  cout << "Total particles = " << m_particles.size() << endl;
}

// ========================================================================
// ================== Initialization Helpers ==============================
// ========================================================================

// Populates a given vector with particles derived from the surface of the given mesh
void MacGrid::getSurfaceParticlesFromMesh(vector<Particle *> &surfaceParticles, const string meshFilepath, const Affine3f &transform)
{
  vector<Vector3f> vertices, normals;
  vector<Vector3i> faces;

  // Load mesh (panic if failed)
  if (!MeshLoader::loadTriMesh(meshFilepath, vertices, normals, faces)) {
    cerr << "MacGrid failed to load mesh \"" << meshFilepath << "\". Exiting!" << endl;
    exit(1);
  }
  cout << "Successfully loaded mesh: \"" << meshFilepath << "\"." << endl;

  // // Spawn particles on the mesh's vertices
  // for (const Vector3f &vertexPosition : vertices) {
  //   Particle * newParticle = new Particle(transform * vertexPosition, whatNormalDudeIdk);
  //   surfaceParticles.push_back(newParticle);
  // }

  // Spawn particles on the mesh's faces
  for (unsigned int i = 0; i < faces.size(); ++i) {

    const Vector3i &face   = faces[i];

    const Vector3f a = transform * vertices[face[0]];
    const Vector3f b = transform * vertices[face[1]];
    const Vector3f c = transform * vertices[face[2]];
    const Vector3f ab = b - a;
    const Vector3f ac = c - a;
    const float abNorm = ab.norm();
    const float acNorm = ac.norm();

    const Vector3f cross = ab.cross(ac);
    const Vector3f normal = cross.normalized();
    assert(normal.norm() > 0.99999);
    const float faceArea = cross.norm() / 2;

    const float numParticles = faceArea * m_maxAverageSurfaceParticlesPerArea;
    const float acAbRatio = acNorm / abNorm;
    const float temp = sqrt(numParticles / acAbRatio);
    const int abStrata = (int) (temp);
    const int acStrata = (int) (acAbRatio * temp);

    // Use stratified sampling
    for (int abStratum = 0; abStratum < abStrata; ++abStratum) {
      for (int acStratum = 0; acStratum < acStrata; ++acStratum) {

        // Get position within triangle face
        float abWeight = (abStratum + getRandomFloat()) / abStrata;
        float acWeight = (acStratum + getRandomFloat()) / acStrata;
        if (abWeight + acWeight > 1) {
          abWeight = 1 - abWeight;
          acWeight = 1 - acWeight;
        }
        const Vector3f surfacePosition = a + abWeight * ab + acWeight * ac;

        // Save particle with normal as velocity
        Particle * newParticle = new Particle(surfacePosition, normal);
        surfaceParticles.push_back(newParticle);
      }
    }
  }
}

struct Vector3iComparator
{
  bool operator() (const Vector3i &lhs, const Vector3i &rhs) const
  {
    if (lhs[0] != rhs[0]) return lhs[0] < rhs[0];
    if (lhs[1] != rhs[1]) return lhs[1] < rhs[1];
    return lhs[2] < rhs[2];
  }
};

// Densely fills grid cells with the given material, starting from a given internal position
void MacGrid::fillCellsFromInternalPosition(const Material material, const Vector3f &internalPosition)
{
  unsigned int count = 0;
  cout << "internal position " << internalPosition << endl;

  const Vector3i internalIndices = positionToIndices(internalPosition);
  cout << "Filling cells from internal position: " << Debug::vectorToString(internalPosition)
      << ", at " << Debug::vectorToString(internalIndices) << endl;
  assert(withinBounds(internalIndices));

  // Check that it a cell doesn't already exist here; if it does, it must be of the specified material
  if (m_cells.find(internalIndices) != m_cells.end()) {
    cout << Debug::vectorToString(internalPosition) << endl;
    cout << Debug::vectorToString(internalIndices) << endl;
    cout << Debug::cellToString(m_cells.at(internalIndices)) << endl;
    assert(m_cells.find(internalIndices)->second->material == material);
    cerr << "Did not fill cells from internal position as specified cell was already filled!" << endl;
    return;
  }

  // Create a cell here and put it in the hash table
  Cell * internalCell = new Cell(material, internalIndices);
  m_cells.insert({internalIndices, internalCell});

  // Fill neighbors (breadth-first, to avoid depth limit)
  set<Vector3i, Vector3iComparator> oldFilledIndices = {internalIndices};
  set<Vector3i, Vector3iComparator> newFilledIndices = {};
  Vector3i neighborIndices;
  while (oldFilledIndices.size() != 0) {

    // For each previously filled index
    for (const Vector3i &indices : oldFilledIndices) {

      // For each neighbor index
      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
        neighborIndices = indices + neighborOffset;

        // Check that it isn't outside of bounds
        if (!withinBounds(neighborIndices)) continue;

        // Check that it a cell doesn't already exist here; if it does, it must be of the specified material
        if (m_cells.find(neighborIndices) != m_cells.end()) {
          assert(m_cells.find(neighborIndices)->second->material == material);
          continue;
        }

        // Create a cell here and put it in the hash table
        count++;
        Cell * newNeighbor = new Cell(material, neighborIndices);
        m_cells.insert({neighborIndices, newNeighbor});

        // Add it to the set
        newFilledIndices.insert(neighborIndices);
      }
    }

    // Swap
    oldFilledIndices.swap(newFilledIndices);
    newFilledIndices.clear();
  }
}

// Assuming that the outermost solid cells have layer = -2 and the rest have layer = -1, propagates normals inward into the solid
void MacGrid::propagateSolidNormals()
{ 
  vector<const Vector3i> iterCellIndices;
  iterCellIndices.reserve(m_cells.size());

  // Create a buffer zone inward, into the solid
  for (int bufferLayer = -3; bufferLayer > -5; --bufferLayer) {
    const int bufferLayerAdd1 = bufferLayer + 1;

    // Save cells which are of the appropriate layer, and are solid
    for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
      Cell * const cell = kv->second;
      if (cell->layer == bufferLayerAdd1 && cell->material == Material::Solid) {
        iterCellIndices.push_back(kv->first);
      }
    }

    // Iterate through those saved cells; these are the ones of the appropriate layer
    for (const Vector3i &cellIndices : iterCellIndices) {

      Cell * cell = m_cells[cellIndices];

      // For each of its six neighbor cells
#pragma omp parallel for
      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
        const Vector3i neighborIndices = cellIndices + neighborOffset;
        auto neighborKV = m_cells.find(neighborIndices);

        // Skip if the neighbor cell does not exist (this must be outside the solid)
        if (neighborKV == m_cells.end()) continue;

        // If the neighbor cell does exist
        Cell * neighbor = neighborKV->second;

        // Skip if the neighbor cell is of the same layer
        if (neighbor->layer == bufferLayerAdd1) continue;

        // If the neighbor's layer hasn't been set yet
        if (neighbor->layer == -1) {
          neighbor->layer = bufferLayer;
        }

        // Add to its normal
        neighbor->normal += cell->normal;
      }
    }

    // Clear the saved cells
    iterCellIndices.clear();
  }

  // Delete unnecessary cells
  auto kv = m_cells.cbegin();
  while (kv != m_cells.cend()) {

    // Do not delete non-solid cells or cells with layer != -1
    if (kv->second->material != Material::Solid || kv->second->layer != -1) {
      ++kv;
      continue;
    }

    delete kv->second;
    kv = m_cells.erase(kv);
  }

  // Fix the normals
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    
    Cell * cell = kv->second;

    // Skip non-solid cells (there shouldn't be any, but better to be safe)
    if (cell->material != Material::Solid) continue;


    if (cell->layer < 2) cell->normal.normalize();
  }
}

// Given a material, populates all cells with that material with particles using stratified sampling
void MacGrid::spawnParticlesInFluidCells()
{
  // m_strata is the number of subdivisions per side, such that there are strata^3 subcells per cell
  const float strataWidth = m_cellWidth / m_strata;

  // For each cell
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    const Vector3i &cellIndices = kv->first;
    Cell * cell = kv->second;

    // Skip if the cell is not fluid
    if (cell->material != Material::Fluid) continue;

    // Do stratified sampling
    for (int x = 0; x < m_strata; ++x) {
      for (int y = 0; y < m_strata; ++y) {
        for (int z = 0; z < m_strata; ++z) {

          // Create a new particle
          Particle * newParticle = new Particle();

          // Repeat to prevent particle from falling in neighboring cell instead of this cell
          do {
            const float a = getRandomFloat(), b = getRandomFloat(), c = getRandomFloat();
            newParticle->position = indicesToBasePosition(cellIndices) + Vector3f(x + a, y + b, z + c) * strataWidth;
          } while (positionToIndices(newParticle->position) != cellIndices);

          // Save the particle
          m_particles.push_back(newParticle);
        }
      }
    }
  }

  cout << "Spawned " << m_particles.size() << " marker particles" << endl;
}

// ========================================================================
// ================== Simulation Helpers ==================================
// ========================================================================

// Calculates the timestep that can be taken while obeying the CFL condition
float MacGrid::calculateCFLTime() const
{
  float maxSpeed = 0;
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    Particle * const particle = m_particles[i];
    float speed = particle->velocity.norm();
    if (speed > maxSpeed) maxSpeed = speed;
  }
  maxSpeed += __FLT_EPSILON__;

  const float returnValue = m_cellWidth / maxSpeed;
  if (returnValue < m_minCFLTime) {
    cerr << "Warning! very low CFL time of " << returnValue << " was clamped to " << m_minCFLTime << endl;
    return m_minCFLTime;
  }
  return min(returnValue, m_maxCFLTime);
}

// Updates the dynamic grid:
// - Sets the material field of cells containing marker particles to Material::Fluid
// - Creates a buffer zone around fluid cells
// - Deletes unnecessary cells
void MacGrid::updateGridExcludingVelocity()
{
  // Set all fluid cells to air
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    if (kv->second->material == Material::Fluid) {
      kv->second->material = Material::Air;
    }
  }

  // Assign all grid cells that currently have marker particles in them
  setCellsBasedOnParticles(Material::Fluid, m_particles, true);

  // Set layer field of non-fluid cells to -1 and fluid cells to 0
  setCellLayerBasedOnMaterial();

  // Save cells to iterate through
  vector<const Vector3i> iterCellIndices;
  iterCellIndices.reserve(m_cells.size());

  // Create a buffer zone around the fluid
  for (int bufferLayer = 1; bufferLayer < BUFFER_LAYERS; ++bufferLayer) {
    const int bufferLayerSub1 = bufferLayer - 1;

    // Save cells which are of the appropriate layer, and are not solid
    for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
      Cell * const cell = kv->second;
      if (cell->layer == bufferLayerSub1 && cell->material != Material::Solid) {
        iterCellIndices.push_back(kv->first);
      }
    }

    // Iterate through those saved cells
    for (const Vector3i &cellIndices : iterCellIndices) {

      // For each of its six neighbor cells
      for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {
        const Vector3i neighborIndices = cellIndices + neighborOffset;
        auto neighborKV = m_cells.find(neighborIndices);

        // If the neighbor cell does not exist
        if (neighborKV == m_cells.end()) {

          // Create the neighbor cell and put it in the hash table
          Cell * newNeighbor = new Cell();
          m_cells.insert({neighborIndices, newNeighbor});

          // Set its fields
          newNeighbor->material    = withinBounds(neighborIndices) ? Material::Air : Material::Solid;
          newNeighbor->cellIndices = neighborIndices;
          newNeighbor->layer       = bufferLayer;

          continue;
        }

        // If the neighbor cell does exist
        Cell * neighbor = neighborKV->second;

        // If the neighbor's layer and material haven't been set yet
        if (neighbor->layer == -1 && neighbor->material != Material::Solid) {
          neighbor->material = Material::Air;
          neighbor->layer = bufferLayer;
        }
      }
    }

    // Clear the saved cells
    iterCellIndices.clear();
  }

  // Delete unnecessary cells
  auto kv = m_cells.cbegin();
  while (kv != m_cells.cend()) {

    // Do not delete solid cells or cells with layer != -1
    if (kv->second->material == Material::Solid || kv->second->layer != -1) {
      ++kv;
      continue;
    }

    delete kv->second;
    kv = m_cells.erase(kv);
  }
}

// Updates the velocity field (grid cell velocities)
void MacGrid::updateGridVelocity()
{
  // Set all grid cell velocities and weights (for weighted averaging) to zero
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    Cell * cell    = kv->second;
    cell->u        = Vector3f::Zero();
    cell->uWeights = Vector3f::Zero();
  }

  // For all particles, contribute to relevant cells
  for (Particle * const particle : m_particles) {

    const Vector3f &velocity = particle->velocity;
    const Vector3f &position = particle->position;
    const Vector3f regularizedPosition = toRegularizedPosition(position);

    contributeToCells(velocity, regularizedPosition - Vector3f(0, 0.5, 0.5), 0);
    contributeToCells(velocity, regularizedPosition - Vector3f(0.5, 0, 0.5), 1);
    contributeToCells(velocity, regularizedPosition - Vector3f(0.5, 0.5, 0), 2);
  }

  // For all cells, divide their accumulated velocities by the accumulated weights
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    Cell * cell = kv->second;
    if (cell->uWeights[0] != 0) cell->u[0] /= cell->uWeights[0];
    if (cell->uWeights[1] != 0) cell->u[1] /= cell->uWeights[1];
    if (cell->uWeights[2] != 0) cell->u[2] /= cell->uWeights[2];
  }
}

// Helper to add particles' weighted velocities to neighboring cells 
void MacGrid::contributeToCells(const Vector3f &velocity, const Vector3f &xyz, int index) const
{
  const Vector3i ijk(floor(xyz[0]), floor(xyz[1]), floor(xyz[2]));

  for (int l = 0; l < 2; ++l) {
    for (int m = 0; m < 2; ++m) {
      for (int n = 0; n < 2; ++n) {

        const Vector3i offset(l, m, n);
        const Vector3i neighborIndices = ijk + offset;

        // Skip if cell does not exist
        if (m_cells.find(neighborIndices) == m_cells.end()) {
          // This is actually fine, due to cells at the bound corners
          continue;
        }
        Cell * const cell = m_cells.at(neighborIndices);

        // Get weights
        Vector3f weights = ijk.cast<float>() + Vector3f::Ones() - xyz;
        if (l == 1) weights[0] = xyz[0] - ijk[0];
        if (m == 1) weights[1] = xyz[1] - ijk[1];
        if (n == 1) weights[2] = xyz[2] - ijk[2];

        // Accumulate
        const float weight = weights[0] * weights[1] * weights[2];
        cell->uWeights[index] += weight;
        cell->u[index] += weight * velocity[index];
      }
    }
  }
}

// Saves a copy of the velocity field for FLIP calculations
void MacGrid::saveCopyOfGridVelocity()
{
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    kv->second->oldU = kv->second->u;
  }
}

// Applies external forces (gravity) to the velocity field
void MacGrid::applyExternalForces(const float deltaTime)
{
  const Vector3f gravityDeltaV = m_gravityVector * deltaTime;

#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    kv->second->u += gravityDeltaV;
  }
}

// Enforces the Neumann boundary condition to prevent flow from air/fluid cells into solid cells
void MacGrid::enforceBoundaryConditions()
{
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    const Vector3i &cellIndices = kv->first;
    Cell * cell = kv->second;

    // Skip if cell is not solid
    if (cell->material != Material::Solid) continue;

    // Prevent flow into solid cells from air and fluid cells

    auto neighborKV = m_cells.find(cellIndices + Vector3i(-1, 0, 0));
    if (neighborKV != m_cells.end() && neighborKV->second->material != Material::Solid) cell->u[0] = min(0.f, cell->u[0]);

    neighborKV = m_cells.find(cellIndices + Vector3i(0, -1, 0));
    if (neighborKV != m_cells.end() && neighborKV->second->material != Material::Solid) cell->u[1] = min(0.f, cell->u[1]);

    neighborKV = m_cells.find(cellIndices + Vector3i(0, 0, -1));
    if (neighborKV != m_cells.end() && neighborKV->second->material != Material::Solid) cell->u[2] = min(0.f, cell->u[2]);

    neighborKV = m_cells.find(cellIndices + Vector3i(1, 0, 0));
    if (neighborKV != m_cells.end() && neighborKV->second->material != Material::Solid) neighborKV->second->u[0] = max(0.f, neighborKV->second->u[0]);

    neighborKV = m_cells.find(cellIndices + Vector3i(0, 1, 0));
    if (neighborKV != m_cells.end() && neighborKV->second->material != Material::Solid) neighborKV->second->u[1] = max(0.f, neighborKV->second->u[1]);

    neighborKV = m_cells.find(cellIndices + Vector3i(0, 0, 1));
    if (neighborKV != m_cells.end() && neighborKV->second->material != Material::Solid) neighborKV->second->u[2] = max(0.f, neighborKV->second->u[2]);
  }
}

// Solves for pressure and remove divergence from the velocity field
void MacGrid::updateGridVelocityByRemovingDivergence()
{
  // Set up solver with preconditioning
  ConjugateGradient<SparseMatrix<float>, Lower|Upper, IncompleteCholesky<float>> m_solver;

  // Number all fluid cells
  int numFluidCells = 0;
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    if (kv->second->material == Fluid) {
      kv->second->index = numFluidCells;
      numFluidCells++;
    } else {
      kv->second->index = -1;
    }
  }
  if (numFluidCells == 0) return;

  // Create A and b arrays
  SparseMatrix<float> A(numFluidCells, numFluidCells);
  vector<T> coefficients;
  VectorXf b(numFluidCells);

  // Fill A and b arrays by iterating over all fluid cells
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    const Vector3i &cellIndices = kv->first;
    const Cell * cell = kv->second;

    // Skip if the cell is non-fluid
    if (cell->material != Material::Fluid) continue;

    // Fill this row of the A matrix (coefficients)
    float centerCoefficient = -6;
    for (const Vector3i &neighborOffset : NEIGHBOR_OFFSETS) {

      if (m_cells.find(cellIndices + neighborOffset) == m_cells.end()) {
        printGrid();
        cout << endl << Debug::vectorToString(cellIndices) << endl << endl;
        cout << endl << Debug::vectorToString(neighborOffset) << endl << endl;
        assert(false);
      }

      if (m_cells[cellIndices + neighborOffset]->material == Solid) {
        centerCoefficient += 1;
      } else if (m_cells[cellIndices + neighborOffset]->material == Fluid) {
        coefficients.push_back(T(cell->index, m_cells[cellIndices + neighborOffset]->index, 1));
      }
    }
    coefficients.push_back(T(cell->index, cell->index, centerCoefficient));

    // Fill this row of the b matrix (divergences) (at this point, velocities into solids should be 0)
    float divergence = 0;
    divergence += (m_cells[cellIndices + Vector3i(1, 0, 0)]->u[0]) - cell->u[0];
    divergence += (m_cells[cellIndices + Vector3i(0, 1, 0)]->u[1]) - cell->u[1];
    divergence += (m_cells[cellIndices + Vector3i(0, 0, 1)]->u[2]) - cell->u[2];
    b(cell->index, 0) = divergence;
  }
  A.setFromTriplets(coefficients.begin(), coefficients.end());

  // Solve for pseudo-pressures
  VectorXf pseudoPressures(numFluidCells);
  m_solver.compute(A);
  pseudoPressures = m_solver.solve(b);

  // Use pseudo-pressures to correct velocities
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    const Vector3i &cellIndices = kv->first;
    Cell * cell = kv->second;

    // Skip if the cell is solid
    if (cell->material == Material::Solid) continue;

    // Update velocity based on pseudopressure

    const float cellPseudoPressure = cell->material == Material::Air ? 0 : pseudoPressures[cell->index];

    auto xMinKV = m_cells.find(cellIndices + Vector3i(-1, 0, 0));
    if (xMinKV != m_cells.end() && xMinKV->second->material != Material::Solid) {
      cell->u[0] -= cellPseudoPressure;
      if (xMinKV->second->material == Material::Fluid) cell->u[0] += pseudoPressures[xMinKV->second->index];
    }

    auto yMinKV = m_cells.find(cellIndices + Vector3i(0, -1, 0));
    if (yMinKV != m_cells.end() && yMinKV->second->material != Material::Solid) {
      cell->u[1] -= cellPseudoPressure;
      if (yMinKV->second->material == Material::Fluid) cell->u[1] += pseudoPressures[yMinKV->second->index];
    }

    auto zMinKV = m_cells.find(cellIndices + Vector3i(0, 0, -1));
    if (zMinKV != m_cells.end() && zMinKV->second->material != Material::Solid) {
      cell->u[2] -= cellPseudoPressure;
      if (zMinKV->second->material == Material::Fluid) cell->u[2] += pseudoPressures[zMinKV->second->index];
    }
  }
}

// Updates particle positions with RK2
void MacGrid::updateParticlePositions(const float deltaTime, const vector<Particle *> &particles)
{
  // Save old positions and velocities
#pragma omp parallel for
  for (unsigned int i = 0; i < particles.size(); ++i) {
    Particle * particle   = particles[i];
    particle->oldPosition = particle->position;
    particle->oldVelocity = particle->velocity;
  }

  // Get their new velocities (relies on current position and old velocity)
  updateParticleVelocities(particles);

  // Take a half step
#pragma omp parallel for
  for (unsigned int i = 0; i < particles.size(); ++i) {
    Particle * particle = particles[i];
    particle->position  = particle->position + particle->velocity * deltaTime / 2;
  }
  resolveParticleCollisions(particles);

  // Get their new velocities again (relies on current position and old velocity)
  updateParticleVelocities(particles);

  // Take a full step from their old positions
#pragma omp parallel for
  for (unsigned int i = 0; i < particles.size(); ++i) {
    Particle * particle = particles[i];
    particle->position  = particle->oldPosition + particle->velocity * deltaTime;
  }
  resolveParticleCollisions(particles);
}

// Helper to updates particle velocities by interpolating PIC and FLIP velocities
void MacGrid::updateParticleVelocities(const vector<Particle *> &particles)
{
  // For every particle
#pragma omp parallel for
  for (unsigned int particleIndex = 0; particleIndex < particles.size(); ++particleIndex) {

    Particle * particle = particles[particleIndex];

    Vector3f pic  = Vector3f::Zero();
    Vector3f flip = Vector3f::Zero();

    const Vector3f &position = particle->position;
    const Vector3f regularizedPosition = toRegularizedPosition(position);

    // For each component of velocity
    for (unsigned int index = 0; index < 3; ++index) {

      // Get the regularized position to interpolate this component of velocity from
      const Vector3f xyz = regularizedPosition - INTERP_OFFSETS[index];

      // Get PIC and FLIP values from grid
      pair<float, float> pf = getInterpolatedPICAndFLIP(xyz, index);
      pic[index]  = pf.first;
      flip[index] = pf.second;
    }

    // Update particle with interpolated PIC/FLIP velocities
    particle->velocity = m_interpolationCoefficient * pic + (1 - m_interpolationCoefficient) * (particle->oldVelocity + flip);
  }
}

// Helper to get the interpolated PIC and FLIP velocities for a given regularized and offset position
pair<float, float> MacGrid::getInterpolatedPICAndFLIP(const Vector3f &xyz, const int index) const
{
  float p = 0, f = 0, weightSum = 0;

  // Get stuff for weight calculation
  const Vector3i ijk = Vector3i(floor(xyz[0]), floor(xyz[1]), floor(xyz[2]));

  // For a 2x2 cell neighborhood around xyz
  for (int l = 0; l < 2; ++l) {
    for (int m = 0; m < 2; ++m) {
      for (int n = 0; n < 2; ++n) {

        // Get cell
        const Vector3i offset(l, m, n);
        const Vector3i ijkOffset = ijk + offset;

        // Skip if cell does not exist
        if (m_cells.find(ijkOffset) == m_cells.end()) continue;
        Cell * const cell = m_cells.at(ijkOffset);

        // Get weights
        Vector3f weights = ijk.cast<float>() + Vector3f::Ones() - xyz;
        if (l == 1) weights[0] = xyz[0] - ijk[0];
        if (m == 1) weights[1] = xyz[1] - ijk[1];
        if (n == 1) weights[2] = xyz[2] - ijk[2];

        // Add to PIC and FLIP particle velocities
        const float weight = weights[0] * weights[1] * weights[2];
        weightSum += weight;
        p += weight * cell->u[index];
        f += weight * (cell->u[index] - cell->oldU[index]);
      }
    }
  }

  return {p / weightSum, f / weightSum};
}

// Helper to resolve particle collisions
void MacGrid::resolveParticleCollisions(const std::vector<Particle *> &particles)
{
  // For each particle
#pragma omp parallel for
  for (unsigned int i = 0; i < particles.size(); ++i) {

    Particle * particle = particles[i];
    Vector3i currIndices = positionToIndices(particle->position);

    unsigned int attemptA = 0, attemptB = 0;
    auto kv = m_cells.find(currIndices);

    while (!withinBounds(currIndices) || (kv != m_cells.end() && kv->second->material == Material::Solid)) {

      // If out of bounds, project it back into the box
      while (!withinBounds(currIndices)) {

        if (attemptA > 20) {
          cout << attemptA << ": repeated failure to project particle back into bounds!" << endl;
          cout << "Particle: " << Debug::particleToString(particle) << endl;
          cout << Debug::vectorToString(currIndices) << endl;
          if (attemptA > 40) assert(false);
        }

        resolveParticleOutOfBoundsHelper(particle, 0);
        resolveParticleOutOfBoundsHelper(particle, 1);
        resolveParticleOutOfBoundsHelper(particle, 2);
        currIndices = positionToIndices(particle->position);
        kv = m_cells.find(currIndices);
        ++attemptA;
      }

      // If in a solid, project it out in the normal direction
      while (withinBounds(currIndices) && kv != m_cells.end() && kv->second->material == Material::Solid) {

        if (attemptB > 20) {
          cout << attemptB << ": repeated failure to project particle out of solid!" << endl;
          cout << "Particle: " << Debug::particleToString(particle) << endl;
          cout << Debug::cellToString(kv->second) << endl;
          if (attemptB > 40) assert(false);
        }

        resolveParticleInSolidHelper(particle, kv->second);
        currIndices = positionToIndices(particle->position);
        kv = m_cells.find(currIndices);
        ++attemptB;
      }
    }
  }
}

// Helper to resolve particle collisions with the bounds of the simulation
void MacGrid::resolveParticleOutOfBoundsHelper(Particle * particle, const int index)
{
  const float halfCellWidth = m_cellWidth / 2;

  if (particle->position[index] <= m_cornerPosition[index] + halfCellWidth) {

    particle->position[index] = m_cornerPosition[index] + halfCellWidth;
    particle->velocity[index] = 0;

  } else if (m_otherCornerPosition[index] - halfCellWidth <= particle->position[index]) {

    particle->position[index] = m_otherCornerPosition[index] - halfCellWidth;
    particle->velocity[index] = 0;
    
  }
}

// Helper to resolve particle collisions with solids within the bounds of the simulation
void MacGrid::resolveParticleInSolidHelper(Particle * particle, Cell * const cell)
{
  particle->position += cell->normal * m_cellWidth / 2;
  particle->velocity -= particle->velocity.dot(cell->normal) * cell->normal;
}

// ========================================================================
// ================== Common (Initialization and Simulation) Helpers ======
// ========================================================================

// Given a material and a vector of particles, sets all cells containing those particles to that material
void MacGrid::setCellsBasedOnParticles(const Material material, const vector<Particle *> &particles, const bool preventBadPositions)
{
  // Clear cells' particle collections
#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {
    kv->second->particles.clear();
    kv->second->particles.reserve(8);
  }

  // Iterate through particles
  for (unsigned int i = 0; i < particles.size(); ++i) {

    Particle *particle = particles[i];
    const Vector3i cellIndices = positionToIndices(particle->position);
    auto kv = m_cells.find(cellIndices);

    // If cell does not exist
    if (kv == m_cells.end()) {

      // Skip if cell is outside simulation bounds
      if (!withinBounds(cellIndices)) {
        if (material == Material::Fluid && preventBadPositions) {
          cout << "Marker particle " << i << " out of bounds!" << endl;
          cout << Debug::particleToString(particle) << endl;
          assert(false);
        }
        continue;
      }

      // Create the cell and put it in the hash table
      Cell * newCell = new Cell();
      m_cells.insert({cellIndices, newCell});

      // Set its fields
      newCell->material    = material;
      newCell->cellIndices = cellIndices;
      newCell->normal      = material == Material::Solid ? particle->velocity : Vector3f::Zero();
      newCell->particles.push_back(particle);

      continue;
    }

    // If cell does exist, and is not solid
    Cell * cell = kv->second;
    cell->particles.push_back(particle);
    if (cell->material != Material::Solid) {
      cell->material = material;
    } else if (material == Material::Fluid && preventBadPositions) {
      cout << "Marker particle " << i << " in solid!" << endl;
      cout << Debug::particleToString(particle) << endl;
      assert(false);
    }
  }
}

// Set layer field of solid cells to -2, air cells to -1, and fluid cells to 0
void MacGrid::setCellLayerBasedOnMaterial()
{
#pragma omp parallel for
  for (auto i = m_cells.begin(); i != m_cells.end(); ++i) {
    switch (i->second->material) {
      case Material::Solid: {
        i->second->layer = -2;
        break;
      }
      case Material::Air: {
        i->second->layer = -1;
        break;
      }
      case Material::Fluid: {
        i->second->layer = 0;
        break;
      };
    }
  }
}

// ========================================================================
// ================== Positional Helpers ==================================
// ========================================================================

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

// ========================================================================
// ================== Miscellaneous Helpers ===============================
// ========================================================================

// Helper function for QtConcurrent to save files asynchronously
extern void saveParticlesHelper(const string filepath, const vector<const Vector3f> particlePositions) {
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

// Function to call to save particles to a file
QFuture<void> MacGrid::saveParticlesToFile(const float time, bool saveOnlyFoamParticles) const
{
  // Todo: add something to indicate if a particle is a foam particle
  const string timeString = to_string(time);
  string filepath = m_outputFolder + "/" + string(10 - timeString.size(), '0') + timeString + ".csv";

  vector<const Vector3f> particlePositions;
  if (saveOnlyFoamParticles) {
    for (Particle * const particle : m_particles) {
      filepath = m_outputFolder + "/foam/" + string(10 - timeString.size(), '0') + timeString + ".csv";
      if (particle->foamParticle) particlePositions.push_back(particle->position);
    }
  } else {
    particlePositions.reserve(m_particles.size());
    for (Particle * const particle : m_particles) particlePositions.push_back(particle->position);
  }
  
  return QtConcurrent::run(saveParticlesHelper, filepath, particlePositions);
}

// Given a cell's indices, returns whether it falls within the simulation grid's bounds
bool MacGrid::withinBounds(const Vector3i &cellIndices) const
{
  return 0 <= cellIndices[0] && cellIndices[0] < m_cellCount[0] &&
      0 <= cellIndices[1] && cellIndices[1] < m_cellCount[1] &&
      0 <= cellIndices[2] && cellIndices[2] < m_cellCount[2];
}

// ========================================================================
// ================== Extension Helpers ===================================
// ========================================================================

// Used for adding fluid: given cell indices, add particles to that cell with given velocity
vector<Particle *> MacGrid::addParticlesToCell(int x, int y, int z, Vector3f velocity)
{
  // m_strata is the number of subdivisions per side, such that there are strata^3 subcells per cell
  const float strataWidth = m_cellWidth / m_strata;

  const Vector3i cellIndices(x,y,z);
  vector<Particle *> newParticles;

#pragma omp parallel for
  for (int x = 0; x < m_strata; ++x) {
    for (int y = 0; y < m_strata; ++y) {
      for (int z = 0; z < m_strata; ++z) {
        // Create a new particle
        Particle * newParticle = new Particle();
        do {
          const float a = getRandomFloat(), b = getRandomFloat(), c = getRandomFloat();
          newParticle->position = indicesToBasePosition(cellIndices) + Vector3f(x + a, y + b, z + c) * strataWidth;
        } while (positionToIndices(newParticle->position) != cellIndices);
        newParticle->velocity = velocity;
        m_particles.push_back(newParticle);
        newParticles.push_back(newParticle);
      }
    }
  }
  return newParticles;
}

// Can be called in simulate to add a horizontal square of fluid particles to the simulation
void MacGrid::addFluid(int x, int y, int z, int sideLength)
{

#pragma omp parallel for
  for (int i = 0; i < sideLength; ++i) {
    for (int j = 0; j < sideLength; ++j) {
      // Create or update cell
      if (m_cells.find(Vector3i(x+i, y+j, z)) == m_cells.end()) {
        Cell * newCell = new Cell();
        newCell->cellIndices = Vector3i(x+i,y+j,z);
        newCell->material = Material::Fluid;
        m_cells.insert({Vector3i(x+i, y+j, z), newCell});
      } else if (m_cells[Vector3i(x+i,y+j,z)]->material == Material::Air) {
        m_cells[Vector3i(x+i,y+j,z)]->material = Material::Fluid;
      } else if (m_cells[Vector3i(x+i,y+j,z)]->material == Material::Solid) {
        continue;
      }

      // Add fluid particles to cell
      addParticlesToCell(x+i, y+j, z, m_fluidVelocity);
    }
  }
}

// Convert all solids within bounds to fluid
void MacGrid::convertToFluid(int xLow, int xHigh, int yLow, int yHigh, int zLow, int zHigh) {
    for (int x = xLow; x < xHigh; x++) {
        for (int y = yLow; y < yHigh; y++) {
            for (int z = zLow; z < zHigh; z++) {
                if (m_cells.find(Vector3i(x,y,z)) == m_cells.end()) continue;
                if (m_cells[Vector3i(x,y,z)]->material != Material::Solid) continue;
                m_cells[Vector3i(x,y,z)]->material = Material::Fluid;
                addParticlesToCell(x,y,z,Vector3f(0,0,0));
            }
        }
    }
}

// Can be called in simulate to set foamParticle = true for particles in voxels where curl is above m_foamParticleBoundary
void MacGrid::assignFoamParticles()
{
#pragma omp parallel for
  for (unsigned int i = 0; i < m_particles.size(); ++i) {
    m_particles[i]->foamParticle = false;
  }

#pragma omp parallel for
  for (auto kv = m_cells.begin(); kv != m_cells.end(); ++kv) {

    const Vector3i &cellIndices = kv->first;
    Cell * cell = kv->second;

    // Skip non-fluid cells
    if (cell->material != Material::Fluid) continue; 

    // Curl calculation uses equation 28 from "Fluid Flow for the Rest of Us"
    Vector3f curl(0,0,0);
    curl[0] += cell->u[2] - m_cells[cellIndices + Vector3i(0,-1,0)]->u[2];
    curl[0] -= cell->u[1] - m_cells[cellIndices + Vector3i(0,0,-1)]->u[1];
    curl[1] += cell->u[0] - m_cells[cellIndices + Vector3i(0,0,-1)]->u[0];
    curl[1] -= cell->u[2] - m_cells[cellIndices + Vector3i(-1,0,0)]->u[2];
    curl[2] += cell->u[1] - m_cells[cellIndices + Vector3i(-1,0,0)]->u[1];
    curl[2] -= cell->u[0] - m_cells[cellIndices + Vector3i(0,-1,0)]->u[0];

    if (curl.norm() > m_foamParticleBoundary) {
      for (Particle * particle : cell->particles) {
        particle->foamParticle = true;
      }
    }
  }
}
