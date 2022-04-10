#include "flip.h"
#include "graphics/MeshLoader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

// ================== Constructor

Flip::Flip() {}

// ================== Destructor

Flip::~Flip() {}

// ================== Initializer

void Flip::init()
{
  vector<Vector3f> vertices;
  vector<Vector3f> normals;
  vector<Vector3i> tets;

  // Load mesh (panic if failed)
  if (!MeshLoader::loadTriMesh("../../../../flip/meshes/teapot.obj", vertices, normals, tets)) {
    cout << "Flip::init() failed to load mesh. Exiting!" << endl;
    exit(1);
  }

  // Initialize shape
  m_shape.init(vertices, tets);
}

// ================== Intermediate Renderer (Optional Task)

void Flip::draw() const
{
  cerr << "Flip::draw() not implemented!" << endl;
}
