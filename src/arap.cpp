#include "arap.h"
#include "graphics/MeshLoader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

ARAP::ARAP() {}

void ARAP::init(Vector3f &min, Vector3f &max)
{
  vector<Vector3f> vertices;
  vector<Vector3f> normals;
  vector<Vector3i> tets;

  // Load mesh (panic if failed)
  if (!MeshLoader::loadTriMesh("../../../../arap-zackchengyk/meshes/teapot.obj", vertices, normals, tets)) {
    cout << "ARAP::init() failed to load mesh. Exiting!" << endl;
    exit(1);
  }

  // Initialize shape
  m_shape.init(vertices, tets);

  // Get min and max for viewport stuff
  MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
  for (unsigned long i = 0; i < vertices.size(); ++i) {
    all_vertices.row(i) = vertices[i];
  }
  min = all_vertices.colwise().minCoeff();
  max = all_vertices.colwise().maxCoeff();
}

void ARAP::move(int vertex, Vector3f pos)
{
  const vector<Vector3f>   &startingPositions = m_shape.getVertices();
  const unordered_set<int> &anchors = m_shape.getAnchors();

  vector<Vector3f> newPositions = startingPositions;

  newPositions[vertex] = pos;

  // Todo
  m_shape.setVertices(newPositions);
}

//////////////////// No need to edit after this /////////////////////////

int ARAP::getClosestVertex(Vector3f start, Vector3f ray)
{
  return m_shape.getClosestVertex(start, ray);
}

void ARAP::draw(Shader *shader, GLenum mode)
{
  m_shape.draw(shader, mode);
}

void ARAP::select(Shader *shader, int vertex)
{
  m_shape.select(shader, vertex);
}

void ARAP::toggleWire()
{
  m_shape.toggleWireframe();
}

bool ARAP::getAnchorPos(int lastSelected, Vector3f& pos, Vector3f ray, Vector3f start)
{
  return m_shape.getAnchorPos(lastSelected, pos, ray, start);
}
