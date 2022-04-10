#include "shape.h"

#include <iostream>

using namespace std;
using namespace Eigen;

// ================== Constructor

Shape::Shape() :
  m_numVertices(),
  m_numFaces(),
  m_modelMatrix(Matrix4f::Identity())
{}

// ================== Destructor

Shape::~Shape() {}

// ================== Misc

void Shape::init(const std::vector<Eigen::Vector3f> &vertices,
                 const std::vector<Eigen::Vector3i> &faces)
{
  m_vertices.clear();
  m_faces.clear();
  copy(vertices.begin(), vertices.end(), back_inserter(m_vertices));
  copy(faces.begin(), faces.end(), back_inserter(m_faces));
}

void Shape::setModelMatrix(const Affine3f &model)  { m_modelMatrix = model.matrix(); }

const vector<const Vector3f> &Shape::getVertices() { return m_vertices; }

const vector<const Vector3i> &Shape::getFaces()    { return m_faces; }
