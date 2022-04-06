#include "shape.h"
#include "graphics/Shader.h"

#include <iostream>

using namespace std;
using namespace Eigen;

Shape::Shape() :
  m_numSurfaceVertices(),
  m_verticesSize(),
  m_modelMatrix(Matrix4f::Identity()),
  m_wireframe(false)
{}


Vector3f Shape::getNormal(const Vector3i &face)
{
  const Vector3f &v1 = m_vertices[face[0]];
  const Vector3f &v2 = m_vertices[face[1]];
  const Vector3f &v3 = m_vertices[face[2]];
  const Vector3f e1 = v2 - v1;
  const Vector3f e2 = v3 - v1;
  const Vector3f n = e1.cross(e2);
  return  n / n.norm();
}

void Shape::updateMesh(const vector<Vector3i> &triangles,
                       const vector<Vector3f> &vertices,
                       vector<Vector3f> &verts,
                       vector<Vector3f> &normals,
                       vector<Vector3f> &colors)
{
  verts.reserve  (triangles.size() * 3);
  normals.reserve(triangles.size() * 3);

  for(const Vector3i &triangle : triangles) {

    // Get normal
    const Vector3f normal = getNormal(triangle);

    for (auto &v : {triangle[0], triangle[1], triangle[2]}) {
      normals.push_back(normal);
      verts.push_back(vertices[v]); // What kind of black magic is going on here

      if (m_anchors.find(v) == m_anchors.end()) {
        colors.push_back(Vector3f(1.f, 0.f, 0.f));
      } else {
        colors.push_back(Vector3f(0.f, 1.f - m_green, 1.f - m_blue));
      }
    }
  }
}



void Shape::init(const vector<Vector3f> &vertices, const vector<Vector3i> &triangles)
{
  m_vertices.clear();
  copy(vertices.begin(), vertices.end(), back_inserter(m_vertices));

  vector<Vector3f> verts;
  vector<Vector3f> normals;
  vector<Vector3f> colors;
  vector<Vector3i> faces;
  faces.reserve(triangles.size());

  for(int s = 0; s < triangles.size() * 3; s+=3) faces.push_back(Vector3i(s, s + 1, s + 2));
  updateMesh(triangles, vertices, verts, normals, colors);

  glGenBuffers(1, &m_surfaceVbo);
  glGenBuffers(1, &m_surfaceIbo);
  glGenVertexArrays(1, &m_surfaceVao);

  glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3) + (colors.size() * 3)), nullptr, GL_DYNAMIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * verts.size() * 3, static_cast<const void *>(verts.data()));
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * verts.size() * 3, sizeof(float) * normals.size() * 3, static_cast<const void *>(normals.data()));
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3)), sizeof(float) * colors.size() * 3, static_cast<const void *>(colors.data()));
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * faces.size(), static_cast<const void *>(faces.data()), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glBindVertexArray(m_surfaceVao);
  glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, static_cast<GLvoid *>(0));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(sizeof(float) * verts.size() * 3));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(sizeof(float) * (verts.size() * 3 + colors.size() * 3)));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  m_numSurfaceVertices = faces.size() * 3;
  m_verticesSize = vertices.size();
  m_faces = triangles;
  m_red = 0.5f + 0.5f * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  m_blue = 0.5f + 0.5f * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  m_green = 0.5f + 0.5f * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  m_alpha = 0.5f * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}


void Shape::setVertices(const vector<Vector3f> &vertices)
{
  m_vertices.clear();
  copy(vertices.begin(), vertices.end(), back_inserter(m_vertices));

  vector<Vector3f> verts;
  vector<Vector3f> normals;
  vector<Vector3f> colors;

  updateMesh(m_faces, vertices, verts, normals, colors);

  glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3) + (colors.size() * 3)), nullptr, GL_DYNAMIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * verts.size() * 3, static_cast<const void *>(verts.data()));
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * verts.size() * 3, sizeof(float) * normals.size() * 3, static_cast<const void *>(normals.data()));
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3)), sizeof(float) * colors.size() * 3, static_cast<const void *>(colors.data()));
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

bool Shape::getAnchorPos(int lastSelected,
                         Vector3f &pos,
                         Vector3f ray,
                         Vector3f start)
{
  bool isAnchor = m_anchors.find(lastSelected) != m_anchors.end();
  if (isAnchor) {
    Vector3f oldPos = m_vertices[lastSelected];
    ParametrizedLine line = ParametrizedLine<float, 3>::Through(start, start+ray);
    pos = line.projection(oldPos);
  }
  return isAnchor;
}

const vector<Vector3f> &Shape::getVertices()
{
  return m_vertices;
}


const vector<Vector3i> &Shape::getFaces()
{
  return m_faces;
}

const unordered_set<int> &Shape::getAnchors()
{
  return m_anchors;
};


int Shape::getClosestVertex(Vector3f start,
                            Vector3f ray,
                            float threshold)
{
  int closest_vertex = -1;
  int i = 0;
  float dist = numeric_limits<float>::max();
  ParametrizedLine line = ParametrizedLine<float, 3>::Through(start, start + ray);
  for(auto &v : m_vertices) {
    float d = line.distance(v);
    if (d<dist) {
      dist = d;
      closest_vertex = i;
    }
    i++;
  }

  if (dist >= threshold) {
    closest_vertex = -1;
  }

  return closest_vertex;
}

bool Shape::select(Shader *shader, int closest_vertex)
{
  bool vertexIsNowSelected = m_anchors.find(closest_vertex) == m_anchors.end();
  if (vertexIsNowSelected) {
    m_anchors.insert(closest_vertex);
  } else {
    m_anchors.erase(closest_vertex);
  }

  vector<Vector3f> verts;
  vector<Vector3f> normals;
  vector<Vector3f> colors;
  updateMesh(m_faces, m_vertices, verts, normals, colors);

  glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3) + (colors.size() * 3)), nullptr, GL_DYNAMIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * verts.size() * 3, static_cast<const void *>(verts.data()));
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * verts.size() * 3, sizeof(float) * normals.size() * 3, static_cast<const void *>(normals.data()));
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3)), sizeof(float) * colors.size() * 3, static_cast<const void *>(colors.data()));
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  return vertexIsNowSelected;
}

void Shape::setModelMatrix(const Affine3f &model)
{
  m_modelMatrix = model.matrix();
}

void Shape::toggleWireframe()
{
  m_wireframe = !m_wireframe;
}

void Shape::draw(Shader *shader, GLenum mode)
{
  switch(mode) {
    case GL_TRIANGLES: {
      shader->setUniform("wire",  0);
      shader->setUniform("m",     m_modelMatrix);
      shader->setUniform("red",   m_red);
      shader->setUniform("green", m_green);
      shader->setUniform("blue",  m_blue);
      shader->setUniform("alpha", m_alpha);
      glBindVertexArray(m_surfaceVao);
      glDrawElements(GL_TRIANGLES,
                     m_numSurfaceVertices,
                     GL_UNSIGNED_INT,
                     reinterpret_cast<GLvoid *>(0));
      glBindVertexArray(0);
      break;
    }
    case GL_POINTS: {
      shader->setUniform("m", m_modelMatrix);
      glBindVertexArray(m_surfaceVao);
      glDrawElements(mode, m_numSurfaceVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
      glBindVertexArray(0);
      break;
    }
  }

}
