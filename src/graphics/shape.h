#ifndef SHAPE_H
#define SHAPE_H

#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <unordered_set>
#include <vector>

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)

class Shader;

class Shape
{
  public:

    Shape();
    ~Shape();

    void init(const std::vector<Eigen::Vector3f> &vertices,
              const std::vector<Eigen::Vector3i> &faces);

    void setModelMatrix(const Eigen::Affine3f &model);

    void toggleWireframe();

    const std::vector<const Eigen::Vector3f> &getVertices();
    const std::vector<const Eigen::Vector3i> &getFaces();

  private:

    unsigned int m_numVertices;
    unsigned int m_numFaces;

    std::vector<const Eigen::Vector3f> m_vertices;
    std::vector<const Eigen::Vector3i> m_faces;

    Eigen::Matrix4f m_modelMatrix;
};

#endif // SHAPE_H
