#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <iostream>
#include <Eigen/StdVector>

using namespace std;
class Reconstruction
{
public:
    Reconstruction();
    ~Reconstruction();

    /**
     * @brief surface_reconstruction - given folder of particles, performs surface reconstruction
     * to convert point clouds into triangle meshes
     * @param input_filepath - input folder of point clouds in .xyz file format
     * @param output_filepath - output folder of triangles meshes in .obj file format
     */
    void surface_reconstruction(string input_filepath, string output_filepath);
    void loadParticles(string input_filepath); //load .csv file, construct grids
    float calculateSignedDistance (int grid);


private:
    float m_searchRadius;
    float m_gridSpace;
    int m_numOfGrids;
    int m_numOfParticles;
    std::vector<Eigen::Vector3f> _particles;
    std::vector<int> *_grids;
    std::vector<float> _gridCorners;
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3f> _faces;



};

#endif // RECONSTRUCTION_H
