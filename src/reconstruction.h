#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <iostream>

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

};

#endif // RECONSTRUCTION_H
