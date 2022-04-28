#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <iostream>
#include <Eigen/StdVector>
#include <unordered_map>
#include <unordered_set>
#include <QSettings>

using namespace std;
class Reconstruction
{
public:
    Reconstruction();
    Reconstruction(std::string folder);
    ~Reconstruction();

    /**
     * @brief surface_reconstruction - given folder of particles, performs surface reconstruction
     * to convert point clouds into triangle meshes
     * @param input_filepath - input folder of point clouds in .xyz file format
     * @param output_filepath - output folder of triangles meshes in .obj file format
     */
    void surface_reconstruction(string input_filepath, string output_filepath);


private:
    void init();
    /**
     * @brief loadParticles - for a given input filepath
     * @param input_filepath - filepath to a text file where each line contains three comma
     * seperated values for x, y, and z
     */
    void loadParticles(string input_filepath);

    /**
     * @brief writeGrid - for a given output filepath, write grid
     * @param output_filepath - filepath to write grid information to
     * Given in format grid corner index, signed distance value (x, y, z, t)
     * x, y, z are integer values (
     */
    void writeGrid(string output_filepath);

    /**
     * @brief calculateSignedDistance - Calculates the signed distance at a grid corner
     * @param grid - index of the grid corner
     * @return the signed distance for that gridpoint
     */
    bool calculateSignedDistance (Eigen::Vector3i grid_corner, float &toWrite);

    /**
     * @brief XYZtoGridID - converts xyz coordinates to grid index, for placing points into cells
     * @param xyz - x,y,z coordinates of the particle
     * @return index to the grid vector
     */
    int XYZtoGridID(Eigen::Vector3f xyz);

    /**
     * @brief GridCornertoGridID - converts grid corner to grid index, for iteration
     * @param xyz - x,y,z coordinates of the grid corner
     * @return index to the grid vector
     */
    int GridCornertoGridID(Eigen::Vector3i xyz);

    /**
     * @brief kernel - kernel function from equation 22
     * @param s - input value
     * @return output value
     */
    float kernel(float s);

    //Member variables
    float m_searchRadius;
    float m_gridSpacing;
    int m_numOfGrids;

    int m_gridHeight; //corresponds with x
    int m_gridWidth; //corresponds with y
    int m_gridLength; // corresponds with z

    Eigen::Vector3f m_center; //Represents the center of the grid

    std::vector<Eigen::Vector3f> _particles;
    std::unordered_map<int, std::unordered_set<int>> m_cellToParticle;

    std::vector<float> _gridCorners;
};

#endif // RECONSTRUCTION_H
