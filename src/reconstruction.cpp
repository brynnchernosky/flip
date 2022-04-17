#include "reconstruction.h"
#include <iostream>
#include <fstream>
#include <string>

const double DOUBLE_MAX = std::numeric_limits<double>::max();

Reconstruction::Reconstruction():
    m_gridSpacing(1),
    m_numOfParticles(64),
    m_gridHeight(2),
    m_gridWidth(2),
    m_gridLength(2)
{
    //TODO:
    //need to set: m_numOfParticles, m_gridLength, m_gridWidth, m_gridHeight, m_gridSpace, m_searchRadius

    m_numOfGrids = m_gridLength*m_gridWidth*m_gridHeight;

    float height_offset = m_gridHeight / 2.f;
    float width_offset = m_gridWidth / 2.f;
    float length_offset = m_gridLength / 2.f;
    m_center = Eigen::Vector3f(height_offset, width_offset, length_offset);
}

Reconstruction::~Reconstruction() {}

void Reconstruction::surface_reconstruction(string input_filepath, string output_filepath) {

    loadParticles(input_filepath);
    for (int x = 0; x < m_gridHeight; x++) {
        for (int y = 0; y < m_gridLength; y++) {
            for (int z = 0; z < m_gridWidth; z++) {
                Eigen::Vector3f corner(x,y,z);
                std::cout << XYZtoGridID(corner) << std::endl;
            }
        }
    }
     //TODO: calculate signed distances for all grid corners
//    std::vector<double> signed_distances;
//    std::vector<int> invalid;
//    signed_distances.reserve(m_numOfGrids);
//    for (int x = 0; x < m_gridHeight; x++) {
//        for (int y = 0; y < m_gridWidth; y++) {
//            for (int z = 0; z < m_gridLength; z++) {
//                //TODO: implement signed distance
//                double toWrite;
//                if (calculateSignedDistance(Eigen::Vector3i(x, y, z), toWrite)) {
//                    signed_distances.push_back(toWrite);
//                } else {
//                    signed_distances.push_back(DOUBLE_MAX);
//                    invalid.push_back(XYZtoGridID(Eigen::Vector3i(x, y, z)));
//                }
//            }
//        }
//    }

}

//Eigen::Vector3i Reconstruction::GridIDtoXYZ(int idx){
//    int z = idx/(m_gridLength*m_gridWidth);
//    int y = (idx/(m_gridLength*m_gridWidth))/m_gridLength;
//    int x = (idx/(m_gridLength*m_gridWidth))%m_gridLength;

//    return Eigen::Vector3i(x,y,z);
//}

//int Reconstruction::XYZtoGridID(Eigen::Vector3i xyz){
//    float x_f = xyz[0]/m_gridSpacing;
//    float y_f = xyz[1]/m_gridSpacing;
//    float z_f = xyz[2]/m_gridSpacing;
//    int x = floor(x_f);
//    int y = floor(y_f);
//    int z = floor(z_f);

//    return m_gridLength*m_gridWidth*x + m_gridLength*y +z;
//}

int Reconstruction::XYZtoGridID(Eigen::Vector3f xyz){
    int x = floor(xyz[0]);
    int y = floor(xyz[1]);
    int z = floor(xyz[2]);

    return m_gridLength*m_gridWidth*x + m_gridLength*y +z;
}

void Reconstruction::loadParticles(string input_filepath){

    // open the input file and load into _particles
    fstream fin;
    fin.open(input_filepath, ios::in);
    if(!fin.good()) {
        std::cout << input_filepath << " could not be opened" << std::endl;
        return;
    }

    // Read the Data from the file as String Vector vector<string> row;
    string line, word, temp;
    //for now, assume every line of .csv file is just particle positions separated by coma
    for(int i = 0; i < m_numOfParticles; i++){
        getline(fin, line);
        stringstream s(line);
        Eigen::Vector3f particle_pos = Eigen::Vector3f(0,0,0);
        int j = 0;
        while (getline(s, word,',')) {
            particle_pos[j] = stof(word);
            j++;
        }
        particle_pos += m_center;
        _particles.push_back(particle_pos);
    }

    //grid index: row-first, then column, then stack
    for(int i = 0; i < m_numOfParticles; i++){
        int index = XYZtoGridID(_particles[i]);
        if (m_cellToParticle.find(index) != m_cellToParticle.end()) {
            m_cellToParticle[index].insert(i);
        } else {
            std::unordered_set<int> setOfParticles = std::unordered_set<int>();
            setOfParticles.insert(i);
            m_cellToParticle[index] = setOfParticles;
        }
    }

}

double Reconstruction::kernel(double s) {
    return std::max(0.0, std::pow(1 - s, 3));
}

bool Reconstruction::calculateSignedDistance (Eigen::Vector3i grid_corner, double &toWrite) {
    //row - width, column - length, depth - height
    int rowStart = max(0, grid_corner[1] - 3);
    int rowEnd = min(m_gridWidth, grid_corner[1] + 3);
    int colStart = max(0, grid_corner[2] - 3);
    int colEnd = min(m_gridLength, grid_corner[2] + 3);
    int depthStart = max(0, grid_corner[0] - 3);
    int depthEnd = min(m_gridHeight, grid_corner[0] + 3);

    Eigen::Vector3f x_g(grid_corner[0], grid_corner[1], grid_corner[2]);
    std::unordered_map<int, double> neighbor_particle_weights; //maps particle_idx to w_i
    double R = 3 * m_gridSpacing;

    //Iterate through each cell in a 9x9 block around the given grid corner
    double total = 0.0;
    for(int i = depthStart; i < depthEnd; i++) {
        for(int j = rowStart; j < rowEnd; j++) {
            for(int k = colStart; k < colEnd; k++) {
                int idx = XYZtoGridID(Eigen::Vector3f(i, j, k));
                std::unordered_set<int> in_cell_particles = m_cellToParticle[idx];
                for (int particle_idx : in_cell_particles) {
                    Eigen::Vector3f vec = _particles[particle_idx] - x_g;
                    double distance = (vec).norm();
                    if (distance <= R) {
                        //Map each particle that is within distance R to corner to a weight, normalize after
                        double w_i = kernel(vec.dot(vec) / std::pow(R, 2));
                        neighbor_particle_weights[particle_idx] = w_i;
                        total += w_i;
                    }
                }
            }
        }
    }

    if (!neighbor_particle_weights.size()) {
        return false;
    }

    double particle_radius = 0.5 * m_gridSpacing;
    //Use those weights to create an average point x_bar, and average radius r_bar
    Eigen::Vector3f x_avg = Eigen::Vector3f::Zero();
    double r_avg = 0.0;
    for (auto i : neighbor_particle_weights) {
        int particle_idx = i.first;
        double w_i = i.second / total;
        x_avg += w_i * _particles[particle_idx];
        r_avg += w_i * particle_radius;
    }

    //Use to calculate signed distance
    toWrite = (x_g - x_avg).norm() - r_avg;
    return true;
}
