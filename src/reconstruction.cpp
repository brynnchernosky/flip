#include "reconstruction.h"
#include <iostream>
#include <fstream>
#include <string>

Reconstruction::Reconstruction():
    m_gridSpacing(1),
    m_gridWidth(4),
    m_gridHeight(4),
    m_gridLength(4)
{
    //TODO:
    //need to set: m_numOfParticles, m_gridLength, m_gridWidth, m_gridHeight, m_gridSpace, m_searchRadius

    m_numOfGrids = m_gridLength*m_gridWidth*m_gridHeight;

}

Reconstruction::~Reconstruction() {}

void Reconstruction::surface_reconstruction(string input_filepath, string output_filepath){

    loadParticles(input_filepath);
     //TODO: calculate signed distances for all grid corners
    for (int x = 0; x < m_gridWidth; x++) {
        for (int y = 0; y < m_gridHeight; y++) {
            for (int z = 0; z < m_gridHeight; z++) {
                //TODO: implement signed distance
                calculateSignedDistance(Eigen::Vector3i(x, y, z));
            }
        }
    }

}

Eigen::Vector3i Reconstruction::GridIDtoXYZ(int idx){
    int z = idx/(m_gridLength*m_gridWidth);
    int y = (idx/(m_gridLength*m_gridWidth))/m_gridLength;
    int x = (idx/(m_gridLength*m_gridWidth))%m_gridLength;

    return Eigen::Vector3i(x,y,z);
}

int Reconstruction::XYZtoGridID(Eigen::Vector3i xyz){
    float x_f = xyz[0]/m_gridSpacing;
    float y_f = xyz[1]/m_gridSpacing;
    float z_f = xyz[2]/m_gridSpacing;
    int x = floor(x_f);
    int y = floor(y_f);
    int z = floor(z_f);

    return m_gridLength*m_gridWidth*x + m_gridLength*y +z;
}

int Reconstruction::XYZtoGridID(Eigen::Vector3f xyz){
    float x_f = xyz[0]/m_gridSpacing;
    float y_f = xyz[1]/m_gridSpacing;
    float z_f = xyz[2]/m_gridSpacing;
    int x = floor(x_f);
    int y = floor(y_f);
    int z = floor(z_f);

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
    getline(fin, line);
    //for now, assume every line of .csv file is just particle positions separated by coma
    for(int i = 0; i < m_numOfParticles; i++){
        stringstream s(line);
        Eigen::Vector3f particle_pos = Eigen::Vector3f(0,0,0);
        int j = 0;
        while (getline(s, word,',')) {
            particle_pos[j] = stof(word);
            j++;
        }
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

double Reconstruction::calculateSignedDistance (Eigen::Vector3i grid) {
    //row - height, column - length, depth - width


}
