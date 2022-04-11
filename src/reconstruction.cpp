#include "reconstruction.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

Reconstruction::Reconstruction()
{
    //TODO:
    //need to set: m_numOfParticles, m_gridLength, m_gridWidth, m_gridHeight, m_gridSpace, m_searchRadius

    m_numOfGrids = m_gridLength*m_gridLength*m_gridHeight;

}

void Reconstruction::surface_reconstruction(string input_filepath, string output_filepath){

    loadParticles(input_filepath);
    //calculate the signed distance for each grid corner
    for(int i = 0; i<m_numOfGrids; i++){
        //TODO: implement signed distance
        calculateSignedDistance(i);
        //TODO: calculate signed distances for all grid corners
    }

}

//helper fuction
Eigen::Vector3i Reconstruction::GridIDtoXYZ(int idx){
    int z = idx/(m_gridLength*m_gridWidth);
    int y = (idx/(m_gridLength*m_gridWidth))/m_gridLength;
    int x = (idx/(m_gridLength*m_gridWidth))%m_gridLength;

    return Eigen::Vector3i(x,y,z);
}

//helper function
int Reconstruction::XYZtoGridID(Eigen::Vector3i xyz){
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
    m_particles2grids = new std::vector<int>[m_numOfGrids];
    for(int i = 0; i < m_numOfParticles; i++){
        int index = XYZtoGridID(_particles[i]);
        m_particles2grids[index].push_back(i);
    }

}

float Reconstruction::calculateSignedDistance (int grid){
    //

}
