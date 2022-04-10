#include "reconstruction.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

Reconstruction::Reconstruction()
{
    //TODO:
    //need to load: m_numOfParticles, m_numOfGrids, m_gridSpace, m_searchRadius

}

void Reconstruction::surface_reconstruction(string input_filepath, string output_filepath){
    //TODO:
    loadParticles(input_filepath);
    //calculate the signed distance for each grid corner, fuction parameter still not decided

}

void Reconstruction::loadParticles(string input_filepath){

    // File pointer
    fstream fin;

    // Open an existing file
    fin.open(input_filepath, ios::in);

    // Get the roll number
    // of which the data is required
    int rollnum, roll2, count = 0;
    rollnum = m_numOfParticles;

    // Read the Data from the file
    // as String Vector
//    vector<string> row;
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

}

float Reconstruction::calculateSignedDistance (int grid){
    //

}
