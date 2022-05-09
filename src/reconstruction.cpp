#include "reconstruction.h"
#include <iostream>
#include <fstream>
#include <string>

#include <QDirIterator>
#include <QFile>
#include <QTextStream>

// ================== Constructors

Reconstruction::Reconstruction(std::string folder)
{
    const std::string configFilepath = folder + "/config.ini";
    QSettings settings(QString::fromStdString(configFilepath), QSettings::IniFormat);

    settings.beginGroup("/Simulation");
    m_corner_offset = - Eigen::Vector3f(settings.value(QString("cornerPositionX")).toFloat(),
                                                 settings.value(QString("cornerPositionY")).toFloat(),
                                                 settings.value(QString("cornerPositionZ")).toFloat());

    int factor = 1;
    m_gridHeight = settings.value(QString("cellCountX")).toInt() * factor;
    m_gridWidth = settings.value(QString("cellCountY")).toInt() * factor;
    m_gridLength = settings.value(QString("cellCountZ")).toInt() * factor;
    m_gridSpacing = settings.value(QString("cellWidth")).toFloat() / (float) factor;
    settings.endGroup();

    m_searchRadius = 3 * m_gridSpacing;
    m_numOfGrids = m_gridLength*m_gridWidth*m_gridHeight;
}

// ================== Destructor

Reconstruction::~Reconstruction() {}

// ================== Generate per file SDFs

void Reconstruction::surface_reconstruction(string input_filepath, string output_filepath) {
    QDirIterator it(QString::fromStdString(input_filepath), QDir::Files);
    while(it.hasNext()) {
        QString filename = it.next();
        loadParticles(filename.toStdString());

        _gridCorners.clear();
        _gridCorners.reserve(m_numOfGrids);
        for (int x = 0; x < m_gridHeight; x++) {
            for (int y = 0; y < m_gridWidth; y++) {
                for (int z = 0; z < m_gridLength; z++) {
                    float toWrite;
                    if (calculateSignedDistance(Eigen::Vector3i(x, y, z), toWrite)) {
                        _gridCorners.push_back(toWrite);
                    } else {
                        //arbitrary, just needs to be a positive value
                        //we're not estimating vertex normals with the SDF, we're doing it with topology so actually doesn't matter
                        _gridCorners.push_back(100);
                    }
                }
            }
        }

        QFileInfo f(filename);
        std::string out = output_filepath + "/" + f.fileName().toStdString();
        writeGrid(out);
    }
}

// ================== Helpers to calculate SDF

int Reconstruction::XYZtoGridID(Eigen::Vector3f xyz){
    Eigen::Vector3f regularized = xyz / m_gridSpacing;
    int x = floor(regularized[0]);
    int y = floor(regularized[1]);
    int z = floor(regularized[2]);

    return m_gridLength*m_gridWidth*x + m_gridLength*y +z;
}

int Reconstruction::GridCornertoGridID(Eigen::Vector3i xyz) {
    return m_gridLength*m_gridWidth*xyz[0] + m_gridLength*xyz[1] + xyz[2];
}

void Reconstruction::loadParticles(string input_filepath){
    QFile particle_file(QString::fromStdString(input_filepath));

    // open the input file and load into _particles
    if (!particle_file.open(QIODevice::ReadOnly)) {
        std::cerr << input_filepath << " could not be opened" << std::endl;
        return;
    }

    QTextStream in(&particle_file);
    int num_particles = 0;
    _particles.clear();
    while (!in.atEnd()) {
        num_particles ++;
        Eigen::Vector3f particle_pos = Eigen::Vector3f(0,0,0);
        QString line = in.readLine();
        QStringList positions = line.split(",");
        particle_pos[0] = positions[0].toFloat();
        particle_pos[1] = positions[1].toFloat();
        particle_pos[2] = positions[2].toFloat();

        particle_pos = (particle_pos + m_corner_offset);
        if (particle_pos[0] * particle_pos[1] * particle_pos[2] < 0) {
            std::cout << "particle negative dimension value" << std::endl;
        }
        _particles.push_back(particle_pos);
    }

    m_cellToParticle.clear();
    for(int i = 0; i < num_particles; i++){
        int index = XYZtoGridID(_particles[i]);
        if (m_cellToParticle.find(index) != m_cellToParticle.end()) {
            m_cellToParticle[index].insert(i);
        } else {
            std::unordered_set<int> setOfParticles = std::unordered_set<int>();
            setOfParticles.insert(i);
            m_cellToParticle[index] = setOfParticles;
        }
    }

    particle_file.close();

    std::cout << "Read " << num_particles << " particles from " << input_filepath << std::endl;
}

void Reconstruction::writeGrid(string output_filepath) {
    QFile sdf_file(QString::fromStdString(output_filepath));

    if (!sdf_file.open(QIODevice::WriteOnly)) {
        std::cerr << output_filepath << " could not be opened" << std::endl;
        return;
    }

    QTextStream fout(&sdf_file);
    for (int x = 0; x < m_gridHeight; x++) {
        for (int y = 0; y < m_gridWidth; y++) {
            for (int z = 0; z < m_gridLength; z++) {
                Eigen::Vector3i corner(x, y, z);
                float signed_distance = _gridCorners[GridCornertoGridID(corner)];
                std::string toWrite =
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(z) + ", " + std::to_string(signed_distance) ;
                fout << QString::fromStdString(toWrite) << endl;
            }
        }
    }

    std::cout << "Wrote to " << output_filepath << std::endl;
    sdf_file.close();
}

float Reconstruction::kernel(float s) {
    return std::max(0.0, std::pow(1 - s, 3));
}

bool Reconstruction::calculateSignedDistance (Eigen::Vector3i grid_corner, float &toWrite) {
    //row - width, column - length, depth - height
    //x
    int depthStart = max(0, grid_corner[0] - 3);
    int depthEnd = min(m_gridHeight, grid_corner[0] + 3);
    // y
    int rowStart = max(0, grid_corner[1] - 3);
    int rowEnd = min(m_gridWidth, grid_corner[1] + 3);
    // z
    int colStart = max(0, grid_corner[2] - 3);
    int colEnd = min(m_gridLength, grid_corner[2] + 3);

    Eigen::Vector3f x_g(grid_corner[0], grid_corner[1], grid_corner[2]);
    x_g *= m_gridSpacing;
    std::unordered_map<int, double> neighbor_particle_weights; //maps particle_idx to w_i

    //Iterate through each cell in a 9x9 block around the given grid corner
    double total = 0.0;
    for(int i = depthStart; i < depthEnd; i++) {
        for(int j = rowStart; j < rowEnd; j++) {
            for(int k = colStart; k < colEnd; k++) {
                int idx = GridCornertoGridID(Eigen::Vector3i(i, j, k));
                std::unordered_set<int> in_cell_particles = m_cellToParticle[idx];
                for (int particle_idx : in_cell_particles) {
                    Eigen::Vector3f vec = _particles[particle_idx] - x_g;
                    double distance = (vec).norm();
                    if (distance <= m_searchRadius) {
                        //Map each particle that is within distance R to corner to a weight, normalize after
                        double w_i = kernel(vec.dot(vec) / std::pow(m_searchRadius, 2));
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
