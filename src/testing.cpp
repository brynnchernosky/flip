#include "testing.h"

//Assumes folders already exist
void Testing::testMeshParticleMesh(std::string folder) {
    MacGrid grid(folder);
    grid.init();
    grid.printParticles(folder + "/particles/volume.csv");

    const string particleFilepath = folder + "/particles";
    const string sdfFilepath = folder + "/sdfs";
    Reconstruction converter(folder);
    converter.surface_reconstruction(particleFilepath, sdfFilepath);
}


