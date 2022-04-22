#include <QCoreApplication>
#include <QCommandLineParser>
#include <QSettings>
#include <QDir>
#include <QFileInfo>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sys/stat.h>

#include "reconstruction.h"
#include "macGrid/MacGrid.h"

using namespace std;
using namespace Eigen;

// This is a command line application with one argument
int main(int argc, char *argv[])
{
  // ================== To delete

  MacGrid grid;
  grid.init();
  grid.createBufferZone();
  grid.printGrid();
  grid.setGridCellVelocity({1, 1, 1}, {1, 1, 1}, {1, 1, 1});
  grid.classifyPseudoPressureGradient();
  cout << "After" << endl;
  grid.printGrid();

  return 0;

  // ================== End to delete

  QCoreApplication a(argc, argv);
  QCommandLineParser parser;

  // Define formal parameters
  parser.addHelpOption();
  parser.addPositionalArgument("method", "simulate or particle-to-mesh");
  parser.addPositionalArgument("folder",  "parent folder for a particular simulation");
  parser.process(a);
  
  // Obtain actual parameters
  const QStringList args = parser.positionalArguments();
  if (args.size() != 2) {
    cerr << "Usage: ./flip <method> <folder>" << endl;
    a.exit(1);
    return 1;
  }

  // Parse parameters, ensure that given folder has the required folders
  const string method = args[0].toStdString();
  const string folder = args[1].toStdString();

  // Load the .ini if it exists
  const QString ini_filepath = QString::fromStdString(folder + "/config.ini");
  QFileInfo ini_file(ini_filepath);
  if (!(ini_file.exists() && ini_file.isFile())) {
      std::cerr << "config.ini does not exist in given folder" << std::endl;
      a.exit(1);
      return 1;
  }
  QSettings settings(ini_filepath, QSettings::IniFormat);

  // Make the directories if they do not exist

  // Start timer
  const auto startTime = chrono::high_resolution_clock::now();

  // Switch on method
  if (method == "simulate")
  {
    const QString obj_filepath = QString::fromStdString(folder + "/fluid.obj");
    QFileInfo obj_file(ini_filepath);
    if (!(obj_file.exists() && obj_file.isFile())) {
        std::cerr << "fluid.obj does not exist in given folder" << std::endl;
        a.exit(1);
        return 1;
    }

    const QString particle_filepath = QString::fromStdString(folder + "/particles");
    QDir particle_dir(particle_filepath);
    if (!particle_dir.exists()) {
        QDir().mkdir(particle_filepath);
    }

    const string input_mesh = obj_filepath.toStdString();
    const string output_folder = particle_filepath.toStdString();

    // Flip flip;
    // flip.init();
    // flip.simulate();
    cout << "Error: simulate not yet implemented!" << endl;
  }
  else if (method == "particle-to-mesh")
  {
    const QString particle_filepath = QString::fromStdString(folder + "/particles");
    QDir particle_dir(particle_filepath);
    if (!particle_dir.exists()) {
        std::cerr << "particles directory does not exist in given folder" << std::endl;
    }

    const QString sdf_filepath = QString::fromStdString(folder + "/sdfs");
    QDir sdf_dir(sdf_filepath);
    if (!sdf_dir.exists()) {
        QDir().mkdir(sdf_filepath);
    }

    const string input_filepath = particle_filepath.toStdString();
    const string output_filepath = sdf_filepath.toStdString();

    settings.beginGroup("/Conversion");
    float gridSpacing = settings.value("gridSpacing").toFloat();
    int numOfParticles = settings.value("numOfParticles").toInt();
    int gridHeight = settings.value("gridHeight").toInt();
    int gridWidth = settings.value("gridWidth").toInt();
    int gridLength = settings.value("gridLength").toInt();
    settings.endGroup();

    Reconstruction converter(gridSpacing, numOfParticles, gridHeight, gridWidth, gridLength);
    converter.surface_reconstruction(input_filepath, output_filepath);
  }
  else
  {
    cout << "Error: unknown method!" << endl;
  }

  // End timer
  const auto endTime = chrono::high_resolution_clock::now();
  const auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
  cout << "Execution took " << duration << " milliseconds." <<endl;

  a.exit();
}

