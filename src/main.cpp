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
#include <testing.h>

using namespace std;
using namespace Eigen;

bool isInvalidPath(string thingName, bool isFile, string filepath)
{
  QFileInfo fileInfo(QString::fromStdString(filepath));
  if (!fileInfo.exists()) {
    cerr << "Error: " << thingName << " does not exist!" << endl;
    return true;
  }
  if (isFile && !fileInfo.isFile()) {
    cerr << "Error: " << thingName << " exists, but is not a file!" << endl;
    return true;
  }
  return false;
}

// This is a command line application with two arguments
int main(int argc, char *argv[])
{
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
    a.exit();
    return 1;
  }
  const string method = args[0].toStdString();
  const string folder = args[1].toStdString();

  // Panic if the config file does not exist in the specified folder
  const string configFilepath = folder + "/config.ini";
  if (isInvalidPath(configFilepath, true, configFilepath)) {
    a.exit();
    return 1;
  }

  // Start timer
  const auto startTime = chrono::high_resolution_clock::now();

  // Switch on method
  if (method == "simulate") {

    if (isInvalidPath("solid obj file", true, folder + "/solid.obj") ||
        isInvalidPath("fluid obj file", true, folder + "/fluid.obj")) {
      a.exit();
      return 1;
    }

    const string particleFilepath = folder + "/particles";
    if (isInvalidPath("particles directory", false, particleFilepath)) {
      QDir().mkdir(QString::fromStdString(particleFilepath));
    }

    MacGrid grid(folder);

    // ================== TO DELETE

    grid.init();
    grid.simulate();

    // ================== END TO DELETE

  } else if (method == "particle-to-mesh") {

    const string particleFilepath = folder + "/particles";
    if (isInvalidPath("particles directory", false, particleFilepath)) {
      a.exit();
      return 1;
    }

    const string sdfFilepath = folder + "/sdfs";
    if (isInvalidPath("sdfs directory", false, sdfFilepath)) {
      QDir().mkdir(QString::fromStdString(sdfFilepath));
    }

    Reconstruction converter(folder);
    converter.surface_reconstruction(particleFilepath, sdfFilepath);

  } else if (method == "test") {
      Testing::testMeshParticleMesh(folder);

  } else {

    cout << "Error: unknown method!" << endl;

  }

  // End timer
  const auto endTime = chrono::high_resolution_clock::now();
  const auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();
  cout << "Execution took " << duration << " microseconds." <<endl;

  a.exit();
}

