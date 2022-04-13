#include <QCoreApplication>
#include <QCommandLineParser>

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "macGrid/MacGrid.h"

using namespace std;
using namespace Eigen;

// This is a command line application with one argument
int main(int argc, char *argv[])
{
  // ================== To delete

  MacGrid grid(1, {3,3,3}, {0,0,0});
  grid.addParticle({1.5,1.5,1.5}, Vector3f::Zero());
  grid.addParticle({1.5,2.5,1.5}, Vector3f::Zero());
  grid.addParticle({2.5,1.5,1.5}, Vector3f::Zero());
  grid.updateGrid();
  grid.printGrid();

  // ================== End to delete

  QCoreApplication a(argc, argv);
  QCommandLineParser parser;

  // Define formal parameters
  parser.addHelpOption();
  parser.addPositionalArgument("method", "simulate or particle-to-mesh");
  parser.addPositionalArgument("args1",  "argument1 for the method");
  parser.addPositionalArgument("args2",  "argument2 for the method");
  parser.process(a);
  
  // Obtain actual parameters
  const QStringList args = parser.positionalArguments();
  if (args.size() < 1) {
    cerr << "Usage: ./flip <method> <args...>" << endl;
    a.exit(1);
    return 1;
  }

  // Parse parameters
  const string method = args[0].toStdString();
  // const string args1  = args[1].toStdString();
  // const string args2  = args[2].toStdString();

  // Start timer
  const auto startTime = chrono::high_resolution_clock::now();

  // Switch on method
  if (method == "simulate")
  {
    // Flip flip;
    // flip.init();
    // flip.simulate();
    cout << "Error: simulate not yet implemented!" << endl;
  }
  else if (method == "particle-to-mesh")
  {
    // ParticleToMesh particleToMesh;
    // particleToMesh.init();
    // particleToMesh.convert();
    cout << "Error: particle-to-mesh not yet implemented!" << endl;
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

