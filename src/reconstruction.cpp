#include "reconstruction.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

Reconstruction::Reconstruction()
{

}

void surface_reconstruction(string input_filepath, string output_filepath){

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
    vector<string> row;
    string line, word, temp;

    while (fin >> temp) {

        row.clear();

        // read an entire row and
        // store it in a string variable 'line'
        getline(fin, line);

        // used for breaking words
        stringstream s(line);

        // read every column data of a row and
        // store it in a string variable, 'word'
        while (getline(s, word)) {

            // add all the column data
            // of a row to a vector
            row.push_back(word);
        }

        // convert string to integer for comparision
        roll2 = stoi(row[0]);

        // Compare the roll number
        if (roll2 == rollnum) {

            // Print the found data
            count = 1;
            cout << "Details of Roll " << row[0] << " : \n";
            cout << "Name: " << row[1] << "\n";
            cout << "Maths: " << row[2] << "\n";
            cout << "Physics: " << row[3] << "\n";
            cout << "Chemistry: " << row[4] << "\n";
            cout << "Biology: " << row[5] << "\n";
            break;
        }
    }
    if (count == 0)
        cout << "Record not found\n";

}

float Reconstruction::calculateSignedDistance (int grid){

}
