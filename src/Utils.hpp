#pragma once

#include "Polyhedron.hpp"
#include "UCDUtilities.hpp"

using namespace std;
using namespace Eigen;

Polyhedron build_platonic_solid(int p, int q);

// Function which exports the polyhedron for Paraview
void export_polyhedron(const Polyhedron& P);

// Function which writes the output
bool write_output(const Polyhedron& P);