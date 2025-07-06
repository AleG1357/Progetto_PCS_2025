#pragma once

#include "Polyhedron.hpp"


// Proietta un vertice sulla sfera unitaria (Geometry.cpp)
void project_on_unit_sphere(Vertex& v);

// Calcola il centroide di una faccia (Geometry.cpp)
Vertex face_centroid(const Polyhedron& poly, unsigned int face_id);

// Assegna le adiacenze tra edge e facce
void assign_edge_adjacencies(Polyhedron& poly);
// Assegna le adiacenze tra vertici, edge e facce
void assign_vertex_adjacencies(Polyhedron& poly);

// Costruisce il poliedro duale
Polyhedron dual_polyhedron(const Polyhedron& poly);