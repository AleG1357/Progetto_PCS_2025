#pragma once

#include <limits>
#include <utility>

#include "Polyhedron.hpp"


// Funzione che aggiunge un vertice solo se non già presente
unsigned int complete_vertex(Polyhedron& poly, const Vector3d& coords);

// Funzione che aggiunge un edge solo se non già presente
unsigned int complete_edge(Polyhedron& poly, unsigned int id1, unsigned int id2, unsigned int forced_id);
// Overload per chiamate senza forced_id (comportamento di default)
inline unsigned int complete_edge(Polyhedron& poly, unsigned int id1, unsigned int id2) {
	return complete_edge(poly, id1, id2, numeric_limits<unsigned int>::max());
}

// Triangolazione Class I e II di un poliedro
Polyhedron triangulate_classI(const Polyhedron& P_old, const unsigned int& delta);
Polyhedron triangulate_classII(const Polyhedron& P_old, const unsigned int& delta);