#include "Topology.hpp"
#include "Triangulation.hpp"

#include <map>
#include <algorithm>

using namespace std;
using namespace Eigen;


// Proietta le coordinate di un vertice sulla sfera unitaria (normalizza il vettore)
void project_on_unit_sphere(Vertex& v) {
	double norm = v.coords.norm();
	if (norm > 0) v.coords /= norm;
}


// Calcola il centroide di una faccia del poliedro
Vertex face_centroid(const Polyhedron& poly, unsigned int face_id) {
	Vertex centroid;
	centroid.coords = {0, 0, 0};
	centroid.id = face_id;
	unsigned int n = poly.faces[face_id].n_vertices();
	for (unsigned int i = 0; i < n; ++i) {
		centroid.coords += poly.vertices[poly.faces[face_id].id_vertices[i]].coords;
	}
	centroid.coords /= n;
	return centroid;
}



// Assegna a ogni spigolo le facce adiacenti
void assign_edge_adjacencies(Polyhedron& poly) {
	for (auto& edge : poly.edges) {
		edge.face_adjacencies.clear();
		for (const auto& face : poly.faces) {
			if (std::find(face.id_edges.begin(), face.id_edges.end(), edge.id) != face.id_edges.end()) {
				edge.face_adjacencies.push_back(face.id);
				if (edge.face_adjacencies.size() == 2) break;
			}
		}
	}
}


// Assegna a ogni vertice le facce e gli spigoli adiacenti
void assign_vertex_adjacencies(Polyhedron& poly) {
	std::unordered_map<unsigned int, std::vector<unsigned int>> edgeMap;
	for (const auto& edge : poly.edges) {
		edgeMap[edge.origin].push_back(edge.id);
		edgeMap[edge.end].push_back(edge.id);
	}
	for (auto& v : poly.vertices) {
		v.face_adjacencies.clear();
		v.edge_adjacencies.clear();
		auto& neighbors = edgeMap[v.id];
		if (neighbors.empty()) continue;
		unsigned int start_edge = neighbors[0];
		unsigned int current_face = poly.edges[start_edge].face_adjacencies[0];
		v.face_adjacencies.push_back(current_face);
		unsigned int first_edge = 0;
		for (unsigned int e_id : poly.faces[current_face].id_edges) {
			if (e_id == start_edge) continue;
			if (std::find(neighbors.begin(), neighbors.end(), e_id) == neighbors.end()) continue;
			first_edge = e_id;
			v.edge_adjacencies.push_back(first_edge);
			break;
		}
		while (first_edge != start_edge) {
			unsigned int next_face = 0;
			for (unsigned int f_id : poly.edges[first_edge].face_adjacencies) {
				if (std::find(v.face_adjacencies.begin(), v.face_adjacencies.end(), f_id) != v.face_adjacencies.end()) continue;
				next_face = f_id;
				v.face_adjacencies.push_back(next_face);
				break;
			}
			unsigned int next_edge = 0;
			for (unsigned int e_id : poly.faces[next_face].id_edges) {
				if (std::find(v.edge_adjacencies.begin(), v.edge_adjacencies.end(), e_id) != v.edge_adjacencies.end()) continue;
				if (std::find(neighbors.begin(), neighbors.end(), e_id) == neighbors.end()) continue;
				next_edge = e_id;
				v.edge_adjacencies.push_back(next_edge);
				break;
			}
			first_edge = next_edge;
		}
	}
}


// Costruisce il poliedro duale (scambia facce e vertici)
Polyhedron dual_polyhedron(const Polyhedron& poly) {
	Polyhedron dual;
	dual.id = poly.id + 2;
	dual.vertices.reserve(poly.n_faces());
	dual.edges.reserve(poly.n_edges());
	dual.faces.reserve(poly.n_vertices());
	for (const auto& face : poly.faces) {
		Vertex v_dual = face_centroid(poly, face.id);
		project_on_unit_sphere(v_dual);
		dual.vertices.push_back(v_dual);
	}
	for (const auto& edge : poly.edges) {
		complete_edge(dual, edge.face_adjacencies[0], edge.face_adjacencies[1], edge.id);
	}
	for (const auto& v : poly.vertices) {
		Face f_dual;
		f_dual.id = v.id;
		for (size_t i = 0; i < v.face_adjacencies.size(); ++i) {
			f_dual.id_vertices.push_back(v.face_adjacencies[i]);
			f_dual.id_edges.push_back(v.edge_adjacencies[i]);
		}
		dual.faces.push_back(f_dual);
	}
	return dual;
}