#pragma once

#include <iostream>
#include <vector>

#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;


// Struttura che memorizza l'ID e le coordinate di un vertice
struct Vertex
{
	unsigned int id = 0;
	Vector3d coords = Vector3d::Zero();
	vector<unsigned int> edge_adjacencies{}; // ID degli spigoli adiacenti
	vector<unsigned int> face_adjacencies{}; // ID delle facce adiacenti
	bool short_path = false; // Per evidenziare i cammini minimi
};

// Struttura che memorizza l'ID di uno spigolo e gli ID dei suoi estremi
struct Edge
{
	unsigned int id = 0;
	unsigned int origin = 0;
	unsigned int end = 0;
	vector<unsigned int> face_adjacencies{}; // ID delle facce adiacenti
	bool short_path = false;
};

// Struttura che memorizza l'ID di una faccia, gli ID dei suoi vertici e spigoli
// e il numero di vertici e spigoli
struct Face
{
	unsigned int id = 0;
	vector<unsigned int> id_vertices{};
	vector<unsigned int> id_edges{};

	// Restituisce il numero di vertici della faccia
	unsigned int n_vertices() const
	{
		return id_vertices.size();
	}

	// Restituisce il numero di spigoli della faccia
	unsigned int n_edges() const
	{
		return id_edges.size();
	}
};

// Struttura che memorizza l'ID di un poliedro, gli ID dei suoi vertici, spigoli e facce
// e il numero di vertici, spigoli e facce
struct Polyhedron
{
	unsigned int id = 0;
	vector<Vertex> vertices{};
	vector<Edge> edges{};
	vector<Face> faces{};

	// Restituisce il numero di vertici del poliedro
	unsigned int n_vertices() const
	{
		return vertices.size();
	}

	// Restituisce il numero di spigoli del poliedro
	unsigned int n_edges() const
	{
		return edges.size();
	}

	// Restituisce il numero di facce del poliedro
	unsigned int n_faces() const
	{
		return faces.size();
	}

	// Controlla che i vertici e gli spigoli di ogni faccia siano coerenti
	bool check_faces() const
	{
		// Itera su tutte le facce del poliedro
		for (const auto& face : faces)
		{
			// Ottieni il numero di spigoli e vertici (ci si aspetta sempre 3)
			unsigned int E = face.n_edges();
			unsigned int V = face.n_vertices();

			// Controlla che siano uguali
			if(E != V)
			{
				cerr << "Numero di spigoli e vertici non corrispondente" << endl;
				return false;
			}

			// Itera su ciascun ID
			for (size_t e = 0; e < E; ++e)
			{
				// Ottieni l'ID dello spigolo e del vertice corrente
				unsigned int id_edge = face.id_edges[e];
				unsigned int id_vertex = face.id_vertices[e];

				// Ottieni lo spigolo corrente
				const Edge& edge = edges[id_edge];

				// L'ID del vertice deve essere uno degli estremi dello spigolo
				if (edge.origin != id_vertex && edge.end != id_vertex)
				{
					cerr << "Errore: vertice e spigolo non corrispondenti" << endl;
					return false;
				}

				// Ottieni l'ID del vertice successivo
				unsigned int id_vertex_next = face.id_vertices[(e + 1) % E];
				
				// Ottieni l'estremo dello spigolo
				unsigned int current_edge_end;
				if (edge.origin == id_vertex)
				{
					current_edge_end = edge.end;
				}
				else
				{
					current_edge_end = edge.origin;
				}

				// L'ID del vertice successivo deve essere l'estremo dello spigolo
				if (current_edge_end != id_vertex_next)
				{
					cerr << "Errore: discontinuità tra spigoli" << endl;
					return false;
				}
			}
		}
		
		return true;
	}
};