#pragma once

#include <queue>
#include <list>

#include "Polyhedron.hpp"


// Struttura grafo: lista di adiacenza (più efficiente per BFS/Dijkstra)
struct Graph {
    std::vector<std::vector<unsigned int>> adjacency_list;
};


// Costruzione grafo e pesi
Graph build_adjacency_graph(const Polyhedron& poly);
MatrixXd build_edge_weight_matrix(const Polyhedron& poly);

// Algoritmi di cammino minimo
std::vector<unsigned int> dijkstra_alg(const Graph& g, unsigned int start, unsigned int end, const MatrixXd& W);
std::vector<unsigned int> bfs_alg(const Graph& g, unsigned int start, unsigned int end);
std::vector<unsigned int> find_shortest_path(const Graph& g, unsigned int start, unsigned int end, bool is_uniform_edge_length, const MatrixXd& W);

// Output e visualizzazione
void display_path(const Polyhedron& poly, const std::vector<unsigned int>& path);
void highlight_path(Polyhedron& poly, const std::vector<unsigned int>& path);