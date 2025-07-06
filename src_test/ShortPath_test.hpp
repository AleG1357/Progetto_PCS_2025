#pragma once

#include <gtest/gtest.h>

#include "Utils.hpp"
#include "ShortPath.hpp"


using namespace std;



// Graph TEST

TEST(GraphTest, BuildAdjacencyGraph_Octahedron)
{
    Polyhedron P = build_platonic_solid(4, 3); // Ottaedro
    assign_edge_adjacencies(P);
    assign_vertex_adjacencies(P);
    Graph graph = build_adjacency_graph(P);
    ASSERT_EQ(graph.adjacency_list.size(), P.n_vertices());
    // Ogni vertice dell'ottaedro ha 4 adiacenze
    for (unsigned int i = 0; i < P.n_vertices(); ++i) {
        ASSERT_EQ(graph.adjacency_list[i].size(), 4);
    }
}

TEST(GraphTest, BuildEdgeWeightMatrix_Symmetry)
{
    Polyhedron P = build_platonic_solid(3, 5); // Icosaedro
    assign_edge_adjacencies(P);
    assign_vertex_adjacencies(P);
    MatrixXd weights = build_edge_weight_matrix(P);
    ASSERT_EQ(weights.rows(), P.n_vertices());
    ASSERT_EQ(weights.cols(), P.n_vertices());
    for (unsigned int i = 0; i < weights.rows(); ++i) {
        for (unsigned int j = 0; j < weights.cols(); ++j) {
            ASSERT_DOUBLE_EQ(weights(i, j), weights(j, i));
        }
    }
}


TEST(GraphTest, FindShortestPath_BFS_Tetrahedron)
{
    Polyhedron P = build_platonic_solid(3, 3); // Tetraedro
    assign_edge_adjacencies(P);
    assign_vertex_adjacencies(P);
    Graph graph = build_adjacency_graph(P);
    MatrixXd weights = build_edge_weight_matrix(P);
    // Tra 0 e 2 ci sono due cammini minimi equivalenti
    unsigned int id_start = 0, id_end = 2;
    std::vector<unsigned int> path = find_shortest_path(graph, id_start, id_end, true, weights);
    ASSERT_GE(path.size(), 2);
    ASSERT_EQ(path.front(), id_start);
    ASSERT_EQ(path.back(), id_end);
}

TEST(GraphTest, FindShortestPath_Dijkstra_Octahedron)
{
    Polyhedron P = build_platonic_solid(4, 3); // Ottaedro
    assign_edge_adjacencies(P);
    assign_vertex_adjacencies(P);
    Graph graph = build_adjacency_graph(P);
    MatrixXd weights = build_edge_weight_matrix(P);
    unsigned int id_start = 0, id_end = 5;
    std::vector<unsigned int> path = find_shortest_path(graph, id_start, id_end, false, weights);
    ASSERT_GE(path.size(), 2);
    ASSERT_EQ(path.front(), id_start);
    ASSERT_EQ(path.back(), id_end);
}