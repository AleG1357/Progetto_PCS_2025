#pragma once

#include <gtest/gtest.h>

#include "Triangulation.hpp"


using namespace std;




// Test complete_vertex 
TEST(TriangleTest, CompleteVertex)
{
	Polyhedron P;
	Vector3d v0 = {2.5, -1.1, 0.0};
	unsigned int id0 = complete_vertex(P, v0);
	ASSERT_EQ(id0, 0);
	ASSERT_EQ(P.n_vertices(), 1);
	// Aggiunta di un duplicato quasi identico
	Vector3d v1 = {2.5 + 1e-13, -1.1 - 1e-13, 0.0};
	unsigned int id1 = complete_vertex(P, v1);
	ASSERT_EQ(id1, 0);
	ASSERT_EQ(P.n_vertices(), 1);
	// Aggiunta di un nuovo vertice
	Vector3d v2 = {0.0, 0.0, 3.3};
	unsigned int id2 = complete_vertex(P, v2);
	ASSERT_EQ(id2, 1);
	ASSERT_EQ(P.n_vertices(), 2);
}



TEST(TriangleTest, CompleteEdge_AndForcedID)
{
	Polyhedron P;
	P.vertices = {{0, {0,0,0}}, {1, {1,0,0}}, {2, {0,2,0}}};
	// Aggiunta di un edge valido
	unsigned int id0 = complete_edge(P, 0, 2);
	ASSERT_EQ(id0, 0);
	ASSERT_EQ(P.n_edges(), 1);
	// Aggiunta di un duplicato (ordine inverso)
	unsigned int id1 = complete_edge(P, 2, 0);
	ASSERT_EQ(id1, 0);
	ASSERT_EQ(P.n_edges(), 1);
	// Edge non valido (stesso vertice)
	unsigned int id2 = complete_edge(P, 1, 1);
	ASSERT_EQ(id2, numeric_limits<unsigned int>::max());
	// Edge con ID forzato
	unsigned int id3 = complete_edge(P, 1, 2, 7);
	ASSERT_EQ(id3, 7);
	ASSERT_EQ(P.n_edges(), 2);
}



// Test triangolazione Class I con delta=3 su ottaedro
TEST(TriangleTest, TriangulateClassI_Octahedron)
{
	Polyhedron P_old = build_platonic_solid(4, 3); // Ottaedro
	Polyhedron P = triangulate_classI(P_old, 3);
	// Valori attesi dipendono dall'implementazione, ma verifichiamo proprietà strutturali
	ASSERT_GT(P.n_vertices(), P_old.n_vertices());
	ASSERT_GT(P.n_edges(), P_old.n_edges());
	ASSERT_GT(P.n_faces(), P_old.n_faces());
	for (const auto& f : P.faces) {
		ASSERT_EQ(f.id_vertices.size(), 3);
		ASSERT_EQ(f.id_edges.size(), 3);
	}
	for (const auto& e : P.edges) {
		ASSERT_NE(e.origin, e.end);
		ASSERT_LT(e.origin, P.n_vertices());
		ASSERT_LT(e.end, P.n_vertices());
	}
	ASSERT_TRUE(P.check_faces());
}

// Test triangolazione Class II con delta=2 su icosaedro
TEST(TriangleTest, TriangulateClassII_Icosahedron)
{
	Polyhedron P_old = build_platonic_solid(3, 5); // Icosaedro
	Polyhedron P = triangulate_classII(P_old, 2);
	ASSERT_GT(P.n_vertices(), P_old.n_vertices());
	ASSERT_GT(P.n_edges(), P_old.n_edges());
	ASSERT_GT(P.n_faces(), P_old.n_faces());
	for (const auto& f : P.faces) {
		ASSERT_EQ(f.id_vertices.size(), 3);
		ASSERT_EQ(f.id_edges.size(), 3);
	}
	for (const auto& e : P.edges) {
		ASSERT_NE(e.origin, e.end);
		ASSERT_LT(e.origin, P.n_vertices());
		ASSERT_LT(e.end, P.n_vertices());
	}
	ASSERT_TRUE(P.check_faces());
}

// Test triangolazione Class I con delta=1 (deve restituire il poliedro originale)
TEST(TriangleTest, TriangulateClassI_Val1_ReturnsOriginal)
{
	Polyhedron P_old = build_platonic_solid(3, 3); // Tetraedro
	Polyhedron P = triangulate_classI(P_old, 1);
	ASSERT_EQ(P.n_vertices(), P_old.n_vertices());
	ASSERT_EQ(P.n_edges(), P_old.n_edges());
	ASSERT_EQ(P.n_faces(), P_old.n_faces());
}