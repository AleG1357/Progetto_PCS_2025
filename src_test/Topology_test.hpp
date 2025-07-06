#pragma once

#include <gtest/gtest.h>

#include "Topology.hpp"
#include "Utils.hpp"

using namespace std;




// Geometry TEST

TEST(GeometryTest, ProjectOnUnitSphere)
{
	Vertex v;
	v.coords = {-2.0, 7.0, 3.5};
	project_on_unit_sphere(v);
	double length = v.coords.norm();
	ASSERT_NEAR(length, 1.0, 1e-12);
	// Check direction is preserved (dot product sign)
	Vector3d original = {-2.0, 7.0, 3.5};
	ASSERT_GT(v.coords.dot(original), 0.0);
}

TEST(GeometryTest, FaceCentroid)
{
	Polyhedron P;
	P.vertices = {
		{0, {2, 1, 0}},
		{1, {4, 1, 0}},
		{2, {2, 5, 0}}
	};
	Face f;
	f.id = 0;
	f.id_vertices = {0, 1, 2};
	P.faces.push_back(f);
	Vertex bc = face_centroid(P, 0);
	Vector3d expected = {(2+4+2)/3.0, (1+1+5)/3.0, 0};
	ASSERT_TRUE(bc.coords.isApprox(expected, 1e-12));
}


// DualTest


TEST(GeometryTest, AssignEdgeAdjacencies_Octahedron)
{
	Polyhedron P = build_platonic_solid(4, 3); // Ottaedro
	assign_edge_adjacencies(P);
	// Ogni spigolo deve appartenere a 2 facce
	for (const auto& e : P.edges) {
		ASSERT_EQ(e.face_adjacencies.size(), 2);
	}
}



TEST(GeometryTest, AssignVertexAdjacencies_Tetrahedron)
{
	Polyhedron P = build_platonic_solid(3, 3); // Tetraedro
	assign_edge_adjacencies(P);
	assign_vertex_adjacencies(P);
	// Ogni vertice deve avere almeno 3 edge_adjacencies e 3 face_adjacencies
	for (const auto& v : P.vertices) {
		ASSERT_GE(v.edge_adjacencies.size(), 3);
		ASSERT_GE(v.face_adjacencies.size(), 3);
	}
}


TEST(GeometryTest, MakeDualPolyhedron_Tetrahedron)
{
	Polyhedron P = build_platonic_solid(3, 3); // Tetraedro
	assign_edge_adjacencies(P);
	assign_vertex_adjacencies(P);
	Polyhedron Q = dual_polyhedron(P);
	// Il duale del tetraedro ha 4 vertici, 6 spigoli, 4 facce
	ASSERT_EQ(Q.n_vertices(), 4);
	ASSERT_EQ(Q.n_edges(), 6);
	ASSERT_EQ(Q.n_faces(), 4);
	// Tutti i vertici devono essere normalizzati
	for (const auto& v : Q.vertices) {
		ASSERT_NEAR(v.coords.norm(), 1.0, 1e-12);
	}
	// Tutte le facce devono essere triangoli
	for (const auto& f : Q.faces) {
		ASSERT_EQ(f.id_vertices.size(), 3);
		ASSERT_EQ(f.id_edges.size(), 3);
	}
	// Il duale deve essere coerente
	ASSERT_TRUE(Q.check_faces());
}