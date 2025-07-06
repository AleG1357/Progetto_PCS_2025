#pragma once

#include <gtest/gtest.h>

#include "Utils.hpp"


using namespace std;


// Test su build_platonic_solid

TEST(UtilsTest, BuildPlatonicSolid_Tetrahedron)
{
	Polyhedron P = build_platonic_solid(3, 3);
	ASSERT_EQ(P.n_vertices(), 4);
	ASSERT_EQ(P.n_edges(), 6);
	ASSERT_EQ(P.n_faces(), 4);
	ASSERT_TRUE(P.check_faces());
}

TEST(UtilsTest, BuildPlatonicSolid_Octahedron)
{
	Polyhedron P = build_platonic_solid(4, 3);
	ASSERT_EQ(P.n_vertices(), 6);
	ASSERT_EQ(P.n_edges(), 12);
	ASSERT_EQ(P.n_faces(), 8);
	ASSERT_TRUE(P.check_faces());
}

TEST(UtilsTest, BuildPlatonicSolid_Icosahedron)
{
	Polyhedron P = build_platonic_solid(3, 5);
	ASSERT_EQ(P.n_vertices(), 12);
	ASSERT_EQ(P.n_edges(), 30);
	ASSERT_EQ(P.n_faces(), 20);
	ASSERT_TRUE(P.check_faces());
}

TEST(UtilsTest, BuildPlatonicSolid_Invalid)
{
	// Parametri non validi: deve lanciare eccezione
	EXPECT_THROW(build_platonic_solid(2, 2), std::exception);
}
