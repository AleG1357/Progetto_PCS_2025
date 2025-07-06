#include <iostream>

#include "Utils_test.hpp"
#include "Topology_test.hpp"
#include "Triangulation_test.hpp"
#include "ShortPath_test.hpp"

#include <gtest/gtest.h>


int main(int argc, char *argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}