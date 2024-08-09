#include "catch2/catch.hpp"
#include "sdu_controllers/sdu_controllers.hpp"

using namespace sdu_controllers;

TEST_CASE("add_one", "[adder]")
{
  REQUIRE(add_one(0) == 1);
  REQUIRE(add_one(123) == 124);
  REQUIRE(add_one(-1) == 0);
}
