#include <gtest/gtest.h>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Way.hpp"

namespace {

Edge MakeEdge(const uint32_t id_base, const double x0, const double y0, const double x1, const double y1) {
  const Node n0(x0, y0, x0, y0, id_base);
  const Node n1(x1, y1, x1, y1, id_base + 1U);
  return Edge(n0, n1);
}

}  // namespace

TEST(WaySafetyTest, QuinEhNoCommonMidpointReturnsReplanWithoutCrash) {
  Params::WayComputer::Way params{};
  params.max_dist_loop_closure = 1.0;
  params.max_angle_diff_loop_closure = 0.6;
  params.vital_num_midpoints = 5;
  Way::init(params);

  Way way_a;
  Way way_b;
  way_a.addEdge(MakeEdge(1U, 0.0, 0.0, 1.0, 0.0));
  way_a.addEdge(MakeEdge(3U, 1.0, 0.0, 2.0, 0.0));
  way_b.addEdge(MakeEdge(11U, 0.0, 1.0, 1.0, 1.0));
  way_b.addEdge(MakeEdge(13U, 1.0, 1.0, 2.0, 1.0));

  EXPECT_TRUE(way_a.quinEhLobjetiuDeLaSevaDiresio(way_b));
}

TEST(WaySafetyTest, GetNextPathPointOnEmptyWayReturnsOrigin) {
  Way way;
  const Point p = way.getNextPathPoint();
  EXPECT_DOUBLE_EQ(p.x, 0.0);
  EXPECT_DOUBLE_EQ(p.y, 0.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
