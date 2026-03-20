#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Way.hpp"
#include "utils/Params.hpp"

#include <autodrive_msgs/HUAT_Cone.h>
#include <gtest/gtest.h>

namespace {

Edge MakeTypedEdge(uint32_t id_left, uint32_t left_type, uint32_t id_right, uint32_t right_type) {
  autodrive_msgs::HUAT_Cone left_msg;
  left_msg.id = id_left;
  left_msg.position_baseLink.x = 0.0;
  left_msg.position_baseLink.y = 1.0;
  left_msg.position_global.x = 0.0;
  left_msg.position_global.y = 1.0;
  left_msg.type = left_type;

  autodrive_msgs::HUAT_Cone right_msg;
  right_msg.id = id_right;
  right_msg.position_baseLink.x = 0.0;
  right_msg.position_baseLink.y = -1.0;
  right_msg.position_global.x = 0.0;
  right_msg.position_global.y = -1.0;
  right_msg.type = right_type;

  return Edge(Node(left_msg), Node(right_msg));
}

}  // namespace

TEST(WayColorSemanticsTest, RedBlueSemanticOverrideSwapsToLeftRedRightBlue) {
  Params::WayComputer::Way params{};
  params.max_dist_loop_closure = 1.0;
  params.max_angle_diff_loop_closure = 0.6;
  params.vital_num_midpoints = 5;
  Way::init(params);

  Way way;
  way.addEdge(MakeTypedEdge(1U, 0U, 2U, 3U));  // geometric left=BLUE, right=RED

  const Tracklimits limits = way.getTracklimits();
  ASSERT_FALSE(limits.first.empty());
  ASSERT_FALSE(limits.second.empty());

  EXPECT_EQ(3U, limits.first.front().type());   // LEFT=RED
  EXPECT_EQ(0U, limits.second.front().type());  // RIGHT=BLUE
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
