// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "test_path_following_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace test_path_following_controller;

TEST_F(FixturePFC, test_setup)
{
  ASSERT_EQ(1,1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
