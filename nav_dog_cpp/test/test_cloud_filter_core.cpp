#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "nav_dog_cpp/cloud_filter_core.hpp"

namespace nav_dog_cpp {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(TransformTest, RotatesYawNinetyAndTranslates) {
  const Transform transform = make_transform(
      0.0, 0.0, std::sin(kPi / 4.0), std::cos(kPi / 4.0), 10.0, 2.0, -0.1);
  const PointXYZI point = transform_point(transform, 2.0F, 0.0F, 0.0F, 7.0F);
  EXPECT_NEAR(point.x, 10.0, 1e-5);
  EXPECT_NEAR(point.y, 4.0, 1e-5);
  EXPECT_NEAR(point.z, -0.1, 1e-5);
  EXPECT_FLOAT_EQ(point.intensity, 7.0F);
}

TEST(VoxelHashSetTest, DeduplicatesAndSupportsGrowth) {
  VoxelHashSet set(2);
  EXPECT_TRUE(set.insert(1));
  EXPECT_FALSE(set.insert(1));
  for (uint64_t value = 2; value < 10000; ++value) {
    EXPECT_TRUE(set.insert(value));
  }
  EXPECT_EQ(set.size(), 9999U);
  EXPECT_GT(set.capacity(), set.size());
}

TEST(VoxelGridTest, KeepsPointsOutsidePackedKeyRange) {
  VoxelGrid grid(0.08, 16);
  EXPECT_TRUE(grid.insert(PointXYZI{1.0e30F, 0.0F, 0.0F, 0.0F}));
}

TEST(VoxelGridTest, KeepsOnePointPerCell) {
  VoxelGrid grid(0.08, 16);
  EXPECT_TRUE(grid.insert(PointXYZI{0.00F, 0.00F, 0.00F, 0.0F}));
  EXPECT_FALSE(grid.insert(PointXYZI{0.01F, 0.01F, 0.01F, 0.0F}));
  EXPECT_TRUE(grid.insert(PointXYZI{0.50F, 0.00F, 0.00F, 0.0F}));
}

TEST(SelfFilterTest, RejectsBodyCylinderOnly) {
  const Transform transform = make_transform(0.0, 0.0, 0.0, 1.0, 10.0, 2.0, 0.0);
  EXPECT_TRUE(is_self_return(PointXYZI{10.2F, 2.1F, 0.1F, 0.0F},
                             transform, 0.7 * 0.7, 0.65));
  EXPECT_FALSE(is_self_return(PointXYZI{11.0F, 2.0F, 0.1F, 0.0F},
                              transform, 0.7 * 0.7, 0.65));
  EXPECT_FALSE(is_self_return(PointXYZI{10.2F, 2.1F, 0.8F, 0.0F},
                              transform, 0.7 * 0.7, 0.65));
}

TEST(TransformTest, RejectsInvalidQuaternion) {
  EXPECT_THROW(make_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
               std::invalid_argument);
}

TEST(TransformTest, RejectsNonFiniteTranslation) {
  EXPECT_THROW(make_transform(0.0, 0.0, 0.0, 1.0,
                              std::numeric_limits<double>::infinity(), 0.0, 0.0),
               std::invalid_argument);
}

}  // namespace
}  // namespace nav_dog_cpp
