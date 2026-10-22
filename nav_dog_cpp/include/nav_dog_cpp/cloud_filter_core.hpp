#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace nav_dog_cpp {

struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;
};
static_assert(sizeof(PointXYZI) == 16, "PointXYZI must remain a compact 16-byte point");
static_assert(std::is_trivially_copyable<PointXYZI>::value,
              "PointXYZI must support contiguous serialization");

struct Transform {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

inline Transform make_transform(const double qx, const double qy, const double qz,
                                const double qw, const double tx, const double ty,
                                const double tz) {
  Eigen::Quaterniond quaternion(qw, qx, qy, qz);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1e-12 || !std::isfinite(tx) ||
      !std::isfinite(ty) || !std::isfinite(tz)) {
    throw std::invalid_argument("invalid pose transform");
  }
  quaternion.normalize();
  Transform transform;
  transform.rotation = quaternion.toRotationMatrix();
  transform.translation = Eigen::Vector3d(tx, ty, tz);
  return transform;
}

inline PointXYZI transform_point(const Transform& transform,
                                 const float x, const float y, const float z,
                                 const float intensity) {
  const Eigen::Matrix3d& r = transform.rotation;
  const Eigen::Vector3d& t = transform.translation;
  return PointXYZI{
      static_cast<float>(r(0, 0) * x + r(0, 1) * y + r(0, 2) * z + t.x()),
      static_cast<float>(r(1, 0) * x + r(1, 1) * y + r(1, 2) * z + t.y()),
      static_cast<float>(r(2, 0) * x + r(2, 1) * y + r(2, 2) * z + t.z()),
      intensity,
  };
}

inline uint64_t pack_voxel_key(int32_t x, int32_t y, int32_t z) {
  constexpr uint64_t kMask = (uint64_t{1} << 21) - 1;
  return ((static_cast<uint64_t>(static_cast<uint32_t>(x)) & kMask) << 42) |
         ((static_cast<uint64_t>(static_cast<uint32_t>(y)) & kMask) << 21) |
         (static_cast<uint64_t>(static_cast<uint32_t>(z)) & kMask);
}

inline bool fast_floor_to_int32(double value, int32_t* result) {
  // pack_voxel_key stores signed 21-bit voxel coordinates.
  constexpr double kHalfRange = 1048576.0;
  if (!std::isfinite(value) || value < -kHalfRange || value >= kHalfRange) {
    return false;
  }
  const int64_t truncated = static_cast<int64_t>(value);
  *result = static_cast<int32_t>(value < static_cast<double>(truncated)
                                     ? truncated - 1
                                     : truncated);
  return true;
}

inline uint64_t splitmix64(uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

// Open-addressed set with no per-point allocation. Capacity is kept at a
// power of two and grows before load factor exceeds 0.70.
class VoxelHashSet {
 public:
  explicit VoxelHashSet(size_t expected_points = 8192) {
    reserve(expected_points);
  }

  void reserve(size_t expected_points) {
    size_t required = std::max<size_t>(16, expected_points + expected_points / 2);
    size_t capacity = 16;
    while (capacity < required) {
      capacity <<= 1;
    }
    if (capacity <= keys_.size()) {
      return;
    }
    rehash(capacity);
  }

  bool insert(uint64_t key) {
    if (keys_.empty()) {
      rehash(16);
    } else if ((size_ + 1) * 10 >= keys_.size() * 7) {
      rehash(keys_.size() << 1);
    }
    const size_t mask = keys_.size() - 1;
    size_t index = static_cast<size_t>(splitmix64(key)) & mask;
    while (used_[index]) {
      if (keys_[index] == key) {
        return false;
      }
      index = (index + 1) & mask;
    }
    used_[index] = 1;
    keys_[index] = key;
    ++size_;
    return true;
  }

  size_t size() const { return size_; }
  size_t capacity() const { return keys_.size(); }

  void clear() {
    std::fill(used_.begin(), used_.end(), uint8_t{0});
    size_ = 0;
  }

 private:
  void rehash(size_t capacity) {
    std::vector<uint64_t> old_keys = std::move(keys_);
    std::vector<uint8_t> old_used = std::move(used_);
    keys_.assign(capacity, 0);
    used_.assign(capacity, 0);
    size_ = 0;
    for (size_t i = 0; i < old_keys.size(); ++i) {
      if (old_used[i]) {
        insert(old_keys[i]);
      }
    }
  }

  std::vector<uint64_t> keys_;
  std::vector<uint8_t> used_;
  size_t size_ = 0;
};

class VoxelGrid {
 public:
  explicit VoxelGrid(double voxel_size, size_t expected_points = 8192)
      : inverse_voxel_size_(voxel_size > 0.0 ? 1.0 / voxel_size : 0.0),
        occupied_(expected_points) {}

  size_t voxel_key(const PointXYZI& point, uint64_t* key) const {
    if (inverse_voxel_size_ <= 0.0) {
      return 0;
    }
    int32_t ix = 0;
    int32_t iy = 0;
    int32_t iz = 0;
    if (!fast_floor_to_int32(point.x * inverse_voxel_size_, &ix) ||
        !fast_floor_to_int32(point.y * inverse_voxel_size_, &iy) ||
        !fast_floor_to_int32(point.z * inverse_voxel_size_, &iz)) {
      return 0;
    }
    *key = pack_voxel_key(ix, iy, iz);
    return 1;
  }

  void reserve(size_t expected_points) { occupied_.reserve(expected_points); }

  void clear() { occupied_.clear(); }

  bool insert(const PointXYZI& point) {
    uint64_t key = 0;
    if (voxel_key(point, &key) == 0) {
      return true;
    }
    return occupied_.insert(key);
  }

 private:
  double inverse_voxel_size_;
  VoxelHashSet occupied_;
};

inline bool is_self_return(const PointXYZI& point, const Transform& transform,
                           double radius_squared, double z_band) {
  const double dx = static_cast<double>(point.x) - transform.translation.x();
  const double dy = static_cast<double>(point.y) - transform.translation.y();
  const double dz = static_cast<double>(point.z) - transform.translation.z();
  return dx * dx + dy * dy <= radius_squared && std::abs(dz) <= z_band;
}


}  // namespace nav_dog_cpp
