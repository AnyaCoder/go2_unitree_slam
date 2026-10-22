#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_dog_cpp/cloud_filter_core.hpp"

namespace nav_dog_cpp {
namespace {

using Clock = std::chrono::steady_clock;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr uint32_t kNoFieldOffset = std::numeric_limits<uint32_t>::max();

struct CloudLayout {
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  uint32_t intensity_offset = kNoFieldOffset;
  uint8_t x_type = PointField::FLOAT32;
  uint8_t y_type = PointField::FLOAT32;
  uint8_t z_type = PointField::FLOAT32;
  uint8_t intensity_type = PointField::FLOAT32;
};

size_t field_size(uint8_t datatype) {
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

template <typename T>
inline T read_unaligned(const uint8_t* address) {
  T value;
  std::memcpy(&value, address, sizeof(T));
  return value;
}

inline float read_numeric(const uint8_t* address, uint8_t datatype) {
  switch (datatype) {
    case PointField::INT8:
      return static_cast<float>(read_unaligned<int8_t>(address));
    case PointField::UINT8:
      return static_cast<float>(read_unaligned<uint8_t>(address));
    case PointField::INT16:
      return static_cast<float>(read_unaligned<int16_t>(address));
    case PointField::UINT16:
      return static_cast<float>(read_unaligned<uint16_t>(address));
    case PointField::INT32:
      return static_cast<float>(read_unaligned<int32_t>(address));
    case PointField::UINT32:
      return static_cast<float>(read_unaligned<uint32_t>(address));
    case PointField::FLOAT32:
      return read_unaligned<float>(address);
    case PointField::FLOAT64:
      return static_cast<float>(read_unaligned<double>(address));
    default:
      throw std::invalid_argument("unsupported PointCloud2 field datatype");
  }
}

const PointField* find_field(const PointCloud2& cloud, const std::string& name) {
  for (const PointField& field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

CloudLayout parse_layout(const PointCloud2& cloud) {
  if (cloud.is_bigendian) {
    throw std::invalid_argument("big-endian PointCloud2 is unsupported");
  }
  if (cloud.point_step == 0) {
    throw std::invalid_argument("point_step must be positive");
  }

  const PointField* x = find_field(cloud, "x");
  const PointField* y = find_field(cloud, "y");
  const PointField* z = find_field(cloud, "z");
  if (x == nullptr || y == nullptr || z == nullptr) {
    throw std::invalid_argument("cloud must contain x/y/z fields");
  }

  CloudLayout layout;
  layout.point_step = cloud.point_step;
  layout.width = cloud.width;
  layout.height = std::max<uint32_t>(1, cloud.height);
  layout.x_offset = x->offset;
  layout.y_offset = y->offset;
  layout.z_offset = z->offset;
  layout.x_type = x->datatype;
  layout.y_type = y->datatype;
  layout.z_type = z->datatype;

  for (const PointField* field : {x, y, z}) {
    const size_t size = field_size(field->datatype);
    if (field->count != 1 || size == 0 ||
        static_cast<uint64_t>(field->offset) + size > layout.point_step) {
      throw std::invalid_argument("invalid x/y/z field layout");
    }
  }

  if (const PointField* intensity = find_field(cloud, "intensity")) {
    const size_t size = field_size(intensity->datatype);
    if (intensity->count != 0 && size != 0 &&
        static_cast<uint64_t>(intensity->offset) + size <= layout.point_step) {
      layout.intensity_offset = intensity->offset;
      layout.intensity_type = intensity->datatype;
    }
  }

  const uint64_t minimum_row_step =
      static_cast<uint64_t>(layout.width) * layout.point_step;
  layout.row_step = cloud.row_step == 0
                        ? static_cast<uint32_t>(std::min<uint64_t>(
                              minimum_row_step, std::numeric_limits<uint32_t>::max()))
                        : cloud.row_step;
  if (layout.row_step < minimum_row_step) {
    throw std::invalid_argument("row_step is smaller than width * point_step");
  }
  if (layout.width != 0) {
    const uint64_t required =
        static_cast<uint64_t>(layout.row_step) * (layout.height - 1U) +
        minimum_row_step;
    if (required > cloud.data.size()) {
      throw std::invalid_argument("PointCloud2 data is shorter than its layout");
    }
  }
  return layout;
}

inline PointXYZI read_point(const uint8_t* address, const CloudLayout& layout) {
  const float intensity =
      layout.intensity_offset == kNoFieldOffset
          ? 0.0F
          : read_numeric(address + layout.intensity_offset, layout.intensity_type);
  return PointXYZI{
      read_numeric(address + layout.x_offset, layout.x_type),
      read_numeric(address + layout.y_offset, layout.y_type),
      read_numeric(address + layout.z_offset, layout.z_type),
      intensity,
  };
}

PointField make_field(const std::string& name, uint32_t offset, uint8_t datatype) {
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

}  // namespace

class CloudToMapFilter : public rclcpp::Node {
 public:
  CloudToMapFilter() : Node("cloud_to_map_filter") {
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    input_frame_ = declare_parameter<std::string>("input_frame", "base_link");
    input_topic_ = declare_parameter<std::string>(
        "input_topic", "/lio_sam_ros2/deskew/cloud_deskewed");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/nav/cloud_registered_map");
    pose_topic_ = declare_parameter<std::string>(
        "pose_topic", "/nav/state_estimation");
    pose_timeout_seconds_ = declare_parameter<double>("pose_timeout", 0.40);
    output_rate_hz_ = declare_parameter<double>("output_rate_hz", 5.0);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.08);
    self_radius_ = declare_parameter<double>("self_radius", 0.70);
    self_z_band_ = std::abs(declare_parameter<double>("self_z_band", 0.65));
    expected_points_ = static_cast<size_t>(
        std::max<int64_t>(1024, declare_parameter<int64_t>("expected_points", 24000)));

    self_radius_squared_ = self_radius_ * self_radius_;
    voxel_grid_ = std::make_unique<VoxelGrid>(voxel_size_, expected_points_);
    output_period_ = output_rate_hz_ > 0.0
                          ? std::chrono::duration<double>(1.0 / output_rate_hz_)
                          : std::chrono::duration<double>::zero();
    resize_scratch(expected_points_);

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    sensor_qos.best_effort().durability_volatile();
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    pose_qos.best_effort().durability_volatile();
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    output_qos.reliable().durability_volatile();

    pose_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        pose_topic_, pose_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          pose_callback(*message);
        });
    cloud_subscription_ = create_subscription<PointCloud2>(
        input_topic_, sensor_qos,
        [this](const PointCloud2::SharedPtr message) {
          cloud_callback(*message);
        });
    publisher_ = create_publisher<PointCloud2>(output_topic_, output_qos);

    output_message_.header.frame_id = target_frame_;
    output_message_.height = 1;
    output_message_.fields = {
        make_field("x", 0, PointField::FLOAT32),
        make_field("y", 4, PointField::FLOAT32),
        make_field("z", 8, PointField::FLOAT32),
        make_field("intensity", 12, PointField::FLOAT32),
    };
    output_message_.point_step = sizeof(PointXYZI);
    output_message_.is_bigendian = false;
    output_message_.is_dense = true;

    RCLCPP_INFO(
        get_logger(),
        "C++ cloud filter: %s [%s] @ <=%.1f Hz -> %s [%s], pose=%s, "
        "voxel=%.2f m, self_radius=%.2f m",
        input_topic_.c_str(), input_frame_.c_str(), output_rate_hz_,
        output_topic_.c_str(), target_frame_.c_str(), pose_topic_.c_str(),
        voxel_size_, self_radius_);
  }

 private:
  void resize_scratch(size_t expected_points) {
    filtered_points_.reserve(expected_points);
    voxel_grid_->reserve(expected_points);
    output_message_.data.reserve(expected_points * sizeof(PointXYZI));
  }

  void pose_callback(const nav_msgs::msg::Odometry& message) {
    const std::string& frame = message.header.frame_id;
    const std::string& child_frame = message.child_frame_id;
    if (frame != target_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "drop navigation pose in frame '%s'; expected '%s'",
          frame.c_str(), target_frame_.c_str());
      return;
    }
    if (child_frame != input_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "drop navigation pose for child frame '%s'; expected '%s'",
          child_frame.c_str(), input_frame_.c_str());
      return;
    }

    const auto& pose = message.pose.pose;
    try {
      pose_transform_ = make_transform(
          pose.orientation.x, pose.orientation.y, pose.orientation.z,
          pose.orientation.w, pose.position.x, pose.position.y,
          pose.position.z);
      pose_received_at_ = Clock::now();
      has_pose_ = true;
    } catch (const std::exception& error) {
      has_pose_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "drop invalid navigation pose: %s", error.what());
    }
  }

  void navigation_transform(const std::string& source_frame, Transform* transform) {
    if (!has_pose_) {
      throw std::runtime_error("navigation pose has not arrived");
    }
    const auto pose_age = Clock::now() - pose_received_at_;
    if (std::chrono::duration<double>(pose_age).count() > pose_timeout_seconds_) {
      throw std::runtime_error("navigation pose is stale");
    }
    if (source_frame == target_frame_) {
      *transform = Transform{};
      return;
    }
    if (source_frame != input_frame_) {
      throw std::runtime_error(
          "unexpected cloud frame '" + source_frame + "'; expected '" +
          input_frame_ + "' or '" + target_frame_ + "'");
    }
    *transform = pose_transform_;
  }

  void cloud_callback(const PointCloud2& message) {
    ++received_;
    const Clock::time_point now = Clock::now();
    if (output_period_ > std::chrono::duration<double>::zero() &&
        last_publish_at_ != Clock::time_point{} &&
        now - last_publish_at_ < output_period_ * 0.90) {
      ++rate_skipped_;
      report_if_due(now);
      return;
    }

    try {
      const std::string source_frame = message.header.frame_id;
      Transform transform;
      navigation_transform(source_frame, &transform);
      const CloudLayout layout = parse_layout(message);

      filtered_points_.clear();
      voxel_grid_->clear();
      size_t dropped_self = 0;
      size_t dropped_voxel = 0;
      const uint64_t point_count =
          static_cast<uint64_t>(layout.width) * layout.height;
      if (point_count > filtered_points_.capacity()) {
        resize_scratch(static_cast<size_t>(point_count));
      }
      filtered_points_.reserve(std::min<size_t>(
          static_cast<size_t>(point_count), filtered_points_.capacity()));

      for (uint32_t row = 0; row < layout.height; ++row) {
        const uint8_t* row_address =
            message.data.data() + static_cast<size_t>(row) * layout.row_step;
        for (uint32_t column = 0; column < layout.width; ++column) {
          const uint8_t* point_address =
              row_address + static_cast<size_t>(column) * layout.point_step;
          const PointXYZI source = read_point(point_address, layout);
          if (!(std::isfinite(source.x) && std::isfinite(source.y) &&
                std::isfinite(source.z))) {
            continue;
          }
          const PointXYZI point =
              transform_point(transform, source.x, source.y, source.z,
                              source.intensity);
          if (!(std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))) {
            continue;
          }
          if (is_self_return(point, transform, self_radius_squared_, self_z_band_)) {
            ++dropped_self;
            continue;
          }
          if (voxel_grid_->insert(point)) {
            filtered_points_.push_back(point);
          } else {
            ++dropped_voxel;
          }
        }
      }

      publish(message);
      ++published_;
      last_publish_at_ = now;
      dropped_self_total_ += dropped_self;
      dropped_voxel_total_ += dropped_voxel;
      last_dropped_self_ = dropped_self;
      last_dropped_voxel_ = dropped_voxel;
      const double milliseconds =
          std::chrono::duration<double, std::milli>(Clock::now() - now).count();
      max_process_ms_ = std::max(max_process_ms_, milliseconds);
      process_ms_sum_ += milliseconds;
      ++processed_frames_;
    } catch (const std::exception& error) {
      ++pose_failures_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "drop cloud until navigation pose/layout is valid: %s", error.what());
    }
    report_if_due(Clock::now());
  }

  void publish(const PointCloud2& source) {
    output_message_.header.stamp = source.header.stamp;
    output_message_.width = static_cast<uint32_t>(filtered_points_.size());
    output_message_.row_step =
        output_message_.width * output_message_.point_step;
    output_message_.data.resize(
        filtered_points_.size() * sizeof(PointXYZI));
    if (!filtered_points_.empty()) {
      std::memcpy(output_message_.data.data(), filtered_points_.data(),
                  output_message_.data.size());
    }
    publisher_->publish(output_message_);
  }

  void report_if_due(Clock::time_point now) {
    if (last_report_at_ != Clock::time_point{} &&
        now - last_report_at_ < std::chrono::seconds(5)) {
      return;
    }
    last_report_at_ = now;
    const double pose_age =
        has_pose_
            ? std::chrono::duration<double>(now - pose_received_at_).count()
            : std::numeric_limits<double>::infinity();
    const double mean_process_ms =
        processed_frames_ == 0 ? 0.0 : process_ms_sum_ / processed_frames_;
    RCLCPP_INFO(
        get_logger(),
        "cloud filter: received=%zu published=%zu rate_skipped=%zu "
        "pose_failures=%zu pose_age=%.3fs points=%zu "
        "self(last/total)=%zu/%zu voxel(last/total)=%zu/%zu "
        "process_ms(mean/max)=%.3f/%.3f",
        received_, published_, rate_skipped_, pose_failures_, pose_age,
        filtered_points_.size(), last_dropped_self_, dropped_self_total_,
        last_dropped_voxel_, dropped_voxel_total_,
        mean_process_ms, max_process_ms_);
    process_ms_sum_ = 0.0;
    processed_frames_ = 0;
    max_process_ms_ = 0.0;
  }

  std::string target_frame_;
  std::string input_frame_;
  std::string input_topic_;
  std::string output_topic_;
  std::string pose_topic_;
  double pose_timeout_seconds_ = 0.40;
  double output_rate_hz_ = 5.0;
  std::chrono::duration<double> output_period_{};
  double voxel_size_ = 0.08;
  double self_radius_ = 0.70;
  double self_radius_squared_ = 0.49;
  double self_z_band_ = 0.65;
  size_t expected_points_ = 24000;

  Transform pose_transform_;
  Clock::time_point pose_received_at_{};
  Clock::time_point last_publish_at_{};
  Clock::time_point last_report_at_{};
  bool has_pose_ = false;

  std::vector<PointXYZI> filtered_points_;
  std::unique_ptr<VoxelGrid> voxel_grid_;
  PointCloud2 output_message_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

  size_t received_ = 0;
  size_t published_ = 0;
  size_t rate_skipped_ = 0;
  size_t pose_failures_ = 0;
  size_t dropped_self_total_ = 0;
  size_t dropped_voxel_total_ = 0;
  size_t last_dropped_self_ = 0;
  size_t last_dropped_voxel_ = 0;
  size_t processed_frames_ = 0;
  double process_ms_sum_ = 0.0;
  double max_process_ms_ = 0.0;
};

}  // namespace nav_dog_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav_dog_cpp::CloudToMapFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
