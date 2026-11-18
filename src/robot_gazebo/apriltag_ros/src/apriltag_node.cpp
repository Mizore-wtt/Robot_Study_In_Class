// ROS 2 standalone-tag adapter for the supplied AprilTag 3.4.2 library.
// Detection messages are retained from the course's BSD-licensed ROS 1 wrapper.
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag36h11.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include "apriltag_ros/msg/april_tag_detection_array.hpp"

class AprilTagNode : public rclcpp::Node {
 public:
  AprilTagNode() : Node("apriltag"), broadcaster_(*this) {
    size_ = declare_parameter("tag_size", 0.24);
    ids_ = declare_parameter<std::vector<int64_t>>("tag_ids", std::vector<int64_t>{});
    const auto threads = declare_parameter("threads", 2);
    const auto decimate = declare_parameter("quad_decimate", 1.0);
    max_hamming_ = declare_parameter("max_hamming", 0);
    publish_tf_ = declare_parameter("publish_tf", true);
    annotate_ = declare_parameter("publish_detections_image", true);
    if (!std::isfinite(size_) || size_ <= 0 || threads < 1 ||
        !std::isfinite(decimate) || decimate < 1 || max_hamming_ < 0 || max_hamming_ > 2) {
      throw std::invalid_argument("Invalid tag_size/threads/quad_decimate/max_hamming");
    }
    family_ = tag36h11_create();
    detector_ = apriltag_detector_create();
    apriltag_detector_add_family_bits(detector_, family_, max_hamming_);
    detector_->nthreads = threads;
    detector_->quad_decimate = decimate;
    detector_->refine_edges = true;
    detections_ = create_publisher<apriltag_ros::msg::AprilTagDetectionArray>("tag_detections", 10);
    images_ = create_publisher<sensor_msgs::msg::Image>("tag_detections_image", 1);
    // Best-effort subscribers accept both reliable Gazebo and sensor-data publishers.
    // Depth 1 avoids processing old frames when simulation runs slowly.
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    calibration_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { info_ = info; });
    camera_ = create_subscription<sensor_msgs::msg::Image>("image", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) { process(image); });
    RCLCPP_INFO(get_logger(), "tag36h11 detector ready; black edge %.3f m", size_);
  }

  ~AprilTagNode() override {
    apriltag_detector_destroy(detector_);
    tag36h11_destroy(family_);
  }

 private:
  void process(const sensor_msgs::msg::Image::ConstSharedPtr & image) {
    // Intrinsics are static in this Gazebo camera. Cache CameraInfo rather than
    // relying on arrival order of two independently delivered topics.
    if (!info_ || info_->header.frame_id != image->header.frame_id ||
        info_->width != image->width || info_->height != image->height ||
        !std::isfinite(info_->k[0]) || !std::isfinite(info_->k[4]) ||
        info_->k[0] <= 0 || info_->k[4] <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Waiting for matching calibrated CameraInfo");
      return;
    }
    // The supplied simulated camera has zero distortion. Reject distorted raw
    // input rather than silently estimating incorrect metric poses.
    if (std::any_of(info_->d.begin(), info_->d.end(), [](double d) { return d != 0.; })) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Input must be undistorted; Gazebo uses zero distortion");
      return;
    }
    cv_bridge::CvImagePtr gray;
    cv_bridge::CvImagePtr overlay;
    try {
      gray = cv_bridge::toCvCopy(image, "mono8");
      if (annotate_) overlay = cv_bridge::toCvCopy(image, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "Image conversion failed: %s", e.what());
      return;
    }
    image_u8_t input{gray->image.cols, gray->image.rows,
                     static_cast<int>(gray->image.step), gray->image.data};
    auto raw = apriltag_detector_detect(detector_, &input);
    apriltag_ros::msg::AprilTagDetectionArray result;
    result.header = image->header;
    for (int i = 0; i < zarray_size(raw); ++i) {
      apriltag_detection_t * tag;
      zarray_get(raw, i, &tag);
      if (tag->hamming > max_hamming_ || (!ids_.empty() &&
          std::find(ids_.begin(), ids_.end(), tag->id) == ids_.end())) continue;
      apriltag_detection_info_t observation{tag, size_, info_->k[0], info_->k[4],
                                          info_->k[2], info_->k[5]};
      apriltag_pose_t pose{};
      const double error = estimate_tag_pose(&observation, &pose);
      bool valid = pose.R && pose.t && std::isfinite(error);
      if (valid) {
        for (int k = 0; k < 9; ++k) valid = valid && std::isfinite(pose.R->data[k]);
        for (int k = 0; k < 3; ++k) valid = valid && std::isfinite(pose.t->data[k]);
        valid = valid && pose.t->data[2] > 0;
      }
      if (valid) {
        apriltag_ros::msg::AprilTagDetection detection;
        detection.id = {tag->id};
        detection.size = {size_};
        detection.pose.header = image->header;
        auto & p = detection.pose.pose.pose;
        p.position.x = pose.t->data[0];
        p.position.y = pose.t->data[1];
        p.position.z = pose.t->data[2];
        tf2::Matrix3x3 rotation(pose.R->data[0], pose.R->data[1], pose.R->data[2],
                              pose.R->data[3], pose.R->data[4], pose.R->data[5],
                              pose.R->data[6], pose.R->data[7], pose.R->data[8]);
        tf2::Quaternion q;
        rotation.getRotation(q);
        q.normalize();
        p.orientation.x = q.x(); p.orientation.y = q.y();
        p.orientation.z = q.z(); p.orientation.w = q.w();
        // AprilTag's solver provides no covariance estimate; leave zeros.
        result.detections.push_back(detection);
        if (publish_tf_) {
          geometry_msgs::msg::TransformStamped transform;
          transform.header = image->header;
          transform.child_frame_id = "tag_" + std::to_string(tag->id);
          transform.transform.translation.x = p.position.x;
          transform.transform.translation.y = p.position.y;
          transform.transform.translation.z = p.position.z;
          transform.transform.rotation = p.orientation;
          broadcaster_.sendTransform(transform);
        }
        if (overlay) {
          for (int j = 0; j < 4; ++j) {
            const int next = (j + 1) % 4;
            cv::line(overlay->image, cv::Point(tag->p[j][0], tag->p[j][1]),
                     cv::Point(tag->p[next][0], tag->p[next][1]), cv::Scalar(0, 255, 0), 2);
          }
          cv::putText(overlay->image, "ID " + std::to_string(tag->id),
                      cv::Point(tag->c[0], tag->c[1]), cv::FONT_HERSHEY_SIMPLEX,
                      0.6, cv::Scalar(0, 0, 255), 2);
        }
      }
      if (pose.R) matd_destroy(pose.R);
      if (pose.t) matd_destroy(pose.t);
    }
    apriltag_detections_destroy(raw);
    detections_->publish(result);  // Empty array means this frame contains no known tags.
    if (overlay) images_->publish(*overlay->toImageMsg());
  }

  apriltag_family_t * family_{};
  apriltag_detector_t * detector_{};
  double size_;
  int max_hamming_;
  bool publish_tf_, annotate_;
  std::vector<int64_t> ids_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr calibration_;
  rclcpp::Publisher<apriltag_ros::msg::AprilTagDetectionArray>::SharedPtr detections_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagNode>());
  rclcpp::shutdown();
  return 0;
}
