// hvl_bdt_node.cpp
// Subscribes to a RealSense coloured PointCloud2 (depth aligned to colour),
// crops it to a bounding box, keeps points of a given HSV colour, picks the
// largest cluster and publishes the ball centre as geometry_msgs/PointStamped.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::PointStamped;
using visualization_msgs::msg::Marker;

class BallDetector : public rclcpp::Node
{
public:
  BallDetector() : Node("hvl_bdt_node")
  {
    // --- topics / frames
    cloud_topic_  = declare_parameter<std::string>("cloud_topic", "/camera/camera/depth/color/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "");  // "" = keep camera frame

    // --- bounding box (in target_frame, or camera optical frame if target_frame is empty)
    box_min_ = declare_parameter<std::vector<double>>("box_min", {-0.3, -0.3, 0.2});
    box_max_ = declare_parameter<std::vector<double>>("box_max", { 0.3,  0.3, 1.2});

    // --- colour filter, HSV: H in degrees [0,360), S and V in [0,1]
    //     If h_min > h_max the range wraps around 0 (useful for red).
    h_min_ = declare_parameter<double>("h_min", 20.0);
    h_max_ = declare_parameter<double>("h_max", 45.0);
    s_min_ = declare_parameter<double>("s_min", 0.45);
    v_min_ = declare_parameter<double>("v_min", 0.25);

    // --- processing
    voxel_size_       = declare_parameter<double>("voxel_size", 0.004);   // 0 disables
    cluster_tol_      = declare_parameter<double>("cluster_tolerance", 0.015);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 30);
    max_cluster_size_ = declare_parameter<int>("max_cluster_size", 50000);

    // --- sphere fit: camera only sees the front half of the ball, so the plain
    //     centroid sits ~radius/2 too close to the camera. A sphere fit fixes that.
    use_sphere_fit_ = declare_parameter<bool>("use_sphere_fit", true);
    radius_min_     = declare_parameter<double>("radius_min", 0.02);
    radius_max_     = declare_parameter<double>("radius_max", 0.05);
    sphere_thresh_  = declare_parameter<double>("sphere_distance_threshold", 0.004);

    publish_debug_ = declare_parameter<bool>("publish_debug_cloud", true);

    if (box_min_.size() != 3 || box_max_.size() != 3)
      throw std::runtime_error("box_min and box_max must have 3 elements");

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BallDetector::cloudCallback, this, std::placeholders::_1));

    point_pub_  = create_publisher<PointStamped>("ball_position", 10);
    marker_pub_ = create_publisher<Marker>("ball_marker", 10);
    debug_pub_  = create_publisher<PointCloud2>("ball_filtered_cloud", rclcpp::SensorDataQoS());
    box_pub_    = create_publisher<Marker>("ball_search_box", 1);

    RCLCPP_INFO(get_logger(), "Listening on %s", cloud_topic_.c_str());
  }

private:
  // ---------------------------------------------------------------- helpers
  static void rgbToHsv(uint8_t r8, uint8_t g8, uint8_t b8, float & h, float & s, float & v)
  {
    const float r = r8 / 255.f, g = g8 / 255.f, b = b8 / 255.f;
    const float mx = std::max({r, g, b}), mn = std::min({r, g, b});
    const float d = mx - mn;
    v = mx;
    s = (mx > 0.f) ? d / mx : 0.f;
    if (d == 0.f) { h = 0.f; return; }
    if (mx == r)      h = 60.f * std::fmod((g - b) / d, 6.f);
    else if (mx == g) h = 60.f * ((b - r) / d + 2.f);
    else              h = 60.f * ((r - g) / d + 4.f);
    if (h < 0.f) h += 360.f;
  }

  bool hueInRange(float h) const
  {
    return (h_min_ <= h_max_) ? (h >= h_min_ && h <= h_max_)
                              : (h >= h_min_ || h <= h_max_);   // wrap-around (red)
  }

  // ---------------------------------------------------------------- main callback
  void cloudCallback(const PointCloud2::SharedPtr msg)
  {
    // 1. Optionally transform into target_frame (e.g. base_link) so the box is robot-fixed
    PointCloud2 cloud_msg;
    if (!target_frame_.empty() && target_frame_ != msg->header.frame_id) {
      try {
        auto tf = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                              msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        pcl_ros::transformPointCloud(target_frame_, tf, *msg, cloud_msg);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF: %s", e.what());
        return;
      }
    } else {
      cloud_msg = *msg;
    }
    const std::string frame = cloud_msg.header.frame_id;
    publishBox(frame, cloud_msg.header.stamp);

    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // 2. Crop to bounding box (also drops NaNs)
    CloudT::Ptr cropped(new CloudT);
    pcl::CropBox<PointT> crop;
    crop.setMin(Eigen::Vector4f(box_min_[0], box_min_[1], box_min_[2], 1.f));
    crop.setMax(Eigen::Vector4f(box_max_[0], box_max_[1], box_max_[2], 1.f));
    crop.setInputCloud(cloud);
    crop.filter(*cropped);

    // 3. Colour filter in HSV
    CloudT::Ptr colored(new CloudT);
    colored->reserve(cropped->size());
    for (const auto & p : cropped->points) {
      if (!std::isfinite(p.z)) continue;
      float h, s, v;
      rgbToHsv(p.r, p.g, p.b, h, s, v);
      if (s >= s_min_ && v >= v_min_ && hueInRange(h)) colored->push_back(p);
    }
    colored->width = colored->size(); colored->height = 1; colored->is_dense = true;

    // 4. Downsample
    CloudT::Ptr filtered = colored;
    if (voxel_size_ > 0.0 && !colored->empty()) {
      filtered.reset(new CloudT);
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(colored);
      vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      vg.filter(*filtered);
    }

    if (publish_debug_) {
      PointCloud2 dbg;
      pcl::toROSMsg(*filtered, dbg);
      dbg.header = cloud_msg.header;
      debug_pub_->publish(dbg);
    }

    if (static_cast<int>(filtered->size()) < min_cluster_size_) {
      RCLCPP_DEBUG(get_logger(), "Not enough coloured points (%zu)", filtered->size());
      return;
    }

    // 5. Euclidean clustering -> keep the largest blob (rejects stray pixels)
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(filtered);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tol_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(clusters);
    if (clusters.empty()) return;

    const auto & best = *std::max_element(clusters.begin(), clusters.end(),
      [](const auto & a, const auto & b) { return a.indices.size() < b.indices.size(); });
    CloudT::Ptr ball(new CloudT(*filtered, best.indices));

    // 6. Centre: sphere fit if possible, otherwise plain centroid
    Eigen::Vector3f center;
    double radius = 0.0;
    bool fitted = false;
    if (use_sphere_fit_ && ball->size() >= 10) {
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_SPHERE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(sphere_thresh_);
      seg.setRadiusLimits(radius_min_, radius_max_);
      seg.setMaxIterations(200);
      seg.setInputCloud(ball);
      pcl::PointIndices inliers;
      pcl::ModelCoefficients coeff;
      seg.segment(inliers, coeff);
      if (inliers.indices.size() > ball->size() / 2 && coeff.values.size() == 4) {
        center = {coeff.values[0], coeff.values[1], coeff.values[2]};
        radius = coeff.values[3];
        fitted = true;
      }
    }
    if (!fitted) {
      Eigen::Vector4f c;
      pcl::compute3DCentroid(*ball, c);
      center = c.head<3>();
      radius = 0.5 * (radius_min_ + radius_max_);
    }

    // 7. Publish
    PointStamped out;
    out.header = cloud_msg.header;
    out.point.x = center.x(); out.point.y = center.y(); out.point.z = center.z();
    point_pub_->publish(out);
    publishMarker(out, radius, fitted);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Ball in %s: [%.3f, %.3f, %.3f] (%zu pts, %s, r=%.3f)",
      frame.c_str(), center.x(), center.y(), center.z(), ball->size(),
      fitted ? "sphere fit" : "centroid", radius);
  }

  // ---------------------------------------------------------------- visualisation
  void publishMarker(const PointStamped & p, double radius, bool fitted)
  {
    Marker m;
    m.header = p.header;
    m.ns = "ball"; m.id = 0;
    m.type = Marker::SPHERE; m.action = Marker::ADD;
    m.pose.position = p.point;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 2.0 * radius;
    m.color.a = 0.6f;
    m.color.g = fitted ? 1.0f : 0.5f; m.color.r = fitted ? 0.0f : 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(m);
  }

  void publishBox(const std::string & frame, const builtin_interfaces::msg::Time & stamp)
  {
    Marker m;
    m.header.frame_id = frame; m.header.stamp = stamp;
    m.ns = "search_box"; m.id = 0;
    m.type = Marker::CUBE; m.action = Marker::ADD;
    m.pose.position.x = 0.5 * (box_min_[0] + box_max_[0]);
    m.pose.position.y = 0.5 * (box_min_[1] + box_max_[1]);
    m.pose.position.z = 0.5 * (box_min_[2] + box_max_[2]);
    m.pose.orientation.w = 1.0;
    m.scale.x = box_max_[0] - box_min_[0];
    m.scale.y = box_max_[1] - box_min_[1];
    m.scale.z = box_max_[2] - box_min_[2];
    m.color.b = 1.0f; m.color.a = 0.15f;
    box_pub_->publish(m);
  }

  // ---------------------------------------------------------------- members
  std::string cloud_topic_, target_frame_;
  std::vector<double> box_min_, box_max_;
  double h_min_, h_max_, s_min_, v_min_;
  double voxel_size_, cluster_tol_;
  int min_cluster_size_, max_cluster_size_;
  bool use_sphere_fit_, publish_debug_;
  double radius_min_, radius_max_, sphere_thresh_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_, box_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
