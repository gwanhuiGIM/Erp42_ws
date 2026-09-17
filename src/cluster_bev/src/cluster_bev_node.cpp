#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/kdtree.h>
#include <Eigen/Core>
#include <cmath>

class ClusterBEVNode : public rclcpp::Node
{
public:
  ClusterBEVNode()
  : Node("cluster_bev_node")
  {
    // ---- 파라미터 (필요시 launch에서 수정 가능) ----
    declare_parameter<std::string>("input_topic", "/velodyne_points");
    declare_parameter<std::string>("output_topic_bev", "/clusters_bev");
    declare_parameter<double>("voxel_leaf_size", 0.05);        // 5cm
    declare_parameter<double>("ransac_dist_thr_ground", 0.03);  // 3cm
    declare_parameter<double>("ransac_dist_thr_wall", 0.03);    // 3cm
    declare_parameter<double>("eps_angle_deg", 10.0);           // 축과 평면법선의 허용각
    declare_parameter<int>("max_wall_removals", 2);             // 큰 벽 2개까지 제거
    declare_parameter<double>("z_min_keep", -1.5);              // 남길 z범위 (VLP-16 장착높이 고려)
    declare_parameter<double>("z_max_keep", 2.0);
    declare_parameter<double>("cluster_tolerance", 0.2);        // 20cm
    declare_parameter<int>("min_cluster_size", 30);
    declare_parameter<int>("max_cluster_size", 20000);

    const auto input_topic  = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic_bev").as_string();

    // Velodyne 센서 데이터 QoS 권장: SensorData
    auto qos = rclcpp::SensorDataQoS();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, qos,
        std::bind(&ClusterBEVNode::cloudCallback, this, std::placeholders::_1));

    pub_bev_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(get_logger(), "ClusterBEVNode subscribed to: %s, publishing: %s",
                input_topic.c_str(), output_topic.c_str());
  }

private:
  using PointT = pcl::PointXYZI;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ROS2 -> PCL
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Empty cloud");
      return;
    }

    // ---- 0) 다운샘플 (속도/잡음 완화) ----
    double leaf = get_parameter("voxel_leaf_size").as_double();
    pcl::PointCloud<PointT>::Ptr cloud_ds(new pcl::PointCloud<PointT>());
    if (leaf > 1e-6) {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(leaf, leaf, leaf);
      vg.filter(*cloud_ds);
    } else {
      cloud_ds = cloud;
    }

    // ---- 1) Ground 제거 : 평면법선이 z축과 "평행"한 평면 (바닥) ----
    pcl::PointCloud<PointT>::Ptr no_ground(new pcl::PointCloud<PointT>());
    removePlane(*cloud_ds,
                /*parallel_to_axis=*/true,
                get_parameter("ransac_dist_thr_ground").as_double(),
                get_parameter("eps_angle_deg").as_double(),
                *no_ground);

    // ---- 2) 벽 제거 : 평면법선이 z축에 "수직"인 큰 평면들을 반복 제거 ----
    pcl::PointCloud<PointT>::Ptr no_walls(new pcl::PointCloud<PointT>(*no_ground));
    const int max_wall_iters = get_parameter("max_wall_removals").as_int();
    for (int i = 0; i < max_wall_iters; ++i) {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      bool removed = removePlane(*no_walls,
                                 /*parallel_to_axis=*/false,
                                 get_parameter("ransac_dist_thr_wall").as_double(),
                                 get_parameter("eps_angle_deg").as_double(),
                                 *tmp);
      if (!removed) break;
      no_walls.swap(tmp);
    }

    // ---- 3) z 범위 필터 (잔여 천장/바닥 잡음 컷) ----
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(no_walls);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(
          get_parameter("z_min_keep").as_double(),
          get_parameter("z_max_keep").as_double());
      pass.filter(*cloud_f);
    }
    if (cloud_f->empty()) {
      RCLCPP_WARN(get_logger(), "All points filtered out");
      return;
    }

    // ---- 4) Euclidean Clustering ----
    std::vector<pcl::PointIndices> cluster_indices;
    {
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
      tree->setInputCloud(cloud_f);

      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(get_parameter("cluster_tolerance").as_double());
      ec.setMinClusterSize(get_parameter("min_cluster_size").as_int());
      ec.setMaxClusterSize(get_parameter("max_cluster_size").as_int());
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_f);
      ec.extract(cluster_indices);
    }
    RCLCPP_DEBUG(get_logger(), "Clusters: %zu", cluster_indices.size());

    // ---- 5) BEV 투영 및 publish (z=0에 투영, intensity에 cluster id 부여) ----
    pcl::PointCloud<PointT>::Ptr bev_cloud(new pcl::PointCloud<PointT>());
    int cid = 1;  // 0은 배경으로 남기고, 1부터 클러스터 id
    bev_cloud->points.reserve(cloud_f->size());
    for (const auto &ci : cluster_indices) {
      for (int idx : ci.indices) {
        const auto &p = cloud_f->points[idx];
        PointT q;
        q.x = p.x;
        q.y = p.y;
        q.z = 0.0f;        // BEV: 높이 제거
        q.intensity = static_cast<float>(cid); // 시각화/후처리용 id
        bev_cloud->points.push_back(q);
      }
      ++cid;
    }
    bev_cloud->width = static_cast<uint32_t>(bev_cloud->points.size());
    bev_cloud->height = 1;
    bev_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*bev_cloud, out);
    out.header = msg->header; // velodyne frame 유지
    pub_bev_->publish(out);
  }

  // parallel_to_axis=true  -> ground(법선이 z축과 평행) 제거
  // parallel_to_axis=false -> walls(법선이 z축과 수직) 제거
  bool removePlane(const pcl::PointCloud<PointT>& in,
                   bool parallel_to_axis,
                   double dist_thr,
                   double eps_angle_deg,
                   pcl::PointCloud<PointT>& out_without_plane)
  {
    if (in.empty()) return false;

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    if (parallel_to_axis)
      seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    else
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);

    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_thr);
    seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f)); // 기준축: z
    seg.setEpsAngle(static_cast<float>(eps_angle_deg * M_PI / 180.0));

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients());
    pcl::PointCloud<PointT>::ConstPtr in_ptr(&in, [](const pcl::PointCloud<PointT>*){});
    seg.setInputCloud(in_ptr);
    seg.segment(*inliers, *coeffs);

    if (inliers->indices.empty()) {
      out_without_plane = in;
      return false;
    }

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(in_ptr);
    ex.setIndices(inliers);
    ex.setNegative(true);    // 평면 제거
    ex.filter(out_without_plane);
    return true;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_bev_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusterBEVNode>());
  rclcpp::shutdown();
  return 0;
}

