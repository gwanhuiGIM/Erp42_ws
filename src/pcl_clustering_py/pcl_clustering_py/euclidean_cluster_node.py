import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

import numpy as np
import pcl
import random

class EuclideanClusterNode(Node):
    def __init__(self):
        super().__init__('euclidean_cluster_node')

        # 파라미터 선언
        self.declare_parameter('point_cloud_topic', '/velodyne_points')
        self.declare_parameter('leaf_size', 0.1)
        self.declare_parameter('plane_dist_thresh', 0.05)
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 50)
        self.declare_parameter('max_cluster_size', 25000)

        # 파라미터 가져오기
        point_cloud_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        self.leaf_size = self.get_parameter('leaf_size').get_parameter_value().double_value
        self.plane_dist_thresh = self.get_parameter('plane_dist_thresh').get_parameter_value().double_value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').get_parameter_value().double_value
        self.min_cluster_size = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        self.max_cluster_size = self.get_parameter('max_cluster_size').get_parameter_value().integer_value

        # 구독자 및 발행자 설정
        self.subscription = self.create_subscription(
            PointCloud2,
            point_cloud_topic,
            self.pointcloud_callback,
            10)
        self.cluster_publisher = self.create_publisher(PointCloud2, '/clustered_points', 10)

        self.get_logger().info(f"PCL Euclidean Clustering Node 시작. 토픽: {point_cloud_topic}")

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            # 1. ROS PointCloud2 -> PCL PointCloud 변환
            cloud = self.ros_to_pcl(msg)
            if cloud.size == 0:
                self.get_logger().warn("입력 포인트 클라우드가 비어있습니다.")
                return

            # 2. Voxel Grid 다운샘플링
            vg = cloud.make_VoxelGrid()
            vg.set_leaf_size(self.leaf_size, self.leaf_size, self.leaf_size)
            cloud_filtered = vg.filter()

            # 3. RANSAC 평면 분할 (지면 제거)
            seg = cloud_filtered.make_segmenter()
            seg.set_model_type(pcl.SACMODEL_PLANE)
            seg.set_method_type(pcl.SAC_RANSAC)
            seg.set_distance_threshold(self.plane_dist_thresh)
            inliers, _ = seg.segment()

            if len(inliers) == 0:
                self.get_logger().warn("지면을 찾지 못했습니다. 전체 클라우드를 대상으로 클러스터링합니다.")
                cloud_objects = cloud_filtered
            else:
                cloud_objects = cloud_filtered.extract(inliers, negative=True)

            if cloud_objects.size == 0:
                self.get_logger().warn("지면 제거 후 남은 포인트가 없습니다.")
                return

            # 4. 유클리디안 클러스터링
            tree = cloud_objects.make_kdtree()
            ec = cloud_objects.make_EuclideanClusterExtraction()
            ec.set_ClusterTolerance(self.cluster_tolerance)
            ec.set_MinClusterSize(self.min_cluster_size)
            ec.set_MaxClusterSize(self.max_cluster_size)
            ec.set_SearchMethod(tree)
            cluster_indices = ec.Extract()

            # 5. 결과 시각화 및 발행
            if cluster_indices:
                colored_cloud = self.colorize_clusters(cloud_objects, cluster_indices)
                ros_cloud_msg = self.pcl_to_ros(colored_cloud, msg.header)
                self.cluster_publisher.publish(ros_cloud_msg)
            else:
                self.get_logger().info("유효한 클러스터를 찾지 못했습니다.")

        except Exception as e:
            self.get_logger().error(f"포인트 클라우드 처리 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()

    def ros_to_pcl(self, ros_cloud: PointCloud2) -> pcl.PointCloud:
        """ ROS PointCloud2 메시지를 PCL PointCloud 객체로 변환 """
        points_list = ''
        for data in pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([data, data[1], data[2]])
        
        pcl_cloud = pcl.PointCloud()
        if points_list:
            pcl_cloud.from_array(np.array(points_list, dtype=np.float32))
        return pcl_cloud

    def colorize_clusters(self, cloud: pcl.PointCloud, cluster_indices: list) -> pcl.PointCloud_PointXYZRGB:
        """ 각 클러스터에 랜덤 색상을 입혀 하나의 포인트 클라우드로 합침 """
        cluster_color = [random.randint(0, 255) for _ in range(3 * len(cluster_indices))]
        
        colored_points = ''
        for i, indices in enumerate(cluster_indices):
            r, g, b = cluster_color[i*3], cluster_color[i*3+1], cluster_color[i*3+2]
            rgb_packed = float((r << 16) | (g << 8) | b)
            for index in indices:
                point = cloud[index]
                colored_points.append([point, point[1], point[2], rgb_packed])

        colored_cloud = pcl.PointCloud_PointXYZRGB()
        if colored_points:
            colored_cloud.from_array(np.array(colored_points, dtype=np.float32))
        return colored_cloud

    def pcl_to_ros(self, pcl_cloud: pcl.PointCloud_PointXYZRGB, header: Header) -> PointCloud2:
        """ PCL PointCloud_PointXYZRGB 객체를 ROS PointCloud2 메시지로 변환 """
        points_list = pcl_cloud.to_list()
        
        fields = ''
        
        ros_msg = pc2.create_cloud(header, fields, points_list)
        return ros_msg

def main(args=None):
    rclpy.init(args=args)
    node = EuclideanClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
