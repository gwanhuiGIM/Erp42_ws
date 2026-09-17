#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from erp_interfaces.msg import ErpCmdMsg
import sensor_msgs_py.point_cloud2 as pc2
from math import atan2, degrees, pi

class ObstacleAvoidance3DNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_3d_node')
        
        # Publisher: 차량 제어 명령
        self.pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd/lidar', 10)
        
        # Subscriber: 3D 라이다 포인트 클라우드
        self.sub = self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_cb, 10)
        
        # 제어 명령 메시지
        self.cmd = ErpCmdMsg()
        
        # 0.1초 (10Hz) 주기로 메인 제어 루프 실행
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # self.points 변수를 None으로 초기화
        self.points = None

    def lidar_cb(self, msg):
        """
        PointCloud2 메시지를 수신하면 pc2.read_points를 통해
        처리하기 쉬운 iterable 객체로 변환하여 저장합니다.
        """
        self.points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    def control_loop(self):
        # 데이터가 수신되기 전까지는 제어 로직을 실행하지 않습니다.
        if self.points is None:
            self.get_logger().info("Waiting for PointCloud2 data...")
            return

        # 장애물 정보 및 회피 경로 계산을 위한 변수 초기화
        obstacle_angles = [] # 유효 장애물의 각도를 저장할 리스트
        is_emergency_stop = False
        
        # --- 3D 포인트 클라우드 처리 및 장애물 탐지 ---
        for x, y, z in self.points:
            # Z축 필터: 주행에 유의미한 높이의 포인트만 고려 (바닥, 너무 높은 장애물 제외)
            if not (0.1 < z < 1.0):
                continue

            # 수평 거리 및 각도 계산
            dist = (x**2 + y**2)**0.5
            ang = degrees(atan2(y, x))

            # 비상 정지 조건: 전방 30도, 0.5m 이내에 장애물 감지 시
            if -15 <= ang <= 15 and 0 < dist < 0.5:
                is_emergency_stop = True
                break # 비상 정지 조건이 충족되면 더 이상 탐색할 필요 없음

            # 회피를 위한 장애물 탐지 조건: 전방 120도, 2.5m 이내
            if -60 <= ang <= 60 and 0 < dist < 2.5:
                obstacle_angles.append(ang)
        
        # 비상 정지 로직
        if is_emergency_stop:
            self.get_logger().warn("EMERGENCY STOP: Obstacle too close!")
            self.cmd.steer = 0
            self.cmd.speed = 0
            self.cmd.brake = 200 # 최대 제동
            self.pub.publish(self.cmd)
            return # 비상 정지 시 회피 로직을 실행하지 않음

        # --- 회피 경로 결정 ---
        target_angle = 0.0
        if obstacle_angles:
            obstacle_angles.sort() # 각도를 오름차순으로 정렬 (-50, -20, 10, 45,...)
            
            largest_gap_size = 0.0
            gap_center_angle = 0.0

            # 장애물 사이의 가장 큰 빈 공간(gap) 찾기
            for i in range(len(obstacle_angles) - 1):
                gap = obstacle_angles[i+1] - obstacle_angles[i]
                if gap > largest_gap_size:
                    largest_gap_size = gap
                    gap_center_angle = obstacle_angles[i] + gap / 2
            
            # 주행 가능한 세 가지 공간 후보 계산
            # 가장 왼쪽 장애물(리스트의 마지막 요소)과 시야각 왼쪽 끝(+60) 사이의 공간
            left_space = 60 - obstacle_angles[-1]
            
            # [오류 수정] 가장 오른쪽 장애물(리스트의 첫 요소)과 시야각 오른쪽 끝(-60) 사이의 공간
            right_space = obstacle_angles + 60

            # 가장 넓은 공간 선택
            max_space = max(left_space, right_space, largest_gap_size)

            if max_space == left_space:
                target_angle = 60 - left_space / 2
                self.get_logger().info('Path: Going left')
            elif max_space == right_space:
                target_angle = -60 + right_space / 2
                self.get_logger().info('Path: Going right')
            else:
                target_angle = gap_center_angle
                self.get_logger().info('Path: Going center gap')
            
            # 목표 각도에 비례하여 조향각 결정 (게인 값 -0.8)
            target_angle = -target_angle * 0.8
        else:
            self.get_logger().info("Clear path — moving forward")

        # --- 차량 제어 명령 생성 및 발행 ---
        steer_cmd = int(target_angle * 71) # 각도를 차량 조향 단위로 변환
        
        self.cmd.steer = steer_cmd
        self.cmd.brake = 0
        
        if not obstacle_angles:
             self.cmd.speed = 100 # 장애물이 없을 때의 주행 속도
        else:
             self.cmd.speed = 80 # 장애물이 있지만 비상상황은 아닐 때의 주행 속도

        self.pub.publish(self.cmd)

def main():
    rclpy.init()
    node = ObstacleAvoidance3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()