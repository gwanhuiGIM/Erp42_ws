#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg
from sensor_msgs.msg import LaserScan
from math import pi

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # 차량 제어 명령을 발행할 퍼블리셔
        self.cmd_pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd', 1)
        
        # 2D 라이다 데이터를 구독할 서브스크라이버
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # 제어 명령 메시지 초기화
        self.cmd_msg = ErpCmdMsg()
        
        # 0.1초(10Hz) 주기로 메인 로직 함수를 호출할 타이머
        self.timer = self.create_timer(0.1, self.main_logic)
        
        # 최신 라이다 메시지를 저장할 변수
        self.lidar_msg = None

    def lidar_callback(self, msg):
        """
        /scan 토픽으로부터 라이다 데이터를 수신할 때마다 호출되는 콜백 함수.
        수신된 최신 메시지를 클래스 변수에 저장합니다.
        """
        self.lidar_msg = msg

    def main_logic(self):
        """
        타이머에 의해 주기적으로 호출되는 메인 제어 로직.
        장애물을 탐지하고 회피하기 위한 조향각을 계산하여 차량에 명령을 내립니다.
        """
        # 라이다 데이터가 아직 수신되지 않았다면 함수를 종료합니다.
        if self.lidar_msg is None:
            self.get_logger().info('Lidar data not received yet.')
            return

        # 장애물 회피 로직에 필요한 변수 초기화
        obstacle_angles = [] # 장애물이 탐지된 각도를 저장할 리스트
        largest_gap_size = 0.0  # 장애물 사이의 가장 큰 간격 크기
        gap_center_angle = 0.0  # 가장 큰 간격의 중심 각도

        # LaserScan 메시지에는 각도 배열이 없으므로, 각 측정값의 각도를 직접 계산합니다.
        angle_min_deg = self.lidar_msg.angle_min * 180 / pi
        angle_increment_deg = self.lidar_msg.angle_increment * 180 / pi
        degrees = [angle_min_deg + i * angle_increment_deg for i in range(len(self.lidar_msg.ranges))]

        # --- 장애물 탐지 ---
        # 모든 라이다 측정값에 대해 반복
        for index, distance in enumerate(self.lidar_msg.ranges):
            angle_deg = degrees[index]
            
            # 관심 영역(ROI): 전방 120도(-60 ~ +60), 0.75m 이내의 장애물만 고려
            if 0 < distance < 0.75 and abs(angle_deg) < 60:
                obstacle_angles.append(angle_deg)                
                # 장애물 사이의 가장 큰 간격(gap)과 그 중심 각도를 찾습니다.
                if len(obstacle_angles) > 1:
                    # 현재 장애물과 바로 이전 장애물 사이의 각도 차이 계산
                    current_gap = obstacle_angles[-1] - obstacle_angles[-2]
                    # 이 간격이 이전에 찾은 가장 큰 간격보다 크면 갱신
                    if current_gap > largest_gap_size:
                        largest_gap_size = current_gap
                        # 간격의 중심 각도 계산
                        gap_center_angle = obstacle_angles[-2] + largest_gap_size / 2

        # --- 회피 경로 결정 ---
        target_angle = 0.0
        if obstacle_angles:  # 장애물이 하나라도 탐지되었다면
            # 주행 가능한 세 가지 공간 후보를 계산합니다.
            # 1. 가장 왼쪽 장애물과 시야각 왼쪽 끝 사이의 공간
            left_space = 60 - obstacle_angles[-1]
            # 2. 가장 오른쪽 장애물과 시야각 오른쪽 끝 사이의 공간
            right_space = 60 + obstacle_angles
            
            # 세 공간(왼쪽, 오른쪽, 중앙 간격) 중 가장 넓은 공간을 선택합니다.
            max_space = max(left_space, right_space, largest_gap_size)
            
            if max_space == left_space:
                # 왼쪽 공간이 가장 넓으면, 그 공간의 중심으로 주행
                target_angle = 60 - left_space / 2
                self.get_logger().info('Path decision: Going left')
            elif max_space == right_space:
                # 오른쪽 공간이 가장 넓으면, 그 공간의 중심으로 주행
                target_angle = -60 + right_space / 2
                self.get_logger().info('Path decision: Going right')
            else:
                # 장애물 사이의 간격이 가장 넓으면, 그 간격의 중심으로 주행
                target_angle = gap_center_angle
                self.get_logger().info('Path decision: Going center gap')
            
            # 비례 제어: 목표 각도에 비례하여 조향각을 결정 (게인 값 -0.5)
            # 음수를 곱하는 것은 차량의 조향 방향과 좌표계를 맞추기 위함
            target_angle = -target_angle * 0.5

        # --- 차량 제어 명령 생성 및 발행 ---
        # 계산된 목표 각도를 차량의 조향 명령 단위로 변환 (차량별 스케일링 팩터: 71)
        steer_cmd = int(target_angle * 71)
        
        # erpCmdMsg 메시지에 조향 값 설정
        self.cmd_msg.steer = steer_cmd
        
        # 차량 제어 명령 발행
        self.cmd_pub.publish(self.cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()