#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import math
from collections import deque
import heapq


class NavigationNode:
    def __init__(self):
        rospy.loginfo("=== Navigation Node Starting ===")
        self.setup_parameters()
        self.setup_variables()
        self.setup_publishers()
        self.setup_subscribers()

        # 주 루프 타이머
        self.navigation_timer = rospy.Timer(rospy.Duration(0.2), self.navigation_loop)
        rospy.loginfo("Navigation Node initialized successfully!")

    def setup_parameters(self):
        """ROS 파라미터 설정"""
        self.map_resolution = rospy.get_param('~map_resolution', 0.1)  # m/pixel
        self.map_width = rospy.get_param('~map_width', 200)  # cells
        self.map_height = rospy.get_param('~map_height', 200)  # cells
        self.obstacle_inflation_radius = rospy.get_param('~obstacle_inflation_radius', 0.5)  # meters
        self.planning_horizon = rospy.get_param('~planning_horizon', 20.0)  # meters
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)  # meters
        self.lidar_range_max = rospy.get_param('~lidar_range_max', 50.0)  # meters
        self.lidar_range_min = rospy.get_param('~lidar_range_min', 0.3)  # meters

    def setup_variables(self):
        """변수 초기화"""
        # 맵 관련
        self.occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_origin_x = -self.map_width * self.map_resolution / 2
        self.map_origin_y = -self.map_height * self.map_resolution / 2

        # 위치 및 목표
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.orientation.w = 1.0
        self.start_point = None
        self.goal_point = None
        self.current_path = []

        # 상태
        self.navigation_active = False
        self.goal_reached = False
        self.map_updated = False

        # 맵 업데이트 관련
        self.last_map_update = rospy.Time.now()
        self.map_update_interval = rospy.Duration(0.5)

        rospy.loginfo("Navigation variables initialized")

    def setup_publishers(self):
        """퍼블리셔 설정"""
        self.map_pub = rospy.Publisher('occupancy_map', OccupancyGrid, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('planned_path', Path, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('nav_goal_point', Point, queue_size=1)
        self.start_pub = rospy.Publisher('nav_start_point', Point, queue_size=1)
        self.nav_status_pub = rospy.Publisher('navigation_status', String, queue_size=1)

        rospy.loginfo("Navigation publishers initialized")

    def setup_subscribers(self):
        """서브스크라이버 설정"""
        self.pointcloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback)
        self.goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback)

        rospy.loginfo("Navigation subscribers initialized")

    def pointcloud_callback(self, data):
        """라이다 포인트클라우드 처리"""
        try:
            if rospy.Time.now() - self.last_map_update < self.map_update_interval:
                return
            self.update_occupancy_grid(data)
            self.last_map_update = rospy.Time.now()
        except Exception as e:
            rospy.logerr(f"Error processing pointcloud: {e}")

    def update_occupancy_grid(self, pointcloud):
        """포인트클라우드 데이터로 점유 격자 업데이트"""

        # 격자 초기화 (자유 공간으로 설정)
        self.occupancy_grid.fill(0)

        # VLP-32는 더 많은 수직 레이어를 가지므로 z 범위를 넓게 설정
        for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[0], point[1], point[2]

            # 높이 필터링 (지면 제외 + 사람/장애물 높이 고려)
            if z < -0.2 or z > 2.0:  # VLP-32의 높은 해상도에 맞춰 지면 잡히는 것 방지
                continue

            # 거리 필터링
            distance = math.sqrt(x*x + y*y)
            if distance < self.lidar_range_min or distance > self.lidar_range_max:
                continue

            # 월드 좌표를 그리드 좌표로 변환
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)

            # 그리드 범위 확인
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                self.occupancy_grid[grid_y, grid_x] = 100  # 장애물 표시

        # 장애물 팽창 (안전 마진 추가)
        self.inflate_obstacles()

        # 점유 격자 퍼블리시
        self.publish_occupancy_grid()
        self.map_updated = True

    def inflate_obstacles(self):
        """장애물 주변 팽창"""
        inflation_cells = int(self.obstacle_inflation_radius / self.map_resolution)
        if inflation_cells <= 0:
            return

        obstacles = np.where(self.occupancy_grid == 100)

        # 각 장애물 주변 팽창 적용
        for i in range(len(obstacles[0])):
            y, x = obstacles[0][i], obstacles[1][i]
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                        new_y, new_x = y + dy, x + dx
                        if 0 <= new_y < self.map_height and 0 <= new_x < self.map_width:
                            if self.occupancy_grid[new_y, new_x] != 100:
                                self.occupancy_grid[new_y, new_x] = 50  # 팽창 영역

    def publish_occupancy_grid(self):
        """점유 격자 퍼블리시"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.map_pub.publish(grid_msg)

    def goal_callback(self, msg):
        """목표점 설정 콜백"""
        self.goal_point = Point()
        self.goal_point.x = msg.pose.position.x
        self.goal_point.y = msg.pose.position.y
        self.goal_point.z = 0.0
        self.navigation_active = True
        self.goal_reached = False
        rospy.loginfo(f"New goal set: ({self.goal_point.x:.2f}, {self.goal_point.y:.2f})")
        self.goal_pub.publish(self.goal_point)

    def navigation_loop(self, event):
        """네비게이션 메인 루프"""
        try:
            if not self.navigation_active or self.goal_point is None:
                return

            if self.check_goal_reached():
                self.navigation_active = False
                self.goal_reached = True
                self.nav_status_pub.publish(String("Goal Reached"))
                rospy.loginfo("Goal reached!")
                return

            if self.map_updated:
                path = self.plan_path()
                if path:
                    self.current_path = path
                    self.publish_path()
                    self.nav_status_pub.publish(String("Path Planning Success"))
                else:
                    self.nav_status_pub.publish(String("Path Planning Failed"))
                    rospy.logwarn("Path planning failed!")

                self.map_updated = False
        except Exception as e:
            rospy.logerr(f"Error in navigation loop: {e}")

    def check_goal_reached(self):
        """목표 도달 확인"""
        if self.goal_point is None:
            return False

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        distance = math.sqrt(
            (current_x - self.goal_point.x)**2 +
            (current_y - self.goal_point.y)**2
        )
        return distance < self.goal_tolerance

    def plan_path(self):
        """A* 알고리즘을 이용한 경로 계획"""
        if self.goal_point is None:
            return None

        start_x = int((self.current_pose.pose.position.x - self.map_origin_x) / self.map_resolution)
        start_y = int((self.current_pose.pose.position.y - self.map_origin_y) / self.map_resolution)
        goal_x = int((self.goal_point.x - self.map_origin_x) / self.map_resolution)
        goal_y = int((self.goal_point.y - self.map_origin_y) / self.map_resolution)

        if not (0 <= start_x < self.map_width and 0 <= start_y < self.map_height):
            rospy.logwarn("Start point is out of map bounds")
            return None
        if not (0 <= goal_x < self.map_width and 0 <= goal_y < self.map_height):
            rospy.logwarn("Goal point is out of map bounds")
            return None

        path = self.astar_search((start_x, start_y), (goal_x, goal_y))
        if path:
            world_path = []
            for x, y in path:
                world_x = x * self.map_resolution + self.map_origin_x
                world_y = y * self.map_resolution + self.map_origin_y
                world_path.append((world_x, world_y))
            return world_path
        return None

    def astar_search(self, start, goal):
        """A* 경로 탐색 알고리즘"""
        open_set = []
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        heapq.heappush(open_set, (f_score[start], start))

        directions = [(-1, -1), (-1, 0), (-1, 1),
                      (0, -1),          (0, 1),
                      (1, -1),  (1, 0), (1, 1)]

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # 경로 재구성
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            closed_set.add(current)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if neighbor[0] < 0 or neighbor[0] >= self.map_width or \
                   neighbor[1] < 0 or neighbor[1] >= self.map_height:
                    continue

                if self.occupancy_grid[neighbor[1], neighbor[0]] >= 50:  # 장애물 또는 팽창 영역 건너뜀
                    continue

                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + (1.414 if dx != 0 and dy != 0 else 1.0)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # 경로 실패

    def heuristic(self, a, b):
        """휴리스틱 함수 (유클리드 거리)"""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def publish_path(self):
        """계획된 경로 퍼블리시"""
        if not self.current_path:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for x, y in self.current_path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('navigation_node')
        navigator = NavigationNode()
        rospy.loginfo("Navigation Node running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Node stopped.")
    except Exception as e:
        rospy.logerr(f"Fatal error in navigation node: {e}")