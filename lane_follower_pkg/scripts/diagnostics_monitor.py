#!/usr/bin/env python3

import rospy
import diagnostic_msgs.msg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, PointCloud2
import time

class DiagnosticsMonitor:
    def __init__(self):
        rospy.init_node('diagnostics_monitor')
        
        # 시스템 상태 변수 초기화
        self.system_state = "INITIALIZING"
        self.last_camera_time = 0
        self.last_lidar_time = 0
        self.last_steering_time = 0
        self.last_drive_time = 0
        self.last_arduino_time = 0
        self.emergency_stop = False
        self.battery_level = 100.0
        
        # 진단 메시지 발행자
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.system_status_pub = rospy.Publisher('/system_status', String, queue_size=10)
        
        # 서브스크라이버 설정
        rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/steering_feedback', Float32, self.steering_callback)
        rospy.Subscriber('/drive_command', Float32, self.drive_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)
        
        # 진단 타이머 설정
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        self.battery_timer = rospy.Timer(rospy.Duration(60.0), self.update_battery_level)
        
        rospy.loginfo("Diagnostics Monitor initialized")

    def camera_callback(self, msg):
        self.last_camera_time = time.time()

    def lidar_callback(self, msg):
        self.last_lidar_time = time.time()

    def steering_callback(self, msg):
        self.last_steering_time = time.time()

    def drive_callback(self, msg):
        self.last_drive_time = time.time()

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def update_battery_level(self, event):
        # 배터리 레벨 시뮬레이션 (실제 구현시 하드웨어에서 읽어와야 함)
        self.battery_level = max(0, self.battery_level - 1.0)
        if self.battery_level < 20.0:
            rospy.logwarn(f"Low battery warning: {self.battery_level}% remaining")

    def check_sensor_timeouts(self):
        current_time = time.time()
        sensor_timeout = 2.0  # 2초 타임아웃
        
        # 카메라 상태 확인
        camera_active = (current_time - self.last_camera_time) < sensor_timeout
        if not camera_active:
            rospy.logerr("Camera feed timeout detected")
        
        # 라이다 상태 확인
        lidar_active = (current_time - self.last_lidar_time) < sensor_timeout
        if not lidar_active:
            rospy.logerr("LiDAR feed timeout detected")
            
        return camera_active, lidar_active

    def check_actuator_timeouts(self):
        current_time = time.time()
        actuator_timeout = 5.0  # 5초 타임아웃
        
        # 조향 시스템 확인
        steering_active = (current_time - self.last_steering_time) < actuator_timeout
        if not steering_active:
            rospy.logerr("Steering command timeout detected")
            
        # 구동 시스템 확인
        drive_active = (current_time - self.last_drive_time) < actuator_timeout
        if not drive_active:
            rospy.logerr("Drive command timeout detected")
            
        return steering_active, drive_active

    def determine_system_state(self):
        camera_active, lidar_active = self.check_sensor_timeouts()
        steering_active, drive_active = self.check_actuator_timeouts()
        
        if self.emergency_stop:
            return "EMERGENCY_STOP"
        elif not camera_active or not lidar_active:
            return "SENSOR_FAILURE"
        elif not steering_active or not drive_active:
            return "ACTUATOR_FAILURE"
        elif self.battery_level < 10.0:
            return "LOW_BATTERY"
        else:
            return "NORMAL"

    def publish_diagnostics(self, event):
        # 시스템 상태 업데이트
        self.system_state = self.determine_system_state()
        
        # 진단 메시지 생성
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        
        # 시스템 상태 메시지
        system_status = DiagnosticStatus()
        system_status.name = "System Status"
        system_status.level = DiagnosticStatus.OK if self.system_state == "NORMAL" else DiagnosticStatus.ERROR
        system_status.message = self.system_state
        system_status.hardware_id = "autonomous_car_system"
        
        # 센서 상태
        camera_active, lidar_active = self.check_sensor_timeouts()
        sensor_status = DiagnosticStatus()
        sensor_status.name = "Sensors"
        sensor_status.level = DiagnosticStatus.OK if camera_active and lidar_active else DiagnosticStatus.ERROR
        sensor_status.message = "Cam: {}, LiDAR: {}".format(
            "Active" if camera_active else "Inactive",
            "Active" if lidar_active else "Inactive"
        )
        sensor_status.values = [
            KeyValue("Camera", "OK" if camera_active else "ERROR"),
            KeyValue("LiDAR", "OK" if lidar_active else "ERROR"),
            KeyValue("Last Camera Update", str(time.time() - self.last_camera_time)),
            KeyValue("Last LiDAR Update", str(time.time() - self.last_lidar_time))
        ]
        
        # 액추에이터 상태
        steering_active, drive_active = self.check_actuator_timeouts()
        actuator_status = DiagnosticStatus()
        actuator_status.name = "Actuators"
        actuator_status.level = DiagnosticStatus.OK if steering_active and drive_active else DiagnosticStatus.ERROR
        actuator_status.message = "Steering: {}, Drive: {}".format(
            "Active" if steering_active else "Inactive",
            "Active" if drive_active else "Inactive"
        )
        actuator_status.values = [
            KeyValue("Steering", "OK" if steering_active else "ERROR"),
            KeyValue("Drive", "OK" if drive_active else "ERROR"),
            KeyValue("Last Steering Command", str(time.time() - self.last_steering_time)),
            KeyValue("Last Drive Command", str(time.time() - self.last_drive_time))
        ]
        
        # 배터리 상태
        battery_status = DiagnosticStatus()
        battery_status.name = "Power System"
        battery_status.level = DiagnosticStatus.WARN if self.battery_level < 20.0 else DiagnosticStatus.OK
        battery_status.message = "Battery: {:.1f}%".format(self.battery_level)
        battery_status.values = [
            KeyValue("Battery Level", "{:.1f}%".format(self.battery_level)),
            KeyValue("Voltage", "{:.2f}V".format(self.battery_level * 0.12 + 10.0))  # 시뮬레이션 값
        ]
        
        # 모든 상태 메시지 추가
        diag_array.status = [system_status, sensor_status, actuator_status, battery_status]
        
        # 발행
        self.diag_pub.publish(diag_array)
        self.system_status_pub.publish(String(self.system_state))

if __name__ == '__main__':
    try:
        monitor = DiagnosticsMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Diagnostics monitor node shutdown")