#!/usr/bin/env python3
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String

class SystemMonitor:
    def __init__(self):
        rospy.init_node('system_monitor')
        
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.status_pub = rospy.Publisher('/system_status', String, queue_size=10)
        
        rospy.Timer(rospy.Duration(1), self.publish_diagnostics)
        
    def publish_diagnostics(self, event):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        
        # 센서 상태 체크
        status = DiagnosticStatus()
        status.name = "Sensors Status"
        status.level = DiagnosticStatus.OK
        status.message = "All sensors operational"
        
        # ... (상세 진단 로직 추가)
        
        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)
        self.status_pub.publish("System OK")

if __name__ == '__main__':
    monitor = SystemMonitor()
    rospy.spin()