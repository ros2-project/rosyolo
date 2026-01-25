import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry #Odometry 메시지 타입 가져오기
import csv #csv모듈 불러오기
import time

class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.csvfile = open('odom_log.csv', 'w', newline='') #csv파일 열고 데이터 기록 모드.
        self.writer = csv.writer(self.csvfile) #파일에 데이터를 쓸 수 있는 write객체 생성
        self.writer.writerow(['time', 'x', 'y', 'theta'])

    def cb(self, msg: Odometry): #x,y는 위치. yaw는 그 위치에서의 방향
        x = msg.pose.pose.position.x #로봇의 좌표
        y = msg.pose.pose.position.y
        # 쿼터니언 → yaw 변환 (간단 버전, 정확히는 tf_transform 필요)
        import math
        q = msg.pose.pose.orientation #로봇의 자세
        #쿼터니언을 yaw로 변환
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        theta = math.atan2(siny_cosp, cosy_cosp) #쿼터니언에서 yaw로 바꾸는 공식

        t = self.get_clock().now().nanoseconds * 1e-9 #현재 시간. 나노는 1e-9임
        self.writer.writerow([t, x, y, theta]) #csva파일에 작성

    def destroy_node(self):
        self.csvfile.close() #닫기
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
