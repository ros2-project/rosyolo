import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from control_pkg.pid import PID
import math

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class Twistt(Node):
    def __init__(self):
        super().__init__("twist_node")

        # 파라미터
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('deadzone', 100)
        self.declare_parameter('velocity', 0.25)
        self.declare_parameter('w', 0.1)
        self.declare_parameter('alpha', 0.3)
        self.declare_parameter('P', 0.4)
        self.declare_parameter('I', 0.0)
        self.declare_parameter('D', 0.00)

        self.add_on_set_parameters_callback(self.param_callback)

        # 초기값 불러오기
        self.frame_w = float(self.get_parameter('frame_width').value)
        self.center = self.frame_w / 2.0
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.v_max = float(self.get_parameter('velocity').value)
        self.w_max = float(self.get_parameter('w').value)
        self.alpha = float(self.get_parameter('alpha').value)

        P = float(self.get_parameter('P').value)
        I = float(self.get_parameter('I').value)
        D = float(self.get_parameter('D').value)

        # PID 컨트롤러 생성. PID 객체 생성
        self.pid = PID(P, I, D, output_limits=(-self.w_max, self.w_max))

        # 상태 변수
        self.e_f = 0.0    # 필터링된 오차
        self.t_prev = None

        # ROS 통신
        self.sub = self.create_subscription(Int32MultiArray, '/chatter', self.cb, 10) # 토픽 받으면 자동으로 self.cb 함수 실행
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def param_callback(self, params):
        for param in params:
            if param.name == 'P':
                self.pid.P = float(param.value)
                self.get_logger().info(f"P updated: {self.pid.P}")
            elif param.name == 'I':
                self.pid.I = float(param.value)
                self.get_logger().info(f"I updated: {self.pid.I}")
            elif param.name == 'D':
                self.pid.D = float(param.value)
                self.get_logger().info(f"D updated: {self.pid.D}")
            elif param.name == 'deadzone':
                self.deadzone = float(param.value)
            elif param.name == 'velocity':
                self.v = float(param.value)
            elif param.name == 'w':
                self.w_max = float(param.value)
                self.pid.max_output = self.w_max
                self.pid.min_output = -self.w_max
            elif param.name == 'alpha':
                self.alpha = float(param.value)
        return SetParametersResult(successful=True)

    def cb(self, msg):
        # 1. 오차
        error = float(msg.data[0]) - self.center

        # 2. 저역통과필터 적용
        self.e_f = self.alpha * error + (1.0 - self.alpha) * self.e_f

        # 3. dt 계산
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.t_prev).nanoseconds * 1e-9) if self.t_prev else 1e-3
        self.t_prev = now

        # 4. PID 출력
        w = -self.pid.update(self.e_f, dt) #오차와 dt 넘겨줌. 즉 필터링된 오차와 dt를 넘겨줌으로 update메소드에서 w계산해줌
        # 리턴받은 out을 -붙여줘야 오차 줄이는 방향으로 회전가능함

        # 5. Twist 메시지 작성
        cmd = Twist()
        s=math.sqrt(max(1,msg.data[1]))
        if abs(self.e_f) < self.deadzone: #데드존 안이면 직진하자
            cmd.angular.z = 0.0
            cmd.linear.x  = 2*self.v_max * (1/s)
        else: #데드존 밖이면 회전하자
            cmd.angular.z = w
            cmd.linear.x  = 1.5*self.v_max * (1/s)

        print(f"error: {self.e_f:.2f}, linear.x: {cmd.linear.x:.2f}, angular.z: {cmd.angular.z:.2f}")
        self.pub.publish(cmd) # twist 토픽 발행


def main():
    rclpy.init()
    node = Twistt()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

