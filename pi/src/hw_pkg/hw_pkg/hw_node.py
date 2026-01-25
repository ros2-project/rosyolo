import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class DifferentialDrive(Node):
    def __init__(self):
        super().__init__('hw_node')

        # 파라미터를 선언한다.(ROS 파라미터 서버에 등록)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('uart_port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed', 0.25) #해당로봇의 최고속도는 얼마다. 실제로 측정해봐야 하는 값임.

        # 값 불러오기.(코드 내부에서 변수로 사용할 수 있도록 한다)
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        uart_port = self.get_parameter('uart_port').value
        baudrate = self.get_parameter('baudrate').value

        self.add_on_set_parameters_callback(self.param_callback)

        # UART 연결
        try:
            #uart객체 생성
            self.ser = serial.Serial(port=uart_port, baudrate=baudrate, timeout=0.05)
            self.get_logger().info(f"UART 연결 성공: {uart_port} @ {baudrate}")
        except Exception as e:
            self.get_logger().error(f"UART 연결 실패: {e}")
            self.ser = None

        # Subscriber (cmd_vel 받음). 구독. 토픽받으면 콜백함수 호출함
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def param_callback(self, params):
        for param in params:
            if param.name == "wheel_base" and param.type_ == param.TYPE_DOUBLE:
                self.wheel_base = param.value
                self.get_logger().info(f"wheel_base 변경됨: {self.wheel_base}")
            elif param.name == "wheel_radius":
                self.wheel_radius = param.value
            elif param.name == "max_speed":
                self.max_speed = param.value
        return SetParametersResult(successful=True)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x # m/s
        w = msg.angular.z # rad/s

        # 좌/우 바퀴 속도 (m/s). 차륜구동공식임. 로봇중심속도와 각속도를 이용해 오른쪽 왼쪽 바퀴 값 줌
        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)

        # int16 범위 매핑 (-32768 ~ +32767). 스케일링 과정. max와 min으로 천장, 바닥값 한정. max_speed로 나눠줌으로 최대속도 대비 현재 바퀴속도 비를 봄
        left_pwm = int(max(-32768, min(32767, (v_l / self.max_speed) * 32767)))
        right_pwm = int(max(-32768, min(32767, (v_r / self.max_speed) * 32767)))

        if self.ser is not None:
            try:
                # ===== 패킷 만들기 (7바이트) =====
                header = 0xAA
                packet_id = 0x01
                packet = bytearray(7)
                packet[0] = header
                packet[1] = packet_id
                packet[2] = left_pwm & 0xFF          # Left LSB
                packet[3] = (left_pwm >> 8) & 0xFF   # Left MSB. 비트 오른쪽으로 8칸 이동
                packet[4] = right_pwm & 0xFF         # Right LSB
                packet[5] = (right_pwm >> 8) & 0xFF  # Right MSB
                packet[6] = packet_id ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]  # 체크섬

                # 송신
                self.ser.write(packet)
                self.get_logger().info(f"[송신] L={left_pwm}, R={right_pwm}, Packet={packet.hex()}")

                # 에코 수신
                data = self.ser.read(7) #7바이트읽어옴
                if len(data) == 7:
                    l_val = int.from_bytes(data[2:4], 'little', signed=True) #바이트를 int로 변환하여 저장
                    r_val = int.from_bytes(data[4:6], 'little', signed=True)
                    self.get_logger().info(f"[에코 수신] Packet={data.hex()} → L={l_val}, R={r_val}")
                elif data:
                    self.get_logger().warn(f"[에코 불완전] {data.hex()}")

            except Exception as e:
                self.get_logger().error(f"UART 전송/수신 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrive()
    rclpy.spin(node) #ros이벤트 루프 돌리자
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
