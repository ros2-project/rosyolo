import cv2, torch, numpy as np #라이브러리 가져오기/ 파이토치: 파이썬에서 딥러닝을 하기 위한 라이브러리
from yolov5.models.common import DetectMultiBackend #models안의 common 파이선 파일에서 이 클래스를 가져온다
from yolov5.utils.augmentations import letterbox
from yolov5.utils.general import non_max_suppression, scale_boxes
from yolov5.utils.torch_utils import select_device
#models: 모델 구조/ 불러오기 관련 코드
#utils: 모델이 잘 돌도록 돕는 도우미 코드 모음

import rclpy #로스 메인 패키지. 라이브러리 가져오기
from rclpy.node import Node #rlcpy안에 있는 node.py파일 안에 있는 클래스 Node
from std_msgs.msg import Int32MultiArray #std_msgs 파캐지 안에 msg폴더 안에 String메시지를 가져와

from sensor_msgs.msg import Image, CompressedImage #Image: 비압축 원복 영상 메시지, Com:압축된 영상 메시지. -> 로스에서 카메라 영상 주고 받기 위해서 사용
from cv_bridge import CvBridge #OpenCV이미지와 ROS2 이미지 메시지를 변환해줌

from rcl_interfaces.msg import SetParametersResult
import time

class YoloPublisher(Node): #Node클래스 상속받고 클래스 생성
    def __init__(self):
        super().__init__('yolo_node') #부모노드 초기화
        self.declare_parameter('weights', '/home/pi/pickachu/pick_ws/src/yolov5/best_0905.pt')
        self.declare_parameter('img', 480)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('skip', 2)
        self.declare_parameter('timer', 0.05)


        self.add_on_set_parameters_callback(self.param_callback) #파라미터 바뀌면 콜백함수 실행

        self.weights = self.get_parameter('weights').value
        self.img = self.get_parameter('img').value
        self.conf = self.get_parameter('conf').value
        self.iou = self.get_parameter('iou').value
        self.skip = self.get_parameter('skip').value
        self.timer_v = self.get_parameter('timer').value
        

        self.pub = self.create_publisher(Int32MultiArray, '/chatter', 10) #타입, 토픽이름, 큐사이즈

        self.img_pub = self.create_publisher(Image, '/yolo/image', 10)# 애는 이미지 보낼 놈
        self.img_pub_comp = self.create_publisher(CompressedImage, '/yolo/image/compressed', 10)#얘는 이미지 보낼 놈
        self.bridge = CvBridge()# 변환기

        self.model = DetectMultiBackend(self.weights, device=select_device('cpu')) #가중치파일을 열어서 cpu에서 돌릴 수 있는 모델객체 생성
        self.model.model.float().eval() #self.model안에 있는 파이토치모델을 까네아 float, eval 하기
        self.stride, self.names = int(self.model.stride), self.model.names #이미지 칸칸, 클래스이름목록

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2) # 카메라 열기. 0: 첫번째 카메라. CAP_V4l2: 리눅스 카메라 드라이버 모드
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #카메라 딜레이 없고 최신 프레임만 가져오기 설정

        self.i = 0 #프레임 카운터 초기화
        self.get_logger().info("YOLO Publisher 시작")

        self.timer = self.create_timer(self.timer_v, self.loop_once) #loop_once함수 계속 실행. timer_v주기로 계속

    def param_callback(self, params):  # 파라미터 콜백 함수
        for param in params:
            if param.name == 'weights':
                self.weights = str(param.value)
                self.get_logger().info(f"weights updated: {self.weights}")

            elif param.name == 'img':
                self.img = int(param.value)
                self.get_logger().info(f"img updated: {self.img}")

            elif param.name == 'conf':
                self.conf = float(param.value)
                self.get_logger().info(f"conf updated: {self.conf}")

            elif param.name == 'iou':
                self.iou = float(param.value)
                self.get_logger().info(f"iou updated: {self.iou}")

            elif param.name == 'skip':
                self.skip = int(param.value)
                self.get_logger().info(f"skip updated: {self.skip}")

            elif param.name == 'timer':
                self.timer_v =float(param.value)
                self.get_logger().info(f"skip updated: {self.timer_v}")
                # 기존 타이머 취소
                self.timer.cancel()
                # 새 타이머 생성
                self.timer = self.create_timer(self.timer_v, self.loop_once)

        return SetParametersResult(successful=True)

    def publish_frames(self, frame_bgr): #압축/비압축 프레임 보내기
        now = self.get_clock().now().to_msg() #노드의 현재시간을 뽑아서 로스 시간메시지로 바꾸기

        # 비압축
        img_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8') #opencv이미지를 로스 메시지로 변환 / frame_bgr: (높이,너비,채널)
        img_msg.header.stamp = now #이 프레임이 찍힌 시간
        img_msg.header.frame_id = 'camera' #어떤 카메라인지 이름표
        self.img_pub.publish(img_msg) #토픽발행

        # 압축 (bandwidth 줄이기용)
        """
        comp_msg = CompressedImage()
        comp_msg.header.stamp = now #시간
        comp_msg.header.frame_id = 'camera' #카메라이름표
        comp_msg.format = 'jpeg' #압축방식
        comp_msg.data =cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])[1].tobytes() #(어떤 포맷으로 압축할지, 입력이미지 정보, 품질).buf만 가져오기.buf를 바이트로 바꾼다
        self.img_pub_comp.publish(comp_msg)
        """
        

    def loop_once(self):
        ok, f = self.cap.read() #카메라에서 한 프레임 읽기.
        if not ok:
            return

        f = cv2.flip(f, 1)

        self.i += 1
        # 추론 스킵: CPU에서 끊김 줄이기
        do_infer = (self.i % (self.skip + 1) == 0) #스킵하기 위함

        draw = f.copy() #프레임 f 복사본

        if do_infer: #do_infer이 0이 아니면 실행

            start=time.time()

            im = letterbox(f, self.img, stride=self.stride, auto=True)[0] #패딩 후 격자에 맞게 크기 정렬해줌.가공된이미지만[0]/ 320으로 해야 욜로추론이 빨라지기 떄문
            im = im[:, :, ::-1].transpose(2, 0, 1) #채널순서를 뒤집은 후 (H,W,3) -> (3,H,W)로 트랜스포즈
            im = np.ascontiguousarray(im) #연속상태로 재배치
            im = torch.from_numpy(im).to(self.model.device).float() / 255.0 #넘파이배열을 텐서로 바꾸고 cpu로 올림. 그리고 float32로 변환. 그리고 255.5로 나눔->0~1 값으로 변환
            if im.ndim == 3:
                im = im.unsqueeze(0) #차원하나 추가해서 (1,3,H,W)로 맞춰줌

            det = non_max_suppression(self.model(im), self.conf, self.iou)[0] #모델에 im넣기. 결과는 좌표, 시뇌도, 클래스번호가 있다

            end = time.time()     # 끝난 시간 기록
            elapsed = (end - start) * 1000  # ms
            fps = 1.0 / (end - start)
            self.get_logger().info(f"YOLO inference: {elapsed:.1f} ms ({fps:.2f} FPS)")

            if det is not None and len(det): #검출결과 있고 박스 있을 때 실행
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], f.shape).round() #(모델입력크기, 좌표, 원본프레임크기).정수픽섹좌표로 변환/ 모델입력좌표에서 원본으로 바꾸는작업
                # 가장 신뢰도 높은 박스 하나 집기 (필요하면 loop로 모두 그림)
                j = det[:, 4].argmax().item() #모든 박스의 신뢰도 열에서 argmax로 가장 큰 인덱스만 반환(신뢰도). 그리고 item하여 파이썬 int로 바꿈
                x1, y1, x2, y2, conf, cls = det[j].tolist()[:6] #j 박스를 꺼낸다. 그후 파이썬 리스트로 바꾸고 앞 6개만 자르기
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2 #중심점 계산

                w_box = int(x2 - x1)

                # 좌표 출력 & chatter 발행
                print(f"x: {cx}, y: {cy}", flush=True)
                msg = Int32MultiArray() #로스 문자열 타입 메시지 만들기
                msg.data = [int(cx), w_box] #메시지 안에 중심좌표(가로축) 담기
                self.pub.publish(msg) #토픽발행

                # 화면에 박스/중심점 그리기
                cv2.rectangle(draw, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2) #좌표, 박스색, 두께
                cv2.circle(draw, (int(cx), int(cy)), 4, (0, 0, 255), -1) #점좌표, 점크기, 점색, 채움모드
                cv2.putText(draw, f"{self.names[int(cls)]} {conf:.2f}",
                            (int(x1), int(y1)-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2) #출력문자열, 신뢰도, 문자출력위치, 글꼴, 글자크기, 글자색, 글자두께

            """ 
            else:
                # 미검출 시 표식 (원하면 주석처리)
            
                cv2.putText(draw, "no detection", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            """   


        self.publish_frames(draw) #화면보내깅/ 욜로박스있는거나 안 된거나 다 발행


def main():
    rclpy.init()
    node = YoloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': # 이 파일이 직접 실행되어야 main을 실행한다
    main()