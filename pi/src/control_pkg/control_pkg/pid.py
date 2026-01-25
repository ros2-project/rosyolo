import time
import math

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0,frame_width=640, fov_deg=60.0,output_limits=(None, None)):
        self.P = P
        self.I = I
        self.D = D
        self.integral = 0.0
        self.prev_error = 0.0
        self.min_output, self.max_output = output_limits

        # 카메라 파라미터
        self.frame_width = frame_width
        self.fov_rad = math.radians(fov_deg)   # 화각을 rad로 저장
        self.rad_per_px = self.fov_rad / self.frame_width

    def update(self, error_px, dt):
        """
        error_px: 픽셀 단위 오차
        dt: 주기 (초)
        출력: 각속도 (rad/s)
        """
        # 🔹 픽셀 → 라디안 변환. 단위를 라디안으로!
        error = error_px * self.rad_per_px

        # 적분항
        self.integral += error * dt

        # 미분항
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID 출력 (단위: rad/s)
        out = self.P * error + self.I * self.integral + self.D * derivative #오차를 이제 안정적이고 빠르게 줄여주는 알고리즘 공식임.
        #내가직접 PID 변수 튜닝해가면서 적절한 PID값을 찾아야함. 근데 P값만 사용할 것 같음ㅋㅋㅋ

        # 출력 제한
        if self.max_output is not None and out > self.max_output:
            out = self.max_output
        if self.min_output is not None and out < self.min_output:
            out = self.min_output

        # 상태 업데이트
        self.prev_error = error
        return out
