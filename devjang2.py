import serial
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# I2C 버스 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# PCA9685 주파수 설정
pca.frequency = 60

# 기본 값 설정 (정지, 중심)
CENTER_DUTY = 1375
STOP_DUTY = 0x179F
FORWARD_DUTY = 1510
LEFT_DUTY = 1200
RIGHT_DUTY = 1800

# 초기 스티어링과 스로틀 설정
pca.channels[3].duty_cycle = CENTER_DUTY  # Center
pca.channels[5].duty_cycle = STOP_DUTY    # Stop

def process_sensor_state(left_state, center_state, right_state):
    """센서 상태에 따라 차량 제어"""

    if left_state == 0 and center_state == 1 and right_state == 1:
        print("왼쪽으로 회전")
        pca.channels[3].duty_cycle = LEFT_DUTY

    elif left_state == 1 and center_state == 0 and right_state == 1:
        print("직진")
        pca.channels[3].duty_cycle = CENTER_DUTY
        pca.channels[5].duty_cycle = FORWARD_DUTY  # 전진 속도 설정

    elif left_state == 1 and center_state == 1 and right_state == 0:
        print("오른쪽으로 회전")
        pca.channels[3].duty_cycle = RIGHT_DUTY

    elif left_state == 0 and center_state == 0 and right_state == 1:
        print("왼쪽과 중앙 활성화됨, 약간 왼쪽 회전")
        pca.channels[3].duty_cycle = LEFT_DUTY + 100

    elif left_state == 1 and center_state == 0 and right_state == 0:
        print("오른쪽과 중앙 활성화됨, 약간 오른쪽 회전")
        pca.channels[3].duty_cycle = RIGHT_DUTY - 100

    elif left_state == 0 and center_state == 1 and right_state == 0:
        print("왼쪽과 오른쪽 활성화됨, 직진 속도 유지")
        pca.channels[3].duty_cycle = CENTER_DUTY
        pca.channels[5].duty_cycle = FORWARD_DUTY

    elif left_state == 0 and center_state == 0 and right_state == 0:
        print("모든 센서 활성화됨, 긴급 정지")
        pca.channels[5].duty_cycle = STOP_DUTY

    else:
        print("센서가 비활성화됨, 정지")
        pca.channels[5].duty_cycle = STOP_DUTY

def mapping_data(value):
    return int((value / 15900) * 65535)

def throttle_change(current_value, target_value, step=150):
    if current_value < target_value:
        return min(current_value + step, target_value)
    elif current_value > target_value:
        return max(current_value - step, target_value)
    return current_value

def steer_change(current_value, target_value):
    diff = abs(target_value - current_value)
    if diff > 5000:
        step = 2000
    elif diff > 1000:
        step = 1000
    else:
        step = 500

    if current_value < target_value:
        return min(current_value + step, target_value)
    elif current_value > target_value:
        return max(current_value - step, target_value)
    
    return current_value

def running():
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        while True:
            content = seri.readline().decode().strip()
            try:
                if len(content) == 0:
                    print("데이터 없음")
                else:
                    values = content.split(',')
                    steer_value = int(values[0].strip())
                    throttle_value = int(values[1].strip())
                    controller_value = int(values[2].strip())
                    left_state = int(values[3].strip())
                    center_state = int(values[4].strip())
                    right_state = int(values[5].strip())
             
                    if controller_value < 1200:
                        print("자동 모드")
                        process_sensor_state(left_state, center_state, right_state)
                    else:
                        print("수동 모드")
                        current_steer_value = pca.channels[3].duty_cycle
                        target_steer_value = mapping_data(steer_value)
                        pca.channels[3].duty_cycle = steer_change(current_steer_value, target_steer_value)

                        current_throttle_value = pca.channels[5].duty_cycle
                        target_throttle_value = mapping_data(throttle_value)
                        pca.channels[5].duty_cycle = throttle_change(current_throttle_value, target_throttle_value)

            except (ValueError, IndexError) as e:
                print(f"데이터 처리 오류: {e}")
                continue

if __name__ == "__main__":
    running()
