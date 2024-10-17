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
pca.channels[3].duty_cycle = 0x7D0  # Center, 0x1806, us is 6150
pca.channels[5].duty_cycle = 0x179F  # Stop, 0x179F, 6047

# 고정된 HIGH값으로 실행시 조작하지않아도 출력이 증가하는 문제가 발생하여 아래와 같은 방식으로 동적으로 변경함
# 데이터 맵핑을 동적으로 처리하기위해 HIGH값만 변수처리하여 실시간으로 아두이노에서 넘어오는 값을 변환함
def mappingData(highValue):
    return int((highValue / 15900) * 65535)  # PCA value 값을 리턴 (15900: 아두이노 신호의 HIGH, LOW 값을 더한 값)

# 쓰로틀값을 변환하는 함수로 기존에는 step값을 50으로 설정하여 실행하였지만 너무 민감하게 반응하여 조작에 어려움을 겪었다.
# step값을 증가시키며 적절한 step값을 찾았다.(step=150) 
# 추후에 라인트래킹시 step값을 조절하여 속도조절?
def throttleConvert(currentValue, targetValue, step=150):  # step을 20으로 작게 설정하여 천천히 변화
    if currentValue < targetValue:
        return min(currentValue + step, targetValue)
    elif currentValue > targetValue:
        return max(currentValue - step, targetValue)
    return currentValue

# 기존에는 스로틀과 스티어링 모두 하나의 함수로 값을 변환했지만 스티어링의 경우 지나치게 느리게 조향되는 문제가 발생하였다.
# 스로틀과 스티어링의 함수를 분리하였고, 목표 각도와 현재 각도의 차이를 분기처리하여 
# 두 값의 차이가 클 수록 더 큰 step으로 조정하여 빠르게 조향할 수 있도록 하였다.
# 조향속도를 느리게 하고자 할 때도 대응하기 위해 차이가 적으면 작은 step을 주었다.
def steerConvert(currentValue, targetValue):
    diff = abs(targetValue - currentValue)
    
    # 차이가 클수록 더 빠르게 이동
    if diff > 5000:
        step = 2000  # 차이가 크면 더 큰 step으로 조정
    elif diff > 1000:
        step = 1000  # 차이가 중간이면 중간 step
    else:
        step = 500   # 차이가 작으면 작은 step

    if currentValue < targetValue:
        return min(currentValue + step, targetValue)
    elif currentValue > targetValue:
        return max(currentValue - step, targetValue)
    
    return currentValue

def running():
    # 시리얼 통신 설정 (아두이노와 연결)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        while True:
            recieveData = seri.readline().decode().strip()
            try:

                values = recieveData.split(',')

                steerValue = int(values[0].strip())  # 첫 번째 값은 스티어링
                throttleValue = int(values[1].strip())  # 두 번째 값은 스로틀

                print(f"steer: {steerValue}, throttle: {throttleValue}")

                currentSteerValue = pca.channels[3].duty_cycle
                targetSteerValue = mappingData(steerValue)
                pca.channels[3].duty_cycle = steerConvert(currentSteerValue, targetSteerValue)

                currentThrottleValue = pca.channels[5].duty_cycle
                targetThrottleValue = mappingData(throttleValue)
                pca.channels[5].duty_cycle = throttleConvert(currentThrottleValue, targetThrottleValue)

                print(f"mapping steer: {pca.channels[3].duty_cycle}, throttle: {pca.channels[5].duty_cycle}")

            except (ValueError, IndexError) as e:
                print(f"Error running: {e}")
                continue

if __name__ == "__main__":
    running()
