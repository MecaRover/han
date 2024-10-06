import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
import threading

# MPU6050 초기화
mpu = mpu6050(0x68)

# 스텝 모터 핀 설정 (예: CW, CCW 핀)
motors = {
    "M1": {"CW": 5, "CCW": 6, "HOLD": 13},  # M1 핀 설정
    "M2": {"CW": 12, "CCW": 16, "HOLD": 19},  # M2 핀 설정
    "M3": {"CW": 20, "CCW": 21, "HOLD": 26},  # M3 핀 설정
    "M4": {"CW": 17, "CCW": 27, "HOLD": 22}   # M4 핀 설정
}

# 필터링을 위한 파라미터
alpha = 0.02  # 필터 계수를 낮추어 자이로 데이터를 부드럽게 필터링
filtered_angle_x = 0  # 필터링된 X축 각도 값을 저장할 변수
filtered_angle_y = 0  # 필터링된 Y축 각도 값을 저장할 변수

# 각도 변화 임계값 설정
angle_threshold = 0.03  # 각도 변화 임계값, 너무 작은 각도 변화 무시 / 각도 임계값 조절 

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
for motor_name, pins in motors.items():
    for pin_name, pin in pins.items():
        if isinstance(pin, int):  # 핀 번호가 정수인지 확인
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

# 펄스 생성 함수
def generate_pulse(pins, steps, delay):
    for step in range(steps):
        for pin in pins:
            GPIO.output(pin, GPIO.HIGH)
        time.sleep(delay)
        for pin in pins:
            GPIO.output(pin, GPIO.LOW)
        time.sleep(delay)
        # 스텝별 펄스 출력
        print(f"펄스 발생: {step+1}/{steps}")

# 모터 상승 시작 (CCW 방향으로 500 펄스)
def initial_pulse_ccw():
    # 모든 모터의 CCW 핀을 활성화하여 회전
    for motor_pins in motors.values():
        GPIO.output(motor_pins["CCW"], GPIO.HIGH)  # CCW 핀을 high로 설정하여 CCW 방향으로 회전

    # 펄스 생성 (500 스텝 발생)
    generate_pulse([motor_pins["CW"] for motor_pins in motors.values()], 00, 0.0005)

    time.sleep(0.5)  # 1초 대기

# 각도를 기반으로 스텝 모터를 회전시키는 함수 (스레드에서 실행)
def step_motor(motor_name, angle_change, reverse=False):
    motor = motors[motor_name]

    if reverse:  # 반대 방향으로 회전
        if angle_change > 0:
            GPIO.output(motor["CCW"], GPIO.HIGH)
            GPIO.output(motor["CW"], GPIO.LOW)
        elif angle_change < 0:
            GPIO.output(motor["CCW"], GPIO.LOW)
            GPIO.output(motor["CW"], GPIO.HIGH)
    else:  # 정방향 회전
        if angle_change > 0:
            GPIO.output(motor["CW"], GPIO.HIGH)
            GPIO.output(motor["CCW"], GPIO.LOW)
        elif angle_change < 0:
            GPIO.output(motor["CW"], GPIO.LOW)
            GPIO.output(motor["CCW"], GPIO.HIGH)

    steps = int(abs(angle_change) * 100)  # 스텝 수를 줄여 더 빠르게 반응하도록 조정
    generate_pulse([motor["CW"]], steps, 0.0005)  # 펄스 발생 속도를 빠르게

# 각 모터의 이전 각도 저장
previous_angles = {
    "M1": 0,
    "M2": 0,
    "M3": 0,
    "M4": 0
}

# 모터 제어를 위한 스레드 함수
def control_motor(motor_name, axis="x", reverse=False):
    global filtered_angle_x, filtered_angle_y

    while True:
        # 각도 변화량 계산
        if axis == "x":  # 피치 (상하 움직임)
            angle_change = filtered_angle_x - previous_angles[motor_name]
            previous_angles[motor_name] = filtered_angle_x
        elif axis == "y":  # 롤 (좌우 움직임)
            angle_change = filtered_angle_y - previous_angles[motor_name]
            previous_angles[motor_name] = filtered_angle_y

        # 각도 변화가 임계값을 넘을 때만 모터를 회전
        if abs(angle_change) > angle_threshold:
            step_motor(motor_name, angle_change, reverse)

        # 딜레이 추가
        time.sleep(0.005)  # 주기를 줄여 모터를 더 자주 제어

try:
    # CCW 방향으로 초기 500 펄스 상승
    initial_pulse_ccw()

    # 각 모터를 위한 스레드 생성 (M1/M3은 X축, M2/M4는 Y축 제어)
    motor_threads = [
        threading.Thread(target=control_motor, args=("M3", "x")),     # M1: X축 기준 정방향
        threading.Thread(target=control_motor, args=("M1", "x", True)),  # M3: X축 기준 반대 방향
        threading.Thread(target=control_motor, args=("M2", "y")),     # M2: Y축 기준 정방향
        threading.Thread(target=control_motor, args=("M4", "y", True))  # M4: Y축 기준 반대 방향
    ]

    # 스레드 시작
    for thread in motor_threads:
        thread.start()

    while True:
        # 자이로에서 X축 및 Y축 각도 값 읽기
        gyro_data = mpu.get_gyro_data()  # X, Y, Z 자이로 데이터를 얻음
        raw_angle_x = gyro_data['x']
        raw_angle_y = gyro_data['y']

        # 저속 통과 필터를 적용하여 자이로 값을 부드럽게 필터링
        filtered_angle_x = alpha * raw_angle_x + (1 - alpha) * filtered_angle_x
        filtered_angle_y = alpha * raw_angle_y + (1 - alpha) * filtered_angle_y

        # X축 및 Y축 필터링된 각도 값을 출력 (소수점 3자리까지만 출력)
        print(f"필터링된 X축 각도: {filtered_angle_x:.3f}, 필터링된 Y축 각도: {filtered_angle_y:.3f}")

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
