
import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import serial
import threading
import time
import re

# 전역 변수 설정
distance_to_marker = 0
speedleft1 = speedright1 = 0
speedleft2 = speedright2 = 0
speedleft3 = speedright3 = 0
distance1 = distance2 = distance3 = distance4 = 0
last_detection_time = 0
emergency_stop_message = ""

# 시리얼 포트 설정
def setup_serial():
    global arduino
    try:
        arduino = serial.Serial('/dev/ttyACM0',115200)  # Jetson Nano에 연결된 포트로 변경 필요
        time.sleep(2)  # 아두이노와의 연결 대기 시간
    except serial.SerialException as e:
        print(f"시리얼 포트 연결 오류: {e}")
        arduino = None

setup_serial()

# 모터 제어 함수
def stop_motors():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    if arduino:
        send_to_arduino(0, 0, 0, 0, 0, 0)
    speedleft1 = speedright1 = 0
    speedleft2 = speedright2 = 0
    speedleft3 = speedright3 = 0
    update_labels()

def smooth_acceleration(target_speeds, step=0.05):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    while True:
        # Accelerate the speed towards the target speeds
        speedleft1 = accelerate_towards(speedleft1, target_speeds[0], step)
        speedright1 = accelerate_towards(speedright1, target_speeds[1], step)
        speedleft2 = accelerate_towards(speedleft2, target_speeds[2], step)
        speedright2 = accelerate_towards(speedright2, target_speeds[3], step)
        speedleft3 = accelerate_towards(speedleft3, target_speeds[4], step)
        speedright3 = accelerate_towards(speedright3, target_speeds[5], step)
        
        set_motor_speeds()
        update_labels()
        
        if all_close([speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3], target_speeds):
            break
        time.sleep(0.1)

def accelerate_towards(current, target, step):
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

def all_close(values, targets, tolerance=0.01):
    return all(abs(v - t) < tolerance for v, t in zip(values, targets))

def set_motor_speeds():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    if arduino:
        send_to_arduino(speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3)

def send_to_arduino(left_speed1, right_speed1, left_speed2, right_speed2, left_speed3, right_speed3):
    if arduino:
        try:
            arduino.write(f"{left_speed1},{right_speed1},{left_speed2},{right_speed2},{left_speed3},{right_speed3}\n".encode())
        except serial.SerialException as e:
            print(f"데이터 전송 오류: {e}")

def update_labels():
    # 모터 속도 값 표시
    left_label1.config(text=f"왼쪽 모터 : {round(speedleft1, 2)}")
    right_label1.config(text=f"오른쪽 모터 : {round(speedright1, 2)}")

    # 긴급 정지 메시지 업데이트
    emergency_label.config(text=emergency_stop_message)

def increase_speed(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        min(speedleft1 + 0.05, 1),
        min(speedright1 + 0.05, 1),
        min(speedleft2 + 0.05, 1),
        min(speedright2 + 0.05, 1),
        min(speedleft3 + 0.05, 1),
        min(speedright3 + 0.05, 1),
    ]
    smooth_acceleration(target_speeds)
    
def decrease_speed(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        max(speedleft1 - 0.05, -1),
        max(speedright1 - 0.05, -1),
        max(speedleft2 - 0.05, -1),
        max(speedright2 - 0.05, -1),
        max(speedleft3 - 0.05, -1),
        max(speedright3 - 0.05, -1),
    ]
    smooth_acceleration(target_speeds)

def steer_right(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        max(speedleft1 - 0.05, -1),
        min(speedright1 + 0.05, 1),
        max(speedleft2 - 0.05, -1),
        min(speedright2 + 0.05, 1),
        max(speedleft3 - 0.05, -1),
        min(speedright3 + 0.05, 1),
    ]
    smooth_acceleration(target_speeds)

def steer_left(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        min(speedleft1 + 0.05, 1),
        max(speedright1 - 0.05, -1),
        min(speedleft2 + 0.05, 1),
        max(speedright2 - 0.05, -1),
        min(speedleft3 + 0.05, 1),
        max(speedright3 - 0.05, -1),
    ]
    smooth_acceleration(target_speeds)

def quit_program(event=None):
    stop_motors()
    if arduino:
        arduino.close()
    cap.release()
    root.destroy()

def detect_aruco_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    markers = []
    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i][0]
            center = np.mean(marker_corners, axis=0).astype(int)
            markers.append({
                'id': ids[i][0],
                'center': center,
                'corners': marker_corners
            })
    
    return markers

def draw_marker_overlay(frame, markers):
    for marker in markers:
        marker_corners = marker['corners']
        cv2.polylines(frame, [marker_corners.astype(np.int32)], True, (0, 255, 0), 2)
        center = marker['center']
        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
        marker_id = marker['id']
        cv2.putText(frame, str(marker_id), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

def aruco_following():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3, distance_to_marker, last_detection_time
    stop_event = threading.Event()

    while not stop_event.is_set():
        if mode_var.get() == 2:  # ArUco 모드일 때만 작동
            ret, frame = cap.read()

            if ret:
                markers = detect_aruco_markers(frame)
                print(f"Detected markers: {markers}")  # 디버깅용

                if markers:
                    marker_center_x = markers[0]['center'][0]
                    frame_center_x = frame.shape[1] // 2

                    deviation = (marker_center_x - frame_center_x) / frame_center_x
                    speed_adjustment = deviation * 0.5

                    marker_size = np.sqrt(np.sum((markers[0]['corners'][0] - markers[0]['corners'][1])**2))
                    focal_length = 1000
                    real_marker_size = 9
                    distance_to_marker = focal_length * real_marker_size / marker_size

                    print(f"Marker size: {marker_size}, Distance to marker: {distance_to_marker}")  # 디버깅용

                    last_detection_time = time.time()

                    desired_distance = 40
                    if distance_to_marker < desired_distance:
                        speedleft1 = speedright1 = speedleft2 = speedright2 = speedleft3 = speedright3 = 0
                    else:
                        speed_base = 0.4
                        speedleft1 = speed_base - speed_adjustment
                        speedright1 = speed_base + speed_adjustment
                        speedleft2 = speed_base - speed_adjustment
                        speedright2 = speed_base + speed_adjustment
                        speedleft3 = speed_base - speed_adjustment
                        speedright3 = speed_base + speed_adjustment

                    # 속도 범위 제한
                    speedleft1 = max(min(speedleft1, 0.7), -0.7)
                    speedright1 = max(min(speedright1, 0.7), -0.7)
                    speedleft2 = max(min(speedleft2, 0.7), -0.7)
                    speedright2 = max(min(speedright2, 0.7), -0.7)
                    speedleft3 = max(min(speedleft3, 0.7), -0.7)
                    speedright3 = max(min(speedright3, 0.7), -0.7)

                    set_motor_speeds()
                    


                    # GUI 업데이트 예약
                    root.after(100, update_labels)
                    root.after(100, update_distance_label)

                else:
                    if time.time() - last_detection_time > 1:
                        stop_motors()

        time.sleep(0.1)

    stop_event.set()

def update_distance_label():
    distance_label.config(text=f"마커까지 거리: {round(distance_to_marker, 2)} cm")

def mode_selection_changed(*args):
    if mode_var.get() == 2:  # ArUco 모드로 변경된 경우
        stop_motors()  # 모든 모터를 멈춤
        aruco_follow_thread = threading.Thread(target=aruco_following, daemon=True)
        aruco_follow_thread.start()

def update_serial_data():
    global distance1, distance2, distance3, distance4, emergency_stop_message
    if arduino and arduino.in_waiting:
        try:
            data = arduino.readline().decode().strip()
            print(f"수신 데이터: {data}")  # 수신 데이터 확인용
            # 정규 표현식을 사용하여 거리 값을 추출
            match = re.match(r'Distance1: (\d+)\s*cm,\s*Distance2: (\d+)\s*cm,\s*Distance3: (\d+)\s*cm,\s*Distance4: (\d+)\s*cm', data)
            if match:
                distance1 = float(match.group(1))
                distance2 = float(match.group(2))
                distance3 = float(match.group(3))
                distance4 = float(match.group(4))
                emergency_stop_message = ""  # 정상 상태로 복구
                update_labels()
            else:
                # 긴급 정지 메시지 감지
                if "Emergency Stop!" in data:
                    emergency_stop_message = "긴급 정지!"
                    stop_motors()
                    update_labels()
        except ValueError as e:
            print(f"데이터 변환 오류: {e}")
    root.after(10, update_serial_data)  # 100 밀리초마다 시리얼 데이터 업데이트

# GUI 초기화
root = tk.Tk()
root.title("로봇 제어 GUI")

# 모터 속도와 거리 표시 레이블
left_label1 = tk.Label(root, text=f"왼쪽 모터 : {round(speedleft1, 2)}")
left_label1.pack()

right_label1 = tk.Label(root, text=f"오른쪽 모터 : {round(speedright1, 2)}")
right_label1.pack()

distance_label = tk.Label(root, text=f"마커까지 거리: {round(distance_to_marker, 2)} cm")
distance_label.pack()

# 긴급 정지 메시지 레이블
emergency_label = tk.Label(root, text=emergency_stop_message, fg="red")
emergency_label.pack()

# 제어 프레임 설정
control_frame = tk.Frame(root)
control_frame.pack()

btn_forward = tk.Button(control_frame, text="전진 (w)", command=increase_speed)
btn_forward.grid(row=0, column=1)

btn_left = tk.Button(control_frame, text="좌회전 (a)", command=steer_left)
btn_left.grid(row=1, column=0)

btn_stop = tk.Button(control_frame, text="정지 (q)", command=stop_motors)
btn_stop.grid(row=1, column=1)

btn_right = tk.Button(control_frame, text="우회전 (d)", command=steer_right)
btn_right.grid(row=1, column=2)

btn_backward = tk.Button(control_frame, text="후진 (s)", command=decrease_speed)
btn_backward.grid(row=2, column=1)

btn_quit = tk.Button(control_frame, text="종료 (x)", command=quit_program)
btn_quit.grid(row=3, column=1)

# 모드 선택 라디오 버튼
mode_var = tk.IntVar()
mode_var.set(1)  # 기본 모드는 원격 제어 모드

mode_frame = tk.Frame(root)
mode_frame.pack()

mode_label = tk.Label(mode_frame, text="모드 선택:")
mode_label.grid(row=0, column=0)

remote_mode = tk.Radiobutton(mode_frame, text="원격 제어", variable=mode_var, value=1)
remote_mode.grid(row=0, column=1)

aruco_mode = tk.Radiobutton(mode_frame, text="ArUco 마커 추적", variable=mode_var, value=2)
aruco_mode.grid(row=0, column=2)

# 모드 변경 추적
mode_var.trace('w', mode_selection_changed)

# 영상 표시 라벨
video_label = tk.Label(root)
video_label.pack()

# 웹캠 및 ArUco 마커 검출 설정
cap = cv2.VideoCapture(0, cv2.CAP_V4L)  # 웹캠 인덱스 0 사용

# 키보드 바인딩
root.bind("<w>", lambda event: increase_speed())
root.bind("<s>", lambda event: decrease_speed())
root.bind("<a>", lambda event: steer_left())
root.bind("<d>", lambda event: steer_right())
root.bind("<q>", lambda event: stop_motors())
root.bind("<x>", lambda event: quit_program())

# 영상 피드 업데이트 함수
def update_video_feed():
    _, frame = cap.read()
    if frame is not None:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if mode_var.get() == 2:  # ArUco 모드인 경우에는 마커 오버레이를 그리지 않음
            markers = detect_aruco_markers(frame)
            if markers:
                draw_marker_overlay(frame, markers)
        frame = Image.fromarray(frame)
        frame = ImageTk.PhotoImage(frame)
        video_label.imgtk = frame
        video_label.config(image=frame)
    video_label.after(10, update_video_feed)  # 10 밀리초마다 영상 업데이트

update_video_feed()
update_serial_data()

root.mainloop()

