import numpy as np
import serial
import struct
import time

# 시리얼 포트 설정 (포트와 속도는 환경에 맞게 수정하세요)
ser = serial.Serial('/dev/ttyUSB0', 9600)

Re_buf = bytearray(11)
a = [0.0, 0.0, 0.0]
w = [0.0, 0.0, 0.0]
angle = [0.0, 0.0, 0.0]
T = 0.0
yaw = 0.0
previous_time = time.time()

accel_offsets = np.zeros(3)
gyro_offsets = np.zeros(3)
angle_offsets = np.zeros(3)

# 캘리브레이션을 위한 데이터 수집 함수
def collect_calibration_data(duration=10):
    global accel_offsets, gyro_offsets, angle_offsets
    
    accel_data = []
    gyro_data = []
    angle_data = []
    
    start_time = time.time()
    
    print("Calibrating... Please keep the IMU stationary.")
    
    while time.time() - start_time < duration:
        if ser.in_waiting:
            Re_buf[:] = ser.read(11)
            if Re_buf[0] == 0x55:
                if Re_buf[1] == 0x51:
                    # 가속도계 데이터 수집
                    ax = struct.unpack('h', Re_buf[3:5])[0] / 32768.0 * 16
                    ay = struct.unpack('h', Re_buf[5:7])[0] / 32768.0 * 16
                    az = struct.unpack('h', Re_buf[7:9])[0] / 32768.0 * 16
                    accel_data.append([ax, ay, az])
                elif Re_buf[1] == 0x52:
                    # 자이로스코프 데이터 수집
                    gx = struct.unpack('h', Re_buf[3:5])[0] / 32768.0 * 2000
                    gy = struct.unpack('h', Re_buf[5:7])[0] / 32768.0 * 2000
                    gz = struct.unpack('h', Re_buf[7:9])[0] / 32768.0 * 2000
                    gyro_data.append([gx, gy, gz])
                elif Re_buf[1] == 0x53:
                    angle_x = struct.unpack('h', Re_buf[3:5])[0] / 32768.0 * 180
                    angle_y = struct.unpack('h', Re_buf[5:7])[0] / 32768.0 * 180
                    angle_z = struct.unpack('h', Re_buf[7:9])[0] / 32768.0 * 180
                    angle_data.append([angle_x, angle_y, angle_z])
    
    # 평균 오프셋 계산
    accel_offsets = np.mean(accel_data, axis=0)
    gyro_offsets = np.mean(gyro_data, axis=0)
    angle_offsets = np.mean(angle_data, axis=0)
    
    # Z축 가속도는 1g(9.81 m/s^2) 값이므로 이를 기준으로 보정
    accel_offsets[2] -= 9.81
    
    print("Calibration complete.")
    print(f"Accelerometer Offsets: {accel_offsets}")
    print(f"Gyroscope Offsets: {gyro_offsets}")
    print(f"Angle Offsets: {angle_offsets}")

# 캘리브레이션된 데이터를 적용하여 처리하는 함수
def process_data():
    global a, w, angle, T, yaw, previous_time, accel_offsets, gyro_offsets, angle_offsets

    current_time = time.time()
    dt = current_time - previous_time  # Δt: 이전 데이터 읽기와의 시간 차이
    previous_time = current_time

    if Re_buf[0] == 0x55:  # 프레임 헤더 확인
        if Re_buf[1] == 0x51:
            a[0] = struct.unpack('h', Re_buf[3:5])[0] / 32768.0 * 16 - accel_offsets[0]
            a[1] = struct.unpack('h', Re_buf[5:7])[0] / 32768.0 * 16 - accel_offsets[1]
            a[2] = struct.unpack('h', Re_buf[7:9])[0] / 32768.0 * 16 - accel_offsets[2]
            T = struct.unpack('h', Re_buf[9:11])[0] / 340.0 + 36.25
        elif Re_buf[1] == 0x52:
            w[0] = struct.unpack('h', Re_buf[3:5])[0] / 32768.0 * 2000 - gyro_offsets[0]
            w[1] = struct.unpack('h', Re_buf[5:7])[0] / 32768.0 * 2000 - gyro_offsets[1]
            w[2] = struct.unpack('h', Re_buf[7:9])[0] / 32768.0 * 2000 - gyro_offsets[2]
            T = struct.unpack('h', Re_buf[9:11])[0] / 340.0 + 36.25
        elif Re_buf[1] == 0x53:
            angle[0] = struct.unpack('h', Re_buf[3:5])[0] / 32768.0 * 180 - angle_offsets[0]  # Roll 보정
            angle[1] = struct.unpack('h', Re_buf[5:7])[0] / 32768.0 * 180 - angle_offsets[1]  # Pitch 보정
            angle[2] = struct.unpack('h', Re_buf[7:9])[0] / 32768.0 * 180 - angle_offsets[2]  # Yaw 보정
            T = struct.unpack('h', Re_buf[9:11])[0] / 340.0 + 36.25

            yaw += (w[2] * dt)

            print(f"Calibrated a: {a[0]:.2f} {a[1]:.2f} {a[2]:.2f} w: {w[0]:.2f} {w[1]:.2f} {w[2]:.2f} angle: {angle[0]:.2f} {angle[1]:.2f} {angle[2]:.2f} yaw; {yaw:.2f}")

def main():
    collect_calibration_data(duration=10)
    counter = 0
    while True:
        if ser.in_waiting:
            Re_buf[counter] = ser.read(1)[0]
            if counter == 0 and Re_buf[0] != 0x55:
                continue
            counter += 1
            if counter == 11:
                counter = 0
                process_data()

if __name__ == '__main__':
    main()
