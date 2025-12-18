#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from ros_controller_pkg.msg import PlcStatus          # PlcStatus.msg (is_empty, fence_open)
from std_srvs.srv import SetBool                      # 검사 서비스용
from std_msgs.msg import Bool                         # door_state, start_task 용

import serial
import threading
import time

# 시리얼 포트 설정 (필요하면 수정)
PORT = "/dev/ttyUSB0"
BAUD = 9600
SLAVE_ID = 3

# PLC BIT 주소 매핑
M0  = 0x0000  # door_state 명령용 (PLC -> STM32)
M1  = 0x0001  # is_empty용
M2  = 0x0002  # 검사 요청
M3  = 0x0014  # 검사 결과 (PC -> PLC)  ※ 통신 테이블에 맞게 설정
M4  = 0x0004  # fence_open 상태
M31 = 0x0011  # 작업 시작 버튼 (M31)  -> /plc/start_task

# coils 메모리 (PC 측 상태 테이블)
coils = [0] * 256


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC16 계산"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class PLCNode(Node):

    def __init__(self):
        super().__init__("plc_node")

        self.get_logger().info("PLC Node Started.")

        # ───── PlcStatus 상태 변수 ─────
        self.is_empty = True
        self.fence_open = False

        # ───── /plc/status_ros 퍼블리셔 ─────
        self.pub_status = self.create_publisher(PlcStatus, '/plc/status_ros', 10)

        # ───── /plc/door_state 퍼블리셔 (M0 → STM32) ─────
        self.pub_door_state = self.create_publisher(Bool, '/plc/door_state', 10)

        # ───── /plc/start_task 퍼블리셔 (M31 → ros_controller → robot_arm) ─────
        self.pub_start_task = self.create_publisher(Bool, '/plc/start_task', 10)

        # ───── 검사 서비스 클라이언트 (/plc/robotarm_detect) ─────
        # ros_controller가 이 서비스 서버가 될 예정
        self.detect_client = self.create_client(SetBool, '/plc/robotarm_detect')

        # ───── M2 / M31 엣지검출 및 busy 플래그 ─────
        self.m2_prev = 0          # 이전 M2 값 기억
        self.detect_busy = False  # 검사 진행중이면 True

        self.m31_prev = 0         # 작업 시작 버튼(M31) 이전 상태

        # ───── Modbus RTU 시리얼 ─────
        self.ser = serial.Serial(
            PORT, BAUD,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.05
        )

        # 시리얼 수신 스레드 시작
        threading.Thread(target=self.serial_loop, daemon=True).start()

    # ==============================
    # 시리얼 루프 (Modbus Slave)
    # ==============================
    def serial_loop(self):
        buf = bytearray()

        while True:
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)

                # 이 예제에서는 8바이트 고정 프레임 처리
                while len(buf) >= 8:
                    frame = bytes(buf[:8])
                    buf = buf[8:]
                    self.handle_request(frame)

            time.sleep(0.01)

    # ==============================
    # Modbus 요청 처리
    # ==============================
    def handle_request(self, frame: bytes):

        if len(frame) < 8:
            return

        slave = frame[0]
        func = frame[1]

        # 슬레이브 ID 체크
        if slave != SLAVE_ID:
            return

        # CRC 체크
        recv_crc = frame[-2] | (frame[-1] << 8)
        if recv_crc != int.from_bytes(crc16(frame[:-2]), 'little'):
            return

        addr = (frame[2] << 8) | frame[3]

        # WRITE SINGLE COIL (PLC → PC)
        if func == 0x05:
            value = (frame[4] == 0xFF)
            coils[addr] = 1 if value else 0

            # 에코 응답
            resp = frame[:-2]
            resp += crc16(resp)
            self.ser.write(resp)

            self.process_plc_bit(addr, coils[addr])

        # READ COILS (PLC → PC)
        elif func == 0x01:
            count = (frame[4] << 8) | frame[5]
            byte_val = 0

            for i in range(count):
                if coils[addr + i]:
                    byte_val |= (1 << i)

            resp = bytes([SLAVE_ID, 0x01, 0x01, byte_val])
            resp += crc16(resp)
            self.ser.write(resp)

    # ==============================
    # PLC 비트 이벤트 처리
    # ==============================
    def process_plc_bit(self, addr, val):
        self.get_logger().info(f"[PLC BIT] addr={addr}, val={val}")

        # ── 검사 요청 (M2, rising edge + busy 체크) ──
        if addr == M2:
            # 0 → 1 변화 + 검사중이 아닐 때만
            if val == 1 and self.m2_prev == 0 and not self.detect_busy:
                self.get_logger().warn(
                    "PLC M2 rising edge → /plc/robotarm_detect 서비스 요청!"
                )
                self.detect_busy = True
                self.call_robot_detect()

            # 이전 값 갱신
            self.m2_prev = val

        # ── door_state 명령 (M0) ──
        if addr == M0:
            msg = Bool()
            msg.data = (val == 1)  # 예: True=문 열어, False=문 닫아
            self.pub_door_state.publish(msg)
            self.get_logger().info(f"[PLC] door_state → /plc/door_state : {msg.data}")

        # ── 작업 시작 버튼 (M31, rising edge) ──
        if addr == M31:
            if val == 1 and self.m31_prev == 0:
                msg = Bool()
                msg.data = True
                self.pub_start_task.publish(msg)
                self.get_logger().info(
                    "[PLC] M31 rising edge → /plc/start_task : True"
                )
            self.m31_prev = val

        # ── is_empty (M1만 사용) ──
        if addr == M1:
            # M1 = 1 → 물건 있음  → is_empty=False
            # M1 = 0 → 비어 있음 → is_empty=True
            self.is_empty = bool(coils[M1])

        # ── fence_open (M4) ──
        if addr == M4:
            self.fence_open = (val == 1)

        # 상태 비트가 바뀌면 /plc/status_ros 갱신
        if addr in (M1, M4):
            self.publish_status()

    # ==============================
    # /plc/status_ros 퍼블리시
    # ==============================
    def publish_status(self):
        msg = PlcStatus()
        msg.is_empty = self.is_empty
        msg.fence_open = self.fence_open

        self.pub_status.publish(msg)

        self.get_logger().info(
            f"[PLC STATUS] is_empty={msg.is_empty}, "
            f"fence_open={msg.fence_open}"
        )

    # ==============================
    # /plc/robotarm_detect 서비스 호출 (SetBool)
    # ==============================
    def call_robot_detect(self):

        if not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/plc/robotarm_detect 서비스 없음! (ros_controller 확인 필요)")
            # 서비스가 아예 없을 때도 busy 풀어줘야 다음 요청 가능
            self.detect_busy = False
            return

        req = SetBool.Request()
        req.data = True   # True → 검사 시작 트리거

        future = self.detect_client.call_async(req)
        future.add_done_callback(self.on_robot_result)

    # ==============================
    # BAD일 때 3초 후 M3 리셋
    # ==============================
    def reset_m3_after_delay(self, delay_sec: float):
        """BAD 결과를 PLC에 3초간만 표시하고 다시 0으로 리셋"""
        time.sleep(delay_sec)
        coils[M3] = 0
        self.get_logger().info(f"[PLC] M3 결과 리셋: {coils[M3]}")

    # ==============================
    # 검사 결과 처리 → PLC M3에 기록 (3초 펄스)
    # ==============================
    def on_robot_result(self, future):
        try:
            resp = future.result()   # SetBool.Response
            # resp.success: True → GOOD, False → BAD
            self.get_logger().info(
                f"[RobotArm] 검사 결과 success={resp.success}, message='{resp.message}'"
            )

            if resp.success:
                # GOOD → M3 = 0
                coils[M3] = 0
                self.get_logger().info(f"[PLC] GOOD 결과, M3 코일 = {coils[M3]}")
            else:
                # BAD → M3 = 1 (3초 동안 유지 후 0으로 리셋)
                coils[M3] = 1
                self.get_logger().info(
                    f"[PLC] BAD 결과, M3 코일 = {coils[M3]} (3초 후 자동 리셋)"
                )

                # 5초 뒤에 M3를 0으로 되돌리는 스레드 실행
                threading.Thread(
                    target=self.reset_m3_after_delay,
                    args=(10.0,),
                    daemon=True
                ).start()

        except Exception as e:
            self.get_logger().error(f"[ERROR] 검사 결과 처리 실패: {e}")
        finally:
            # 검사 완료 → 다시 다음 요청 받을 수 있게
            self.detect_busy = False


def main():
    rclpy.init()
    node = PLCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
