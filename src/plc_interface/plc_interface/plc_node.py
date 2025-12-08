#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from ros_controller_pkg.msg import PlcStatus          # PlcStatus.msg (is_empty, fence_open, door_open)
from std_srvs.srv import SetBool                      # 검사 서비스용
from std_msgs.msg import Bool                         # door_state용

import serial
import threading
import time

# 시리얼 포트 설정 (필요하면 수정)
PORT = "/dev/ttyUSB0"
BAUD = 9600
SLAVE_ID = 3

# PLC BIT 주소 매핑
M0 = 0x0000  # door_state 명령용 (PLC -> STM32)
M1 = 0x0001  # is_empty용
M2 = 0x0002  # 검사 요청
M3 = 0x0003  # 검사 결과
M4 = 0x0004  # fence_open 상태
M5 = 0x0005  # door_open 상태

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
        self.door_open = False

        # ───── /plc/status_ros 퍼블리셔 ─────
        self.pub_status = self.create_publisher(PlcStatus, '/plc/status_ros', 10)

        # ───── /plc/door_state 퍼블리셔 (M0 → STM32) ─────
        self.pub_door_state = self.create_publisher(Bool, '/plc/door_state', 10)

        # ───── 검사 서비스 클라이언트 (/plc/robotarm_detect) ─────
        # ros_controller가 이 서비스 서버가 될 예정
        self.detect_client = self.create_client(SetBool, '/plc/robotarm_detect')

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

        # ── 검사 요청 (M2) ──
        if addr == M2 and val == 1:
            self.get_logger().warn("PLC M2=1 → /plc/robotarm_detect 서비스 요청!")
            self.call_robot_detect()

        # ── door_state 명령 (M0) ──
        if addr == M0:
            msg = Bool()
            msg.data = (val == 1)  # 예: True=문 열어, False=문 닫아
            self.pub_door_state.publish(msg)
            self.get_logger().info(f"[PLC] door_state → /plc/door_state : {msg.data}")

        # ── is_empty (M1만 사용) ──
        if addr == M1:
            # M1 = 1 → 물건 있음  → is_empty=False
            # M1 = 0 → 비어 있음 → is_empty=True
            self.is_empty = not bool(coils[M1])

        # ── fence_open (M4) ──
        if addr == M4:
            self.fence_open = (val == 1)

        # ── door_open (M5) ──
        if addr == M5:
            self.door_open = (val == 1)

        # 상태 비트가 바뀌면 /plc/status_ros 갱신
        if addr in (M1, M4, M5):
            self.publish_status()

    # ==============================
    # /plc/status_ros 퍼블리시
    # ==============================
    def publish_status(self):
        msg = PlcStatus()
        msg.is_empty = self.is_empty
        msg.fence_open = self.fence_open
        msg.door_open = self.door_open

        self.pub_status.publish(msg)

        self.get_logger().info(
            f"[PLC STATUS] is_empty={msg.is_empty}, "
            f"fence_open={msg.fence_open}, door_open={msg.door_open}"
        )

    # ==============================
    # /plc/robotarm_detect 서비스 호출 (SetBool)
    # ==============================
    def call_robot_detect(self):

        if not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/plc/robotarm_detect 서비스 없음! (ros_controller 확인 필요)")
            return

        req = SetBool.Request()
        req.data = True   # True → 검사 시작 트리거

        future = self.detect_client.call_async(req)
        future.add_done_callback(self.on_robot_result)

    # ==============================
    # 검사 결과 처리 → PLC M3에 기록
    # ==============================
    def on_robot_result(self, future):
        try:
            resp = future.result()   # SetBool.Response
            # resp.success: True → GOOD, False → BAD
            self.get_logger().info(
                f"[RobotArm] 검사 결과 success={resp.success}, message='{resp.message}'"
            )

            # GOOD → M3 = 0, BAD → M3 = 1
            coils[M3] = 0 if resp.success else 1

            self.get_logger().info(f"[PLC] 검사 결과 M3 코일에 반영: {coils[M3]}")

        except Exception as e:
            self.get_logger().error(f"[ERROR] 검사 결과 처리 실패: {e}")


def main():
    rclpy.init()
    node = PLCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
