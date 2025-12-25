#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from ros_controller_pkg.msg import PlcStatus          # PlcStatus.msg (is_empty, fence_open)
from std_srvs.srv import SetBool                      # 검사 서비스용
from std_msgs.msg import Bool                         # door_state용

import serial
import threading
import time
import mysql.connector   # 🔥 DB 사용

# ==============================
# 시리얼 포트 / Modbus 설정
# ==============================
PORT = "/dev/ttyUSB1"
BAUD = 9600
SLAVE_ID = 3

# ==============================
# PLC BIT / WORD 주소 매핑
# ==============================
# ── 기존 plc_node 용 ──
M0 = 0x0000  # door_state 명령용 (PLC -> STM32)
M1 = 0x0001  # is_empty용
M2 = 0x0002  # 검사 요청
M3 = 0x0014  # 검사 결과
M4 = 0x0004  # fence_open 상태
# M5 = 0x0005  # 🔥 door_open 은 사용 안 함

# ── DB 로깅용 (기존 db.py에서 쓰던 주소) ──
COIL_M0_ADDR  = 0x0000    # M0  → 안전펜스 ON/OFF
COIL_M4_ADDR  = 0x0004    # M4  → 추가 신호 ON/OFF
COIL_M70_ADDR = 0x0070    # M70 → 컨베이어 ON/OFF

REG_D100_ADDR = 0x0020    # D100 → 주파수
REG_D500_ADDR = 0x0021    # D500 → 전류

# ==============================
# 메모리 공간
# ==============================
# db.py 기준으로 넉넉하게 맞춰줌
coils = [0] * 1024          # BIT
holding_regs = [0] * 65536  # WORD

# ==============================
# 🔥 MySQL 설정
# ==============================
DB_HOST = "172.30.1.96"
DB_USER = "rosuser"
DB_PASSWORD = "1234"
DB_NAME = "dbdb"


def insert_plc_to_db(m0, m4, m70, d100, d500):
    """
    PLC에서 받은 M0/M4/M70/D100/D500 값을
    plc_conveyor_log 테이블에 INSERT

    - m0  : 0 또는 1 (안전펜스)
    - m4  : 0 또는 1 (추가 신호)
    - m70 : 0 또는 1 (컨베이어 RUN)
    - d100: 주파수 원시 값
    - d500: 전류 원시 값
    """

    # 필요하면 여기서 스케일 조정 (예: /10.0, /100.0 등)
    m0_state      = int(m0)
    m4_state      = int(m4)
    running       = int(m70)
    frequency     = float(d100)
    motor_current = float(d500)

    try:
        conn = mysql.connector.connect(
            host=DB_HOST,
            user=DB_USER,
            password=DB_PASSWORD,
            database=DB_NAME,
        )
        cursor = conn.cursor()

        sql = """
            INSERT INTO plc_conveyor_log (running, m0_state, m4_state, frequency, motor_current)
            VALUES (%s, %s, %s, %s, %s)
        """
        cursor.execute(sql, (running, m0_state, m4_state, frequency, motor_current))
        conn.commit()

        print(
            f"[DB] INSERT plc_conveyor_log OK  "
            f"(M0={m0_state}, M4={m4_state}, run={running}, "
            f"freq={frequency}, cur={motor_current})"
        )

    except Exception as e:
        print(f"[DB ERROR] PLC INSERT 실패: {e}")

    finally:
        try:
            cursor.close()
            conn.close()
        except Exception:
            pass


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

        # ───── 검사 서비스 클라이언트 (/plc/robotarm_detect) ─────
        self.detect_client = self.create_client(SetBool, '/plc/robotarm_detect')

        # ───── M2 엣지검출 및 busy 플래그 ─────
        self.m2_prev = 0          # 이전 M2 값 기억
        self.detect_busy = False  # 검사 진행중이면 True

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

        # DB 모니터링 스레드 시작 (M0/M4/M70/D100/D500)
        threading.Thread(target=self.monitor_loop, daemon=True).start()

    # ==============================
    # 시리얼 루프 (Modbus Slave)
    # ==============================
    def serial_loop(self):
        buf = bytearray()

        while True:
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)

                # 8바이트 고정 프레임 처리
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

            print(f"[PLC→PC BIT] WRITE addr={addr:#06x}, value={coils[addr]}")

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

            print(f"[PC→PLC BIT] READ addr={addr:#06x}, send={byte_val:#04x}")

        # WRITE SINGLE REGISTER (PLC → PC) 0x06
        elif func == 0x06:
            value = (frame[4] << 8) | frame[5]
            holding_regs[addr] = value

            resp = frame[:-2]
            resp += crc16(resp)
            self.ser.write(resp)

            print(f"[PLC→PC WORD] WRITE REG addr={addr:#06x}, value={value}")

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
                coils[M3] = 0 
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

        # ── is_empty (M1만 사용) ──
        if addr == M1:
            # M1 = 1 → 물건 있음  → is_empty=False
            # M1 = 0 → 비어 있음 → is_empty=True
            self.is_empty = not bool(coils[M1])

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
        finally:
            # 검사 완료 → 다시 다음 요청 받을 수 있게
            self.detect_busy = False

    # ==============================
    # DB 모니터링 스레드
    # ==============================
    def monitor_loop(self):
        """
        M0, M4, M70, D100, D500 값이 바뀔 때마다 DB에 한 줄씩 INSERT.
        (기존 db.py의 monitor_loop 통합)
        """
        prev_m0 = None
        prev_m4 = None
        prev_m70 = None
        prev_d100 = None
        prev_d500 = None

        while True:
            m0   = coils[COIL_M0_ADDR]          # 안전펜스
            m4   = coils[COIL_M4_ADDR]          # 추가 신호
            m70  = coils[COIL_M70_ADDR]         # 컨베이어 RUN
            d100 = holding_regs[REG_D100_ADDR]  # 주파수
            d500 = holding_regs[REG_D500_ADDR]  # 전류

            if (m0, m4, m70, d100, d500) != (prev_m0, prev_m4, prev_m70, prev_d100, prev_d500):
                print(f"[MONITOR] M0={m0}, M4={m4}, M70={m70}, D100={d100}, D500={d500}")
                prev_m0, prev_m4, prev_m70, prev_d100, prev_d500 = m0, m4, m70, d100, d500

                # 🔥 DB로 한 줄 INSERT
                insert_plc_to_db(m0, m4, m70, d100, d500)

            time.sleep(0.1)  # 100ms마다 체크


def main():
    rclpy.init()
    node = PLCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
