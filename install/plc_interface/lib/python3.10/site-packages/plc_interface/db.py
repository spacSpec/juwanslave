#!/usr/bin/env python3

# -*- coding: utf-8 -*-



import serial

import threading

import time

import mysql.connector  # 🔥 DB 사용


# ==========================================

# 슬레이브 PC 기본 설정

# ==========================================

PORT = "/dev/ttyUSB0"   # 🔹 이 코드 돌리는 PC의 시리얼 포트 이름

                 #   (윈도우: "COM3", "COM10" / 리눅스: "/dev/ttyUSB0")

BAUD = 9600

SLAVE_ID = 3


# ==========================================

# BIT / WORD 주소 매핑

# ==========================================

COIL_M70_ADDR   = 0x0070    # M70  (BIT, Modbus: 0x00070)  → 컨베이어 ON/OFF

REG_D100_ADDR   = 0x0020    # D100 (WORD, Modbus: 0x40020) → 주파수용 (예시)

REG_D500_ADDR   = 0x0021    # D500 (WORD, Modbus: 0x40021) → 전류용 (예시)



# ==========================================

# 메모리 공간

# ==========================================

coils = [0] * 1024          # BIT (필요한 만큼 넉넉히)

holding_regs = [0] * 65536  # WORD



# ==========================================

# 🔥 MySQL 설정 (DB 서버: 172.30.1.96)

# ==========================================

DB_HOST = "172.30.1.96"   # 🔹 MySQL 설치된 PC IP (네가 말한 IP)

DB_USER = "rosuser"       # 🔹 우리가 만든 계정

DB_PASSWORD = "1234"

DB_NAME = "dbdb"





def insert_plc_to_db(m70, d100, d500):

    """

    PLC에서 받은 M70/D100/D500 값을

    plc_conveyor_log 테이블에 INSERT



    - m70  : 0 또는 1 (컨베이어 정지/동작)

    - d100 : 주파수 원시 값 (PLC에서 온 정수)

    - d500 : 전류 원시 값 (PLC에서 온 정수)

    """



    # 🔧 필요하면 여기서 스케일 조정

    # 예: D100 = 600 → 60.0 Hz 이면 이렇게:

    # frequency = d100 / 10.0

    # 예: D500 = 23 → 2.3 A 이면:

    # motor_current = d500 / 10.0



    running = int(m70)

    frequency = float(d100)       # 필요하면 /10.0 등으로 수정

    motor_current = float(d500)   # 필요하면 /10.0 등으로 수정


    try:

        conn = mysql.connector.connect(

            host=DB_HOST,

            user=DB_USER,

            password=DB_PASSWORD,

            database=DB_NAME

        )

        cursor = conn.cursor()



        sql = """

            INSERT INTO plc_conveyor_log (running, frequency, motor_current)

            VALUES (%s, %s, %s)

        """

        cursor.execute(sql, (running, frequency, motor_current))

        conn.commit()


        print(f"[DB] INSERT plc_conveyor_log OK  "

              f"(run={running}, freq={frequency}, cur={motor_current})")


    except Exception as e:

        print(f"[DB ERROR] PLC INSERT 실패: {e}")



    finally:

        try:

            cursor.close()

            conn.close()

        except:

            pass





# ==========================================

# CRC16 (Modbus RTU)

# ==========================================

def crc16(data: bytes) -> bytes:

    crc = 0xFFFF

    for b in data:

        crc ^= b

        for _ in range(8):

            if crc & 1:

                crc = (crc >> 1) ^ 0xA001

            else:

                crc >>= 1

    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])





# ==========================================

# 요청 처리 함수

# ==========================================

def handle_request(frame: bytes, ser: serial.Serial):

    if len(frame) < 8:

        return



    slave = frame[0]

    func = frame[1]



    # 슬레이브 ID 체크

    if slave != SLAVE_ID:

        return



    # CRC 체크

    recv_crc = frame[-2] | (frame[-1] << 8)

    calc_crc = int.from_bytes(crc16(frame[:-2]), 'little')

    if recv_crc != calc_crc:

        # CRC 안 맞으면 무시

        return



    # 주소 파싱

    addr = (frame[2] << 8) | frame[3]



    # ----------------------------------------------------------

    # 0x05 - WRITE SINGLE COIL (PLC → PC)

    # ----------------------------------------------------------

    if func == 0x05:

        value = (frame[4] == 0xFF)

        coils[addr] = 1 if value else 0



        # 에코 응답

        resp = frame[:-2]

        resp += crc16(resp)

        ser.write(resp)


        print(f"[PLC→PC BIT] WRITE addr={addr:#06x}, value={coils[addr]}")



    # ----------------------------------------------------------

    # 0x01 - READ COILS (PC → PLC)

    # ----------------------------------------------------------

    elif func == 0x01:

        count = (frame[4] << 8) | frame[5]

        byte_val = 0

        for i in range(count):

            if coils[addr + i]:

                byte_val |= 1 << i



        resp = bytes([

            SLAVE_ID,

            0x01,

            0x01,         # 바이트 수 (지금은 1바이트만 전송)

            byte_val

        ])

        resp += crc16(resp)

        ser.write(resp)



        print(f"[PC→PLC BIT] READ addr={addr:#06x}, send={byte_val:#04x}")



    # ----------------------------------------------------------

    # 0x06 - WRITE SINGLE REGISTER (PLC → PC)

    # ----------------------------------------------------------

    elif func == 0x06:

        value = (frame[4] << 8) | frame[5]

        holding_regs[addr] = value



        resp = frame[:-2]

        resp += crc16(resp)

        ser.write(resp)



        print(f"[PLC→PC WORD] WRITE REG addr={addr:#06x}, value={value}")





# ==========================================

# 시리얼 수신 스레드

# ==========================================

def serial_loop(ser: serial.Serial):

    buf = bytearray()



    while True:

        if ser.in_waiting:

            buf += ser.read(ser.in_waiting)



            # 모드버스 프레임 최소 8바이트 기준으로 잘라서 처리

            while len(buf) >= 8:

                frame = bytes(buf[:8])

                buf = buf[8:]

                handle_request(frame, ser)



        time.sleep(0.01)




# ==========================================

# 모니터링 스레드 (M70, D100, D500)

# ==========================================

def monitor_loop():

    prev_m70 = None

    prev_d100 = None

    prev_d500 = None



    while True:

        m70  = coils[COIL_M70_ADDR]            # BIT

        d100 = holding_regs[REG_D100_ADDR]     # WORD

        d500 = holding_regs[REG_D500_ADDR]     # WORD



        # 값이 바뀔 때만 출력 + DB 인서트 (로그 폭주 방지)

        if (m70, d100, d500) != (prev_m70, prev_d100, prev_d500):

            print(f"[MONITOR] M70={m70}, D100={d100}, D500={d500}")

            prev_m70, prev_d100, prev_d500 = m70, d100, d500



            # 🔥 여기서 DB로 한 줄 INSERT

            insert_plc_to_db(m70, d100, d500)



        time.sleep(0.1)  # 100ms마다 체크 (원하면 조절 가능)




# ==========================================

# 메인 실행부

# ==========================================

def main():

    ser = serial.Serial(

        PORT, BAUD,

        bytesize=8,

        parity='N',

        stopbits=1,

        timeout=0.05

    )



    print("\n===== PC Modbus Slave 시작 =====")

    print(f"포트: {PORT}, 속도: {BAUD}, 슬레이브ID: {SLAVE_ID}")

    print(f"DB: host={DB_HOST}, user={DB_USER}, db={DB_NAME}\n")



    # Modbus 수신 스레드

    threading.Thread(target=serial_loop, args=(ser,), daemon=True).start()


    # 모니터링 스레드

    threading.Thread(target=monitor_loop, daemon=True).start()


    # 메인 스레드는 그냥 대기용

    try:

        while True:

            time.sleep(1.0)

    except KeyboardInterrupt:

        print("\n종료 요청됨(CTRL+C)")

    finally:

        ser.close()

        print("시리얼 포트 닫음, 프로그램 종료")




if __name__ == "__main__":

    main()