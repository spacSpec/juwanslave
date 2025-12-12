#!/usr/bin/env python3
# db_func.py
import mysql.connector
# ==============================
# MySQL 접속 설정 (우분투 → 윈도우)
# ==============================
DB_HOST = "172.30.1.96"   # :작은_파란색_다이아몬드: 윈도우 노트북 IP (ipconfig에서 본 IPv4 주소)
DB_USER = "rosuser"       # :작은_파란색_다이아몬드: 아까 만든 계정 이름
DB_PASSWORD = "1234"
DB_NAME = "dbdb"
# 모듈 import 될 때 한 번만 연결
conn = mysql.connector.connect(
    host=DB_HOST,
    user=DB_USER,
    password=DB_PASSWORD,
    database=DB_NAME
)
cursor = conn.cursor()
print(f"[db_func] MySQL 연결 완료 (host={DB_HOST}, user={DB_USER})")
# ==============================
# 로봇암 전류/온도 INSERT 함수
# ==============================
def insert_robot_power(currents, temps):
    """
    로봇암 6축 전류/온도 값을 robot_power_log 테이블에 INSERT
    currents: 길이 6 리스트/튜플 [c1, c2, c3, c4, c5, c6]
    temps   : 길이 6 리스트/튜플 [t1, t2, t3, t4, t5, t6]
    """
    if len(currents) != 6 or len(temps) != 6:
        raise ValueError("currents, temps는 길이 6 리스트여야 합니다.")
    sql = """
        INSERT INTO robot_power_log (
            j1_current, j2_current, j3_current, j4_current, j5_current, j6_current,
            j1_temp,    j2_temp,    j3_temp,    j4_temp,    j5_temp,    j6_temp
        ) VALUES (%s, %s, %s, %s, %s, %s,
                  %s, %s, %s, %s, %s, %s)
    """
    data = (
        float(currents[0]), float(currents[1]), float(currents[2]),
        float(currents[3]), float(currents[4]), float(currents[5]),
        float(temps[0]),    float(temps[1]),    float(temps[2]),
        float(temps[3]),    float(temps[4]),    float(temps[5]),
    )
    cursor.execute(sql, data)
    conn.commit()
    print("[db_func] INSERT robot_power_log OK")
# ==============================
# 종료용 함수 (원하면 노드 종료 시 호출)
# ==============================
def close_db():
    cursor.close()
    conn.close()
    print("[db_func] DB 연결 종료")
