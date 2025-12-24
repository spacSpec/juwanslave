#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import mysql.connector

# 🔹 윈도우 MySQL 서버 정보 (네 환경 그대로 사용)
DB_HOST = "172.30.1.96"   # 윈도우 PC IP
DB_USER = "rosuser"       # 이미 만들어 둔 계정
DB_PASSWORD = "1234"
DB_NAME = "dbdb"


def insert_agv_battery(battery_level, current=None):
    """
    AGV 배터리 상태를 agv_power_log 테이블에 1줄 INSERT

    battery_level : 배터리 퍼센트 (0~100)
    current       : 전류(A) 값, 없으면 None 으로 넘기면 DB에 NULL 저장
    """
    try:
        conn = mysql.connector.connect(
            host=DB_HOST,
            user=DB_USER,
            password=DB_PASSWORD,
            database=DB_NAME,
        )
        cursor = conn.cursor()

        sql = """
            INSERT INTO agv_power_log (battery_level, current)
            VALUES (%s, %s)
        """

        # current 값을 안 쓰면 None → DB에서 NULL
        data = (float(battery_level), float(current) if current is not None else None)

        cursor.execute(sql, data)
        conn.commit()

        print(f"[db_agv] INSERT agv_power_log OK (batt={battery_level}, curr={current})")

    except Exception as e:
        print("[db_agv] DB INSERT 실패:", e)

    finally:
        try:
            cursor.close()
            conn.close()
        except:
            pass